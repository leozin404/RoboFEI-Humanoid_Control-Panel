#include "control_panel_pkg/usb_monitor_node.hpp"

#include <sys/stat.h>

#include <rclcpp/executors/single_threaded_executor.hpp>

namespace control_panel_pkg {

template <typename MsgT>
void UsbMonitorNode::watch(const std::string &topic, rclcpp::Time &stamp) {
  // SensorDataQoS (best effort) connects to both reliable and best-effort
  // publishers, so the monitor never stays deaf because of a QoS mismatch.
  subs_.push_back(this->create_subscription<MsgT>(
      topic, rclcpp::SensorDataQoS(),
      [this, &stamp](const typename MsgT::SharedPtr) { stamp = this->now(); }));
}

UsbMonitorNode::UsbMonitorNode() : Node("usb_monitor_node") {
  comm_board_.path = this->declare_parameter<std::string>("comm_board_device", "/dev/motors");
  imu_.path = this->declare_parameter<std::string>("imu_device", "/dev/imu");
  cam_.path = this->declare_parameter<std::string>("cam_device", "/dev/camera");

  traffic_window_ms_ = this->declare_parameter<int>("traffic_window_ms", 500);
  cam_window_ms_ = this->declare_parameter<int>("cam_traffic_window_ms", 1500);

  watch<custom_interfaces::msg::JointState>(
      this->declare_parameter<std::string>("motors_topic", "/all_joints_position"), last_motors_);
  watch<sensor_msgs::msg::Imu>(
      this->declare_parameter<std::string>("imu_topic", "/imu/data"), last_imu_);
  watch<std_msgs::msg::Int32>(
      this->declare_parameter<std::string>("cam_topic", "/goalpost_count"), last_cam_);

  comm_board_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
      "/painel_controle/usb_status/comm_board", 10);
  imu_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
      "/painel_controle/usb_status/imu", 10);
  cam_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
      "/painel_controle/usb_status/cam", 10);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(POLL_PERIOD_MS), std::bind(&UsbMonitorNode::poll, this));
}

bool UsbMonitorNode::exists(const std::string &path) {
  struct stat st{};
  return stat(path.c_str(), &st) == 0;   // follows the udev symlink
}

bool UsbMonitorNode::recent(const rclcpp::Time &stamp, int windowMs) const {
  if (stamp.nanoseconds() == 0) return false;   // never received anything
  return (this->now() - stamp).nanoseconds() < static_cast<int64_t>(windowMs) * 1'000'000LL;
}

void UsbMonitorNode::poll() {
  const bool motorsTraffic = recent(last_motors_, traffic_window_ms_);
  const bool imuTraffic = recent(last_imu_, traffic_window_ms_);
  const bool camTraffic = recent(last_cam_, cam_window_ms_);

  publishState("Comm board", comm_board_, motorsTraffic, comm_board_pub_);
  publishState("IMU", imu_, imuTraffic, imu_pub_);
  publishState("Camera", cam_, camTraffic, cam_pub_);
}

void UsbMonitorNode::publishState(
    const char *label, Device &dev, bool traffic,
    const rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr &pub) {
  const bool present = exists(dev.path);
  if (present != dev.present) {
    RCLCPP_INFO(this->get_logger(), "%s (%s) %s", label, dev.path.c_str(),
                present ? "connected" : "disconnected");
    dev.present = present;
  }

  std_msgs::msg::UInt8 msg;
  msg.data = !present ? MODE_OFF : (traffic ? MODE_BLINK : MODE_ON);
  pub->publish(msg);
}

}  // namespace control_panel_pkg

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<control_panel_pkg::UsbMonitorNode>();
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
