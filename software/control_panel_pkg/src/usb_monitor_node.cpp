#include "control_panel_pkg/usb_monitor_node.hpp"

#include <sys/stat.h>
#include <rclcpp/executors/single_threaded_executor.hpp>

namespace control_panel_pkg {

UsbMonitorNode::UsbMonitorNode() : Node("usb_monitor_node") {
  comm_board_presence_.devicePath =
      this->declare_parameter<std::string>("comm_board_device", "/dev/commboard");
  imu_presence_.devicePath =
      this->declare_parameter<std::string>("imu_device", "/dev/imu");
  cam_presence_.devicePath =
      this->declare_parameter<std::string>("cam_device", "/dev/camera");

  std::string imuTopic = this->declare_parameter<std::string>("imu_topic", "/imu/data");
  std::string camTopic = this->declare_parameter<std::string>("cam_topic", "/image_raw");

  imu_data_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imuTopic, 10,
      [this](const sensor_msgs::msg::Imu::SharedPtr) { last_imu_msg_time_ = this->now(); });

  cam_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      camTopic, 10,
      [this](const sensor_msgs::msg::Image::SharedPtr) { last_cam_msg_time_ = this->now(); });

  comm_board_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
      "/painel_controle/usb_status/comm_board", 10);
  imu_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
      "/painel_controle/usb_status/imu", 10);
  cam_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
      "/painel_controle/usb_status/cam", 10);

  poll_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(POLL_PERIOD_MS),
      std::bind(&UsbMonitorNode::pollPresence, this));
}

bool UsbMonitorNode::checkPresence(const std::string &path) const {
  struct stat st{};
  return stat(path.c_str(), &st) == 0;
}

bool UsbMonitorNode::isRecentTraffic(const rclcpp::Time &lastMsgTime) const {
  if (lastMsgTime.nanoseconds() == 0) return false;   // never received anything
  auto elapsed = this->now() - lastMsgTime;
  return elapsed.nanoseconds() < static_cast<int64_t>(TRAFFIC_WINDOW_MS) * 1'000'000;
}

void UsbMonitorNode::pollPresence() {
  bool commPresentNow = checkPresence(comm_board_presence_.devicePath);
  bool imuPresentNow  = checkPresence(imu_presence_.devicePath);
  bool camPresentNow  = checkPresence(cam_presence_.devicePath);

  if (commPresentNow != comm_board_presence_.present) {
    RCLCPP_INFO(this->get_logger(), "%s %s.", comm_board_presence_.devicePath.c_str(),
                commPresentNow ? "connected" : "disconnected");
  }
  if (imuPresentNow != imu_presence_.present) {
    RCLCPP_INFO(this->get_logger(), "%s %s.", imu_presence_.devicePath.c_str(),
                imuPresentNow ? "connected" : "disconnected");
  }
  if (camPresentNow != cam_presence_.present) {
    RCLCPP_INFO(this->get_logger(), "%s %s.", cam_presence_.devicePath.c_str(),
                camPresentNow ? "connected" : "disconnected");
  }

  comm_board_presence_.present = commPresentNow;
  imu_presence_.present = imuPresentNow;
  cam_presence_.present = camPresentNow;

  publishAll();
}

void UsbMonitorNode::publishAll() {
  // comm_board: no dedicated topic of its own, approximated as "IMU
  // traffic flowed recently" (see header TODO — extend with motors
  // feedback once a suitable topic is confirmed).
  bool commTraffic = isRecentTraffic(last_imu_msg_time_);
  bool imuTraffic  = isRecentTraffic(last_imu_msg_time_);
  bool camTraffic  = isRecentTraffic(last_cam_msg_time_);

  std_msgs::msg::UInt8 commMsg;
  commMsg.data = !comm_board_presence_.present ? MODE_OFF
                 : commTraffic                  ? MODE_BLINK
                                                  : MODE_ON;
  comm_board_pub_->publish(commMsg);

  std_msgs::msg::UInt8 imuMsg;
  imuMsg.data = !imu_presence_.present ? MODE_OFF
                : imuTraffic            ? MODE_BLINK
                                         : MODE_ON;
  imu_pub_->publish(imuMsg);

  std_msgs::msg::UInt8 camMsg;
  camMsg.data = !cam_presence_.present ? MODE_OFF
                : camTraffic            ? MODE_BLINK
                                         : MODE_ON;
  cam_pub_->publish(camMsg);
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
