#include "control_panel_pkg/system_status_node.hpp"
#include <rclcpp/executors/single_threaded_executor.hpp>

namespace control_panel_pkg {

SystemStatusNode::SystemStatusNode() : Node("system_status_node") {

  gc_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/game_controller_connected", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) { gc_connected_ = msg->data; });

  overload_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/motors/overload", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) { motor_overload_ = msg->data; });

  comm_board_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
      "/painel_controle/usb_status/comm_board", 10,
      [this](const std_msgs::msg::UInt8::SharedPtr msg) { comm_board_mode_ = msg->data; });

  imu_usb_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
      "/painel_controle/usb_status/imu", 10,
      [this](const std_msgs::msg::UInt8::SharedPtr msg) { imu_usb_mode_ = msg->data; });

  cam_usb_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
      "/painel_controle/usb_status/cam", 10,
      [this](const std_msgs::msg::UInt8::SharedPtr msg) { cam_usb_mode_ = msg->data; });

  button_event_sub_ = this->create_subscription<custom_interfaces::msg::ButtonEvent>(
      "/painel_controle/button_event", 10,
      std::bind(&SystemStatusNode::onButtonEvent, this, std::placeholders::_1));

  process_launched_["decision"] = false;
  process_launched_["control"] = false;
  process_launched_["robot_bringup"] = false;

  // Robot_Bringup aggregates: decision, vision, motors, um7, control (per Tabela 15).
  // Localization is tracked separately (LED_LOCALIZATION), not part of this group.
  registerHeartbeatSub("decision", "/decision/heartbeat");
  registerHeartbeatSub("vision", "/vision/heartbeat");
  registerHeartbeatSub("motors", "/motors/heartbeat");
  registerHeartbeatSub("um7", "/um7/heartbeat");
  registerHeartbeatSub("control", "/control/heartbeat");
  registerHeartbeatSub("localization", "/localization/heartbeat");

  status_pub_ = this->create_publisher<custom_interfaces::msg::PanelStatus>(
      "/painel_controle/status", 10);

  publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(PUBLISH_PERIOD_MS),
      std::bind(&SystemStatusNode::publishStatus, this));
}

void SystemStatusNode::registerHeartbeatSub(const std::string &name, const std::string &topic) {
  // Initialize as "never seen" — isHeartbeatAlive() will correctly report
  // false until the first heartbeat actually arrives. This is what lets
  // this node run standalone before other packages implement their side.
  last_heartbeat_[name] = rclcpp::Time(0, 0, RCL_ROS_TIME);

  heartbeat_subs_[name] = this->create_subscription<std_msgs::msg::Empty>(
      topic, 10,
      [this, name](const std_msgs::msg::Empty::SharedPtr) {
        last_heartbeat_[name] = this->now();
      });
}

bool SystemStatusNode::isHeartbeatAlive(const std::string &name) const {
  auto it = last_heartbeat_.find(name);
  if (it == last_heartbeat_.end()) return false;

  // Never received anything yet (still at the sentinel zero time).
  if (it->second.nanoseconds() == 0) return false;

  auto elapsed = this->now() - it->second;
  return elapsed.nanoseconds() < static_cast<int64_t>(HEARTBEAT_TIMEOUT_MS) * 1'000'000;
}

void SystemStatusNode::publishStatus() {
  custom_interfaces::msg::PanelStatus msg;

  msg.led_localization   = isHeartbeatAlive("localization") ? MODE_ON : MODE_OFF;
  msg.led_gamecontroller  = gc_connected_ ? MODE_ON : MODE_OFF;
  msg.led_comm_board      = comm_board_mode_;
  msg.led_overload        = motor_overload_ ? MODE_ON : MODE_OFF;
  msg.led_cam             = cam_usb_mode_;
  msg.led_imu             = imu_usb_mode_;

  bool bringupAlive = isHeartbeatAlive("decision") &&
                       isHeartbeatAlive("vision") &&
                       isHeartbeatAlive("motors") &&
                       isHeartbeatAlive("um7") &&
                       isHeartbeatAlive("control");
  msg.led_rbt_bu = bringupAlive ? MODE_ON : MODE_OFF;

  msg.stamp = this->now();
  status_pub_->publish(msg);
}

void SystemStatusNode::onButtonEvent(
    const custom_interfaces::msg::ButtonEvent::SharedPtr msg) {

  if (msg->button_id != 2) return;   // Button 1 is handled by panel_teleop_node

  // TODO: confirm the exact launch package/file names below against the
  // real RoboFEI-HT_2023_SOFTWARE repo before field use. These are
  // placeholders following the naming conventions discussed, not
  // confirmed launch commands.
  switch (msg->click_type) {
    case custom_interfaces::msg::ButtonEvent::SINGLE_CLICK:
      toggleProcess("decision",
                     "ros2 launch decision decision.launch.py &",
                     "decision_node");
      break;

    case custom_interfaces::msg::ButtonEvent::DOUBLE_CLICK:
      toggleProcess("control",
                     "ros2 launch control control.launch.py &",
                     "control_node");
      break;

    case custom_interfaces::msg::ButtonEvent::HOLD:
      toggleProcess("robot_bringup",
                     "ros2 launch start robot_bringup.launch.py &",
                     "robot_bringup");
      break;

    default:
      RCLCPP_WARN(this->get_logger(), "Unknown click_type %d on button 2",
                  msg->click_type);
  }
}

void SystemStatusNode::toggleProcess(const std::string &name,
                                       const std::string &launchCmd,
                                       const std::string &killPattern) {
  bool currentlyLaunched = process_launched_[name];

  if (currentlyLaunched) {
    RCLCPP_INFO(this->get_logger(), "Toggling '%s' OFF", name.c_str());
    std::string killCmd = "pkill -f '" + killPattern + "'";
    std::system(killCmd.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Toggling '%s' ON", name.c_str());
    std::system(launchCmd.c_str());
  }

  process_launched_[name] = !currentlyLaunched;

  // NOTE: this flag reflects intent, not confirmed reality. If the
  // launch/kill command silently fails, this flag will disagree with
  // the actual process state until the next toggle. The heartbeat
  // subscribers already tracked in this node are the source of truth
  // for whether the node is *actually* alive — consider cross-checking
  // isHeartbeatAlive(name) here in a future iteration, e.g. warning the
  // operator if a toggle doesn't produce the expected heartbeat change
  // within a few seconds.
}

}  // namespace control_panel_pkg

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<control_panel_pkg::SystemStatusNode>();
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
