#include "control_panel_pkg/system_status_node.hpp"

#include <rclcpp/executors/single_threaded_executor.hpp>

#include <vector>

namespace control_panel_pkg {

// A message arriving on `topic` counts as a sign of life of node `name`.
// SensorDataQoS (best effort) connects to both reliable and best-effort
// publishers, so a QoS mismatch never leaves this monitor deaf.
template <typename MsgT>
void SystemStatusNode::watchSignal(const std::string &name, const std::string &topic) {
  signal_subs_.push_back(this->create_subscription<MsgT>(
      topic, rclcpp::SensorDataQoS(),
      [this, name](const typename MsgT::SharedPtr) { last_heartbeat_[name] = this->now(); }));
}

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

  // --- process control (Button 2) ---
  repo_dir_ = this->declare_parameter<std::string>("repo_dir", "~/RoboFEI-HT_2023_SOFTWARE");
  if (!process_utils::repoDirExists(repo_dir_)) {
    RCLCPP_ERROR(this->get_logger(),
                 "repo_dir '%s' does not exist: nothing started by Button 2 will work. "
                 "Launch with repo_dir:=<path to RoboFEI-HT_2023_SOFTWARE>.",
                 repo_dir_.c_str());
  }
  const std::string imuCmd =
      this->declare_parameter<std::string>("imu_command", "ros2 run um7 um7_node");

  using process_utils::launchSpec;
  using process_utils::ProcSpec;

  decision_spec_ = launchSpec("decision", "decision_bringup.launch.py");
  control_spec_ = launchSpec("control", "control_bringup.launch.py");

  // The "Robot Bringup" is the set of vital nodes the team starts by hand
  // today (one terminal each). robot_bringup.launch.py is NOT used: in the
  // repo it only includes vision and decision.
  bringup_ = std::make_unique<process_utils::BringupGroup>(
      repo_dir_,
      std::vector<ProcSpec>{
          launchSpec("motors", "motors_bringup.launch.py"),
          ProcSpec("imu", process_utils::splitWords(imuCmd)),
          launchSpec("vision", "vision_bringup.launch.py"),
          decision_spec_,
          control_spec_});

  // Robot_Bringup LED aggregates: decision, vision, motors, um7, control (Tabela 15).
  // Localization is tracked separately (LED_LOCALIZATION), not part of this group.
  registerHeartbeatSub("decision", "/decision/heartbeat");
  registerHeartbeatSub("vision", "/vision/heartbeat");
  registerHeartbeatSub("motors", "/motors/heartbeat");
  registerHeartbeatSub("um7", "/um7/heartbeat");
  registerHeartbeatSub("control", "/control/heartbeat");
  registerHeartbeatSub("localization", "/localization/heartbeat");

  // Natural signals: same table, no edits in the team packages.
  watchSignal<std_msgs::msg::Int32>("vision", "/goalpost_count");
  watchSignal<sensor_msgs::msg::Imu>("um7", "/imu/data");
  watchSignal<custom_interfaces::msg::JointState>("motors", "/all_joints_position");

  // localization publishes /localization_active (constant true) every 500 ms.
  signal_subs_.push_back(this->create_subscription<std_msgs::msg::Bool>(
      "/localization_active", rclcpp::SensorDataQoS(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) last_heartbeat_["localization"] = this->now();
      }));

  // control only reacts to goals, so "alive" = its action server is discoverable.
  control_client_ =
      rclcpp_action::create_client<custom_interfaces::action::Control>(this, "control_action");

  status_pub_ = this->create_publisher<custom_interfaces::msg::PanelStatus>(
      "/painel_controle/status", 10);

  publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(PUBLISH_PERIOD_MS),
      std::bind(&SystemStatusNode::publishStatus, this));
}

void SystemStatusNode::registerHeartbeatSub(const std::string &name, const std::string &topic) {
  // Initialize as "never seen": isHeartbeatAlive() reports false until the
  // first heartbeat actually arrives. This is what lets this node run
  // standalone before other packages implement their side.
  last_heartbeat_[name] = rclcpp::Time(0, 0, RCL_ROS_TIME);

  heartbeat_subs_[name] = this->create_subscription<std_msgs::msg::Empty>(
      topic, 10,
      [this, name](const std_msgs::msg::Empty::SharedPtr) {
        last_heartbeat_[name] = this->now();
      });
}

// True for "decision_node" and "<namespace>/decision_node", but not for
// something like "my_decision_node".
static bool isNodeNamed(const std::string &fullName, const std::string &base) {
  if (fullName == base) return true;
  if (fullName.size() <= base.size()) return false;
  const size_t start = fullName.size() - base.size();
  return fullName.compare(start, base.size(), base) == 0 && fullName[start - 1] == '/';
}

// Signals that come from the ROS graph instead of a message stream.
void SystemStatusNode::pollGraphSignals() {
  if (control_client_ && control_client_->action_server_is_ready()) {
    last_heartbeat_["control"] = this->now();
  }

  // decision publishes nothing periodic; its presence in the graph is the
  // best passive sign of life (fully qualified name is "/decision_node").
  for (const auto &name : this->get_node_names()) {
    if (isNodeNamed(name, "decision_node")) {
      last_heartbeat_["decision"] = this->now();
      break;
    }
  }
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
  pollGraphSignals();

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

  switch (msg->click_type) {
    case custom_interfaces::msg::ButtonEvent::SINGLE_CLICK:
      if (bringup_->active()) {
        RCLCPP_WARN(this->get_logger(),
                    "Decision toggle blocked: bringup is active (hold button 2 to stop it).");
        return;
      }
      toggleLaunch(decision_spec_);
      break;

    case custom_interfaces::msg::ButtonEvent::DOUBLE_CLICK:
      toggleLaunch(control_spec_);
      break;

    case custom_interfaces::msg::ButtonEvent::HOLD: {
      const bool on = bringup_->toggle();
      RCLCPP_INFO(this->get_logger(), "Bringup toggled %s", on ? "ON" : "OFF");
      for (const auto &name : bringup_->failedToStart()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Bringup: could not start '%s' (repo_dir '%s' not found?). Log: /tmp/panel_%s.log",
                     name.c_str(), repo_dir_.c_str(), name.c_str());
      }
      break;
    }

    default:
      RCLCPP_WARN(this->get_logger(), "Unknown click_type %d on button 2",
                  msg->click_type);
  }
}

// State comes from the OS (pgrep), not from a flag, so a process started by
// hand in a terminal is also seen as running and toggled off correctly.
void SystemStatusNode::toggleLaunch(const process_utils::ProcSpec &spec) {
  if (process_utils::isRunning(spec)) {
    RCLCPP_INFO(this->get_logger(), "Toggling '%s' OFF", spec.name.c_str());
    process_utils::stop(spec);
  } else {
    RCLCPP_INFO(this->get_logger(), "Toggling '%s' ON", spec.name.c_str());
    if (!process_utils::start(repo_dir_, spec)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Could not start '%s' (repo_dir '%s' not found?). Log: /tmp/panel_%s.log",
                   spec.name.c_str(), repo_dir_.c_str(), spec.name.c_str());
    }
  }
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
