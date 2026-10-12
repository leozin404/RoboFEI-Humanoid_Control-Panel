#include "control_panel_pkg/panel_teleop_node.hpp"
#include "control_panel_pkg/process_utils.hpp"

#include <rclcpp/executors/single_threaded_executor.hpp>

#include <chrono>
#include <string>

namespace control_panel_pkg {

PanelTeleopNode::PanelTeleopNode() : Node("panel_teleop_node") {
  repo_dir_ = this->declare_parameter<std::string>("repo_dir", "~/RoboFEI-HT_2023_SOFTWARE");
  team_number_ = this->declare_parameter<int>("team_number", 7);
  pos1_action_ = this->declare_parameter<int>("pos1_action_number", 1);
  macarena_action_ = this->declare_parameter<int>("macarena_action_number", 36);

  if (!process_utils::repoDirExists(repo_dir_)) {
    RCLCPP_ERROR(this->get_logger(),
                 "repo_dir '%s' does not exist: the GameController button will not work. "
                 "Launch with repo_dir:=<path to RoboFEI-HT_2023_SOFTWARE>.",
                 repo_dir_.c_str());
  }

  control_client_ = rclcpp_action::create_client<Control>(this, "control_action");

  button_event_sub_ = this->create_subscription<custom_interfaces::msg::ButtonEvent>(
      "/painel_controle/button_event", 10,
      std::bind(&PanelTeleopNode::onButtonEvent, this, std::placeholders::_1));

  gc_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/game_controller_connected", 10,
      std::bind(&PanelTeleopNode::onGameControllerStatus, this, std::placeholders::_1));
}

void PanelTeleopNode::onGameControllerStatus(const std_msgs::msg::Bool::SharedPtr msg) {
  gc_connected_ = msg->data;
}

void PanelTeleopNode::onButtonEvent(
    const custom_interfaces::msg::ButtonEvent::SharedPtr msg) {

  if (msg->button_id != 1) return;   // Button 2 is handled by system_status_node

  switch (msg->click_type) {
    case custom_interfaces::msg::ButtonEvent::SINGLE_CLICK:
      if (gc_connected_) {
        RCLCPP_WARN(this->get_logger(), "Pos1 ignored: GameController has priority.");
        return;
      }
      sendMotion(pos1_action_, "Pos1 (Stand Still)");
      break;

    case custom_interfaces::msg::ButtonEvent::DOUBLE_CLICK:
      if (gc_connected_) {
        RCLCPP_WARN(this->get_logger(), "Macarena ignored: GameController has priority.");
        return;
      }
      sendMotion(macarena_action_, "Macarena");
      break;

    case custom_interfaces::msg::ButtonEvent::HOLD:
      connectGameController();
      break;

    default:
      RCLCPP_WARN(this->get_logger(), "Unknown click_type %d on button 1", msg->click_type);
  }
}

void PanelTeleopNode::sendMotion(int32_t actionNumber, const char *label) {
  if (!control_client_->wait_for_action_server(std::chrono::milliseconds(500))) {
    RCLCPP_WARN(this->get_logger(),
                "%s ignored: /control_action server unavailable (is `control` running?)", label);
    return;
  }

  Control::Goal goal;
  goal.action_number = actionNumber;

  // No goal_response_callback on purpose: its signature differs between
  // Foxy (shared_future) and Humble (shared_ptr), and we do not need it.
  auto options = rclcpp_action::Client<Control>::SendGoalOptions();
  options.result_callback =
      [this, label](const GoalHandleControl::WrappedResult &result) {
        RCLCPP_INFO(this->get_logger(), "%s finished (result code %d)", label,
                    static_cast<int>(result.code));
      };

  control_client_->async_send_goal(goal, options);
  RCLCPP_INFO(this->get_logger(), "Sent %s (action_number=%d)", label, actionNumber);
}

// Only starts the listener node (`game_controller connect`: UDP 3838 in,
// 3939 out). The link itself comes from the official GameController
// software broadcasting on the same network with the right team number;
// /game_controller_connected (and the LED) follow that.
void PanelTeleopNode::connectGameController() {
  const auto spec = process_utils::launchSpec(
      "gamecontroller", "gamecontroller_bringup.launch.py",
      {"team_number:=" + std::to_string(team_number_)});

  if (process_utils::isRunning(spec)) {
    RCLCPP_WARN(this->get_logger(), "GameController listener already running, ignoring.");
    return;
  }
  RCLCPP_INFO(this->get_logger(),
              "Starting GameController listener (team %d). The LED lights up once its UDP packets arrive.",
              team_number_);
  if (!process_utils::start(repo_dir_, spec)) {
    RCLCPP_ERROR(this->get_logger(),
                 "Could not start the GameController listener (repo_dir '%s' not found?).",
                 repo_dir_.c_str());
  }
}

}  // namespace control_panel_pkg

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<control_panel_pkg::PanelTeleopNode>();
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
