#include "control_panel_pkg/panel_teleop_node.hpp"

#include <cstdlib>
#include <rclcpp/executors/single_threaded_executor.hpp>

namespace control_panel_pkg {

PanelTeleopNode::PanelTeleopNode() : Node("panel_teleop_node") {
  button_event_sub_ = this->create_subscription<custom_interfaces::msg::ButtonEvent>(
      "/painel_controle/button_event", 10,
      std::bind(&PanelTeleopNode::onButtonEvent, this, std::placeholders::_1));

  gc_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/game_controller_connected", 10,
      std::bind(&PanelTeleopNode::onGameControllerStatus, this, std::placeholders::_1));

  // TODO: placeholder topic — see header note.
  command_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/control/high_level_command", 10);
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
        RCLCPP_WARN(this->get_logger(),
                    "Pos1 ignored: GameController has priority while connected.");
        return;
      }
      sendCommand("POS1");
      break;

    case custom_interfaces::msg::ButtonEvent::DOUBLE_CLICK:
      if (gc_connected_) {
        RCLCPP_WARN(this->get_logger(),
                    "Macarena ignored: GameController has priority while connected.");
        return;
      }
      sendCommand("MACARENA");
      break;

    case custom_interfaces::msg::ButtonEvent::HOLD:
      // Not subject to GC arbitration — see header note.
      connectGameController();
      break;

    default:
      RCLCPP_WARN(this->get_logger(), "Unknown click_type %d on button 1",
                  msg->click_type);
  }
}

void PanelTeleopNode::sendCommand(const std::string &command) {
  std_msgs::msg::String msg;
  msg.data = command;
  command_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Sent high-level command: %s", command.c_str());
}

void PanelTeleopNode::connectGameController() {
  RCLCPP_INFO(this->get_logger(), "Launching GameController connection...");
  // TODO: confirm exact launch/executable name against
  // src/game_controller/game_controller/connect.py's launch entry point.
  std::system("ros2 launch game_controller game_controller.launch.py &");
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
