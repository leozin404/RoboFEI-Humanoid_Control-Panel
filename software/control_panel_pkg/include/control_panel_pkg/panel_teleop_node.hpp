/**
 * @file    panel_teleop_node.hpp
 * @brief   Consumes Button 1 events: Pos1, Macarena, GameController connect.
 *
 * NOTE ON PLACEMENT: this node conceptually belongs to the `control`
 * package of the main RoboFEI-HT_2023_SOFTWARE repo (it issues the same
 * high-level commands robot_joy_control already issues), not to this
 * repo's control_panel_pkg. It lives here temporarily while this repo is
 * developed standalone; move it into `control` when merging.
 *
 * Arbitration rule: while the GameController is connected, single/double
 * click commands (Pos1/Macarena) are ignored — the GC has priority over
 * manual button commands for movement, matching how robot_joy_control
 * behaves. The HOLD gesture (connect GameController) is NOT subject to
 * this arbitration — it's the action that establishes the GC link in the
 * first place, so it is always attempted regardless of current state.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <custom_interfaces/msg/button_event.hpp>

namespace control_panel_pkg {

class PanelTeleopNode : public rclcpp::Node {
 public:
  PanelTeleopNode();

 private:
  void onButtonEvent(const custom_interfaces::msg::ButtonEvent::SharedPtr msg);
  void onGameControllerStatus(const std_msgs::msg::Bool::SharedPtr msg);

  void sendCommand(const std::string &command);
  void connectGameController();

  rclcpp::Subscription<custom_interfaces::msg::ButtonEvent>::SharedPtr button_event_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gc_status_sub_;

  // TODO: replace this placeholder String-command topic with whatever
  // real service/action robot_joy_control actually calls once this node
  // is merged into the `control` package — this is a stand-in interface,
  // not a confirmed one.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;

  bool gc_connected_ = false;
};

}  // namespace control_panel_pkg
