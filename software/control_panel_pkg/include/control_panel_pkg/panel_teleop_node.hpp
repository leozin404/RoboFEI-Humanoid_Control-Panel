/**
 * @file    panel_teleop_node.hpp
 * @brief   Consumes Button 1 events: Pos 1, Macarena, GameController.
 *
 * Pos 1 and Macarena are goals on the `control` node's /control_action
 * server (custom_interfaces/action/Control), the same interface that
 * robot_joy_control uses. Action numbers come from
 * control.cpp::choose_movement: 1 = Stand Still, 36 = Macarena.
 *
 * Arbitration: while the GameController is connected, single and double
 * click (Pos 1 / Macarena) are ignored, the GC has priority. HOLD (start
 * the GC listener) is exempt, since it is what establishes the GC link.
 *
 * NOTE: conceptually this belongs in the `control` package of the main
 * repo; it lives here while the panel is developed standalone.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <custom_interfaces/msg/button_event.hpp>
#include <custom_interfaces/action/control.hpp>

#include <cstdint>
#include <string>

namespace control_panel_pkg {

class PanelTeleopNode : public rclcpp::Node {
 public:
  using Control = custom_interfaces::action::Control;
  using GoalHandleControl = rclcpp_action::ClientGoalHandle<Control>;

  PanelTeleopNode();

 private:
  void onButtonEvent(const custom_interfaces::msg::ButtonEvent::SharedPtr msg);
  void onGameControllerStatus(const std_msgs::msg::Bool::SharedPtr msg);

  void sendMotion(int32_t actionNumber, const char *label);
  void connectGameController();

  rclcpp::Subscription<custom_interfaces::msg::ButtonEvent>::SharedPtr button_event_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gc_status_sub_;
  rclcpp_action::Client<Control>::SharedPtr control_client_;

  bool gc_connected_ = false;
  int32_t pos1_action_ = 1;
  int32_t macarena_action_ = 36;
  int team_number_ = 7;
  std::string repo_dir_;
};

}  // namespace control_panel_pkg
