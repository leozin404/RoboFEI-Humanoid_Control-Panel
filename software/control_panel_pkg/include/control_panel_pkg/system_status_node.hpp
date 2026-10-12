/**
 * @file    system_status_node.hpp
 * @brief   Aggregates every status source into a single PanelStatus,
 *          published for stm_communication_pkg to forward to the STM32.
 *
 * Also owns Button 2 (process toggles):
 *   click        -> toggle decision   (blocked while the bringup is active)
 *   double click -> toggle control
 *   hold         -> toggle the whole bringup (motors, IMU, vision, decision, control)
 *
 * Subscribes to sources that may not exist yet in the wider ROS graph
 * (e.g. /motors/overload, node heartbeats). This is intentional: this
 * node can be developed and tested standalone by publishing mock values
 * from the CLI, without waiting for those other packages to implement
 * their side.
 *
 * Liveness ("alive" = a sign of life within HEARTBEAT_TIMEOUT_MS). Two
 * sources feed the same table, so either one keeps a node alive:
 *   1. explicit std_msgs/Empty topics /<node>/heartbeat (none of the team
 *      packages publishes them yet, kept for when they do);
 *   2. signals the nodes ALREADY publish, so no edits in team packages:
 *        vision       /goalpost_count          every processed frame
 *        um7          /imu/data                IMU samples
 *        motors       /all_joints_position     8 ms loop (commanded values:
 *                                              proves the loop runs, NOT bus health)
 *        localization /localization_active     every 500 ms, only data == true
 *        control      /control_action server discoverable (control has no
 *                                              periodic output, it only reacts)
 *        decision     node "decision_node" present in the ROS graph
 *                                              (weakest signal: exists, not "works";
 *                                              a heartbeat from players_behavior()
 *                                              would be functional)
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <custom_interfaces/action/control.hpp>
#include <custom_interfaces/msg/button_event.hpp>
#include <custom_interfaces/msg/joint_state.hpp>
#include <custom_interfaces/msg/panel_status.hpp>

#include "control_panel_pkg/process_utils.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace control_panel_pkg {

class SystemStatusNode : public rclcpp::Node {
 public:
  SystemStatusNode();

 private:
  static constexpr uint8_t MODE_OFF = 0;
  static constexpr uint8_t MODE_ON  = 1;

  static constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 2000;
  static constexpr uint32_t PUBLISH_PERIOD_MS    = 200;

  void publishStatus();
  bool isHeartbeatAlive(const std::string &name) const;
  void registerHeartbeatSub(const std::string &name, const std::string &topic);
  template <typename MsgT>
  void watchSignal(const std::string &name, const std::string &topic);
  void pollGraphSignals();
  void onButtonEvent(const custom_interfaces::msg::ButtonEvent::SharedPtr msg);
  void toggleLaunch(const process_utils::ProcSpec &spec);

  /* --- direct status sources --- */
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gc_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr overload_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr comm_board_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr imu_usb_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr cam_usb_sub_;
  rclcpp::Subscription<custom_interfaces::msg::ButtonEvent>::SharedPtr button_event_sub_;

  bool gc_connected_ = false;
  bool motor_overload_ = false;
  uint8_t comm_board_mode_ = MODE_OFF;
  uint8_t imu_usb_mode_ = MODE_OFF;
  uint8_t cam_usb_mode_ = MODE_OFF;

  /* --- heartbeats --- */
  std::unordered_map<std::string, rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr>
      heartbeat_subs_;
  std::unordered_map<std::string, rclcpp::Time> last_heartbeat_;

  /* --- signals the monitored nodes already publish --- */
  std::vector<rclcpp::SubscriptionBase::SharedPtr> signal_subs_;
  rclcpp_action::Client<custom_interfaces::action::Control>::SharedPtr control_client_;

  /* --- process control (Button 2) --- */
  std::string repo_dir_;
  process_utils::ProcSpec decision_spec_;
  process_utils::ProcSpec control_spec_;
  std::unique_ptr<process_utils::BringupGroup> bringup_;

  rclcpp::Publisher<custom_interfaces::msg::PanelStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace control_panel_pkg
