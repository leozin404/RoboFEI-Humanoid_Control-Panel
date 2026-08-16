/**
 * @file    system_status_node.hpp
 * @brief   Aggregates every status source into a single PanelStatus,
 *          published for stm_communication_pkg to forward to the STM32.
 *
 * Subscribes to sources that may not exist yet in the wider ROS graph
 * (e.g. /motors/overload, node heartbeats). This is intentional: this
 * node can be developed and tested standalone by publishing mock values
 * from the CLI (see design-decisions.md / development notes), without
 * waiting for those other packages to implement their side.
 *
 * Heartbeat timeout convention (resolves the open item from
 * design-decisions.md): a node is considered "alive" if a heartbeat was
 * received within HEARTBEAT_TIMEOUT_MS. Heartbeats are std_msgs/Empty,
 * published periodically by each monitored node — only arrival time
 * matters, not message content.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <custom_interfaces/msg/panel_status.hpp>
#include <custom_interfaces/msg/button_event.hpp>
#include <cstdlib>   // std::system

#include <string>
#include <unordered_map>

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
  void onButtonEvent(const custom_interfaces::msg::ButtonEvent::SharedPtr msg);
  void toggleProcess(const std::string &name,
                      const std::string &launchCmd,
                      const std::string &killPattern);

  /* --- direct status sources --- */
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gc_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr overload_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr comm_board_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr imu_usb_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr cam_usb_sub_;
  rclcpp::Subscription<custom_interfaces::msg::ButtonEvent>::SharedPtr button_event_sub_;

  // Tracks whether each managed node group is currently believed to be
  // running. This is our own bookkeeping, separate from heartbeat
  // liveness — heartbeat confirms the process is *actually* alive and
  // responsive; this flag just tracks "did we last tell it to launch or
  // kill", so toggling decides the opposite of what we last commanded.

  bool gc_connected_ = false;
  bool motor_overload_ = false;
  uint8_t comm_board_mode_ = MODE_OFF;
  uint8_t imu_usb_mode_ = MODE_OFF;
  uint8_t cam_usb_mode_ = MODE_OFF;

  /* --- heartbeats --- */
  std::unordered_map<std::string, rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr>
      heartbeat_subs_;
  std::unordered_map<std::string, rclcpp::Time> last_heartbeat_;

  std::unordered_map<std::string, bool> process_launched_;

  rclcpp::Publisher<custom_interfaces::msg::PanelStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace control_panel_pkg
