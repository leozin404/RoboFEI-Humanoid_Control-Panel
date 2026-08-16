/**
 * @file    usb_monitor_node.hpp
 * @brief   Monitors USB device presence (via /dev/* symlinks) and traffic
 *          (via message arrival on existing topics) for the comm board,
 *          IMU, and camera.
 *
 * Presence (constant/off) is checked via stat() on the udev symlink —
 * this works reliably, since it only depends on the device file existing.
 *
 * Traffic (blinking) is NOT derived from stat()/mtime on the device file.
 * Character devices generally don't update mtime on read()/write() the
 * way regular files do, so that approach was tried and rejected (see
 * design-decisions.md). Instead, this node subscribes to topics that
 * other nodes (um7, vision_pkg) already publish, and treats a recent
 * message arrival as "traffic detected" — no extra access to the raw
 * device file is needed for this part.
 *
 * The comm board doesn't have a single topic of its own (it's a
 * transport, not a sensor) — its activity is inferred as "either motors
 * or IMU data flowed recently", since both travel through it.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <string>
#include <unordered_map>

namespace control_panel_pkg {

class UsbMonitorNode : public rclcpp::Node {
 public:
  UsbMonitorNode();

 private:
  struct PresenceState {
    std::string devicePath;
    bool present = false;
  };

  void pollPresence();
  void publishAll();

  bool checkPresence(const std::string &path) const;
  bool isRecentTraffic(const rclcpp::Time &lastMsgTime) const;

  /* --- presence (stat-based) --- */
  PresenceState comm_board_presence_;
  PresenceState imu_presence_;
  PresenceState cam_presence_;

  /* --- traffic (topic-based) --- */
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_data_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_image_sub_;
  // Motors doesn't have a convenient existing topic subscribed here yet;
  // comm board traffic is approximated from IMU traffic alone for now.
  // TODO: subscribe to a motors feedback topic once one is confirmed,
  // and OR it in with imu traffic for comm_board's "recent activity".

  rclcpp::Time last_imu_msg_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cam_msg_time_{0, 0, RCL_ROS_TIME};

  /* --- publishers --- */
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr comm_board_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr cam_pub_;

  rclcpp::TimerBase::SharedPtr poll_timer_;

  static constexpr uint8_t MODE_OFF   = 0;
  static constexpr uint8_t MODE_ON    = 1;
  static constexpr uint8_t MODE_BLINK = 2;

  static constexpr uint32_t POLL_PERIOD_MS       = 500;
  static constexpr uint32_t TRAFFIC_WINDOW_MS    = 500;   // "recent" = within this window
};

}  // namespace control_panel_pkg
