#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <custom_interfaces/msg/joint_state.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace control_panel_pkg {

/**
 * One LedMode (0=OFF, 1=ON, 2=BLINK) per device on
 * /painel_controle/usb_status/{comm_board,imu,cam}.
 *
 *   OFF   : device node missing under /dev
 *   ON    : device present, but its owner node has not published recently
 *   BLINK : device present and its owner node published within the window
 *
 * "Traffic" is message arrival on a topic the owner already publishes (no
 * byte counting, no extra access to the device file):
 *   comm board -> /all_joints_position (motors_pkg). CAVEAT: motors_pkg
 *                 publishes it every 8 ms regardless of bus health, so BLINK
 *                 only proves the motors node is looping, not that the
 *                 Dynamixel bus answers.
 *   IMU        -> /imu/data (UM7)
 *   camera     -> /goalpost_count (vision `detect` opens /dev/camera itself
 *                 via OpenCV; there is no /image_raw)
 *
 * Devices (udev, branch main): /dev/motors and /dev/imu are the two child
 * ports of the same communication board, /dev/camera is the C920.
 */
class UsbMonitorNode : public rclcpp::Node {
 public:
  UsbMonitorNode();

 private:
  struct Device {
    std::string path;
    bool present = false;
  };

  template <typename MsgT>
  void watch(const std::string &topic, rclcpp::Time &stamp);

  void poll();
  void publishState(const char *label, Device &dev, bool traffic,
                    const rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr &pub);
  bool recent(const rclcpp::Time &stamp, int windowMs) const;
  static bool exists(const std::string &path);

  Device comm_board_;
  Device imu_;
  Device cam_;

  rclcpp::Time last_motors_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cam_{0, 0, RCL_ROS_TIME};

  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr comm_board_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr cam_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int traffic_window_ms_ = 500;
  int cam_window_ms_ = 1500;   // YOLO on CPU can be slow; frames may be >500 ms apart

  static constexpr uint8_t MODE_OFF = 0;
  static constexpr uint8_t MODE_ON = 1;
  static constexpr uint8_t MODE_BLINK = 2;
  static constexpr uint32_t POLL_PERIOD_MS = 500;
};

}  // namespace control_panel_pkg
