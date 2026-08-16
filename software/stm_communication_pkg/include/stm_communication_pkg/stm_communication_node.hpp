/**
 * @file    stm_communication_node.hpp
 * @brief   ROS2 <-> STM32 serial bridge. Pure I/O layer: no panel logic here.
 *
 * Subscribes to PanelStatus (from control_panel_pkg) and forwards it to the
 * STM32 over USB CDC using the same [SYNC][LEN][payload][CHECKSUM] framing
 * implemented in firmware/Core/Src/main.cpp. Reads button events back from
 * the STM32 and republishes them as ButtonEvent messages.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <custom_interfaces/msg/panel_status.hpp>
#include <custom_interfaces/msg/button_event.hpp>

#include <libserial/SerialStream.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace stm_communication_pkg {

class StmCommunicationNode : public rclcpp::Node {
 public:
  StmCommunicationNode();

 private:
  /* framing constants (must match firmware) */
  static constexpr uint8_t SYNC_BYTE           = 0xAA;
  static constexpr uint8_t PANEL_STATUS_LEN    = 7;   // one byte per LED
  static constexpr uint8_t BUTTON_EVENT_LEN    = 2;   // button_id, click_type
  static constexpr uint32_t RECONNECT_PERIOD_MS = 2000;
  static constexpr uint32_t READ_POLL_PERIOD_MS = 20;   // 50 Hz poll

  /* lifecycle */
  void openSerialPort();
  bool isPortOpen() const;

  /* outbound: PanelStatus -> STM32 */
  void onPanelStatus(const custom_interfaces::msg::PanelStatus::SharedPtr msg);
  void sendFrame(const uint8_t *payload, uint8_t len);
  static uint8_t checksum(const uint8_t *data, uint8_t len);

  /* inbound: STM32 -> ButtonEvent */
  void pollSerialInput();
  void feedByte(uint8_t byte);
  bool tryParseButtonEvent();

  /* ROS interfaces */
  rclcpp::Subscription<custom_interfaces::msg::PanelStatus>::SharedPtr panel_status_sub_;
  rclcpp::Publisher<custom_interfaces::msg::ButtonEvent>::SharedPtr button_event_pub_;
  rclcpp::TimerBase::SharedPtr read_timer_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;

  /* serial */
  std::unique_ptr<LibSerial::SerialStream> serial_;
  std::string port_path_;

  /* inbound ring buffer (simple vector-based, single-threaded access) */
  std::vector<uint8_t> rx_buffer_;
};

}  // namespace stm_communication_pkg

