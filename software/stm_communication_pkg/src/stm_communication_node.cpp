#include "stm_communication_pkg/stm_communication_node.hpp"

#include <chrono>
#include <rclcpp/executors/single_threaded_executor.hpp>

using namespace std::chrono_literals;

namespace stm_communication_pkg {

StmCommunicationNode::StmCommunicationNode()
    : Node("stm_communication_node") {

  port_path_ = this->declare_parameter<std::string>("serial_port", "/dev/stm_panel");

  panel_status_sub_ = this->create_subscription<custom_interfaces::msg::PanelStatus>(
      "/painel_controle/status", 10,
      std::bind(&StmCommunicationNode::onPanelStatus, this, std::placeholders::_1));

  button_event_pub_ = this->create_publisher<custom_interfaces::msg::ButtonEvent>(
      "/painel_controle/button_event", 10);

  openSerialPort();

  read_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(READ_POLL_PERIOD_MS),
      std::bind(&StmCommunicationNode::pollSerialInput, this));

  reconnect_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(RECONNECT_PERIOD_MS),
      [this]() {
        if (!isPortOpen()) {
          RCLCPP_WARN(this->get_logger(), "Serial port not open, retrying...");
          openSerialPort();
        }
      });
}


/*  Lifecycle  */

void StmCommunicationNode::openSerialPort() {
  try {
    serial_ = std::make_unique<LibSerial::SerialStream>();
    serial_->Open(port_path_);

    if (!serial_->IsOpen()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port_path_.c_str());
      serial_.reset();
      return;
    }

    serial_->SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    serial_->SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_->SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    serial_->SetParity(LibSerial::Parity::PARITY_NONE);
    serial_->SetStopBits(LibSerial::StopBits::STOP_BITS_1);

    RCLCPP_INFO(this->get_logger(), "Serial port opened: %s", port_path_.c_str());
    rx_buffer_.clear(); 
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception opening serial port %s: %s",
                 port_path_.c_str(), e.what());
    serial_.reset();
  }
}

void StmCommunicationNode::pollSerialInput() {
  if (!isPortOpen()) return;

  try {
    while (serial_->IsDataAvailable()) {
      char c;
      serial_->get(c);
      feedByte(static_cast<uint8_t>(c));
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Serial read failed: %s. Marking port closed.",
                 e.what());
    serial_.reset();
    rx_buffer_.clear(); 
    return;
  }

  tryParseButtonEvent();
}

bool StmCommunicationNode::isPortOpen() const {
  return serial_ && serial_->IsOpen();
}


/*  Outbound: PanelStatus -> STM32 */


uint8_t StmCommunicationNode::checksum(const uint8_t *data, uint8_t len) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < len; i++) sum ^= data[i];
  return sum;
}

void StmCommunicationNode::sendFrame(const uint8_t *payload, uint8_t len) {
  if (!isPortOpen()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Serial port not open, dropping outgoing frame.");
    return;
  }

  try {
    (*serial_) << static_cast<char>(SYNC_BYTE);
    (*serial_) << static_cast<char>(len);
    for (uint8_t i = 0; i < len; i++) {
      (*serial_) << static_cast<char>(payload[i]);
    }
    (*serial_) << static_cast<char>(checksum(payload, len));
    serial_->flush();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Serial write failed: %s. Marking port closed.",
                 e.what());
    serial_.reset();   // force reconnect_timer_ to reopen on next tick
  }
}

void StmCommunicationNode::onPanelStatus(
    const custom_interfaces::msg::PanelStatus::SharedPtr msg) {

  // Field order here MUST match PANEL_STATUS_LEN and the firmware's
  // expected byte order (see main.cpp LedIndex enum).
  uint8_t payload[PANEL_STATUS_LEN] = {
      msg->led_localization,
      msg->led_gamecontroller,
      msg->led_comm_board,
      msg->led_overload,
      msg->led_cam,
      msg->led_imu,
      msg->led_rbt_bu,
  };

  sendFrame(payload, PANEL_STATUS_LEN);
}


/*  Inbound: STM32 -> ButtonEvent */


void StmCommunicationNode::feedByte(uint8_t byte) {
  rx_buffer_.push_back(byte);

  // Simple cap to avoid unbounded growth if we never see a valid frame
  // (e.g. noise with no SYNC byte at all).
  constexpr size_t MAX_BUFFER = 256;
  if (rx_buffer_.size() > MAX_BUFFER) {
    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + (rx_buffer_.size() - MAX_BUFFER));
  }
}

bool StmCommunicationNode::tryParseButtonEvent() {
  // Consume as many complete, valid frames as are currently buffered.
  bool publishedAny = false;

  while (true) {
    // Find a SYNC byte, discarding anything before it.
    auto syncIt = std::find(rx_buffer_.begin(), rx_buffer_.end(), SYNC_BYTE);
    if (syncIt == rx_buffer_.end()) {
      rx_buffer_.clear();
      break;
    }
    rx_buffer_.erase(rx_buffer_.begin(), syncIt);   // SYNC now at index 0

    if (rx_buffer_.size() < 2) break;   // need SYNC + LEN at least

    uint8_t len = rx_buffer_[1];

    // Reject unexpected LEN immediately — same reasoning as the firmware
    // parser: never wait for a frame size that isn't the one we expect.
    if (len != BUTTON_EVENT_LEN) {
      rx_buffer_.erase(rx_buffer_.begin());   // drop just the SYNC, resync
      continue;
    }

    size_t frameSize = 2 + BUTTON_EVENT_LEN + 1;   // SYNC+LEN+payload+CHK
    if (rx_buffer_.size() < frameSize) break;       // wait for more bytes

    uint8_t payload[BUTTON_EVENT_LEN] = { rx_buffer_[2], rx_buffer_[3] };
    uint8_t receivedChecksum = rx_buffer_[4];

    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + frameSize);

    if (checksum(payload, BUTTON_EVENT_LEN) != receivedChecksum) {
      RCLCPP_WARN(this->get_logger(), "ButtonEvent checksum mismatch, frame dropped.");
      continue;
    }

    auto msg = custom_interfaces::msg::ButtonEvent();
    msg.button_id = payload[0];
    msg.click_type = payload[1];
    msg.stamp = this->now();

    button_event_pub_->publish(msg);
    publishedAny = true;
  }

  return publishedAny;
}

}  // namespace stm_communication_pkg

/*  main()                                                                   */

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Explicit SingleThreadedExecutor: sendFrame()/pollSerialInput()/
  // onPanelStatus() share serial_ and rx_buffer_ without synchronization,
  // relying on all callbacks running sequentially on the same thread.
  // Do not switch this node to a multi-threaded executor or compose it
  // into a multi-threaded container without adding a mutex first.
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<stm_communication_pkg::StmCommunicationNode>();
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
