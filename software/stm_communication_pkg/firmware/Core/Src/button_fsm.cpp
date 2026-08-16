#include "button_fsm.hpp"

ButtonFSM::ButtonFSM(GPIO_TypeDef *port, uint16_t pin)
    : port_(port), pin_(pin) {}

bool ButtonFSM::readRaw() const {
  bool physicallyHigh = (HAL_GPIO_ReadPin(port_, pin_) == GPIO_PIN_SET);
  return ACTIVE_LOW ? !physicallyHigh : physicallyHigh;
}

bool ButtonFSM::readDebounced(uint32_t nowMs) {
  bool raw = readRaw();

  if (raw != lastRawState_) {
    lastRawChangeMs_ = nowMs;
    lastRawState_ = raw;
  }

  if (nowMs - lastRawChangeMs_ > DEBOUNCE_MS) {
    stableState_ = raw;
  }

  return stableState_;
}

std::optional<ClickType> ButtonFSM::update(uint32_t nowMs) {
  bool pressed = readDebounced(nowMs);

  switch (state_) {

    case State::IDLE:
      if (pressed) {
        t0_ = nowMs;
        holdFired_ = false;
        state_ = State::PRESSED;
      }
      break;

    case State::PRESSED:
      if (pressed && !holdFired_ && (nowMs - t0_ >= HOLD_MS)) {
        holdFired_ = true;
        clk_ = 0;
        return ClickType::HOLD;   // fires exactly once, guarded by holdFired_
      }

      if (!pressed) {
        if (holdFired_) {
          // hold already resolved this press cycle — this release doesn't
          // count as a click, just return to idle.
          clk_ = 0;
          state_ = State::IDLE;
        } else {
          clk_++;
          tWait_ = nowMs;
          state_ = State::WAIT_MORE;
        }
      }
      break;

    case State::WAIT_MORE:
      if (pressed) {
        // new press arrived within the double-click window — go back to
        // PRESSED, keep counting clicks, keep watching for hold too.
        t0_ = nowMs;
        holdFired_ = false;
        state_ = State::PRESSED;
      } else if (nowMs - tWait_ >= DOUBLE_CLICK_MS) {
        ClickType result = (clk_ >= 2) ? ClickType::DOUBLE_CLICK
                                         : ClickType::SINGLE_CLICK;
        clk_ = 0;
        state_ = State::IDLE;
        return result;
      }
      break;
  }

  return std::nullopt;
}

