#include "led_driver.hpp"

void LedDriver::registerLed(uint8_t index, GPIO_TypeDef *port, uint16_t pin) {
  if (index >= MAX_LEDS) return;

  leds_[index].port = port;
  leds_[index].pin = pin;
  leds_[index].registered = true;
  leds_[index].mode = LedMode::OFF;
  leds_[index].blinkState = false;
  leds_[index].lastToggleMs = 0;

  writePin(leds_[index], false);
}

void LedDriver::setMode(uint8_t index, LedMode mode) {
  if (index >= MAX_LEDS || !leds_[index].registered) return;

  LedSlot &slot = leds_[index];

  // Reset the blink phase whenever we (re)enter BLINK mode from something
  // else, so every transition into blinking starts from a known state
  // instead of inheriting stale timing.
  if (mode == LedMode::BLINK && slot.mode != LedMode::BLINK) {
    slot.blinkState = false;
    slot.lastToggleMs = 0;   // forces an immediate first toggle in update()
  }

  slot.mode = mode;
}

void LedDriver::writePin(const LedSlot &slot, bool on) {
  HAL_GPIO_WritePin(slot.port, slot.pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void LedDriver::update(uint32_t nowMs) {
  for (auto &slot : leds_) {
    if (!slot.registered) continue;

    switch (slot.mode) {
      case LedMode::OFF:
        writePin(slot, false);
        break;

      case LedMode::ON:
        writePin(slot, true);
        break;

      case LedMode::BLINK:
        if (nowMs - slot.lastToggleMs >= BLINK_INTERVAL_MS) {
          slot.blinkState = !slot.blinkState;
          writePin(slot, slot.blinkState);
          slot.lastToggleMs = nowMs;
        }
        break;

      default:
        // Unknown/corrupted mode value — fail safe to OFF rather than
        // silently freezing the pin at its last physical state.
        writePin(slot, false);
        break;
    }
  }
}

