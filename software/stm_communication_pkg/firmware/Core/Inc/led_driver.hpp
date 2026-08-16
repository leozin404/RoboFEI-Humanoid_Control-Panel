/**
 * @file    led_driver.hpp
 * @brief   Generic non-blocking driver for N status LEDs.
 *
 * Each LED is independently OFF, ON, or BLINKING (toggling every
 * BLINK_INTERVAL_MS). The driver has no knowledge of *why* a given LED is
 * in a given mode — that decision is made entirely on the NUC side and
 * arrives as a PanelStatus frame; this class only renders the mode onto
 * the physical pin.
 *
 * Must be polled every main-loop iteration via update(); never blocks.
 */

#pragma once

#include "main.h"   // GPIO_TypeDef, HAL GPIO types
#include <cstdint>
#include <array>

enum class LedMode : uint8_t {
  OFF   = 0,
  ON    = 1,
  BLINK = 2
};

class LedDriver {
 public:
  static constexpr uint8_t MAX_LEDS = 8;   // headroom above the current 7 LEDs

  /** Associates a logical LED index with its physical GPIO port/pin. */
  void registerLed(uint8_t index, GPIO_TypeDef *port, uint16_t pin);

  /** Sets the desired mode for a given LED index. Safe to call at any time. */
  void setMode(uint8_t index, LedMode mode);

  /** Call every main-loop iteration with the current tick count. */
  void update(uint32_t nowMs);

 private:
  struct LedSlot {
    GPIO_TypeDef *port      = nullptr;
    uint16_t      pin       = 0;
    LedMode       mode      = LedMode::OFF;
    bool          blinkState = false;
    uint32_t      lastToggleMs = 0;
    bool          registered = false;
  };

  void writePin(const LedSlot &slot, bool on);

  std::array<LedSlot, MAX_LEDS> leds_{};

  static constexpr uint32_t BLINK_INTERVAL_MS = 500;
};
