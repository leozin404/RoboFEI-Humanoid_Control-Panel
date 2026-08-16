/**
 * @file    button_fsm.hpp
 * @brief   Non-blocking finite state machine for a single physical button.
 *
 * Detects three gestures per button: single click, double click, and a
 * 5-second hold. Debounces in hardware-agnostic fashion (works against any
 * HAL GPIO port/pin pair). Must be polled every main-loop iteration via
 * update(); never blocks, never calls HAL_Delay().
 *
 * Timing constants match docs/design-decisions.md:
 *   - Debounce window:      25 ms
 *   - Double-click window: 400 ms
 *   - Hold threshold:     5000 ms
 */

#pragma once

#include "main.h"   // for GPIO_TypeDef, GPIO HAL types
#include <cstdint>
#include <optional>

enum class ClickType : uint8_t {
  SINGLE_CLICK = 1,
  DOUBLE_CLICK = 2,
  HOLD         = 3
};

class ButtonFSM {
 public:
  ButtonFSM(GPIO_TypeDef *port, uint16_t pin);

  /**
   * Call every main-loop iteration with the current tick count
   * (e.g. HAL_GetTick()). Returns a ClickType only in the single
   * iteration where a gesture is fully resolved; std::nullopt otherwise.
   */
  std::optional<ClickType> update(uint32_t nowMs);

 private:
  enum class State {
    IDLE,
    PRESSED,
    WAIT_MORE
  };

  bool readDebounced(uint32_t nowMs);
  bool readRaw() const;

  GPIO_TypeDef *port_;
  uint16_t      pin_;

  /* --- debounce filter state --- */
  bool     lastRawState_    = false;   // last raw level seen
  bool     stableState_     = false;   // debounced, trusted level
  uint32_t lastRawChangeMs_ = 0;

  /* --- gesture FSM state --- */
  State    state_    = State::IDLE;
  uint32_t clk_      = 0;
  uint32_t t0_       = 0;       // press timestamp (for hold detection)
  uint32_t tWait_    = 0;       // release timestamp (for double-click window)
  bool     holdFired_ = false;

  static constexpr uint32_t DEBOUNCE_MS      = 25;
  static constexpr uint32_t DOUBLE_CLICK_MS  = 400;
  static constexpr uint32_t HOLD_MS          = 5000;

  /* Active level: buttons are wired to pull-up + switch to GND (active LOW) */
  static constexpr bool ACTIVE_LEVEL = true;
};
