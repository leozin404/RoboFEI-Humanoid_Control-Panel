/**
 * @file    main.cpp
 * @brief   RoboFEI Control Panel — STM32U575CGT6 firmware entry point.
 *
 * Responsibilities of this file:
 *   - HAL/clock/peripheral init (GPIO, USB CDC)
 *   - Drive the two ButtonFSM instances (one per physical button)
 *   - Drive the LedDriver (7 LEDs, OFF/ON/BLINK)
 *   - Serial framing: encode ButtonEvent -> USB CDC, decode PanelStatus <- USB CDC
 *
 * Pin mapping reference (see docs/design-decisions.md / netlist Control-Panel-V5):
 *   BTTN1 -> PA3                    BTTN2 -> PA2
 *   LED_LOCALIZATION    -> PA7      LED_GAMECONTROLLER -> PA6
 *   LED_COMM_BOARD      -> PA5      LED_OVERLOAD       -> PA4
 *   LED_CAM             -> PA8      LED_IMU            -> PA9
 *   LED_ROBOT_BRINGUP   -> PA10
 */

#include "main.h"
#include "usbd_cdc_if.h"
#include "button_fsm.hpp"
#include "led_driver.hpp"

#include <cstring>
#include <cstdint>

/*  Pin definitions */

#define BUTTON1_PORT            GPIOA
#define BUTTON1_PIN             GPIO_PIN_3   // PA3
#define BUTTON2_PORT            GPIOA
#define BUTTON2_PIN             GPIO_PIN_2   // PA2

#define LED_LOCALIZATION_PORT   GPIOA
#define LED_LOCALIZATION_PIN    GPIO_PIN_7   // PA7
#define LED_GAMECONTROLLER_PORT GPIOA
#define LED_GAMECONTROLLER_PIN  GPIO_PIN_6   // PA6
#define LED_COMM_BOARD_PORT     GPIOA
#define LED_COMM_BOARD_PIN      GPIO_PIN_5   // PA5
#define LED_OVERLOAD_PORT       GPIOA
#define LED_OVERLOAD_PIN        GPIO_PIN_4   // PA4
#define LED_CAM_PORT            GPIOA
#define LED_CAM_PIN             GPIO_PIN_8   // PA8
#define LED_IMU_PORT            GPIOA
#define LED_IMU_PIN             GPIO_PIN_9   // PA9
#define LED_ROBOT_BRINGUP_PORT  GPIOA
#define LED_ROBOT_BRINGUP_PIN   GPIO_PIN_10  // PA10

/* Index convention shared with PanelStatus.msg field order */
enum LedIndex : uint8_t {
  LED_IDX_LOCALIZATION = 0,
  LED_IDX_GAMECONTROLLER,
  LED_IDX_COMM_BOARD,
  LED_IDX_OVERLOAD,
  LED_IDX_CAM,
  LED_IDX_IMU,
  LED_IDX_ROBOT_BRINGUP,
  LED_COUNT
};

/*  Serial framing protocol  */
/*  [SYNC][LEN][payload...][CHECKSUM] */

static constexpr uint8_t SYNC_BYTE       = 0xAA;
static constexpr uint8_t PANEL_STATUS_LEN = LED_COUNT;   // 7 bytes
static constexpr uint8_t BUTTON_EVENT_LEN = 2;           // button_id, click_type

/* Incoming ring buffer, filled by CDC_Receive_FS() callback (see usbd_cdc_if.c) */
static uint8_t  rxRing[256];
static volatile uint16_t rxHead = 0;
static volatile uint16_t rxTail = 0;

extern "C" void ControlPanel_CDC_RxCallback(uint8_t *buf, uint32_t len) {
  for (uint32_t i = 0; i < len; i++) {
    uint16_t next = (rxHead + 1) % sizeof(rxRing);
    if (next != rxTail) {           // drop byte silently if buffer is full
      rxRing[rxHead] = buf[i];
      rxHead = next;
    }
  }
}

static bool ringAvailable(uint16_t &count) {
  count = (rxHead >= rxTail) ? (rxHead - rxTail) : (sizeof(rxRing) - rxTail + rxHead);
  return count > 0;
}

static uint8_t ringPop() {
  uint8_t b = rxRing[rxTail];
  rxTail = (rxTail + 1) % sizeof(rxRing);
  return b;
}

static uint8_t checksum(const uint8_t *data, uint8_t len) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < len; i++) sum ^= data[i];
  return sum;
}

/**
 * Non-blocking parser: consumes whatever is in rxRing, and whenever a
 * complete, valid PanelStatus frame is found, writes it into `out` and
 * returns true. Safe to call every main-loop iteration.
 *
 * Any LEN byte different from PANEL_STATUS_LEN is rejected immediately
 * (not "waited on") — this is what prevents a corrupted LEN byte from
 * permanently stalling the parser waiting for bytes that could never
 * all arrive at once.
 */
static bool tryParsePanelStatus(uint8_t out[PANEL_STATUS_LEN]) {
  uint16_t available;
  while (ringAvailable(available)) {
    if (rxRing[rxTail] != SYNC_BYTE) {
      ringPop();               // resync: discard bytes until SYNC_BYTE
      continue;
    }
    if (available < 2) return false;   // need at least SYNC+LEN

    uint8_t len = rxRing[(rxTail + 1) % sizeof(rxRing)];

    // Reject any unexpected LEN right away instead of waiting for a
    // frame size that may never fit in the buffer.
    if (len != PANEL_STATUS_LEN) {
      ringPop();   // drop just the SYNC byte; next iteration resyncs
      continue;
    }

    uint16_t frameSize = 2 + PANEL_STATUS_LEN + 1;   // SYNC+LEN+payload+CHK
    if (available < frameSize) return false;         // wait for more bytes

    ringPop();   // SYNC
    ringPop();   // LEN

    uint8_t payload[PANEL_STATUS_LEN];
    for (uint8_t i = 0; i < PANEL_STATUS_LEN; i++) {
      payload[i] = ringPop();
    }
    uint8_t receivedChecksum = ringPop();

    if (checksum(payload, PANEL_STATUS_LEN) != receivedChecksum) continue; // corrupted

    memcpy(out, payload, PANEL_STATUS_LEN);
    return true;
  }
  return false;
}

/** Encodes and transmits a ButtonEvent over USB CDC. */
static void sendButtonEvent(uint8_t buttonId, uint8_t clickType) {
  uint8_t payload[BUTTON_EVENT_LEN] = { buttonId, clickType };
  uint8_t frame[2 + BUTTON_EVENT_LEN + 1];
  frame[0] = SYNC_BYTE;
  frame[1] = BUTTON_EVENT_LEN;
  memcpy(&frame[2], payload, BUTTON_EVENT_LEN);
  frame[2 + BUTTON_EVENT_LEN] = checksum(payload, BUTTON_EVENT_LEN);

  CDC_Transmit_FS(frame, sizeof(frame));
}

/*  Safety timeout: if no valid PanelStatus arrives for a while, fall back  */
/*  to a visible "link lost" pattern instead of freezing on stale status.   */

static constexpr uint32_t LINK_TIMEOUT_MS = 2000;
static uint32_t lastPanelStatusRxTime = 0;

/*  Application objects */

static ButtonFSM button1(BUTTON1_PORT, BUTTON1_PIN);
static ButtonFSM button2(BUTTON2_PORT, BUTTON2_PIN);
static LedDriver  leds;   // internally holds all 7 LED pin/port pairs

/* HAL-generated init functions (from CubeMX) */
extern "C" void SystemClock_Config(void);
extern "C" void MX_GPIO_Init(void);
extern "C" void MX_USB_Device_Init(void);

int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USB_Device_Init();

  leds.registerLed(LED_IDX_LOCALIZATION,   LED_LOCALIZATION_PORT,   LED_LOCALIZATION_PIN);
  leds.registerLed(LED_IDX_GAMECONTROLLER, LED_GAMECONTROLLER_PORT, LED_GAMECONTROLLER_PIN);
  leds.registerLed(LED_IDX_COMM_BOARD,     LED_COMM_BOARD_PORT,     LED_COMM_BOARD_PIN);
  leds.registerLed(LED_IDX_OVERLOAD,       LED_OVERLOAD_PORT,       LED_OVERLOAD_PIN);
  leds.registerLed(LED_IDX_CAM,            LED_CAM_PORT,            LED_CAM_PIN);
  leds.registerLed(LED_IDX_IMU,            LED_IMU_PORT,            LED_IMU_PIN);
  leds.registerLed(LED_IDX_ROBOT_BRINGUP,  LED_ROBOT_BRINGUP_PORT,  LED_ROBOT_BRINGUP_PIN);

  lastPanelStatusRxTime = HAL_GetTick();

  while (1) {
    /* --- 1. Update button FSMs, forward any resolved click event --- */
    if (auto ev = button1.update(HAL_GetTick())) {
      sendButtonEvent(1, static_cast<uint8_t>(*ev));
    }
    if (auto ev = button2.update(HAL_GetTick())) {
      sendButtonEvent(2, static_cast<uint8_t>(*ev));
    }

    /* --- 2. Consume any incoming PanelStatus frame --- */
    uint8_t status[PANEL_STATUS_LEN];
    if (tryParsePanelStatus(status)) {
      for (uint8_t i = 0; i < LED_COUNT; i++) {
        leds.setMode(i, static_cast<LedMode>(status[i]));
      }
      lastPanelStatusRxTime = HAL_GetTick();
    }

    /* --- 3. Link-loss safety: all LEDs blink if NUC goes silent --- */
    if (HAL_GetTick() - lastPanelStatusRxTime > LINK_TIMEOUT_MS) {
      for (uint8_t i = 0; i < LED_COUNT; i++) {
        leds.setMode(i, LedMode::BLINK);
      }
    }

    /* --- 4. Advance LED blink timers (non-blocking) --- */
    leds.update(HAL_GetTick());
  }
}

