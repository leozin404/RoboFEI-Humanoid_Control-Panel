/**
 * @file    panel.cpp
 * @brief   RoboFEI Control Panel, firmware entry point.
 *
 * Two build targets from the same source:
 *   - default      : PCB final (STM32U575CGT6), transporte USB CDC nativo (PA11/PA12)
 *   - NUCLEO_TEST  : NUCLEO-U575ZI-Q em protoboard, transporte LPUART1 (PG7/PG8)
 *                    pela Virtual COM do ST-LINK. Nesta placa SB24/SB26 estao ON e
 *                    SB32/SB34 OFF (UM2861, Tabela 11), entao a VCP e a LPUART1.
 *
 * Para compilar a versao de teste, defina NUCLEO_TEST no CubeIDE:
 *   Properties > C/C++ Build > Settings > MCU G++ Compiler > Preprocessor
 *
 * O protocolo [SYNC][LEN][payload][XOR] e identico nos dois alvos, entao o
 * stm_communication_node nao muda. So muda a porta (/dev/ttyACM0 no teste).
 */

#include "main.h"
#include "button_fsm.hpp"
#include "led_driver.hpp"

#include <cstring>
#include <cstdint>

#ifdef NUCLEO_TEST
extern UART_HandleTypeDef hlpuart1;
#else
#include "usbd_cdc_if.h"
#endif

/*  Pin definitions  */

#define BUTTON1_PORT            GPIOA
#define BUTTON1_PIN             GPIO_PIN_3   // PA3 (igual nos dois alvos)
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
#ifdef NUCLEO_TEST
/* Pinagem conforme o .ioc do teste: CAM PB0, IMU PB1, BRINGUP PB2. */
#define LED_CAM_PORT            GPIOB
#define LED_CAM_PIN             GPIO_PIN_0   // PB0
#define LED_IMU_PORT            GPIOB
#define LED_IMU_PIN             GPIO_PIN_1   // PB1
#define LED_ROBOT_BRINGUP_PORT  GPIOB
#define LED_ROBOT_BRINGUP_PIN   GPIO_PIN_2   // PB2
#else
#define LED_CAM_PORT            GPIOA
#define LED_CAM_PIN             GPIO_PIN_8   // PA8
#define LED_IMU_PORT            GPIOA
#define LED_IMU_PIN             GPIO_PIN_9   // PA9
#define LED_ROBOT_BRINGUP_PORT  GPIOA
#define LED_ROBOT_BRINGUP_PIN   GPIO_PIN_10  // PA10
#endif

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

/*  Serial framing protocol: [SYNC][LEN][payload...][CHECKSUM]  */

static constexpr uint8_t SYNC_BYTE        = 0xAA;
static constexpr uint8_t PANEL_STATUS_LEN = LED_COUNT;   // 7 bytes
static constexpr uint8_t BUTTON_EVENT_LEN = 2;           // button_id, click_type

/* Incoming ring buffer, filled by the transport RX callback */
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

/*  Transport layer (unico ponto que difere entre os alvos)  */

#ifdef NUCLEO_TEST
static uint8_t uartRxByte;

static void transportStartRx() {
  HAL_UART_Receive_IT(&hlpuart1, &uartRxByte, 1);
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == LPUART1) {
    ControlPanel_CDC_RxCallback(&uartRxByte, 1);
    HAL_UART_Receive_IT(&hlpuart1, &uartRxByte, 1);   // re-arma para o proximo byte
  }
}

/* Sem isso, um overrun (ex.: PC mandando enquanto o micro reseta) mata a RX para sempre */
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == LPUART1) {
    HAL_UART_Receive_IT(&hlpuart1, &uartRxByte, 1);
  }
}

static void transportSend(uint8_t *data, uint16_t len) {
  HAL_UART_Transmit(&hlpuart1, data, len, 10);   // 5 bytes a 115200 ~ 0,4 ms
}
#else
static void transportStartRx() { /* CDC_Receive_FS ja chama o callback */ }

static void transportSend(uint8_t *data, uint16_t len) {
  CDC_Transmit_FS(data, len);
}
#endif

/*  Ring buffer helpers / parser (inalterados)  */

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

static bool tryParsePanelStatus(uint8_t out[PANEL_STATUS_LEN]) {
  uint16_t available;
  while (ringAvailable(available)) {
    if (rxRing[rxTail] != SYNC_BYTE) {
      ringPop();
      continue;
    }
    if (available < 2) return false;

    uint8_t len = rxRing[(rxTail + 1) % sizeof(rxRing)];
    if (len != PANEL_STATUS_LEN) {
      ringPop();
      continue;
    }

    uint16_t frameSize = 2 + PANEL_STATUS_LEN + 1;
    if (available < frameSize) return false;

    ringPop();   // SYNC
    ringPop();   // LEN

    uint8_t payload[PANEL_STATUS_LEN];
    for (uint8_t i = 0; i < PANEL_STATUS_LEN; i++) {
      payload[i] = ringPop();
    }
    uint8_t receivedChecksum = ringPop();

    if (checksum(payload, PANEL_STATUS_LEN) != receivedChecksum) continue;

    memcpy(out, payload, PANEL_STATUS_LEN);
    return true;
  }
  return false;
}

static void sendButtonEvent(uint8_t buttonId, uint8_t clickType) {
  uint8_t payload[BUTTON_EVENT_LEN] = { buttonId, clickType };
  uint8_t frame[2 + BUTTON_EVENT_LEN + 1];
  frame[0] = SYNC_BYTE;
  frame[1] = BUTTON_EVENT_LEN;
  memcpy(&frame[2], payload, BUTTON_EVENT_LEN);
  frame[2 + BUTTON_EVENT_LEN] = checksum(payload, BUTTON_EVENT_LEN);

  transportSend(frame, sizeof(frame));
}

/*  Safety timeout  */

static constexpr uint32_t LINK_TIMEOUT_MS = 2000;
static uint32_t lastPanelStatusRxTime = 0;

/*  Application objects  */

static ButtonFSM button1(BUTTON1_PORT, BUTTON1_PIN);
static ButtonFSM button2(BUTTON2_PORT, BUTTON2_PIN);
static LedDriver  leds;

/*  Entry point
 *
 *  O main() fica no main.c gerado pelo CubeMX, que faz HAL_Init, clock e
 *  MX_*_Init. Ele so precisa chamar app_main() em USER CODE BEGIN 2.
 *  Assim nada se perde quando o projeto e regenerado.
 */

extern "C" void app_main(void) {
  transportStartRx();

  leds.registerLed(LED_IDX_LOCALIZATION,   LED_LOCALIZATION_PORT,   LED_LOCALIZATION_PIN);
  leds.registerLed(LED_IDX_GAMECONTROLLER, LED_GAMECONTROLLER_PORT, LED_GAMECONTROLLER_PIN);
  leds.registerLed(LED_IDX_COMM_BOARD,     LED_COMM_BOARD_PORT,     LED_COMM_BOARD_PIN);
  leds.registerLed(LED_IDX_OVERLOAD,       LED_OVERLOAD_PORT,       LED_OVERLOAD_PIN);
  leds.registerLed(LED_IDX_CAM,            LED_CAM_PORT,            LED_CAM_PIN);
  leds.registerLed(LED_IDX_IMU,            LED_IMU_PORT,            LED_IMU_PIN);
  leds.registerLed(LED_IDX_ROBOT_BRINGUP,  LED_ROBOT_BRINGUP_PORT,  LED_ROBOT_BRINGUP_PIN);

  lastPanelStatusRxTime = HAL_GetTick();

  while (1) {
    if (auto ev = button1.update(HAL_GetTick())) {
      sendButtonEvent(1, static_cast<uint8_t>(*ev));
    }
    if (auto ev = button2.update(HAL_GetTick())) {
      sendButtonEvent(2, static_cast<uint8_t>(*ev));
    }

    uint8_t status[PANEL_STATUS_LEN];
    if (tryParsePanelStatus(status)) {
      for (uint8_t i = 0; i < LED_COUNT; i++) {
        leds.setMode(i, static_cast<LedMode>(status[i]));
      }
      lastPanelStatusRxTime = HAL_GetTick();
    }

    if (HAL_GetTick() - lastPanelStatusRxTime > LINK_TIMEOUT_MS) {
      for (uint8_t i = 0; i < LED_COUNT; i++) {
        leds.setMode(i, LedMode::BLINK);
      }
    }

    leds.update(HAL_GetTick());
  }
}
