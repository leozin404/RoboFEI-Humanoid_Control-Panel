# Design Decisions Log

This document records the key architectural and engineering decisions made
throughout the development of the RoboFEI Control Panel, along with the
reasoning behind each choice. It exists so that future readers (including
future me) understand *why* the system looks the way it does, not just *what*
it looks like.

---

## Phase 1 — Initial prototype: ESP32-S3

- **MCU:** ESP32-S3-DevKitC-1-N8R2
- **Power:** TPS54332DDAR buck converter, fed directly from the robot's
  14.8V LiPo battery bus (via the power distribution board)
- **Communication:** USB CDC, ASCII-based serial protocol
  (newline-terminated messages)
- **Software bridge:** Python script on the NUC using `rclpy`, invoking
  ROS2 nodes via system calls based on received button messages
- **Interface:** ~10 dedicated buttons (one per major ROS2 node) and ~15 LEDs
- Documented in the IC partial report (`docs/reports/IC_partial_report_2025.pdf`)
  and archived under `hardware/legacy/fusion360_esp32s3/`

This version validated the core concept (physical buttons reduce
initialization and diagnosis time significantly — see partial report,
Section 4.3) but was superseded before implementation was finalized.

---

## Phase 2 — Migration to STM32U575CGT6

### MCU change: ESP32-S3 → STM32U575CGT6

**Reasoning:**
- Native USB OTG_FS peripheral, same practical benefit as the ESP32-S3's
  USB CDC support
- Cortex-M33 core leaves headroom for a possible future migration to
  micro-ROS
- Better fits a from-scratch KiCad design (project shifted from
  breadboard/dev-kit prototyping to a proper custom PCB)

### Power architecture change

- **Before:** buck converter (TPS54332) stepping down 14.8V from the
  battery bus to 3.3V
- **After:** LD1117 (SOT-223) linear regulator, powered directly from the
  NUC's own USB 5V rail
- **Reasoning:** the panel no longer needs to tap the high-voltage battery
  bus at all. Confirmed comfortable current margin (LD1117 rated up to
  800mA vs. expected ~150-200mA load) and dropout margin (worst-case USB
  4.75V input, 3.3V output, ~1.2V dropout at max current)
- Thermal check (SOT-223, RθJA ≈ 110°C/W): safe at expected load, ΔT ≈ 33°C
  above ambient

### Connector and interface reduction

- Interface reduced from ~10 buttons/~15 LEDs (Phase 1 concept) to
  **2 buttons / 7 LEDs**, using multi-function button gestures (single
  click, double click, 5s hold)

### Communication protocol change: ASCII → binary framed

- **Before:** ASCII messages, newline-terminated
- **After:** binary protocol with sync byte + length + checksum framing:
  `[SYNC=0xAA][LEN][payload][CHECKSUM(XOR)]`
- **Reasoning:** the link is full-duplex with two independent message
  types (status NUC→MCU, button events MCU→NUC) on the same connection.
  Explicit framing with checksum allows automatic resync after byte loss

### USB class: CDC (confirmed) vs HID (evaluated, rejected)

- **Decision:** USB CDC (Virtual COM Port)
- HID was evaluated (would allow standalone testing outside ROS) but
  rejected: no concrete standalone use case identified, CDC simpler on
  both firmware and NUC side, processing/latency difference negligible
  for this data volume
- **VBUS sensing:** disabled ("No VBUS Sensing") — board has no alternate
  power source to switch between

### Software architecture: dedicated C++ ROS2 packages

- `stm_communication_pkg` — serial I/O only (the "dumb bridge"), mirrors
  the existing `arduino_communication` package pattern
- `control_panel_pkg` — status aggregation logic (the "brain"), mirrors
  the `decision_pkg` pattern
- Message types (`PanelStatus.msg`, `ButtonEvent.msg`) live in
  `custom_interfaces`, not inside either package, since both depend on them

### LED behavior: three-state driver (OFF/ON/BLINKING)

- Each of 7 LEDs driven by a generic non-blocking state machine, controlled
  by a mode byte from the NUC — firmware has no hardcoded knowledge of
  *why* an LED is on
- LED color: **green** (confirmed comfortable Vf headroom at 3.3V rail)

### Node toggle logic: decided full toggle (on ↔ off)

- Kept full toggle for `Decisão`, `Controle`, `Robot_Bringup` — accessible
  kill-switch without SSH/terminal, good safety practice
- Process lifecycle managed by `system_status_node` (owns the heartbeat
  tracking, so also owns launch/kill)

### Rejected/deferred alternatives

- **micro-ROS on the STM32:** deferred due to setup complexity vs. timeline
- **`rosnode list`/process-polling for liveness:** rejected in favor of a
  heartbeat-topic pattern (timeout-based, more robust than one-shot checks)

---

## Firmware architecture: platform-agnostic core

- `button_fsm.hpp/.cpp` and `led_driver.hpp/.cpp` take GPIO access via
  injected `std::function` (read/write callbacks) instead of direct
  `GPIO_TypeDef*`/HAL calls
- **Reasoning:** only `main.cpp` needs to know pin numbers or which board
  it's running on. The FSM/driver logic is byte-for-byte identical across
  the production STM32U575CGT6 PCB and any test rig (Nucleo dev board),
  so test coverage on a dev board directly validates production logic
- Requires **C++17** (`std::optional` in `button_fsm.hpp`) — documented as
  a hard build requirement in both `firmware/CMakeLists.txt` and, later,
  discovered to need explicit configuration in STM32CubeIDE (default
  project language standard is C++14, not C++17 — must be changed in
  Project Properties → C/C++ Build → Settings → MCU G++ Compiler →
  General → Language standard)

---

## USB traffic detection (usb_monitor_node): stat()/mtime → topic-based

- **Initially attempted:** detect USB traffic via `stat()`/`mtime` polling
  on `/dev/*` udev symlinks
- **Rejected:** character devices generally don't update `mtime` on
  ongoing `read()`/`write()` the way regular files do — would likely have
  made every device report "constant" forever, never "blinking"
- **Decision:** presence (constant/off) still via `stat()` on `/dev/*`.
  Traffic (blinking) instead comes from subscribing to topics the owning
  nodes already publish (`/imu/data` for IMU, `/image_raw` for camera),
  using recent message arrival as the signal
- **Known gap:** comm board has no obvious feedback topic yet; approximated
  as "IMU traffic present" for now (both travel through the same board)

---

## Closing the ButtonEvent consumer gap

- **Gap identified:** through initial implementation, every `ButtonEvent`
  published had no subscriber anywhere — pressing a button triggered nothing
- **Button 2 (toggle) → `system_status_node`:** owns launching/killing
  `decision`, `control`, `robot_bringup` on toggle. A `process_launched_`
  map tracks intent separately from heartbeat-confirmed liveness (can
  disagree if a launch/kill silently fails — documented as future
  cross-check improvement, not yet implemented)
- **Button 1 (movement/GameController) → new `panel_teleop_node`:**
  conceptually belongs in the `control` package of the main
  `RoboFEI-HT_2023_SOFTWARE` repo (issues the same commands
  `robot_joy_control` does); lives temporarily in `control_panel_pkg`
  while developed standalone — **move it into `control` when merging**
- **Arbitration rule:** while `/game_controller_connected` is true,
  single/double click (Pos1/Macarena) are ignored — GC has priority for
  movement commands. HOLD (connect GameController) is exempt, since it's
  the action that establishes the GC link in the first place
- **Known placeholders (not yet confirmed against real codebase):** exact
  launch file names/paths (`decision.launch.py`, etc.); the real high-level
  command interface `panel_teleop_node` should use instead of the current
  `std_msgs/String` stand-in topic

---

## Local build adjustments to RoboFEI-HT_2023_SOFTWARE (separate document)

Edits required to build the **main team repo** (not this repo's own
packages) on a clean Ubuntu 20.04 / ROS2 Foxy VM are tracked separately in
`docs/robofei-repo-adjustments.md`, not here — that document covers
patches to `decision_pkg_cpp`, `control`, `vision_msgs` compatibility, etc.,
which are out of scope for this project's own architecture decisions.
Several of those fixes (real Foxy API signature bugs, not environment
workarounds) are flagged there as worth upstreaming to the team repo.

---

## Hardware validation: testing on NUCLEO-U575ZI-Q before the custom PCB

- **Context:** custom PCB (STM32U575CGT6) design finished but not yet
  fabricated/assembled. A NUCLEO-U575ZI-Q (same MCU family, LQFP144
  package instead of the PCB's UFQFPN48) was available for early
  software validation
- **Reasoning it's valid:** thanks to the platform-agnostic firmware
  architecture (GPIO/UART access injected via `std::function`),
  `button_fsm`/`led_driver` require zero changes to run on the Nucleo —
  only pin mapping and the communication transport layer in `main.cpp`
  differ. This validates 100% of the FSM/driver/framing logic against
  real mechanical buttons (including real debounce behavior) before any
  electrical risk is taken with the untested custom PCB
- **Nucleo test rig pin mapping (breadboard, temporary):**
  `PA2`=BTTN2, `PA3`=BTTN1, `PA4`=OVERLOAD, `PA5`=COMM_BOARD,
  `PA6`=GAMECONTROLLER, `PA7`=LOCALIZATION, `PB0`=CAM, `PB1`=IMU,
  `PB2`=ROBOT_BRINGUP — confirmed free of conflict with onboard
  peripherals (Nucleo's own VCP uses LPUART1 on PA9/PA10, not PA2/PA3
  as on many Nucleo-64 boards)
- **TrustZone:** left disabled — no security requirement for this project;
  enabling it would add significant complexity (secure/non-secure project
  split, SAU configuration) with no benefit. Off by default on the chip
  (`TZEN` option byte), consistent with not touching it

### USB stack discovery: CubeMX only offers USBX for STM32U5, not classic USB_DEVICE

- **Discovered:** ST removed the classic USB Device Library option from
  CubeMX for the entire STM32U5 family — only USBX (Azure RTOS USB stack)
  is offered. Confirmed via multiple ST Community threads; this is a
  known pain point, not a local misconfiguration
- Our original firmware plan (`main.cpp` using `CDC_Transmit_FS`/
  `CDC_Receive_FS` from the classic `usbd_cdc_if.c`) assumed the classic
  stack and needs revisiting **specifically for the final PCB**, since the
  PCB has no onboard debugger/VCP bridge and must use the STM32's native
  USB peripheral
- USBX can run in a "Standalone" mode without full ThreadX RTOS
  integration, but community reports show real friction getting this
  working reliably on current STM32U5 firmware packages — treated as a
  real risk to timeline, not a solved problem
- **Decision for the Nucleo test phase specifically:** sidestep USBX
  entirely. The Nucleo's onboard ST-Link already exposes a Virtual COM
  Port via LPUART1 (PA9/PA10) — configured through the Board Project
  Options "Virtual Com Port" checkbox — which appears as a standard
  serial port to the NUC without touching USBX/CDC at all. This validates
  100% of the button FSM / LED driver / framing protocol logic; only the
  transport layer (`HAL_UART_Transmit`/`HAL_UART_Receive_IT` via
  `hcom_uart[COM1]`, the BSP-exposed handle, instead of `CDC_Transmit_FS`)
  differs from the eventual PCB firmware
- **Open item — deferred, not solved:** how the final PCB will handle
  USB communication is still undecided. Two options identified to
  evaluate when the PCB arrives: (a) get USBX standalone mode working
  despite the reported friction, or (b) add a discrete USB-to-UART bridge
  chip (e.g. CP2102) wired to a real UART peripheral, sidestepping USBX
  entirely at the cost of one extra BOM component

### CubeIDE project setup gotchas (for reproducing the build environment)

- Project must be created via **Board Selector** (`NUCLEO-U575ZI-Q`), not
  MCU Selector, to get correct default pin/peripheral assignments
- Toolchain/IDE must be explicitly set to **STM32CubeIDE** in Project
  Manager — CubeMX defaults to IAR/EWARM otherwise if not changed
- CubeIDE generates `main.c` even when C++ was selected at project
  creation — renaming to `.cpp` must be done **inside the IDE's Project
  Explorer** (not via Windows Explorer/external file manager), otherwise
  the Eclipse/CDT build configuration doesn't pick up the change and the
  linker fails with `undefined reference to main` despite the file
  existing on disk
- Renaming/copying `.cpp` files into the project via Windows Explorer
  requires a `Project → Clean` + full rebuild afterward — incremental
  build can silently use a stale `objects.list` and skip the new files
- Even with `.cpp` files present, the project needs **C++ Nature**
  explicitly added (`right-click project → Convert to a C++ Project`) if
  it wasn't picked up correctly — otherwise `.cpp` files are silently
  not compiled (no error, just missing from the object list) and the
  link fails on a missing `main`
- Default C++ language standard in a fresh CubeIDE project is
  **C++14** — must be manually raised to **C++17** (Project Properties →
  C/C++ Build → Settings → MCU G++ Compiler → General → Language
  standard → GNU C++17), required for `std::optional` in `button_fsm.hpp`

---

## Open items / pending decisions

- [ ] **USB transport for the final PCB** — USBX standalone vs. external
      USB-UART bridge chip (see discovery above); currently deferred
- [ ] Confirm a suitable `motors_pkg` feedback topic to fold into comm
      board's traffic detection (currently approximated via IMU traffic
      alone)
- [ ] Confirm actual topic names (`/imu/data`, `/image_raw`) against
      um7/vision_pkg source before field testing `usb_monitor_node`
- [ ] Confirm real launch file names/paths for decision, control, and
      robot_bringup before relying on `toggleProcess()` in the field
- [ ] Confirm how `robot_joy_control` actually issues high-level movement
      commands to `control`, and replace `panel_teleop_node`'s placeholder
      String topic with the real interface
- [ ] Move `panel_teleop_node` into the `control` package once merging
      with `RoboFEI-HT_2023_SOFTWARE`
- [ ] Consider cross-checking `toggleProcess()`'s intent flag against
      actual heartbeat liveness, to catch silently-failed launch/kill
- [ ] Confirm Hardware Error Status register address for the Dynamixel
      model in use (needed for `/motors/overload`)
- [ ] Heartbeat publishers still need to be added to `decision`,
      `vision_pkg`, `motors_pkg`, `um7`, `control` (only the subscriber
      side exists so far, in `system_status_node`) — these are edits to
      the main team repo, out of scope for this repo alone
- [ ] New udev rules for the STM32 (`/dev/stm_panel`) and comm board
      (`/dev/commboard`) in `robot_plugins/robot-usb-ports.rules`
