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
  click, double click, 2 s hold)

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

## Button timing (final values)

- **Hold threshold: 2 s.** The IC report (Table 14) still says 5 s and has to be aligned.
- **Debounce window: 50 ms** (was 25 ms). Long presses were being misdetected on a breadboard
  rig with real mechanical buttons, and the larger window fixed it.
- **Double-click window: 400 ms** (unchanged).
- The constants live in `button_fsm.hpp` (`HOLD_MS`, `DEBOUNCE_MS`, `DOUBLE_CLICK_MS`).

---

## USB presence and traffic (usb_monitor_node)

- **Presence** (OFF vs ON): `stat()` on the udev symlink. Devices, from the `main`
  rules: `/dev/motors` and `/dev/imu` are the two child ports of the same
  communication board (the "Placa nova" block of `robot-usb-ports.rules`), and
  `/dev/camera` is the C920. `/dev/motors` stands for the communication board, so
  no `/dev/commboard` rule is needed. If the board is unplugged, the comm board
  and IMU LEDs go off together; if only the IMU LED is off, the fault is the sensor.
- **Traffic** (BLINK): message arrival, within a time window, on a topic that the
  owner node already publishes. No byte counting and no access to the device file:
  - IMU: `/imu/data` (`sensor_msgs/Imu`, UM7), 500 ms window
  - camera: `/goalpost_count` (`std_msgs/Int32`), 1.5 s window. The vision
    `detect` node opens `/dev/camera` itself through OpenCV, so there is no
    `/image_raw`. The topic is published on every processed frame, also with 0
    goalposts, and not at all if the camera cannot be read
  - comm board: `/all_joints_position` (`custom_interfaces/JointState`), 500 ms window
- **Why not `stat()`/`mtime` for traffic:** character devices do not update `mtime`
  on `read()`/`write()` the way regular files do, so every device would report
  "constant" forever.
- **Limitation (comm board):** `motors_communication` publishes
  `/all_joints_position` every 8 ms regardless of bus health, and its values are the
  commanded positions. BLINK proves that the motors node loop is running, not that
  the Dynamixel bus answers (see "Overload detection").
- **Not verified at the source:** `um7_node` publishing `imu/data` as
  `sensor_msgs/Imu` (confirmed only from the consumer side, `decision_node`).
- Traffic subscriptions use `SensorDataQoS` (best effort), which connects to both
  reliable and best-effort publishers.

---

## Button actions and process control (execution layer)

Every `ButtonEvent` published by `stm_communication_pkg` has a consumer.

**Button 1, `panel_teleop_node`**
- Click: Pos 1. Double click: Macarena. Both are goals on the `control` action server
  (`custom_interfaces/action/Control`, `/control_action`, `int32 action_number`), the same
  interface `robot_joy_control` uses. From `control.cpp::choose_movement`: Pos 1 = 1
  (Stand Still), Macarena = 36.
- The client waits 500 ms for the server. If `control` is not up, the command is ignored
  with a log warning (no queue). `control` took about 13 s to be ready on the development VM.
- Hold: starts the GameController listener (`gamecontroller_bringup`, argument
  `team_number`, default 7). The node `game_controller connect` listens to the official
  GameController broadcast (UDP 3838) and answers on 3939. The connection itself depends on
  that software being on the same network with the right team number. The LED follows
  `/game_controller_connected` (true on the first packet, false after 20 s of silence).
- Arbitration: while `/game_controller_connected` is true, click and double click are
  ignored (the GameController has priority). Hold is exempt, since it is what starts the link.
- Conceptually this node belongs in the `control` package of the team repository (it issues the
  same commands as `robot_joy_control`). It lives in `control_panel_pkg` while the panel is
  developed standalone: **move it into `control` when merging**.

**Button 2, `system_status_node`**
- Click: toggle `decision`. Double click: toggle `control`. Hold: toggle the Robot Bringup.
- **Robot Bringup** = motors + IMU (UM7) + vision + decision + control, started as separate
  processes, as the team does by hand today (one terminal each). `robot_bringup.launch.py` is
  not used: on `main` it only includes vision and decision.
- The decision toggle is blocked while the bringup is active. The bringup state drops back to
  inactive if all its members die (after a 5 s grace period, because right after the hold the
  children have not finished starting).
- Real commands: `ros2 launch robot_bringup {decision,control,motors,vision,gamecontroller}_bringup.launch.py`
  and, for the IMU, `ros2 run um7 um7_node` (parameter `imu_command`).

**How processes are managed (`process_utils.hpp`)**
- State comes from the operating system (`pgrep`), not from an intent flag, so a process started
  by hand in a terminal is also seen as running and is toggled off correctly.
- Processes are started with a double `fork/exec`, detached (`setsid`), from `repo_dir` (`control`
  resolves its motion JSON relative to the working directory), with output in
  `/tmp/panel_<name>.log`. They keep running if the panel stops.
- They are stopped with SIGINT (the equivalent of Ctrl+C). Tested: a process started with
  `sh -c "... &"` from a non-interactive shell inherits SIGINT ignored, so `pkill -INT` did
  nothing. With `fork/exec` it stops, and the stop was confirmed on a real `ros2 launch`.
- Failures are loud: a missing `repo_dir` is logged as an error at startup and on every start
  attempt, and a bringup in which nothing started is not reported as active.

---

## Liveness: natural signals instead of heartbeat publishers

The first design had every monitored node publishing `/<node>/heartbeat`. After reading the
code of each package, almost none of them needs to be edited: the nodes already produce a
usable sign of life. `system_status_node` feeds these signals into the same table as the
explicit heartbeat topics (which are still accepted), with a 2 s timeout.

| Node | Existing signal | What it proves |
|---|---|---|
| vision | `/goalpost_count`, every processed frame | camera delivering frames and model running |
| um7 | `/imu/data` | IMU samples flowing (checked from the consumer side only) |
| motors | `/all_joints_position`, every 8 ms | the loop runs (commanded values, not bus health) |
| localization | `/localization_active`, every 500 ms, always true | the publisher is alive (package not built on the development VM) |
| control | `/control_action` server discoverable | the server exists (`control` has no periodic output) |
| decision | node `decision_node` present in the ROS graph | the weakest signal: it exists, not that it works |

Reasons for not adding heartbeat publishers: it would mean editing `motors_pkg`, `control` and
`decision` (walking-critical code in the team repository), the existing signals are stronger
evidence than a timer, and it works without coordinating changes with the team. The only
optional patch worth considering is a heartbeat from `RobotBehavior::players_behavior()` (the
8 ms behavior timer), which would make the decision signal functional.

---

## Overload detection: not implemented in motors_pkg

- `/motors/overload` has a subscriber (`system_status_node`) and no publisher. The aggregation
  (topic to LED) was tested with a mock publisher; the detection itself does not exist.
- In `motors_communication.cpp` the 8 ms loop is write-only: `setAllJointPos()` is a SyncWrite
  (which has no status reply), and the SyncRead in `getNoTorquePos()` only covers motors whose
  torque is off. With torque on, the node receives nothing from the motors, so overload cannot
  be inferred from the current traffic, and `/all_joints_position` echoes commanded positions.
- Detecting it needs a new periodic SyncRead of the Hardware Error Status register. On the
  X series (XM430/XM540, Protocol 2, the table this code uses) it is at address 70, with bit 5 as
  Overload Error (to be confirmed in the e-Manual). `robot_number` 4 uses Protocol 1, which has
  no such register. With the default shutdown configuration the error stays latched, so slow
  polling should be enough (also to be confirmed).
- Risk and plan: the read shares the bus with the 8 ms position writes that hold the walking.
  It must run at a low rate or in round-robin, behind a parameter that is off by default, and be
  tested on the bench with the robot suspended. The code belongs to the team, so the change must
  be discussed with the owner of `motors_pkg`.
- By-product: the same read gives real bus health (replies received), which would make the comm
  board LED and a motors heartbeat functional instead of "the loop is running".

---

## Validation status

**Validated in simulation** (development VM; events and signals injected with `ros2 topic pub`;
real `control`, `decision` and `gamecontroller_bringup` processes; no motors, camera, IMU or
STM32 in the loop):
- Button 1 click: Pos 1 goal sent and finished with `SUCCEEDED`.
- Button 1 click and double click with the GameController connected: ignored with the priority
  warning. Hold is not blocked and starts the listener.
- Button 2: decision and control start and stop as real processes, with a clean SIGINT shutdown.
- Button 2 hold: bringup toggles ON and OFF, the decision toggle is blocked while it is active.
- LEDs: localization (natural topic), overload (aggregation only, mocked), Robot Bringup (five
  signals, with control and decision real and the other three injected).
- A wrong `repo_dir` is reported as an error instead of failing silently.

**Not validated yet:**
- Physical path with the final hardware: real button, serial, ROS 2, and back to a real LED.
  Record the result of the hardware test here.
- GameController with real UDP packets on the competition network.
- Camera, IMU and comm board LEDs with real devices and real traffic.
- Real overload detection (see "Overload detection").
- Movement on the real robot, and `um7_node` publishing `imu/data` (checked from the consumer
  side only).

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

- [ ] **USB transport for the final PCB**: USBX standalone vs. external USB-UART bridge chip
      (see the USB stack discovery above); currently deferred
- [ ] **Overload detection**: opt-in periodic SyncRead of Hardware Error Status in `motors_pkg`
      (address 70, bit 5 on the X series, to be confirmed; `robot_number` 4 uses Protocol 1).
      Discuss with the owner of `motors_pkg`; bench test with the robot suspended. It would
      also give real bus health for the comm board LED
- [ ] Move `panel_teleop_node` into the `control` package, and merge `PanelStatus.msg` and
      `ButtonEvent.msg` into the team's `custom_interfaces`, when integrating with
      `RoboFEI-HT_2023_SOFTWARE`
- [ ] New udev rule for the STM32 (`/dev/stm_panel`) in `robot_plugins/robot-usb-ports.rules`
- [ ] Test the GameController with real UDP packets, and decide the `team_number` for the
      competition (the default is 7)
- [ ] Decide whether the Control toggle (Button 2, double click) must also be blocked while the
      bringup is active (today it is not, and it stops `control` if it is running)
- [ ] Pos 1 pressed before `control` is ready is ignored with a log-only warning, and there is no
      monitor on the robot. LED_RBT_BU is the natural feedback channel
- [ ] Verify at the source that `um7_node` publishes `imu/data` as `sensor_msgs/Imu`
- [ ] Optional: heartbeat from `RobotBehavior::players_behavior()` in `decision`, to make its
      liveness functional instead of "present in the graph"

Resolved: real launch names and the `control` interface (action), the motors feedback topic for
the comm board LED (with the limitation above), the comm board device (`/dev/motors`, so the
`/dev/commboard` rule is not needed), and the heartbeat publishers (not needed, see "Liveness").
