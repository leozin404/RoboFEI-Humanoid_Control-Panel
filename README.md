# RoboFEI Control Panel

Physical control panel with buttons and status LEDs for the RoboFEI Humanoid
robot. This is an undergraduate research project (Iniciação Científica) developed at Centro
Universitário FEI, advised by Prof. Dr. Reinaldo Augusto da C. Bianchi.

## Overview

The panel has two physical buttons that each recognize a single click, a
double click, and a 2-second hold, plus seven status LEDs. An STM32U575CGT6
microcontroller reads the buttons, drives the LEDs, and bridges to the
robot's NUC over USB using a virtual COM port. It integrates
into the robot's ROS2 Humble software stack.

## Repository structure

- `docs/` — IC reports, design decision log, images
- `hardware/kicad/` — current PCB design (STM32U575)
- `hardware/legacy/` — archived ESP32-S3 prototype (Fusion 360, superseded)
- `software/` — ROS2 packages (`control_panel_pkg`, `stm_communication_pkg`, `custom_interfaces` additions)
- `software/stm_communication_pkg/firmware/` — embedded code running on the STM32 (button FSM + LED
  driver), ignored by `colcon` through a `COLCON_IGNORE` file and built with its own toolchain

## Architecture

- `software/stm_communication_pkg/firmware/` — button click/double-click/hold detection, generic
  OFF/ON/BLINKING LED driver
- `software/control_panel_pkg/` — status aggregation (USB presence and
  traffic, motor overload, node liveness, GameController connection) and the
  actions behind the buttons (motions through the `control` action server,
  GameController listener, bringup of the vital nodes)
- `software/stm_communication_pkg/` — serial bridge between the STM32 and ROS2

See `docs/design-decisions.md` for the rationale behind hardware/protocol choices.

## Button actions

| Button | Gesture | Action |
|---|---|---|
| 1 | click | Pos 1 (goal `action_number` 1 on `/control_action`) |
| 1 | double click | Macarena (`action_number` 36) |
| 1 | hold (2 s) | start the GameController listener (`gamecontroller_bringup`) |
| 2 | click | toggle `decision` (blocked while the bringup is active) |
| 2 | double click | toggle `control` |
| 2 | hold (2 s) | toggle the Robot Bringup: motors, IMU (UM7), vision, decision, control |

While the GameController is connected (`/game_controller_connected`), Pos 1
and Macarena are ignored: the GameController has priority. The hold on
Button 1 is not subject to this rule, since it is what starts the connection.

The hold threshold is 2 s, the debounce window 50 ms and the double-click window 400 ms
(`HOLD_MS`, `DEBOUNCE_MS` and `DOUBLE_CLICK_MS` in `button_fsm.hpp`).

Pos 1 and Macarena move the robot. Test them with the robot suspended.

## Status LEDs

| LED | Source |
|---|---|
| LOCALIZATION | `/localization_active`, or `/localization/heartbeat` |
| GAMECONTROLLER | `/game_controller_connected` |
| COMM BOARD | constant: `/dev/motors` present; blinking: `/all_joints_position` flowing |
| OVERLOAD | `/motors/overload` (publisher still to be implemented in `motors_pkg`) |
| CAM | constant: `/dev/camera` present; blinking: `/goalpost_count` flowing |
| IMU | constant: `/dev/imu` present; blinking: `/imu/data` flowing |
| RBT_BU | decision, vision, motors, um7 and control all alive |

If the NUC stops sending status for 2 seconds, all LEDs blink.

## Building and running

`control_panel_pkg` is built inside the `RoboFEI-HT_2023_SOFTWARE` workspace.
It depends on that repository's `custom_interfaces` (`Control.action`,
`JointState.msg`), so this repository's `PanelStatus.msg` and `ButtonEvent.msg`
must be added to it (messages and `rosidl_generate_interfaces`).

```bash
colcon build --packages-select custom_interfaces control_panel_pkg stm_communication_pkg
source install/setup.bash

ros2 launch stm_communication_pkg stm_communication.launch.py
ros2 launch control_panel_pkg control_panel_bringup.launch.py
```

Launch arguments of `control_panel_bringup.launch.py`:

| Argument | Default | Meaning |
|---|---|---|
| `repo_dir` | `~/RoboFEI-HT_2023_SOFTWARE` | team repository; processes started by the panel run from here |
| `team_number` | `7` | GameController team number |
| `imu_command` | `ros2 run um7 um7_node` | command that starts the IMU node |

Processes started by the panel run detached, so they keep running if the panel
stops. Their output goes to `/tmp/panel_<name>.log`. They are stopped with
SIGINT, the same as Ctrl+C in a terminal. If `repo_dir` does not exist, the
panel logs an error at startup and on every start attempt.

## Hardware

- MCU: STM32U575CGT6
- Regulator: LD1117 3.3V (SOT-223), powered from the NUC's USB 5V rail
- 7 green SMD LEDs, 2 Omron SMD pushbuttons
- Communication: USB virtual COM port (USB stack for the final PCB still being
  decided, see `docs/design-decisions.md`)

## Development status

- [x] Firmware: button FSM and LED driver (validated on a NUCLEO-U575ZI-Q breadboard rig)
- [ ] Firmware on the final PCB (USB transport decision pending)
- [x] `control_panel_pkg` (validated in simulation, see `docs/design-decisions.md`)
- [ ] `stm_communication_pkg`: end-to-end test with the final PCB
- [ ] Motor overload detection (`motors_pkg` patch, to be discussed with the team)
- [ ] Integration with the team repository (move `panel_teleop_node` into `control`, merge the interfaces)

## Reports

Partial and final IC reports are available in `docs/`.

## Author

Leonardo Aparecido da Silva — Scientific Initiation (IC/PIBIC-FEI),
RoboFEI, Centro Universitário FEI.
