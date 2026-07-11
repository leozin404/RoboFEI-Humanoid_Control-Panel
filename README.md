# RoboFEI Control Panel

Physical control panel with buttons and status LEDs for the RoboFEI Humanoid
robot. This is an undergraduate research project (Iniciação Científica) developed at Centro
Universitário FEI, advised by Prof. Dr. Reinaldo Augusto da C. Bianchi.

## Overview

The panel has two physical buttons that each recognize a single click, a
double click, and a 5-second hold, plus seven status LEDs. An STM32U575CGT6
microcontroller reads the buttons, drives the LEDs, and bridges to the
robot's NUC over USB using the CDC virtual COM port class. It integrates
into the robot's ROS2 Humble software stack.

## Repository structure

- `docs/` — IC reports, design decision log, images
- `hardware/kicad/` — current PCB design (STM32U575)
- `hardware/legacy/` — archived ESP32-S3 prototype (Fusion 360, superseded)
- `firmware/` — embedded code running on the STM32 (button FSM + LED driver)
- `software/` — ROS2 packages (control_panel_pkg, stm_communication_pkg)

## Architecture

- `firmware/` — button click/double-click/hold detection, generic
  OFF/ON/BLINKING LED driver
- `software/control_panel_pkg/` — status aggregation logic (USB monitoring,
  motor overload, node heartbeats, GameController connection)
- `software/stm_communication_pkg/` — serial bridge between the STM32 and ROS2

See `docs/design-decisions.md` for the rationale behind hardware/protocol choices.

## Hardware

- MCU: STM32U575CGT6
- Regulator: LD1117 3.3V (SOT-223), powered from the NUC's USB 5V rail
- 7 green SMD LEDs, 2 Omron SMD pushbuttons
- Communication: USB CDC (Virtual COM Port)

## Development status

- [ ] Firmware: button FSM
- [ ] Firmware: LED driver
- [ ] `stm_communication_pkg`
- [ ] `control_panel_pkg`
- [ ] Integration with `robot_bringup`

## Reports

Partial and final IC reports are available in `docs/reports/`.

## Author

Leonardo Aparecido da Silva — Scientific Initiation (IC/PIBIC-FEI),
RoboFEI, Centro Universitário FEI.
