# PanelStatus.msg
# Status of the 7 control panel LEDs, published by
# control_panel_pkg/system_status_node and consumed by
# stm_communication_pkg to drive the physical panel.
# The order of the fields below is the byte order of the serial frame:
# stm_communication_node and the firmware (main.cpp) must agree on it.

# LED mode values (shared by every field below)
uint8 OFF     = 0
uint8 ON      = 1
uint8 BLINK   = 2

uint8 led_localization      # ON: /localization_active is true, or /localization/heartbeat received recently
uint8 led_gamecontroller    # ON: /game_controller_connected is true
uint8 led_comm_board        # OFF: /dev/motors missing, ON: present, BLINK: /all_joints_position flowing
uint8 led_overload          # ON: /motors/overload is true (publisher not implemented in motors_pkg yet)
uint8 led_cam               # OFF: /dev/camera missing, ON: present, BLINK: /goalpost_count flowing
uint8 led_imu               # OFF: /dev/imu missing, ON: present, BLINK: /imu/data flowing
uint8 led_rbt_bu            # ON: decision, vision, motors, um7 and control are all alive

builtin_interfaces/Time stamp   # when this status was assembled
