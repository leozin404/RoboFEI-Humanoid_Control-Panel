# PanelStatus.msg
# Add status of all 7 control panel LEDs, published by
# control_panel_pkg/system_status_node and consumed by
# stm_communication_pkg to drive the physical panel.

# LED mode values (shared by every field below)
uint8 OFF     = 0
uint8 ON      = 1
uint8 BLINK   = 2

uint8 led_localization      # active: heartbeat from localization node.
uint8 led_gamecontroller    # /game_controller_connected.
uint8 led_comm_board        # USB presence + traffic on the comm board port.
uint8 led_overload          # /motors/overload.
uint8 led_cam               # USB presence + traffic on the camera port.
uint8 led_imu               # USB presence + traffic on the IMU port.
uint8 led_rbt_bu            # added heartbeat: decision, vision, motors, imu, contro.l

builtin_interfaces/Time stamp   # when this status was assembled

