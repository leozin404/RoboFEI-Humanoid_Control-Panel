# ButtonEvent.msg
# Published by stm_communication_pkg whenever the STM32 reports a
# completed button gesture (single click, double click, or 5s hold).

uint8 SINGLE_CLICK = 1
uint8 DOUBLE_CLICK = 2
uint8 HOLD         = 3

uint8 button_id      # 1 or 2
uint8 click_type      # one of the constants above

builtin_interfaces/Time stamp

