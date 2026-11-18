# Generated from openpilot car.capnp:CarState.ButtonEvent by scripts/generate_msgs.py; do not edit by hand.

uint8 TYPE_UNKNOWN=0
uint8 TYPE_LEFT_BLINKER=1
uint8 TYPE_RIGHT_BLINKER=2
uint8 TYPE_ACCEL_CRUISE=3
uint8 TYPE_DECEL_CRUISE=4
uint8 TYPE_CANCEL=5
uint8 TYPE_LKAS=6
uint8 TYPE_ALT_BUTTON2=7
uint8 TYPE_MAIN_CRUISE=8
uint8 TYPE_SET_CRUISE=9
uint8 TYPE_RESUME_CRUISE=10
uint8 TYPE_GAP_ADJUST_CRUISE=11

bool pressed
uint8 type
