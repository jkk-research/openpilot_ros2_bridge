# Generated from openpilot car.capnp:CarControl.HUDControl by scripts/generate_msgs.py; do not edit by hand.

uint8 VISUAL_ALERT_NONE=0
uint8 VISUAL_ALERT_FCW=1
uint8 VISUAL_ALERT_STEER_REQUIRED=2
uint8 VISUAL_ALERT_BRAKE_PRESSED=3
uint8 VISUAL_ALERT_WRONG_GEAR=4
uint8 VISUAL_ALERT_SEATBELT_UNBUCKLED=5
uint8 VISUAL_ALERT_SPEED_TOO_HIGH=6
uint8 VISUAL_ALERT_LDW=7
uint8 AUDIBLE_ALERT_NONE=0
uint8 AUDIBLE_ALERT_ENGAGE=1
uint8 AUDIBLE_ALERT_DISENGAGE=2
uint8 AUDIBLE_ALERT_REFUSE=3
uint8 AUDIBLE_ALERT_WARNING_SOFT=4
uint8 AUDIBLE_ALERT_WARNING_IMMEDIATE=5
uint8 AUDIBLE_ALERT_PROMPT=6
uint8 AUDIBLE_ALERT_PROMPT_REPEAT=7
uint8 AUDIBLE_ALERT_PROMPT_DISTRACTED=8

bool speed_visible
float32 set_speed
bool lanes_visible
bool lead_visible
uint8 visual_alert
bool right_lane_visible
bool left_lane_visible
bool right_lane_depart
bool left_lane_depart
int8 lead_distance_bars
uint8 audible_alert
