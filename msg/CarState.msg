# Generated from openpilot car.capnp:CarState by scripts/generate_msgs.py; do not edit by hand.
# Published on /<namespace>/carState.

std_msgs/Header header
uint64 log_mono_time  # openpilot Event.logMonoTime, ns
bool valid  # openpilot Event.valid

uint8 GEAR_SHIFTER_UNKNOWN=0
uint8 GEAR_SHIFTER_PARK=1
uint8 GEAR_SHIFTER_DRIVE=2
uint8 GEAR_SHIFTER_NEUTRAL=3
uint8 GEAR_SHIFTER_REVERSE=4
uint8 GEAR_SHIFTER_SPORT=5
uint8 GEAR_SHIFTER_LOW=6
uint8 GEAR_SHIFTER_BRAKE=7
uint8 GEAR_SHIFTER_ECO=8
uint8 GEAR_SHIFTER_MANUMATIC=9

bool can_valid
bool can_timeout
uint32 can_error_counter
float32 cum_lag_ms
float32 v_ego
float32 a_ego
float32 v_ego_raw
float32 v_ego_cluster
float32 v_cruise
float32 v_cruise_cluster
float32 yaw_rate
bool standstill
WheelSpeeds wheel_speeds
bool gas_pressed
bool brake_pressed
bool regen_braking
bool parking_brake
bool brake_hold_active
float32 steering_angle_deg
float32 steering_angle_offset_deg
float32 steering_rate_deg
float32 steering_torque
float32 steering_torque_eps
bool steering_pressed
bool steering_disengage
bool steer_fault_temporary
bool steer_fault_permanent
bool invalid_lkas_setting
bool stock_aeb
bool stock_lkas
bool stock_fcw
bool esp_disabled
bool acc_faulted
bool car_faulted_non_critical
bool esp_active
bool vehicle_sensors_invalid
bool low_speed_alert
bool block_pcm_enable
bool car_not_ready
CruiseState cruise_state
uint8 gear_shifter
ButtonEvent[] button_events
bool button_enable
bool left_blinker
bool right_blinker
bool generic_toggle
bool door_open
bool seatbelt_unlatched
bool left_blindspot
bool right_blindspot
float32 fuel_gauge
bool charging
