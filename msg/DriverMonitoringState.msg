# Generated from openpilot log.capnp:DriverMonitoringState by scripts/generate_msgs.py; do not edit by hand.
# Published on /<namespace>/driverMonitoringState.

std_msgs/Header header
uint64 log_mono_time  # openpilot Event.logMonoTime, ns
bool valid  # openpilot Event.valid

uint8 ALERT_LEVEL_NONE=0
uint8 ALERT_LEVEL_ONE=1
uint8 ALERT_LEVEL_TWO=2
uint8 ALERT_LEVEL_THREE=3
uint8 ACTIVE_POLICY_WHEELTOUCH=0
uint8 ACTIVE_POLICY_VISION=1

bool lockout
int8 lockout_count
int8 lockout_minutes_remaining
int8 alert3_count
int8 no_response_count
bool no_response_force_decel
bool always_on
bool always_on_lockout
uint8 alert_level
uint8 active_policy
bool is_rhd
CalibrationState rhd_calibration
VisionPolicyState vision_policy_state
WheeltouchPolicyState wheeltouch_policy_state
