# Generated from openpilot car.capnp:CarControl by scripts/generate_msgs.py; do not edit by hand.
# Published on /<namespace>/carControl.

std_msgs/Header header
uint64 log_mono_time  # openpilot Event.logMonoTime, ns
bool valid  # openpilot Event.valid

bool enabled
bool lat_active
bool long_active
Actuators actuators
bool left_blinker
bool right_blinker
float32[] orientation_ned
float32[] angular_velocity
float32 current_curvature
bool driver_monitoring_escalation
CruiseControl cruise_control
HUDControl hud_control
