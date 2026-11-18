# Generated from openpilot log.capnp:DriverStateV2 by scripts/generate_msgs.py; do not edit by hand.
# Published on /<namespace>/driverStateV2.

std_msgs/Header header
uint64 log_mono_time  # openpilot Event.logMonoTime, ns
bool valid  # openpilot Event.valid

uint32 frame_id
float32 model_execution_time
float32 gpu_execution_time
float32 wheel_on_right_prob
DriverData left_driver_data
DriverData right_driver_data
