# Generated from openpilot log.capnp:ExtrinsicsCalibration by scripts/generate_msgs.py; do not edit by hand.
# Published on /<namespace>/extrinsicsCalibration.

std_msgs/Header header
uint64 log_mono_time  # openpilot Event.logMonoTime, ns
bool valid  # openpilot Event.valid

uint8 CAL_STATUS_UNCALIBRATED=0
uint8 CAL_STATUS_CALIBRATED=1
uint8 CAL_STATUS_INVALID=2
uint8 CAL_STATUS_RECALIBRATING=3

uint8 cal_status
int32 cal_cycle
int8 cal_perc
int32 valid_blocks
float32[] extrinsic_matrix
float32[] rpy_calib
float32[] rpy_calib_spread
float32[] wide_from_device_euler
float32[] height
