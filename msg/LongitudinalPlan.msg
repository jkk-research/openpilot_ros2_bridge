# Generated from openpilot log.capnp:LongitudinalPlan by scripts/generate_msgs.py; do not edit by hand.
# Published on /<namespace>/longitudinalPlan.

std_msgs/Header header
uint64 log_mono_time  # openpilot Event.logMonoTime, ns
bool valid  # openpilot Event.valid

uint8 LONGITUDINAL_PLAN_SOURCE_CRUISE=0
uint8 LONGITUDINAL_PLAN_SOURCE_LEAD0=1
uint8 LONGITUDINAL_PLAN_SOURCE_LEAD1=2
uint8 LONGITUDINAL_PLAN_SOURCE_LEAD2=3
uint8 LONGITUDINAL_PLAN_SOURCE_E2E=4

uint64 model_mono_time
bool has_lead
bool fcw
uint8 longitudinal_plan_source
float32 processing_delay
float32[] accels
float32[] speeds
float32[] jerks
float32 a_target
bool should_stop
bool allow_throttle
bool allow_brake
float32 accel_boost
float32 solver_execution_time
