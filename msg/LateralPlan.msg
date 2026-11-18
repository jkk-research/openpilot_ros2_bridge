# Generated from openpilot log.capnp:LateralPlan by scripts/generate_msgs.py; do not edit by hand.
# Published on /<namespace>/lateralPlan.

std_msgs/Header header
uint64 log_mono_time  # openpilot Event.logMonoTime, ns
bool valid  # openpilot Event.valid

uint8 DESIRE_NONE=0
uint8 DESIRE_TURN_LEFT=1
uint8 DESIRE_TURN_RIGHT=2
uint8 DESIRE_LANE_CHANGE_LEFT=3
uint8 DESIRE_LANE_CHANGE_RIGHT=4
uint8 DESIRE_KEEP_LEFT=5
uint8 DESIRE_KEEP_RIGHT=6
uint8 LANE_CHANGE_STATE_OFF=0
uint8 LANE_CHANGE_STATE_PRE_LANE_CHANGE=1
uint8 LANE_CHANGE_STATE_LANE_CHANGE_STARTING=2
uint8 LANE_CHANGE_STATE_LANE_CHANGE_FINISHING=3
uint8 LANE_CHANGE_DIRECTION_NONE=0
uint8 LANE_CHANGE_DIRECTION_LEFT=1
uint8 LANE_CHANGE_DIRECTION_RIGHT=2

uint64 model_mono_time
float32[] d_path_points
bool mpc_solution_valid
uint8 desire
uint8 lane_change_state
uint8 lane_change_direction
bool use_lane_lines
float32[] psis
float32[] curvatures
float32[] curvature_rates
float32 solver_execution_time
float32 solver_cost
SolverState solver_state
