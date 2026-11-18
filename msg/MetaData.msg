# Generated from openpilot log.capnp:ModelDataV2.MetaData by scripts/generate_msgs.py; do not edit by hand.

uint8 LANE_CHANGE_STATE_OFF=0
uint8 LANE_CHANGE_STATE_PRE_LANE_CHANGE=1
uint8 LANE_CHANGE_STATE_LANE_CHANGE_STARTING=2
uint8 LANE_CHANGE_STATE_LANE_CHANGE_FINISHING=3
uint8 LANE_CHANGE_DIRECTION_NONE=0
uint8 LANE_CHANGE_DIRECTION_LEFT=1
uint8 LANE_CHANGE_DIRECTION_RIGHT=2

float32 engaged_prob
float32[] desire_prediction
float32[] desire_state
DisengagePredictions disengage_predictions
bool hard_brake_predicted
uint8 lane_change_state
uint8 lane_change_direction
