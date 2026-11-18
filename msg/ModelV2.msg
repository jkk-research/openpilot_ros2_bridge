# Generated from openpilot log.capnp:ModelDataV2 by scripts/generate_msgs.py; do not edit by hand.
# Published on /<namespace>/modelV2.

std_msgs/Header header
uint64 log_mono_time  # openpilot Event.logMonoTime, ns
bool valid  # openpilot Event.valid

uint8 CONFIDENCE_RED=0
uint8 CONFIDENCE_YELLOW=1
uint8 CONFIDENCE_GREEN=2

uint32 frame_id
uint32 frame_id_extra
uint32 frame_age
float32 frame_drop_perc
uint64 timestamp_eof
float32 model_execution_time
bool big
XYZTData position
XYZTData orientation
XYZTData velocity
XYZTData orientation_rate
XYZTData acceleration
XYZTData[] lane_lines
float32[] lane_line_probs
float32[] lane_line_stds
XYZTData[] road_edges
float32[] road_edge_stds
LeadDataV3[] leads_v3
MetaData meta
uint8 confidence
Action action
