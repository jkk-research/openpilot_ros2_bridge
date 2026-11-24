# Generated from openpilot log.capnp:RadarState by scripts/generate_msgs.py; do not edit by hand.
# Published on /<namespace>/radarState.

std_msgs/Header header
uint64 log_mono_time  # openpilot Event.logMonoTime, ns
bool valid  # openpilot Event.valid

uint64 md_mono_time
RadarDataError radar_errors
LeadData lead_one
LeadData lead_two
