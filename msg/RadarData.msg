# Generated from openpilot car.capnp:RadarData by scripts/generate_msgs.py; do not edit by hand.
# Published on /<namespace>/radarTracks.

std_msgs/Header header
uint64 log_mono_time  # openpilot Event.logMonoTime, ns
bool valid  # openpilot Event.valid

RadarDataError errors
RadarPoint[] points
