# Generated from openpilot car.capnp:CarOutput by scripts/generate_msgs.py; do not edit by hand.
# Published on /<namespace>/carOutput.

std_msgs/Header header
uint64 log_mono_time  # openpilot Event.logMonoTime, ns
bool valid  # openpilot Event.valid

Actuators actuators_output
