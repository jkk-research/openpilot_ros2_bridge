# One CAN message decoded by the drivetrain node with opendbc: every data signal of the DBC message, by name.
# Published on /<namespace>/can, one message per received frame.

std_msgs/Header header
uint64 log_mono_time  # openpilot Event.logMonoTime of the can batch the frame came in, ns
bool valid  # openpilot Event.valid of that can batch

string dbc  # opendbc DBC name, e.g. toyota_nodsu_pt_generated
uint8 bus
uint32 address
string name  # DBC message name, e.g. GEAR_PACKET_HYBRID

# Parallel lists in DBC order, values in DBC units; checksum and counter signals are left out where opendbc checks them
string[] signal_names
float64[] values
