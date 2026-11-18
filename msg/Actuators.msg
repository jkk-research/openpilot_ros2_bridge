# Generated from openpilot car.capnp:CarControl.Actuators by scripts/generate_msgs.py; do not edit by hand.

uint8 LONG_CONTROL_STATE_OFF=0
uint8 LONG_CONTROL_STATE_PID=1
uint8 LONG_CONTROL_STATE_STOPPING=2
uint8 LONG_CONTROL_STATE_STARTING=3

float32 torque
float32 steering_angle_deg
float32 curvature
float32 accel
uint8 long_control_state
float32 gas
float32 brake
float32 torque_output_can
float32 speed
