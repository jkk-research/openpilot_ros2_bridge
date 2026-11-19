# A drive command for openpilot's joystick mode, sent from ROS 2 to the Comma (start_control:=true).
# Publish it on /<namespace>/joystick_command at 10 Hz or faster. The newest command is held for control_timeout
# (0.25 s by default) after it arrives; after that the car gets accel 0 and steer 0 until the next one.
# Commands only act while the driver has engaged openpilot with the car's cruise control. openpilot and the panda
# keep their own limits, and the brake pedal disengages as usual. Joystick mode doesn't follow lead cars or keep
# the lane: these commands are all openpilot does.

std_msgs/Header header  # not used: commands are timed by when they arrive

float32 accel  # m/s², positive speeds up, negative brakes; limited to -4 to 4 (openpilot's joystick range).
               # 0 holds the speed; below 0.3 m/s it stops the car. Only cars with openpilot longitudinal control
               # follow it.
float32 steer  # -1 to 1, positive steers left. On cars steered by torque, the share of the car's maximum steering
               # torque (0: no torque); on cars steered by angle or curvature, the share of the angle that gives
               # 3 m/s² lateral acceleration (0: straight ahead).
