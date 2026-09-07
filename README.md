# ROS 2 bridge for openpilot

ROS 2 bridge for Comma 3/X and Comma 4 devices running openpilot.

## What the bridge publishes

The bridge now exposes:

- `modelV2` predictions, lane lines, road edges, leads, temporal pose, disengage predictions, and desired curvature
- `longitudinalPlan` as ROS 2 topics for speeds, accelerations, jerks, and planner state
- `lateralPlan` as ROS 2 topics for path points, curvature, solver state, and lane-change state
- `carState` and `carControl` summaries as ROS 2 topics
- RViz marker topics for plan, lane lines, road edges, and leads
- optional compressed camera output

## Prerequisites

1. Install ROS 2 Jazzy.
2. Install and set up the openpilot environment by following the official instructions: https://github.com/commaai/openpilot/tree/master/tools#native-setup-on-ubuntu-2404-and-macos
3. Build this repository in a ROS 2 workspace:

   ```bash
   cd /path/to/ros2_ws
   colcon build --symlink-install --packages-select openpilot_ros2_bridge
   ```

## Launching the bridge

If you want the Comma device to start its side of the cereal bridge through SSH, enable `start_remote_bridge`.
Otherwise start it manually on the device:

```bash
cd openpilot/cereal/messaging
./bridge
```

Then launch the ROS 2 bridge stack:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/openpilot/.venv/bin/activate
source /path/to/ros2_ws/install/setup.bash

ros2 launch openpilot_ros2_bridge openpilot_bridge.launch.py \
  openpilot_root:=/path/to/openpilot \
  comma_ip:=<comma_device_ip> \
  start_bridge:=true \
  start_camera:=false
```

Useful optional arguments:

- `start_remote_bridge:=true`
- `remote_bridge_target:=comma@<comma_device_ip>`
- `bridge_topics:=modelV2,carControl,carState,longitudinalPlan,lateralPlan`
- `camera_ip:=<comma_device_ip>`
- `camera_port:=8002`
- `camera_name:=roadCamera`

## Verifying the topics

After launch, verify that the lane and plan topics are active:

```bash
ros2 topic list | grep -E 'lane|plan|car_state|car_control|desired_curvature'
```

Useful topics include:

- `lane_lines`
- `lane_line_probs`
- `position`
- `longitudinal_plan`
- `longitudinal_plan_state`
- `lateral_plan`
- `lateral_plan_state`
- `car_state_motion`
- `car_control_actuators`

For RViz visualization:

```bash
ros2 topic echo /plan_markers
ros2 topic echo /lanes_markers
```

## Virtual environment / container workflow

The `docker/Dockerfile` provides a ROS 2 Jazzy plus openpilot environment.
Inside the container, build this package with `colcon build --symlink-install`, source ROS 2 and the openpilot `.venv`, and launch the stack with the same `ros2 launch` command shown above.

[@tambetm](https://github.com/tambetm) Thanks for the original ROS1 version.

