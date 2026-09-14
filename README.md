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
2. Have an openpilot checkout available locally.
3. Build the openpilot messaging bridge in that checkout by following the official openpilot setup instructions: https://github.com/commaai/openpilot/tree/master/tools#native-setup-on-ubuntu-2404-and-macos
4. Create the bridge virtual environment:

   ```bash
   cd /path/to/openpilot_ros2_bridge
   ./scripts/setup_bridge_venv.sh
   ```

5. Build this repository in a ROS 2 workspace:

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
source /path/to/openpilot_ros2_bridge/.venv/bin/activate
source /path/to/ros2_ws/install/setup.bash
export OPENPILOT_ROS2_BRIDGE_VENV=/path/to/openpilot_ros2_bridge/.venv

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

The `docker/Dockerfile` provides a ROS 2 Jazzy environment for the bridge.
It copies the local repository into `/workspace/ros2_ws/src/openpilot_ros2_bridge`, creates a dedicated bridge `.venv`, installs the required Python packages there, builds the ROS 2 package with `colcon`, and expects only an openpilot source checkout to be bind-mounted at runtime.

Build the image manually:

```bash
cd /path/to/openpilot_ros2_bridge
docker build -t openpilot_ros2_bridge:local -f docker/Dockerfile .
```

Validate ROS 2 and openpilot inside the container:

```bash
docker run --rm --network host --ipc host \
  -v /path/to/openpilot:/workspace/openpilot \
  openpilot_ros2_bridge:local \
  bash -lc 'source /opt/ros/jazzy/setup.bash && \
            source /workspace/ros2_ws/src/openpilot_ros2_bridge/.venv/bin/activate && \
            source /workspace/ros2_ws/install/setup.bash && \
            export PYTHONPATH=/workspace/openpilot/openpilot:${PYTHONPATH} && \
            python3 -c "import rclpy, cereal.messaging; print(\"ROS 2 and openpilot imports OK\")" && \
            ros2 pkg prefix openpilot_ros2_bridge && \
            test -x /workspace/openpilot/cereal/messaging/bridge'
```

Launch the containerized bridge directly from a ROS 2 launch file on the host:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/ros2_ws/install/setup.bash

ros2 launch openpilot_ros2_bridge openpilot_bridge_docker.launch.py \
  host_openpilot_root:=/path/to/openpilot \
  image_name:=ghcr.io/jkk-research/openpilot_ros2_bridge:latest \
  image_source:=pull \
  comma_ip:=<comma_device_ip> \
  ros_domain_id:=0 \
  start_bridge:=true
```

For local development from source, swap to:

```bash
ros2 launch openpilot_ros2_bridge openpilot_bridge_docker.launch.py \
  repo_root:=/path/to/openpilot_ros2_bridge \
  host_openpilot_root:=/path/to/openpilot \
  image_name:=openpilot_ros2_bridge:local \
  image_source:=build
```

The Docker launch file either pulls or builds the image, starts the container with `--network host` and `--ipc host`, forwards `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION`, and then runs the in-container `openpilot_bridge.launch.py`.

For a quick runtime smoke test without a Comma device connection:

```bash
docker run --rm --network host --ipc host openpilot_ros2_bridge:local \
  -v /path/to/openpilot:/workspace/openpilot \
  bash -lc 'source /opt/ros/jazzy/setup.bash && \
            source /workspace/ros2_ws/src/openpilot_ros2_bridge/.venv/bin/activate && \
            source /workspace/ros2_ws/install/setup.bash && \
            export PYTHONPATH=/workspace/openpilot/openpilot:${PYTHONPATH} && \
            timeout 15 ros2 launch openpilot_ros2_bridge openpilot_bridge.launch.py start_bridge:=false start_camera:=false'
```

## Published container image

The repository now includes `/home/runner/work/openpilot_ros2_bridge/openpilot_ros2_bridge/.github/workflows/publish-docker.yml`, which publishes the Docker image to GHCR on pushes to `main`, version tags, and manual runs.

After the workflow publishes an image, users can pull it with:

```bash
docker pull ghcr.io/jkk-research/openpilot_ros2_bridge:latest
```

[@tambetm](https://github.com/tambetm) Thanks for the original ROS1 version.
