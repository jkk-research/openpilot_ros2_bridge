# ROS 2 bridge for openpilot

ROS 2 bridge for Comma 3/X and Comma 4 devices running openpilot.

## Quick start

To run on a vehicle PC whose ROS 2 stack uses CycloneDDS, including SSH setup for the Comma, see [QUICKSTART.md](QUICKSTART.md).

Both options need ROS 2 Jazzy, an openpilot checkout, and this package built in a ROS 2 workspace on the host:

```bash
cd /path/to/ros2_ws
colcon build --symlink-install --packages-select openpilot_ros2_bridge
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### Option A: Docker (recommended)

No Python setup on the host. The launch file pulls the prebuilt image and runs the bridge in a container:

```bash
ros2 launch openpilot_ros2_bridge openpilot_bridge_docker.launch.py \
  host_openpilot_root:=/path/to/openpilot \
  comma_ip:=<comma_device_ip>
```

To build the image from your local source instead, add `repo_root:=/path/to/openpilot_ros2_bridge image_name:=openpilot_ros2_bridge:local image_source:=build`.

### Option B: Local install

Build openpilot natively ([instructions](https://github.com/commaai/openpilot/tree/master/tools#native-setup-on-ubuntu-2404-and-macos)), create the bridge venv once with `./scripts/setup_bridge_venv.sh`, then:

```bash
export OPENPILOT_ROS2_BRIDGE_VENV=/path/to/openpilot_ros2_bridge/.venv
ros2 launch openpilot_ros2_bridge openpilot_bridge.launch.py \
  openpilot_root:=/path/to/openpilot \
  comma_ip:=<comma_device_ip>
```

In both cases, the Comma device must also run its side of the bridge (`cd openpilot/cereal/messaging && ./bridge`). Alternatively, add `start_remote_bridge:=true remote_bridge_target:=comma@<comma_device_ip>` to start it over SSH. The sections below cover the details.

## What the bridge publishes

The bridge now exposes:

- `modelV2` predictions, lane lines, road edges, leads, temporal pose, disengage predictions, and desired curvature
- `longitudinalPlan` as ROS 2 topics for speeds, accelerations, jerks, and planner state
- `lateralPlan` as ROS 2 topics for path points, curvature, solver state, and lane-change state
- `carState` and `carControl` summaries as ROS 2 topics
- RViz marker topics for plan, lane lines, road edges, and leads
- TF frames for openpilot's calibrated frame and the Comma device, driven by openpilot calibration
- optional compressed camera output

All topics are published under the `openpilot` namespace (for example `/openpilot/lane_lines`). Change it with `namespace:=<name>`.
Fields that a given openpilot version no longer has are published as `NaN`, so array layouts stay the same across versions.

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
- `bridge_topics:=modelV2,carControl,carState,longitudinalPlan,lateralPlan,liveCalibration,extrinsicsCalibration`
- `namespace:=openpilot`
- `camera_ip:=<comma_device_ip>`
- `camera_port:=8002`
- `camera_name:=roadCamera`

## TF frames

With `start_tf:=true` (default), `openpilot_tf_publisher` broadcasts:

- `base_frame` → `openpilot`: openpilot's calibrated frame (forward, right, down), with its origin at the Comma device. Model outputs and markers live in this frame. The translation is the device's mount position in `base_frame`. When `use_calibration_height:=true`, the height comes from openpilot calibration minus `base_frame_height`, the height of `base_frame` above the road.
- `openpilot` → `openpilot_device`: the physical device orientation, the inverse of calibration `rpyCalib`, updated as calibration changes.

Calibration is read from `liveCalibration`, or from `extrinsicsCalibration` on openpilot versions that use that name. Example for a car whose `base_link` is 0.37 m above the road, with the Comma 1.6 m ahead of it:

```bash
ros2 launch openpilot_ros2_bridge openpilot_bridge.launch.py \
  openpilot_root:=/path/to/openpilot comma_ip:=<comma_device_ip> \
  base_frame:=base_link device_x:=1.6 device_y:=0.0 base_frame_height:=0.37
```

## Verifying the topics

After launch, verify that the lane and plan topics are active:

```bash
ros2 topic list | grep /openpilot/
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
ros2 topic echo /openpilot/plan_markers
ros2 topic echo /openpilot/lanes_markers
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
