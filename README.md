# ROS 2 bridge for openpilot

ROS 2 bridge for Comma 3/X and Comma 4 devices running openpilot.

## Quick start

To run on a vehicle PC whose ROS 2 stack uses CycloneDDS, including SSH setup for the Comma, see [QUICKSTART.md](QUICKSTART.md).

### Option A: Docker (recommended)

Nothing to build. The launch file runs straight from this checkout, and the published image already contains the bridge, its Python packages, and CycloneDDS.
The host only needs Docker, ROS 2 (Humble or newer) to run `ros2 launch`, and an openpilot checkout:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch /path/to/openpilot_ros2_bridge/launch/openpilot_bridge_docker.launch.py \
  host_openpilot_root:=/path/to/openpilot \
  comma_ip:=<comma_device_ip>
```

- If the openpilot checkout isn't built, add `prepare_openpilot:=true`. The launch then checks it out at the Comma's commit and builds the two parts the bridge uses, msgq and the cereal bridge, inside the container. This needs SSH access to the Comma; see [QUICKSTART.md](QUICKSTART.md).
- If the image can't be pulled, for example offline in the car, the launch uses the copy pulled last time.
- To use an image built from your local source instead, add `image_name:=openpilot_ros2_bridge:local image_source:=build`.

### Option B: Local install

Build openpilot natively ([instructions](https://github.com/commaai/openpilot/tree/master/tools#native-setup-on-ubuntu-2404-and-macos)) and create the bridge venv once with `./scripts/setup_bridge_venv.sh`. Then build this package in a ROS 2 Jazzy workspace and launch it:

```bash
cd /path/to/ros2_ws
colcon build --symlink-install --packages-select openpilot_ros2_bridge
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export OPENPILOT_ROS2_BRIDGE_VENV=/path/to/openpilot_ros2_bridge/.venv
ros2 launch openpilot_ros2_bridge openpilot_bridge.launch.py \
  openpilot_root:=/path/to/openpilot \
  comma_ip:=<comma_device_ip>
```

In both cases, the Comma device must also run its side of the bridge (`cd openpilot/cereal/messaging && ./bridge`). Alternatively, add `start_remote_bridge:=true remote_bridge_target:=comma@<comma_device_ip>` to start it over SSH. The sections below cover the details.

Driver monitoring topics are published by default. To also stream the cameras, add `start_camera:=true cameras:=road,wide_road,driver` (see [Cameras](#cameras)).

## What the bridge publishes

The bridge now exposes:

- `modelV2` predictions, lane lines, road edges, leads, temporal pose, disengage predictions, and desired curvature
- `longitudinalPlan` as ROS 2 topics for speeds, accelerations, jerks, and planner state
- `lateralPlan` as ROS 2 topics for path points, curvature, solver state, and lane-change state
- `carState` and `carControl` summaries as ROS 2 topics
- RViz marker topics for plan, lane lines, road edges, and leads
- TF frames for openpilot's calibrated frame and the Comma device, driven by openpilot calibration
- driver monitoring model output (`driverStateV2`) and policy state (`driverMonitoringState`)
- optional road, wide road, and driver camera streams as raw and JPEG-compressed images

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
- `bridge_topics:=modelV2,carControl,carState,longitudinalPlan,lateralPlan,liveCalibration,extrinsicsCalibration,driverStateV2,driverMonitoringState`
- `namespace:=openpilot`
- `start_driver_monitoring:=false`
- `start_camera:=true`
- `cameras:=road,wide_road,driver`
- `camera_ip:=<comma_device_ip>` (defaults to `comma_ip`)
- `camera_scale:=0.5`
- `camera_jpeg_quality:=80`

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

## Cameras

With `start_camera:=true`, the launch file starts one camera node per entry in `cameras`.
Each node connects directly to the cereal bridge on the device (`camera_ip`, which defaults to `comma_ip`), so the camera streams do not need to be in `bridge_topics`.
It decodes the video with PyAV and publishes:

- `/openpilot/<camera>_camera/image_raw` (`sensor_msgs/Image`, `bgr8`)
- `/openpilot/<camera>_camera/image_raw/compressed` (`sensor_msgs/CompressedImage`, JPEG)

| `cameras` entry | openpilot stream | Video |
| --- | --- | --- |
| `road` | `narrowRoadEncodeData` (`roadEncodeData` on older openpilot) | full resolution HEVC, 5-10 Mbit/s |
| `wide_road` | `wideRoadEncodeData` | full resolution HEVC, 5-10 Mbit/s |
| `driver` | `cabinEncodeData` (`driverEncodeData` on older openpilot) | full resolution HEVC, 5-10 Mbit/s |
| `q_road` | `qNarrowRoadEncodeData` (`qRoadEncodeData` on older openpilot) | 526x330 H.264, 256 kbit/s |

```bash
ros2 launch openpilot_ros2_bridge openpilot_bridge.launch.py \
  openpilot_root:=/path/to/openpilot \
  comma_ip:=<comma_device_ip> \
  start_camera:=true \
  cameras:=road,wide_road,driver \
  camera_scale:=0.5
```

Notes:

- openpilot only runs its video encoder while the device is onroad, so the camera topics stay silent offroad.
- `camera_scale` resizes the images before publishing, and `camera_jpeg_quality` (1-100) sets the JPEG quality.
- A node always decodes its stream, but only converts and compresses images while something subscribes to that topic.
- Over a weak Wi-Fi link, stream fewer cameras or use `q_road`.
- If you created the bridge venv before camera support was added, rerun `./scripts/setup_bridge_venv.sh` to install PyAV.

## Driver monitoring

The driver monitoring node is on by default (`start_driver_monitoring:=true`).
It reads `driverStateV2` and `driverMonitoringState` through the local bridge, so keep both in `bridge_topics` if you override it.
All topics are `std_msgs/Float32MultiArray`, in the same style as the other topics.

Model output (`driverStateV2`). For the face topics, row 0 is the left-hand drive seat and row 1 is the right-hand drive seat:

| Topic | Shape | Contents |
| --- | --- | --- |
| `driver_state` | 4 | frame id, model execution time, GPU execution time, wheel on right probability |
| `driver_face_orientation` | 2x3 | pitch, yaw, roll (rad) |
| `driver_face_orientation_std` | 2x3 | standard deviation of the above |
| `driver_face_position` | 2x2 | x, y of the face in the driver camera image, relative to the image center |
| `driver_face_position_std` | 2x2 | standard deviation of the above |
| `driver_face_probs` | 2x8 | face, left eye, right eye, left blink, right blink, sunglasses, phone, sleep probabilities (NaN when the openpilot version does not provide one) |

Policy state (`driverMonitoringState`):

| Topic | Shape | Contents |
| --- | --- | --- |
| `driver_monitoring_state` | 13 | alert level, active policy (0 wheeltouch, 1 vision), is RHD, lockout, lockout count, lockout minutes remaining, alert 3 count, no response count, no response force decel, always on, always on lockout, RHD calibrated percent, RHD offset |
| `driver_monitoring_vision` | 18 | awareness percent, awareness step, is distracted, face detected, distracted by pose / eye / phone / sleep, pose pitch, pose yaw, pose calibrated, pose uncertainty, pitch calibrated percent, pitch offset, yaw calibrated percent, yaw offset, wheeltouch fallback percent, uncertain offroad alert percent |
| `driver_monitoring_wheeltouch` | 3 | awareness percent, awareness step, driver interacting |
| `driver_monitoring_legacy_state` | 15 | Only for older openpilot versions: face detected, is distracted, distracted type, awareness status, awareness active, awareness passive, step change, pose pitch offset, pose pitch valid count, pose yaw offset, pose yaw valid count, is low std, hi std count, is active mode, is RHD |

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
docker run --rm --network host --ipc host --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v /path/to/openpilot:/workspace/openpilot \
  ghcr.io/jkk-research/openpilot_ros2_bridge:latest \
  bash -lc 'source /opt/ros/jazzy/setup.bash && \
            source /workspace/ros2_ws/src/openpilot_ros2_bridge/.venv/bin/activate && \
            source /workspace/ros2_ws/install/setup.bash && \
            export PYTHONPATH=/workspace/openpilot:/workspace/openpilot/openpilot:/workspace/openpilot/opendbc_repo:/workspace/openpilot/msgq_repo:${PYTHONPATH} && \
            python3 -c "import rclpy, av, cereal.messaging; print(\"ROS 2 and openpilot imports OK\")" && \
            ros2 pkg prefix openpilot_ros2_bridge && \
            { test -x /workspace/openpilot/openpilot/cereal/messaging/bridge || test -x /workspace/openpilot/cereal/messaging/bridge; } && \
            echo "cereal bridge found"'
```

Launch the containerized bridge from the host. No colcon build is needed when you run the launch file by path:

```bash
source /opt/ros/jazzy/setup.bash

ros2 launch /path/to/openpilot_ros2_bridge/launch/openpilot_bridge_docker.launch.py \
  host_openpilot_root:=/path/to/openpilot \
  comma_ip:=<comma_device_ip> \
  ros_domain_id:=0
```

For local development from source, build the image from this checkout instead of pulling it:

```bash
ros2 launch /path/to/openpilot_ros2_bridge/launch/openpilot_bridge_docker.launch.py \
  host_openpilot_root:=/path/to/openpilot \
  image_name:=openpilot_ros2_bridge:local \
  image_source:=build
```

The Docker launch file pulls (or builds) the image, starts the container with `--network host` and `--ipc host`, forwards `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION` and `CYCLONEDDS_URI`, and then runs the in-container `openpilot_bridge.launch.py`.
The container runs as your user. A root container can't share Fast DDS shared memory with your host nodes, so they would see topics but receive no data.
Inside the container, msgq uses its own `OPENPILOT_PREFIX`, so it never touches the queues of an openpilot running natively on the host.

For a quick runtime smoke test without a Comma device connection:

```bash
docker run --rm --network host --ipc host --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v /path/to/openpilot:/workspace/openpilot \
  ghcr.io/jkk-research/openpilot_ros2_bridge:latest \
  timeout 15 bash /workspace/ros2_ws/src/openpilot_ros2_bridge/docker/run_openpilot_bridge_container.sh \
    start_bridge:=false start_camera:=false
```

## Published container image

`.github/workflows/publish-docker.yml` publishes the Docker image to GHCR on pushes to `main`, version tags, and manual runs.
The image includes CycloneDDS and the tools `prepare_openpilot` needs, so no local image build is required.

To pull it by hand:

```bash
docker pull ghcr.io/jkk-research/openpilot_ros2_bridge:latest
```

[@tambetm](https://github.com/tambetm) Thanks for the original ROS1 version.
