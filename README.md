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

Driver monitoring and drivetrain topics are published by default (see [Drivetrain](#drivetrain)). To also stream the cameras, add `start_camera:=true cameras:=road,wide_road,driver` (see [Cameras](#cameras)).

## What the bridge publishes

The bridge now exposes:

- `modelV2` predictions, lane lines, road edges, leads, temporal pose, disengage predictions, and desired curvature
- `longitudinalPlan` as ROS 2 topics for speeds, accelerations, jerks, and planner state
- `lateralPlan` as ROS 2 topics for path points, curvature, solver state, and lane-change state
- `carState` and `carControl` summaries as ROS 2 topics
- RViz marker topics for plan, lane lines, road edges, and leads
- TF frames for openpilot's calibrated frame and the Comma device, driven by openpilot calibration
- driver monitoring model output (`driverStateV2`) and policy state (`driverMonitoringState`)
- drivetrain state from `carState`, `carOutput` and `carParams`, plus drivetrain CAN messages (engine RPM, pedal positions, brake force, gear and more) decoded with opendbc
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
- `bridge_topics:=modelV2,carControl,carState,longitudinalPlan,lateralPlan,liveCalibration,extrinsicsCalibration,driverStateV2,driverMonitoringState,carOutput,carParams,can`
- `namespace:=openpilot`
- `start_driver_monitoring:=false`
- `start_drivetrain:=false`
- `drivetrain_can_messages:=ENGINE_RPM,GEAR_PACKET`
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

## Drivetrain

The drivetrain node is on by default (`start_drivetrain:=true`).
It reads `carState`, `carOutput`, `carParams` and the raw `can` stream through the local bridge, so keep all four in `bridge_topics` if you override it.
If you turn the node off, you can drop `carOutput`, `carParams` and `can` from `bridge_topics`. Keep `carState`, because the `car_state_*` topics use it. The raw `can` stream is the largest of them: 100 batches per second, about 100 kB/s on a Lexus RX.

### From openpilot's car interface

These work on every car. All are `std_msgs/Float32MultiArray` except `drivetrain_car`.

| Topic | Shape | Contents |
| --- | --- | --- |
| `drivetrain_state` | 16 | gear shifter (0 unknown, 1 park, 2 drive, 3 neutral, 4 reverse, 5 sport, 6 low, 7 brake, 8 eco, 9 manumatic), speed (m/s), acceleration (m/s²), raw speed (m/s), standstill, gas pressed, brake pressed, regen braking, parking brake, brake hold active, cruise enabled, cruise available, fuel or battery level (0-1), charging, engine RPM, clutch pressed |
| `drivetrain_command` | 5 | what openpilot actually sent to the car, after the car's limits (`carOutput`): acceleration (m/s²), gas, brake, longitudinal control state (0 off, 1 pid, 2 stopping, 3 starting), speed (m/s). Gas and brake are in the car's own command units: 0-1 on Honda Nidec, raw CAN values on GM and Honda Bosch, m/s² on Ford, and 0 on most other cars, Toyota included |
| `drivetrain_params` | 11 | latched, from `carParams` (every 50 s): transmission type (0 unknown, 1 automatic, 2 manual, 3 direct drive, 4 CVT), mass (kg), wheelbase (m), center of mass to front axle (m), steer ratio, wheel speed factor, minimum enable speed (m/s), openpilot longitudinal control, PCM cruise, stop acceleration (m/s²), longitudinal actuator delay (s) |
| `drivetrain_car` | String | latched: brand, fingerprint, and the DBC, bus and messages used for CAN decoding |

Current openpilot versions no longer report engine RPM or the clutch in `carState`, so those two fields are NaN there.
The CAN topics below carry the engine RPM on most combustion and hybrid cars; the exceptions are Nissan, Hyundai/Kia CAN-FD, Chrysler CUSW and Volkswagen MQB/PQ.

### From the raw CAN bus

The node decodes the car's drivetrain messages with opendbc, using the DBC that opendbc assigns to the car's fingerprint.
It waits for the first `carParams` to know the car, which takes up to 50 s after start.
Each message is published on `drivetrain_can/<message name in lower case>`, for example `/openpilot/drivetrain_can/engine_rpm`.
The data holds the message's signals in DBC order and DBC units, and the layout label lists the signal names in the same order (`RPM,ENGINE_RUNNING`).
Frames with a bad checksum are dropped where opendbc checks the brand's checksum (Toyota, Honda, Hyundai CAN-FD, Volkswagen, Subaru global, Chrysler and Tesla among them), and counter errors drop frames only after several in a row.
GM, Hyundai/Kia CAN, Mazda, Nissan and Subaru pre-global frames are not checked, and their checksum and counter signals are published as data.
If the Comma reports a different car, the node switches to that car's DBC.
This needs openpilot 0.10.0 or newer, whose opendbc has the pure-Python CAN parser; with older versions the node logs a warning and publishes only the topics above.
On openpilot 0.11.0 and older, opendbc's `*_generated` DBC files only exist after a full openpilot build.

Default messages per opendbc brand. A brand's list covers all its DBC variants, and the node keeps the ones the car's DBC defines:

| Brand | Default messages |
| --- | --- |
| Toyota, Lexus | `ENGINE_RPM`, `GAS_PEDAL`, `GAS_PEDAL_HYBRID`, `BRAKE_MODULE`, `BRAKE`, `GEAR_PACKET`, `GEAR_PACKET_HYBRID`, `CLUTCH` |
| BYD | `DRIVE_STATE`, `PEDAL` |
| Chrysler | `ECM_1`, `ECM_5`, `ESP_8`, `Transmission_Status`, `GEAR`, `EPB_1`, `ACCEL_GAS`, `BRAKE_1` |
| Ford | `EngVehicleSpThrottle`, `EngBrakeData`, `ABS_BrkBst_Data`, `BrakeSnData_4`, `PowertrainData_10`, `VehicleOperatingModes`, `DesiredTorqBrk` |
| GM | `ECMEngineStatus`, `AcceleratorPedal2`, `ECMAcceleratorPos`, `ECMPRDNL2`, `EBCMRegenPaddle`, `BECMBatteryVoltageCurrent`, `BCMGeneralPlatformStatus` |
| Honda, Acura | `ENGINE_DATA`, `POWERTRAIN_DATA`, `GAS_PEDAL_2`, `VSA_STATUS`, `GEARBOX_AUTO`, `GEARBOX_CVT`, `EPB_STATUS`, `GEARBOX_ALT_2` |
| Hyundai, Kia, Genesis | `TCS13`, `E_EMS11`, `EMS12`, `EMS11`, `EMS_366`, `ESP12`, `ACCELERATOR`, `ACCELERATOR_ALT`, `GEAR_SHIFTER`, `BRAKE`, `LVR12`, `ELECT_GEAR`, `TCU12`, `CLU15`, `EMS20`, `FCEV_ACCELERATOR`, `GEAR_ALT`, `GEAR_ALT_2` |
| Mazda | `ENGINE_DATA`, `GEAR`, `BRAKE`, `EPB` |
| MG | `GW_HSC2_HCU_FrP00`, `EHBS_HSC2_FrP00`, `GW_HSC2_ECM_FrP04` |
| Nissan | `CRUISE_THROTTLE`, `GAS_PEDAL`, `BRAKE_PEDAL`, `GEARBOX` |
| PSA (Peugeot) | `Dyn_CMM`, `Dyn2_FRE`, `Dyn_EasyMove` |
| Rivian | `VDM_PropStatus`, `iBESP2`, `ESP_Status`, `ESP_Brake_Pressure`, `VDM_Torque_Front`, `VDM_Torque_Rear`, `ESP_EpbStatus`, `VDM_ModSw_DrvModPosSts_GW` |
| Subaru | `Throttle`, `Transmission`, `Brake_Pedal`, `Throttle_Hybrid` |
| Tesla | `DI_systemStatus`, `DI_torque`, `ESP_status`, `IBST_status`, `DI_state` |
| Volkswagen group | `Motor_14`, `Motor_20`, `ESP_05`, `Getriebe_11`, `Motor_51`, `ESC_50`, `Motor_EV_01`, `Motor_3`, `Motor_03`, `Getriebe_1` |

Toyota and Lexus defaults are checked against logs from a Lexus RX hybrid.
There, `ENGINE_RPM` arrives at 42 Hz, `GEAR_PACKET_HYBRID` at 61 Hz with `FDRVREAL` as the driving force at the wheels in N (negative under regen), and `BRAKE` at 80 Hz with `BRAKE_FORCE` as the friction brake force in N.
`ENGINE_RUNNING` stays 0 on that car, so use `RPM`.
Non-hybrids send `GAS_PEDAL` instead of `GAS_PEDAL_HYBRID` (accelerator 0-1). On most models it carries the accelerator in % and the engine torque in Nm; on the SecOC models (RAV4 Prime, Sienna, Yaris) it is a different message with `GAS_PEDAL_USER`.
`GEAR_PACKET` `GEAR` is 0 D, 1 S, 8 N, 16 R, 32 P; `GEAR_PACKET_HYBRID` `GEAR` is 0 P, 1 R, 2 N, 3 D, 4 B.

The other brands' defaults are checked against opendbc's DBC files and car code, not on real cars.
Frames are read from CAN bus 0, with opendbc's exceptions: Honda Bosch cars with a radar and Hyundai CAN-FD cars with LKA steering use bus 1 (from the `carParams` flags), and so do the Nissan Altima and Subaru hybrids.
Some cars split their drivetrain messages over two buses, Subaru hybrids for example; use `drivetrain_can_bus` and `drivetrain_can_messages` to pick the other ones.

A message that the car doesn't send never publishes.

Useful arguments:

- `drivetrain_can_messages:=ENGINE_RPM,GEAR_PACKET`: publish these DBC messages instead of the defaults.
- `drivetrain_can_dbc:=toyota_tnga_k_pt_generated`: use this opendbc DBC name (the `dbc=` value that `drivetrain_car` shows) instead of looking it up from the fingerprint. Decoding then starts without waiting for `carParams`, but without the car's flags: Honda Bosch cars with a radar, Hyundai CAN-FD cars with LKA steering and the Nissan Altima also need `drivetrain_can_bus:=1`.
- `drivetrain_can_bus:=0`: decode frames from this CAN bus instead of the brand's default.

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
