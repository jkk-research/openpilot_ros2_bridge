# ROS 2 bridge for openpilot

ROS 2 bridge for Comma 3/X and Comma 4 devices running openpilot.

## Quick start

To run on a vehicle PC whose ROS 2 stack uses CycloneDDS, including SSH setup for the Comma, see [QUICKSTART.md](QUICKSTART.md).

### Option A: Docker (recommended)

Nothing to build to run it. The launch file runs straight from this checkout, and the published image already contains the bridge, its Python packages, and CycloneDDS.
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
- To use the typed messages in your own nodes on the host, build this package in your workspace for its message types; see [Typed messages](#typed-messages).

### Option B: Local install

Build openpilot natively ([instructions](https://github.com/commaai/openpilot/tree/master/tools#native-setup-on-ubuntu-2404-and-macos)), install the standard CAN message types (`sudo apt install ros-jazzy-can-msgs ros-jazzy-ros2-socketcan-msgs`) and create the bridge venv once with `./scripts/setup_bridge_venv.sh`. Then build this package in a ROS 2 Jazzy workspace and launch it:

```bash
cd /path/to/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select openpilot_ros2_bridge
source install/setup.bash
export OPENPILOT_ROS2_BRIDGE_VENV=/path/to/openpilot_ros2_bridge/.venv
ros2 launch openpilot_ros2_bridge openpilot_bridge.launch.py \
  openpilot_root:=/path/to/openpilot \
  comma_ip:=<comma_device_ip>
```

In both cases, the Comma device must also run its side of the bridge (`cd openpilot/cereal/messaging && ./bridge`). Alternatively, add `start_remote_bridge:=true remote_bridge_target:=comma@<comma_device_ip>` to start it over SSH. The sections below cover the details.

Everything is published as typed ROS 2 messages that mirror openpilot's own (see [Typed messages](#typed-messages)); add `publish_arrays:=true` to also get the older `Float32MultiArray` topics. To also stream the cameras, add `start_camera:=true cameras:=road,wide_road,driver` (see [Cameras](#cameras)).

## What the bridge publishes

- typed ROS 2 messages that mirror openpilot's own, one topic per openpilot service: `carState`, `carControl`, `carOutput`, `carParams`, `modelV2`, `longitudinalPlan`, `lateralPlan` (older openpilot), `driverStateV2`, `driverMonitoringState` and `extrinsicsCalibration` (see [Typed messages](#typed-messages))
- drivetrain CAN messages (engine RPM, pedal positions, brake force, gear and more), decoded with opendbc and published as typed `CanMessage`s (see [Drivetrain](#drivetrain))
- every raw CAN frame the Comma's panda receives or sends, undecoded, as standard `can_msgs/Frame` messages (see [Raw CAN frames](#raw-can-frames))
- optional radar data: openpilot's `radarState` and `radarTracks`, and the radar's own CAN messages decoded with opendbc (see [Radar](#radar))
- RViz marker topics for plan, lane lines, road edges, and leads
- TF frames for openpilot's calibrated frame and the Comma device, driven by openpilot calibration
- optional road, wide road, and driver camera streams as raw and JPEG-compressed images, each with its camera calibration
- optional, with `publish_arrays:=true`: the older `Float32MultiArray` topics for the model, plans, car state and control, driver monitoring and drivetrain

All topics are published under the `openpilot` namespace (for example `/openpilot/carState`). Change it with `namespace:=<name>`.

With `start_control:=true`, the bridge also takes acceleration and steering commands from ROS 2 and passes them to openpilot's joystick mode (see [Controlling the car](#controlling-the-car)).

## Typed messages

Each openpilot service the bridge reads is published as one ROS 2 message that mirrors openpilot's own struct, on a topic named after the service:

| Topic | Type (`openpilot_ros2_bridge/msg/...`) | Rate |
| --- | --- | --- |
| `carState` | `CarState` | 100 Hz |
| `carControl` | `CarControl` | 100 Hz |
| `carOutput` | `CarOutput` | 100 Hz |
| `carParams` | `CarParams` | every 50 s, latched |
| `modelV2` | `ModelV2` | 20 Hz |
| `longitudinalPlan` | `LongitudinalPlan` | 20 Hz |
| `lateralPlan` | `LateralPlan` | 20 Hz, older openpilot only |
| `driverStateV2` | `DriverStateV2` | 20 Hz |
| `driverMonitoringState` | `DriverMonitoringState` | 20 Hz; needs openpilot 0.11.1 or newer |
| `extrinsicsCalibration` | `ExtrinsicsCalibration` | 4 Hz; the topic is `liveCalibration` on older openpilot |
| `radarState` | `RadarState` | 20 Hz, with `start_radar:=true` |
| `radarTracks` | `RadarData` | 20 Hz, with `start_radar:=true`; the topic is `liveTracks` on older openpilot |
| `can` | `CanMessage` | one per decoded drivetrain CAN frame |
| `radar_can` | `CanMessage` | one per decoded radar CAN frame, with `start_radar_can:=true` |

Each topic needs its openpilot service in `bridge_topics`; the default list has them all.

How they map to openpilot:

- Field names are openpilot's in snake_case, in openpilot's order: `carState.vEgo` is `v_ego` and `modelV2.leadsV3` is `leads_v3`. Nested structs are their own types, such as `CruiseState`, `XYZTData` and `LeadDataV3`.
- Enums are `uint8` fields with named constants, for example `CarState.GEAR_SHIFTER_DRIVE`.
- Every top-level message starts with `header` (stamped when received; `modelV2` uses the `openpilot` frame), `log_mono_time` (openpilot's timestamp, ns) and `valid`.
- Deprecated fields, unions and raw data blobs are left out. A float field the running openpilot version doesn't have reads `NaN`, also inside a nested struct it doesn't have; other missing fields keep their default.
- A struct the running openpilot has only in an older layout isn't published, and the log says so. That is `driverMonitoringState` before openpilot 0.11.1; `publish_arrays:=true` publishes that one as `driver_monitoring_legacy_state`.
- `CanMessage` holds one decoded CAN frame: the DBC, bus, address and message name, and the signal names with their values.

Show any definition with `ros2 interface show openpilot_ros2_bridge/msg/CarState`, or read it in [msg/](msg/).

To use the messages in your own nodes, build this package in your workspace (`colcon build --packages-select openpilot_ros2_bridge`). That also holds for a host that only runs the Docker version.
It builds on ROS 2 Humble too, and a Humble node receives the messages from the Jazzy container over CycloneDDS and over Fast DDS. On Fast DDS, Humble prints a harmless `sequence size exceeds remaining buffer` warning a few times.

Your nodes and the bridge must be built with the same message definitions: with different ones, your nodes read wrong values, without any error.
So rebuild the package after every `git pull`. The Docker launch pulls the `latest` image, which is built from `main`, so keep your checkout on `main`. For a release, set `image_name:=ghcr.io/jkk-research/openpilot_ros2_bridge:<tag>` to the release's `v*` tag; for any other checkout, build the image from it (`image_name:=openpilot_ros2_bridge:local image_source:=build`).
The Docker launch warns when they differ. It checks the build your shell would use, or else the one in the colcon workspace that holds this checkout.

The definitions are generated from openpilot's schema. After updating openpilot, regenerate them with a Python that has pycapnp, such as the bridge venv, and rebuild:

```bash
.venv/bin/python3 scripts/generate_msgs.py --openpilot /path/to/openpilot
```

The Docker version then needs an image built from your checkout as well (`image_name:=openpilot_ros2_bridge:local image_source:=build`), because the image holds its own build of the definitions.

## Prerequisites

1. Install ROS 2 Jazzy.
2. Have an openpilot checkout available locally.
3. Build the openpilot messaging bridge in that checkout by following the official openpilot setup instructions: https://github.com/commaai/openpilot/tree/master/tools#native-setup-on-ubuntu-2404-and-macos
4. Install the standard CAN message types that the raw CAN frames use:

   ```bash
   sudo apt install ros-jazzy-can-msgs ros-jazzy-ros2-socketcan-msgs
   ```

5. Create the bridge virtual environment:

   ```bash
   cd /path/to/openpilot_ros2_bridge
   ./scripts/setup_bridge_venv.sh
   ```

6. Build this repository in a ROS 2 workspace:

   ```bash
   cd /path/to/ros2_ws
   source /opt/ros/jazzy/setup.bash
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
- `publish_arrays:=true`
- `start_driver_monitoring:=false`
- `start_drivetrain:=false`
- `drivetrain_can_messages:=ENGINE_RPM,GEAR_PACKET`
- `start_can_frames:=false` (see [Raw CAN frames](#raw-can-frames))
- `start_radar:=true`, `start_radar_can:=true` (see [Radar](#radar))
- `start_camera:=true`
- `cameras:=road,wide_road,driver`
- `camera_ip:=<comma_device_ip>` (defaults to `comma_ip`)
- `camera_scale:=0.5`
- `camera_jpeg_quality:=80`
- `start_control:=true` (see [Controlling the car](#controlling-the-car))
- `control_timeout:=0.25`
- `control_host_ip:=<pc_ip>`
- `comma_openpilot_dir:=/data/openpilot`

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

After launch, check that the typed topics are active:

```bash
ros2 topic list | grep /openpilot/
ros2 topic hz /openpilot/carState
ros2 topic echo /openpilot/modelV2 --once
```

`ros2 topic hz` and `ros2 topic echo` need the message types, so on a host that runs the Docker version, build the package first (see [Typed messages](#typed-messages)).

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
- `/openpilot/<camera>_camera/camera_info` (`sensor_msgs/CameraInfo`, the camera's calibration; see [Camera calibration](#camera-calibration))

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

### Camera calibration

Each camera node also publishes `camera_info` with every image: the same stamp, the same `frame_id` (`openpilot_<camera>_camera`), and the size of the published image, so `camera_scale` is taken into account.
That is what `image_proc`, RViz's Camera display and other `image_transport` subscribers pair with the images.

The intrinsics are openpilot's own, the ones its driving model and its UI use (`DEVICE_CAMERAS` in openpilot's `common/transformations/camera.py`):

| Image sensor | Full resolution stream | Road camera focal length | Wide road and driver camera focal length |
| --- | --- | --- | --- |
| AR0231, OX03C10 | 1928x1208 | 2648 px | 567 px |
| OS04C10 | 1344x760, binned from 2688x1520 | 1141.5 px | 425.25 px |

Each node logs which sensor's intrinsics it uses. The OS04C10 values hold from openpilot 0.9.8 on; before that, openpilot ran that sensor in other modes, so its full resolution streams get no `camera_info` there and `q_road`'s is off.

- `k` and `p` hold that focal length and the image center as the principal point, scaled from the full frame to the published size. `r` is the identity.
- openpilot has no distortion model and treats every camera as a pinhole camera, so `distortion_model` is `plumb_bob` with five zeros in `d`.
  That suits the road camera. The wide road and driver cameras have fisheye lenses, so for them the intrinsics only hold near the image center; openpilot's own source notes that their focal length is probably off.
  For accurate work with those two, calibrate them yourself, for example with `ros2 run camera_calibration cameracalibrator --no-service-check` on their `image_raw`, and save the result. The node can't store a calibration and keeps publishing openpilot's intrinsics on `camera_info`, so publish yours on another topic.
- The road, wide road and driver nodes tell the sensor from the size of their full resolution frames.
  `q_road` is a 526x330 copy of the road camera whose size doesn't say which sensor took it, so that node reads the road camera's sensor from openpilot's `narrowRoadCameraState` (`roadCameraState` on older openpilot), through the same bridge connection. Until that arrives, or with a frame size or sensor it doesn't know, a node publishes no `camera_info` and logs why.
- The `openpilot_<camera>_camera` frames are not in TF, so place them yourself if you need the images in 3D, for example in RViz.
  They are optical frames, as `CameraInfo` expects (x right, y down, z forward), so they need a rotation from the forward-right-down `openpilot_device` frame. For the road camera, which openpilot puts at the device's origin, that is openpilot's own `device_frame_from_view_frame`:
  `ros2 run tf2_ros static_transform_publisher --frame-id openpilot_device --child-frame-id openpilot_road_camera --qx 0.5 --qy 0.5 --qz 0.5 --qw 0.5`. The driver camera faces the cabin and needs its own rotation.

## Driver monitoring

Driver monitoring is on by default (`start_driver_monitoring:=true`): the typed `driverStateV2` and `driverMonitoringState` topics (see [Typed messages](#typed-messages)).
They come through the local bridge, so keep both in `bridge_topics` if you override it.
With `publish_arrays:=true`, the driver monitoring node also publishes them as these `std_msgs/Float32MultiArray` topics:

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
It decodes the car's drivetrain CAN messages and publishes every frame as a typed `CanMessage` on `/openpilot/can` (see below).
The car's state, what openpilot sent to it and the car's parameters are in the typed `carState`, `carOutput` and `carParams` topics.
The node reads `carParams` and the raw `can` stream through the local bridge, so keep both in `bridge_topics` if you override it.
If you turn the node off, and the raw CAN frames (`start_can_frames:=false`, see [Raw CAN frames](#raw-can-frames)) and `start_radar_can` are off too, you can drop `can` from `bridge_topics`: it is the largest stream, 100 batches per second and about 100 kB/s on a Lexus RX.

### Array topics

With `publish_arrays:=true`, the node also publishes these. All are `std_msgs/Float32MultiArray`; `drivetrain_car` is a `std_msgs/String` that is always published.

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
Every decoded frame is published as a `CanMessage` on `/openpilot/can`: its DBC, bus, address and message name, with every data signal by name, in DBC order and DBC units.
With `publish_arrays:=true`, each message is also published on `drivetrain_can/<message name in lower case>`, for example `/openpilot/drivetrain_can/engine_rpm`, with the signal names in the layout label (`RPM,ENGINE_RUNNING`).
Frames with a bad checksum are dropped where opendbc checks the brand's checksum (Toyota, Honda, Hyundai CAN-FD, Volkswagen, Subaru global, Chrysler and Tesla among them), and counter errors drop frames only after several in a row.
GM, Hyundai/Kia CAN, Mazda, Nissan and Subaru pre-global frames are not checked, and their checksum and counter signals are published as data.
If the Comma reports a different car, the node switches to that car's DBC.
This needs openpilot 0.10.0 or newer, whose opendbc has the pure-Python CAN parser; with older versions the node logs a warning and decodes nothing.
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
There, `ENGINE_RPM` arrives at 42 Hz, `GEAR_PACKET_HYBRID` at 61 Hz with `FDRVREAL` as the driving force at the wheels in N (negative under coasting regen), and `BRAKE` at 80 Hz with `BRAKE_FORCE` as the braking force in N.
The DBC calls `BRAKE_FORCE` friction only, but on that car it carries all pedal braking, regen included, so `FDRVREAL` minus `BRAKE_FORCE` is the total force at the wheels while moving; at standstill `BRAKE_FORCE` shows the brake hold instead.
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

## Raw CAN frames

The raw CAN frame node is on by default (`start_can_frames:=true`).
It publishes every frame of openpilot's raw `can` stream, undecoded, as a standard [`can_msgs/Frame`](https://github.com/ros-industrial/ros_canopen/blob/dashing-devel/can_msgs/msg/Frame.msg) on `/openpilot/can_frames`: the frames the Comma's panda receives, and the ones it sends.
That is one message per frame, from all buses in one stream, in the order openpilot read them.
The node reads the `can` stream through the local bridge, so keep `can` in `bridge_topics` if you override it.
For decoded signals instead, see `can` under [Drivetrain](#drivetrain) and `radar_can` under [Radar](#radar).

| Field | Contents |
| --- | --- |
| `header.stamp` | When the frame's batch arrived. openpilot sends the frames in batches, 100 per second, without a time per frame, so all frames of a batch get the same stamp. |
| `header.frame_id` | The CAN bus, as openpilot numbers it. `can0`, `can1` and `can2` are frames the panda received on that bus. `can0_sent` to `can2_sent` are frames the panda sent on that bus: openpilot's own, and the ones it passes on between bus 0 and bus 2 (see below). The panda reports them when it queues them for sending, not once they are on the bus. `can0_blocked` to `can2_blocked` are frames from openpilot that the panda's safety checks blocked, so they never reached the bus. |
| `id` | The CAN ID. |
| `is_extended` | True for IDs from 0x800 up. openpilot doesn't pass on whether a frame used an extended ID, so a 29-bit ID below 0x800 reads as a standard one; openpilot itself sends frames the same way. |
| `dlc` | The number of data bytes, 0 to 8. |
| `data` | The data bytes, padded with zeros to 8. |
| `is_rtr`, `is_error` | Always false. openpilot doesn't say whether a frame was a remote frame, so one shows up as a data frame whose bytes mean nothing, and error frames don't reach it. |

Which bus carries what depends on the car and its harness.
Usually the harness sits between the car and its own camera: bus 0 is the car side, bus 2 the camera side, and the panda passes the frames on between the two, except the ones openpilot sends in their place.
So `can0` and `can0_sent` together are everything on the car side, and a frame passed on shows up twice: as `can2` when the camera sends it, and as `can0_sent` when the panda passes it to the car.
On the Lexus RX, the radar is on bus 1 (see [Radar](#radar)).
On openpilot 0.10.3 and older with a second panda, such as a red panda on a comma three, that panda's buses are numbered from 4: `can4` to `can6`, with matching `_sent` and `_blocked` names.

CAN FD frames longer than 8 bytes, which Hyundai, Kia and Genesis CAN-FD cars among others send, don't fit a `can_msgs/Frame` and are left out of `can_frames`.
For CAN FD cars, `/openpilot/can_fd_frames` carries every frame, of any length, as a [`ros2_socketcan_msgs/FdFrame`](https://github.com/autowarefoundation/ros2_socketcan/blob/main/ros2_socketcan_msgs/msg/FdFrame.msg), the type ros2_socketcan uses for CAN FD: the same `header`, `id` and `is_extended`, with `len` and `len` bytes of `data`.
It is the complete stream in order, so on a CAN FD car, subscribe to it instead of `can_frames`.

Every frame is its own message, a few thousand per second (about 4,000 on a Lexus RX, judging by the size of its `can` stream), so the node needs a noticeable share of a CPU core while something subscribes to one of its topics. While nothing does, it does no work.
Each batch arrives as a burst of dozens of messages, so a subscriber needs a deep queue, about 1000, or it loses frames without notice; `ros2 topic echo` keeps only 5 by default. `ros2 bag record` keeps up.
When pandad loses frames reading them from the panda, the node logs a warning; those frames are missing from the topics too.

To receive the frames, a host needs the two message packages, which ROS 2 Humble and Jazzy have; the Docker image already includes them:

```bash
sudo apt install ros-$ROS_DISTRO-can-msgs ros-$ROS_DISTRO-ros2-socketcan-msgs
ros2 topic echo --qos-depth 1000 /openpilot/can_frames
ros2 bag record /openpilot/can_frames
```

## Radar

Two options publish the car's radar, both off by default.

`start_radar:=true` publishes openpilot's radar services as typed messages:

| Topic | Type | Rate | Contents |
| --- | --- | --- | --- |
| `radarState` | `RadarState` | 20 Hz | The two leads openpilot tracks, `lead_one` and `lead_two`: distance from the front bumper (`d_rel`), lateral offset (`y_rel`, positive to the left), relative and absolute speed, filtered lead speed and acceleration, and whether the radar sees the lead (`radar`, `radar_track_id`) or only the camera model (`model_prob`). Also the radar's error flags. |
| `radarTracks` | `RadarData` | 20 Hz | Every track the car's radar reports, as openpilot reads it: `track_id`, `d_rel`, `y_rel` and `v_rel` per point, and the error flags. On openpilot before August 2026 the topic is `liveTracks`. Needs openpilot 0.10.0 or newer. |

The launch adds both services to the bridged ones itself, also when you set `bridge_topics`.
On openpilot 0.9.8 and older, `radarState.radar_errors` had another layout and keeps its default.

`start_radar_can:=true` decodes the radar's own CAN messages with opendbc and publishes every frame as a `CanMessage` on `/openpilot/radar_can`, the same way the drivetrain node does on `can`.
By default it decodes the messages openpilot itself reads from the car's radar: the DBC, bus and messages of opendbc's radar interface for the fingerprint in `carParams`. So it starts after the first `carParams`, up to 50 s after launch.
Values are in the DBC's units and signs, which can differ from openpilot's. On the Lexus RX, `LAT_DIST` is positive to the right, while `y_rel` in `radarTracks` is positive to the left.
Like the drivetrain decoding, this needs openpilot 0.10.0 or newer.

On a Lexus RX (TSS2), the radar is on CAN bus 1 and uses the `toyota_tss2_adas` DBC:

- `TRACK_A_0` to `TRACK_A_15`: one per radar track, with `LONG_DIST` and `LAT_DIST` (m), `REL_SPEED` (m/s), `VALID` and `NEW_TRACK`.
- `TRACK_B_0` to `TRACK_B_15`: the same tracks' `REL_ACCEL` and `SCORE`.
- `STATUS_MSG`: `RADAR_STATUS`, `INVALID_COUNTER` and `RADAR_PRE_FAULT`.

Each track message arrives at 20 Hz and `STATUS_MSG` at 10 Hz, about 630 frames per second in all.
Cars where openpilot doesn't read the radar (`carParams.radar_unavailable`) publish no front radar frames on `radar_can` unless you set `radar_can_dbc`.

Useful arguments:

- `radar_can_dbc:=toyota_tss2_adas`: decode with this opendbc DBC instead of the car's radar DBC. Decoding then starts without waiting for `carParams`, on bus 1 unless `radar_can_bus` says otherwise, and with every message in the DBC unless `radar_can_messages` lists some.
- `radar_can_bus:=1`: decode frames from this CAN bus.
- `radar_can_messages:=TRACK_A_0,TRACK_B_0`: decode only these DBC messages.
- `radar_can_blind_spot:=false`: leave out the blind spot messages (see [Rear radars and blind spot monitoring](#rear-radars-and-blind-spot-monitoring)).

### Rear radars and blind spot monitoring

On the Lexus RX, the rear corner radars don't send their tracks to the CAN buses the Comma sees. What reaches them is the blind spot monitor's verdict per side. For other brands, too, opendbc only describes such per-side messages.
`start_radar_can:=true` publishes those messages on `radar_can` too: the ones openpilot reads for `carState.left_blindspot` and `right_blindspot`, from the car's own DBCs, on whichever of buses 0 to 2 they arrive:

| Brand | Messages |
| --- | --- |
| Toyota, Lexus | `BSM` |
| Hyundai, Kia, Genesis | `LCA11`, `ADAS_CMD_50_50ms` (CAN FD) |
| GM | `BCMBlindSpotMonitor` |
| Ford | `Side_Detect_L_Stat`, `Side_Detect_R_Stat` |
| Chrysler | `BSM_1`, `BSM_LEFT`, `BSM_RIGHT` |
| Subaru | `BSD_RCTA` (also rear cross traffic) |
| Volkswagen group | `SWA_01`, `SWA_1`, `MEB_Side_Assist_01` |
| Honda, Acura | `BSM_STATUS_LEFT`, `BSM_STATUS_RIGHT` (only with B-CAN forwarded to the Comma) |
| Mazda | `BSM` |
| Tesla | `DAS_status` |
| BYD | `BSD_RADAR` |

They're decoded whenever the car sends them, also when openpilot itself doesn't use them, so they can show more than `carState`.
On the Lexus RX, for example, openpilot doesn't turn on blind spot monitoring, so `left_blindspot` and `right_blindspot` always read false. A likely reason is that `BSM` is too slow for openpilot's first second of car detection.
There, `BSM` arrives on bus 0 about once per second, with `L_ADJACENT`, `L_APPROACHING`, `R_ADJACENT` and `R_APPROACHING` (a vehicle next to the car or closing in, per side) and `ADJACENT_ENABLED` and `APPROACHING_ENABLED` (whether the driver has each warning switched on).
The logs this was checked against had no vehicle in the blind spot, so the warning bits were never seen set.
Distances, speeds or tracks from the rear radars aren't on the bus. The radar bus also carries 28 messages that opendbc doesn't describe; what they hold is unknown.
Turn the blind spot messages off with `radar_can_blind_spot:=false`.

## Controlling the car

With `start_control:=true`, your ROS 2 nodes can drive the car through openpilot's joystick mode. They send acceleration and steering commands, and openpilot passes them to the car in place of its own plan.
openpilot only acts on them while the driver has engaged it. Its own limits and the panda's safety checks stay in place, and pressing the brake disengages as usual.
In joystick mode openpilot doesn't plan anything itself: it doesn't follow a lead car, stop for obstacles or keep the lane. Your commands are all it does.

> **Warning:** this lets your software drive a real car. Only use it with a driver ready to take over, somewhere safe such as a closed course.

It needs openpilot 0.9.5 or newer, and is tested with the `joystickd` of current openpilot (0.9.8 and newer).

Add it to either launch, together with the remote bridge:

```bash
ros2 launch /path/to/openpilot_ros2_bridge/launch/openpilot_bridge_docker.launch.py \
  host_openpilot_root:=/path/to/openpilot comma_ip:=<comma_device_ip> \
  start_remote_bridge:=true start_control:=true
```

Then start the car, engage openpilot with the cruise control, and send commands:

```bash
ros2 topic pub -r 20 /openpilot/joystick_command openpilot_ros2_bridge/msg/JoystickCommand "{accel: 0.5, steer: 0.0}"
```

The Comma's screen shows `Joystick Mode` with the gas and steering it applies.

`JoystickCommand` has two fields:

| Field | Meaning |
| --- | --- |
| `accel` | m/s². Positive speeds up, negative brakes; limited to -4 to 4. 0 holds the current speed, and below 0.3 m/s it stops the car and holds it. Only cars where openpilot controls the speed (openpilot longitudinal control, `carParams.openpilot_longitudinal_control`) follow it. On other cars, the car's own cruise control keeps the speed. |
| `steer` | -1 to 1, positive steers left. On cars steered by torque, such as the Lexus RX, it is the share of the car's maximum steering torque, and 0 means no torque. On cars steered by angle or curvature, it is the share of the steering angle that gives 3 m/s² of lateral acceleration at the current speed, and 0 means straight ahead. |

On cars steered by torque, `steer` is a torque, not a steering angle. To follow a path, close the loop in your node, for example on `carState.steering_angle_deg`.

Send commands at 10 Hz or faster. The newest one is used for `control_timeout` after it arrives: 0.25 s by default, at most 0.5 s. After that, the car gets accel 0 and steer 0 until the next command, and the log says so. openpilot does the same on the Comma when nothing arrives from the PC for 0.2 s.
So when commands stop, the car holds its speed and, depending on the car, stops steering or steers straight ahead. The driver has to take over.

`/openpilot/carControl` shows what is in effect:

- `enabled` says whether openpilot is engaged.
- `lat_active` and `long_active` say whether the steering and acceleration commands are applied.
- `actuators.accel` and `actuators.torque` are the commands openpilot made from yours.

`/openpilot/carOutput` shows what reached the car after openpilot's limits.

How it works:

- `openpilot_joystick_sender` subscribes to `joystick_command` and sends openpilot's `testJoystick` message to the Comma at 100 Hz, also when there's no command, over ZMQ on TCP port 57078 of this PC. That is the port openpilot's cereal bridge uses for this message.
- On the Comma, a small relay, [scripts/comma_joystick_relay.py](scripts/comma_joystick_relay.py), runs with openpilot's own Python. The launch copies it to the Comma and starts it over SSH, like the remote bridge, and starts it again when the connection drops. Nothing has to be installed on the Comma.
- The relay checks each message and passes it to openpilot's `joystickd`, which runs instead of `controlsd` in joystick mode and turns it into `carControl`. Messages that arrive more than 0.1 s late, such as a backlog after a Wi-Fi stall, are dropped.
- In joystick mode, openpilot also starts its own `joystick_control`, for a gamepad plugged into the Comma. Without a gamepad, it sends zeros that would replace the PC's commands, so the relay takes the message back from it.
- The PC side runs in the bridge's own image, so there is no separate image for it.

Joystick mode:

- The relay turns joystick mode on for the next drive once the car has been off for 3 s and the PC's sender is running. If nothing arrives from the PC for 5 s while the car is off, it turns joystick mode off again until the PC is back.
- It only switches joystick mode while the car is off, because openpilot would swap its controls mid-drive. If the car was already on when you launched, that drive uses openpilot's normal controls and the relay says so. Turn the car off and on again to use joystick mode.
- openpilot turns joystick mode off at the end of every drive. While the launch runs, the relay turns it on again for the next drive.
- If you turn joystick mode off on the Comma (Developer settings), it stays off until the launch is restarted. It also stays off while one of openpilot's maneuver modes is on.
- Stopping the launch while the car is off turns joystick mode off, so the next drive uses openpilot normally. Stopping during a drive leaves joystick mode on until that drive ends, with accel 0 and steer 0 for the rest of it.

Network:

- The Comma connects to TCP port 57078 on this PC. If the PC runs a firewall, allow that port, for example `sudo ufw allow from <comma_device_ip> to any port 57078 proto tcp`.
- The relay reaches the PC at the address its SSH session comes from. If the Comma reaches the PC at a different address, set `control_host_ip:=<pc_ip>`.
- Without `start_remote_bridge:=true`, start the Comma side yourself: `scripts/start_comma_joystick_relay.sh comma@<comma_device_ip>`.

## Virtual environment / container workflow

The `docker/Dockerfile` provides a ROS 2 Jazzy environment for the bridge.
It installs the standard CAN message types, copies the local repository into `/workspace/ros2_ws/src/openpilot_ros2_bridge`, creates a dedicated bridge `.venv`, installs the required Python packages there, builds the ROS 2 package with `colcon`, and expects only an openpilot source checkout to be bind-mounted at runtime.

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
The image includes CycloneDDS, the standard CAN message types and the tools `prepare_openpilot` needs, so no local image build is required.

To pull it by hand:

```bash
docker pull ghcr.io/jkk-research/openpilot_ros2_bridge:latest
```

[@tambetm](https://github.com/tambetm) Thanks for the original ROS1 version.
