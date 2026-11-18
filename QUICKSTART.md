# Quick start: running the driver from the vehicle PC

This is how to start the bridge on a vehicle PC whose ROS 2 stack runs on CycloneDDS, for example ROS 2 Humble with `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.
The driver runs in a ROS 2 Jazzy container. Topics appear under `/openpilot/...`, and TF frames are published under your `base_frame`.

The examples use the Comma at `192.168.10.26`. Change it if your Comma has a different address.

## 1. SSH access to the Comma (one-time)

The Comma only accepts SSH keys that it downloads from a GitHub account.

1. On the Comma, open **Settings → Developer**, turn on **Enable SSH**, and set **SSH Keys** to a GitHub username.
   That account's public keys must include this PC's key (`~/.ssh/id_rsa.pub` or `~/.ssh/id_ed25519.pub`).
   To check which keys an account publishes: `curl -s https://github.com/<github-user>.keys`
2. Connect once by hand, so the Comma's host key gets accepted:

   ```bash
   ssh comma@192.168.10.26 'echo ok'
   ```

If SSH fails, here is what each error means:

| Error | Fix |
|---|---|
| `Permission denied (publickey)` | The GitHub account set on the Comma doesn't hold this PC's key. Change the account or add the key on GitHub. |
| `REMOTE HOST IDENTIFICATION HAS CHANGED` | Expected after the Comma is reflashed, or when the IP used to belong to another device. Once you're sure it's your Comma, run `ssh-keygen -R 192.168.10.26` and connect again. |

## 2. Start the driver

Nothing needs to be built: the launch file runs straight from your clone of this repository (here `~/ros2_ws/src/openpilot_ros2_bridge`), and the image comes prebuilt from GHCR.
In a shell that has your stack's ROS settings (`ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, `CYCLONEDDS_URI`), run:

```bash
source /opt/ros/humble/setup.bash
ros2 launch ~/ros2_ws/src/openpilot_ros2_bridge/launch/openpilot_bridge_docker.launch.py \
  comma_ip:=192.168.10.26 start_remote_bridge:=true \
  host_openpilot_root:=$HOME/openpilot prepare_openpilot:=true \
  base_frame:=lexus3/base_link device_x:=1.6 device_y:=0 base_frame_height:=0.37
```

The topics are typed messages that mirror openpilot's own (`openpilot_ros2_bridge/msg/CarState` and so on; see [Typed messages](README.md#typed-messages)).
Your nodes, and `ros2 topic echo` or `hz`, need these message types, so build the package in your workspace. It builds on Humble:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select openpilot_ros2_bridge
source install/setup.bash
```

Rebuild it after every `git pull` of this repository. The launch pulls the newest image, which follows `main`, and with different message definitions on the two sides your nodes read wrong values without any error. The launch compares your build in `~/ros2_ws` (or the one your shell has sourced) with the image and prints a warning when they differ.

After that, `ros2 launch openpilot_ros2_bridge openpilot_bridge_docker.launch.py ...` with the same arguments works too.

The launch file does the following, and does each step only when it's needed:

1. **Comma side:** starts `cereal/messaging/bridge` on the Comma over SSH, unless it's already running there. It stops again when you stop the launch.
2. **Image:** pulls `ghcr.io/jkk-research/openpilot_ros2_bridge:latest`, which already includes CycloneDDS and the build tools. Nothing is built. When the PC is offline, it uses the image it pulled last time.
3. **openpilot checkout** (`prepare_openpilot:=true`): reads the Comma's openpilot repository and commit over SSH.
   - If `host_openpilot_root` is missing, it creates a checkout there.
   - If the checkout is on a different commit and has no local changes, it switches to the Comma's commit.
   - It builds only msgq and `cereal/messaging/bridge`, inside the published image, and only when the commit changed since the last build.
   - If SSH fails, it uses the checkout as it is.
4. **Driver:** starts the container. It runs on your `RMW_IMPLEMENTATION`, `ROS_DOMAIN_ID` and `CYCLONEDDS_URI`; the CycloneDDS profile is mounted automatically.

Set these for your car:

- `base_frame` and `base_frame_height`: your base frame, and its height above the road in meters.
- `device_x` and `device_y`: the Comma's measured mount position in `base_frame`. The height comes from openpilot calibration.

It's ready when the log shows `OpenPilot typed publisher is ready.` and `Calibration calibrated: ...`.

To stop it, press Ctrl+C. That stops the container and the bridge on the Comma.

Other useful arguments:

| Argument | What it does |
|---|---|
| `namespace:=openpilot` | Topic namespace. |
| `comma_openpilot_dir:=/data/openpilot` | Where openpilot lives on the Comma. |
| `remote_bridge_target:=comma@<ip>` | SSH user and host. Defaults to `comma@<comma_ip>`. |
| `cyclonedds_uri:=file:///path.xml` | Overrides `CYCLONEDDS_URI`. |
| `start_tf:=false` | Turns off the TF frames. |
| `start_camera:=true cameras:=road,wide_road,driver` | Streams the cameras to `/openpilot/<camera>_camera/image_raw` and `.../compressed`. |
| `start_driver_monitoring:=false` | Turns off the driver monitoring topics. |
| `start_drivetrain:=false` | Turns off the decoded CAN messages on `/openpilot/can`. |
| `publish_arrays:=true` | Also publishes the older `Float32MultiArray` topics (`car_state_*`, `driver_*`, `drivetrain_*` and more). |
| `mount_source:=true` | Runs this repository's scripts and launch files instead of the copies in the image, for testing local changes. Changes to the message definitions (`msg/`) need `image_name:=openpilot_ros2_bridge:local image_source:=build` instead. |

## 3. Check it

Run this in a host shell with the same ROS settings:

```bash
ros2 topic list | grep /openpilot/
ros2 topic hz /openpilot/carState
ros2 run tf2_ros tf2_echo lexus3/base_link openpilot_device
```

`ros2 topic hz /openpilot/carState` needs the package built (see above); `ros2 topic hz /openpilot/plan_markers` works without it.

Expected rates:

| Topics | Rate |
|---|---|
| `carState`, `carControl`, `carOutput` | ~100 Hz |
| `modelV2`, `longitudinalPlan`, `driverStateV2`, `driverMonitoringState` and the markers | ~20 Hz; `driverMonitoringState` needs openpilot 0.11.1 or newer |
| `extrinsicsCalibration` (`liveCalibration` on older openpilot) | ~4 Hz |
| `carParams` | every 50 s, latched |
| `can` | every decoded drivetrain frame, about 280 per second on a Lexus RX, starting after the first `carParams` (up to 50 s) |
| `*_camera/image_raw*` (with `start_camera:=true`) | ~20 Hz while onroad |
| `lateralPlan` | only on older openpilot versions that still publish it |

The topics show up in `ros2 topic list`, but the driver's nodes won't appear in `ros2 node list` from a Humble host. That's expected: the container runs Jazzy, and Humble can't decode its node info.

In RViz or Foxglove, set the fixed frame to your `base_frame` and add the `/openpilot/*_markers` topics.
