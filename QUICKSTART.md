# Quick start: running the driver from the vehicle PC

This is how to start the bridge on a vehicle PC whose ROS 2 stack runs on CycloneDDS, for example ROS 2 Humble with `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.
The driver runs in a ROS 2 Jazzy container. Topics appear under `/openpilot/...`, and TF frames are published under your `base_frame`.

The examples use the Comma at `192.168.10.26`. Change it if your Comma has a different address.

It works with openpilot 0.10 and newer, including 0.11.3 and current master, which no longer have openpilot's cereal bridge; see [Supported openpilot versions](README.md#supported-openpilot-versions).

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

The raw CAN frames on `/openpilot/can_frames` don't use this package's types but the standard `can_msgs/Frame` (see [Raw CAN frames](README.md#raw-can-frames)). To receive them on the host, install it once, with the CAN FD type next to it:

```bash
sudo apt install ros-humble-can-msgs ros-humble-ros2-socketcan-msgs
```

The launch file does the following, and does each step only when it's needed:

1. **Comma side:** copies [scripts/comma_bridge.py](scripts/comma_bridge.py) to the Comma and starts it over SSH with openpilot's own Python; it works with every openpilot version from 0.10 on. If another PC's launch already runs it there, or openpilot's own cereal bridge is running, it uses that one. It stops again when you stop the launch, and starts again after the Comma was out of reach.
2. **Image:** pulls `ghcr.io/jkk-research/openpilot_ros2_bridge:latest`, which already includes CycloneDDS. Nothing is built. When the PC is offline, it uses the image it pulled last time.
3. **openpilot checkout** (`prepare_openpilot:=true`): reads the Comma's openpilot repository and commit over SSH.
   - If `host_openpilot_root` is missing, it creates a checkout there.
   - If the checkout is on a different commit and has no local changes, it switches to the Comma's commit.
   - Nothing in it is built: the bridge reads its message definitions, service list and opendbc.
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
| `comma_bridge:=cereal` | Runs openpilot's own cereal bridge on the Comma instead of `comma_bridge.py`, on openpilot 0.11.2 and older (`auto` picks it where it exists). |
| `remote_bridge_target:=comma@<ip>` | SSH user and host. Defaults to `comma@<comma_ip>`. |
| `cyclonedds_uri:=file:///path.xml` | Overrides `CYCLONEDDS_URI`. |
| `start_tf:=false` | Turns off the TF frames. |
| `start_camera:=true cameras:=road,wide_road,driver` | Streams the cameras to `/openpilot/<camera>_camera/image_raw` and `.../compressed`. |
| `start_driver_monitoring:=false` | Turns off the driver monitoring topics. |
| `start_drivetrain:=false` | Turns off the decoded CAN messages on `/openpilot/can`. |
| `start_can_frames:=false` | Turns off the raw CAN frames on `/openpilot/can_frames` and `/openpilot/can_fd_frames`. |
| `start_radar:=true` | Publishes openpilot's radar data: the leads it tracks on `/openpilot/radarState` and every radar track on `/openpilot/radarTracks`. |
| `start_radar_can:=true` | Publishes the radar's own CAN messages and the blind spot monitor's messages, decoded with opendbc, on `/openpilot/radar_can`. |
| `publish_arrays:=true` | Also publishes the older `Float32MultiArray` topics (`car_state_*`, `driver_*`, `drivetrain_*` and more). |
| `start_control:=true` | Lets ROS 2 drive the car through openpilot's joystick mode, from `/openpilot/joystick_command`. See [Drive the car from ROS 2](#4-drive-the-car-from-ros-2-optional). |
| `mount_source:=true` | Runs this repository's scripts and launch files instead of the copies in the image, for testing local changes. Changes to the message definitions (`msg/`), `docker/Dockerfile` or `requirements.txt` need `image_name:=openpilot_ros2_bridge:local image_source:=build` instead. |

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
| `can_frames`, `can_fd_frames` | every raw CAN frame of all buses, a few thousand per second; `can_frames` leaves out CAN FD frames longer than 8 bytes |
| `radarState`, `radarTracks` (with `start_radar:=true`) | ~20 Hz |
| `radar_can` (with `start_radar_can:=true`) | every decoded radar frame, about 630 per second on a Lexus RX, starting after the first `carParams` (up to 50 s) |
| `*_camera/image_raw*` (with `start_camera:=true`) | ~20 Hz while onroad |
| `lateralPlan` | only on older openpilot versions that still publish it |

The topics show up in `ros2 topic list`, but the driver's nodes won't appear in `ros2 node list` from a Humble host. That's expected: the container runs Jazzy, and Humble can't decode its node info.

In RViz or Foxglove, set the fixed frame to your `base_frame` and add the `/openpilot/*_markers` topics.

## 4. Drive the car from ROS 2 (optional)

With `start_control:=true`, your nodes can send acceleration and steering commands to openpilot's joystick mode. openpilot applies them while the driver has engaged it, keeps its own and the panda's limits, and disengages when the brake is pressed.
In joystick mode, openpilot doesn't follow lead cars or keep the lane: your commands are all it does.

> **Warning:** this lets your software drive the car. Only use it with a driver ready to take over, somewhere safe such as a closed course.

1. Add `start_control:=true` to the launch command from step 2, with the car off. Within a few seconds the log shows `Joystick mode is on for the next drive`.
2. If the vehicle PC runs a firewall, let the Comma reach TCP port 57078:

   ```bash
   sudo ufw allow from 192.168.10.26 to any port 57078 proto tcp
   ```

3. Start the car and engage openpilot with the cruise control. The Comma's screen shows `Joystick Mode`.
4. Send commands at 10 Hz or faster, from your node or from a host shell with the package built:

   ```bash
   ros2 topic pub -r 20 /openpilot/joystick_command openpilot_ros2_bridge/msg/JoystickCommand "{accel: 0.5, steer: 0.0}"
   ```

   `accel` is in m/s² (-4 to 4). `steer` is -1 to 1, positive to the left; on the Lexus RX it is a share of the maximum steering torque.
   0.25 s after the last command, the car gets accel 0, which holds the speed, and steer 0, which on the Lexus means no steering torque.

`/openpilot/carControl` shows whether openpilot is engaged (`enabled`) and applying your commands (`lat_active`, `long_active`, `actuators`).
Stop the launch with the car off to switch openpilot back to normal driving. If you stop it during a drive, the car gets accel 0 and steer 0, and openpilot switches back when the drive ends.
More details are in [Controlling the car](README.md#controlling-the-car).
