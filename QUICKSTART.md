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

Build the package once (`colcon build --symlink-install --packages-select openpilot_ros2_bridge`). Then, in a shell that has your stack's ROS settings (`ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, `CYCLONEDDS_URI`), run:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch openpilot_ros2_bridge openpilot_bridge_docker.launch.py \
  comma_ip:=192.168.10.26 start_remote_bridge:=true \
  image_source:=cyclone image_name:=openpilot_ros2_bridge:cyclone \
  host_openpilot_root:=$HOME/openpilot prepare_openpilot:=true mount_source:=true \
  base_frame:=lexus3/base_link device_x:=1.6 device_y:=0 base_frame_height:=0.37
```

The launch file does the following, and does each step only when it's needed:

1. **Comma side:** starts `cereal/messaging/bridge` on the Comma over SSH, unless it's already running there. It stops again when you stop the launch.
2. **Image** (`image_source:=cyclone`): builds `openpilot_ros2_bridge:cyclone` from `docker/Dockerfile.cyclone` if that image doesn't exist yet. It's the published image plus CycloneDDS and build tools.
3. **openpilot checkout** (`prepare_openpilot:=true`): reads the Comma's openpilot repository and commit over SSH.
   - If `host_openpilot_root` is missing, it creates a checkout there.
   - If the checkout is on a different commit and has no local changes, it switches to the Comma's commit.
   - It builds only msgq and `cereal/messaging/bridge`, and only when the commit changed since the last build.
   - If SSH fails, it uses the checkout as it is.
4. **Driver:** starts the container. It runs on your `RMW_IMPLEMENTATION`, `ROS_DOMAIN_ID` and `CYCLONEDDS_URI`; the CycloneDDS profile is mounted automatically. With `mount_source:=true`, it runs this repository's scripts and launch files rather than the copies baked into the image.

Set these for your car:

- `base_frame` and `base_frame_height`: your base frame, and its height above the road in meters.
- `device_x` and `device_y`: the Comma's measured mount position in `base_frame`. The height comes from openpilot calibration.

It's ready when the log shows `OpenPilot prediction publisher is ready.` and `Calibration calibrated: ...`.

To stop it, press Ctrl+C. That stops the container and the bridge on the Comma.

Other useful arguments:

| Argument | What it does |
|---|---|
| `namespace:=openpilot` | Topic namespace. |
| `comma_openpilot_dir:=/data/openpilot` | Where openpilot lives on the Comma. |
| `remote_bridge_target:=comma@<ip>` | SSH user and host. Defaults to `comma@<comma_ip>`. |
| `cyclonedds_uri:=file:///path.xml` | Overrides `CYCLONEDDS_URI`. |
| `start_tf:=false` | Turns off the TF frames. |

## 3. Check it

Run this in a host shell with the same ROS settings:

```bash
ros2 topic list | grep /openpilot/
ros2 topic hz /openpilot/car_state_motion
ros2 run tf2_ros tf2_echo lexus3/base_link openpilot_device
```

Expected rates:

| Topics | Rate |
|---|---|
| modelV2-based topics and markers | ~20 Hz |
| `car_state_*`, `car_control_*` | ~100 Hz |
| `temporal_pose_*`, `lateral_plan_*` | silent on openpilot versions that no longer publish them |

The topics show up in `ros2 topic list`, but the driver's nodes won't appear in `ros2 node list` from a Humble host. That's expected: the container runs Jazzy, and Humble can't decode its node info.

In RViz or Foxglove, set the fixed frame to your `base_frame` and add the `/openpilot/*_markers` topics.
