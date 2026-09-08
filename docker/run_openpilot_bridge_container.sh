#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"
OPENPILOT_ROOT="${OPENPILOT_ROOT:-/workspace/openpilot}"
ROS2_WS="${ROS2_WS:-/workspace/ros2_ws}"

source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
source "${OPENPILOT_ROOT}/.venv/bin/activate"
source "${ROS2_WS}/install/setup.bash"

exec ros2 launch openpilot_ros2_bridge openpilot_bridge.launch.py "$@"
