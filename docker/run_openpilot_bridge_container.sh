#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"
OPENPILOT_ROOT="${OPENPILOT_ROOT:-/workspace/openpilot}"
ROS2_WS="${ROS2_WS:-/workspace/ros2_ws}"
BRIDGE_VENV="${BRIDGE_VENV:-${ROS2_WS}/src/openpilot_ros2_bridge/.venv}"

source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
if [[ ! -d "${OPENPILOT_ROOT}/openpilot" ]]; then
  echo "Missing openpilot source tree at ${OPENPILOT_ROOT}/openpilot" >&2
  exit 1
fi
if [[ ! -f "${BRIDGE_VENV}/bin/activate" ]]; then
  echo "Missing bridge virtual environment at ${BRIDGE_VENV}/bin/activate" >&2
  exit 1
fi
source "${BRIDGE_VENV}/bin/activate"
source "${ROS2_WS}/install/setup.bash"
export PYTHONPATH="${OPENPILOT_ROOT}/openpilot:${PYTHONPATH:-}"

exec ros2 launch openpilot_ros2_bridge openpilot_bridge.launch.py "$@"
