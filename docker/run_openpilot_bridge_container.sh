#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"
OPENPILOT_ROOT="${OPENPILOT_ROOT:-/workspace/openpilot}"
ROS2_WS="${ROS2_WS:-/workspace/ros2_ws}"
BRIDGE_VENV="${BRIDGE_VENV:-${ROS2_WS}/src/openpilot_ros2_bridge/.venv}"

if [[ ! -d "${OPENPILOT_ROOT}/openpilot" ]]; then
  echo "Missing openpilot source tree at ${OPENPILOT_ROOT}/openpilot" >&2
  exit 1
fi
if [[ ! -f "${BRIDGE_VENV}/bin/activate" ]]; then
  echo "Missing bridge virtual environment at ${BRIDGE_VENV}/bin/activate" >&2
  exit 1
fi
# ROS setup scripts read unset variables (e.g. AMENT_TRACE_SETUP_FILES), so relax -u while sourcing
set +u
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
source "${BRIDGE_VENV}/bin/activate"
source "${ROS2_WS}/install/setup.bash"
set -u
export PYTHONPATH="${OPENPILOT_ROOT}/openpilot:${PYTHONPATH:-}"

# /dev/shm is shared with the host, so keep the bridge's msgq queues apart from any openpilot running
# natively there: msgq wakes readers by signalling thread ids, which don't match across PID namespaces.
# Older msgq versions keep prefixed queues in /dev/shm/<prefix>, newer ones in /dev/shm/msgq_<prefix>.
export OPENPILOT_PREFIX="${OPENPILOT_PREFIX:-ros2_bridge_$(id -u)}"
mkdir -p "/dev/shm/${OPENPILOT_PREFIX}" "/dev/shm/msgq_${OPENPILOT_PREFIX}"

exec ros2 launch openpilot_ros2_bridge openpilot_bridge.launch.py "$@"
