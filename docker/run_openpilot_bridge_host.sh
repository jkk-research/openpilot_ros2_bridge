#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$1"
IMAGE_NAME="$2"
IMAGE_SOURCE="$3"
CONTAINER_NAME="$4"
ROS_DOMAIN_ID_VALUE="$5"
RMW_IMPLEMENTATION_VALUE="$6"
ROS_DISTRO_NAME="$7"
HOST_OPENPILOT_ROOT="$8"
shift 8

if [[ -z "${HOST_OPENPILOT_ROOT}" ]]; then
  echo "host_openpilot_root must point to an existing openpilot checkout" >&2
  exit 1
fi

if [[ ! -d "${HOST_OPENPILOT_ROOT}/openpilot" ]]; then
  echo "Missing openpilot source tree under ${HOST_OPENPILOT_ROOT}/openpilot" >&2
  exit 1
fi

docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

if [[ "${IMAGE_SOURCE}" == "build" ]]; then
  docker build -t "${IMAGE_NAME}" -f "${REPO_ROOT}/docker/Dockerfile" "${REPO_ROOT}"
else
  docker pull "${IMAGE_NAME}"
fi

exec docker run --rm \
  --name "${CONTAINER_NAME}" \
  --network host \
  --ipc host \
  --add-host host.docker.internal:host-gateway \
  -v "${HOST_OPENPILOT_ROOT}:/workspace/openpilot" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}" \
  -e ROS_LOCALHOST_ONLY=0 \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION_VALUE}" \
  -e ROS_DISTRO_NAME="${ROS_DISTRO_NAME}" \
  -e OPENPILOT_ROOT=/workspace/openpilot \
  -e ROS2_WS=/workspace/ros2_ws \
  -e BRIDGE_VENV=/workspace/ros2_ws/src/openpilot_ros2_bridge/.venv \
  "${IMAGE_NAME}" \
  bash /workspace/ros2_ws/src/openpilot_ros2_bridge/docker/run_openpilot_bridge_container.sh "$@"
