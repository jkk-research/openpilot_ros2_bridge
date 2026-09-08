#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$1"
IMAGE_NAME="$2"
CONTAINER_NAME="$3"
ROS_DOMAIN_ID_VALUE="$4"
RMW_IMPLEMENTATION_VALUE="$5"
ROS_DISTRO_NAME="$6"
HOST_OPENPILOT_ROOT="$7"
shift 7

if [[ -z "${HOST_OPENPILOT_ROOT}" ]]; then
  echo "host_openpilot_root must point to an existing openpilot checkout" >&2
  exit 1
fi

if [[ ! -f "${HOST_OPENPILOT_ROOT}/.venv/bin/activate" ]]; then
  echo "Missing openpilot virtual environment under ${HOST_OPENPILOT_ROOT}/.venv/bin/activate" >&2
  exit 1
fi

docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true
docker build -t "${IMAGE_NAME}" -f "${REPO_ROOT}/docker/Dockerfile" "${REPO_ROOT}"

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
  "${IMAGE_NAME}" \
  bash /workspace/ros2_ws/src/openpilot_ros2_bridge/docker/run_openpilot_bridge_container.sh "$@"
