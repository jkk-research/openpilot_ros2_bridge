#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$1"
IMAGE_NAME="$2"
CONTAINER_NAME="$3"
ROS_DOMAIN_ID_VALUE="$4"
RMW_IMPLEMENTATION_VALUE="$5"
ROS_DISTRO_NAME="$6"
shift 6

docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true
docker build -t "${IMAGE_NAME}" -f "${REPO_ROOT}/docker/Dockerfile" "${REPO_ROOT}"

exec docker run --rm \
  --name "${CONTAINER_NAME}" \
  --network host \
  --ipc host \
  --add-host host.docker.internal:host-gateway \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}" \
  -e ROS_LOCALHOST_ONLY=0 \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION_VALUE}" \
  -e ROS_DISTRO_NAME="${ROS_DISTRO_NAME}" \
  -e OPENPILOT_ROOT=/workspace/openpilot \
  -e ROS2_WS=/workspace/ros2_ws \
  "${IMAGE_NAME}" \
  bash /workspace/ros2_ws/src/openpilot_ros2_bridge/docker/run_openpilot_bridge_container.sh "$@"
