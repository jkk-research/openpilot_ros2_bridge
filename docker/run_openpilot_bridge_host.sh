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

# Optional settings passed through the environment by openpilot_bridge_docker.launch.py
COMMA_TARGET="${COMMA_TARGET:-}"
COMMA_OPENPILOT_DIR="${COMMA_OPENPILOT_DIR:-/data/openpilot}"
PREPARE_OPENPILOT="${PREPARE_OPENPILOT:-false}"
MOUNT_SOURCE="${MOUNT_SOURCE:-false}"
CYCLONEDDS_URI_VALUE="${CYCLONEDDS_URI_VALUE:-}"

# Default to the tree this script lives in (the source tree for --symlink-install builds)
if [[ -z "${REPO_ROOT}" ]]; then
  REPO_ROOT="$(dirname "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")")"
fi

HOST_OPENPILOT_ROOT="${HOST_OPENPILOT_ROOT/#\~/${HOME}}"
if [[ -z "${HOST_OPENPILOT_ROOT}" && "${PREPARE_OPENPILOT}" == "true" ]]; then
  HOST_OPENPILOT_ROOT="${HOME}/openpilot"
fi
if [[ -z "${HOST_OPENPILOT_ROOT}" ]]; then
  echo "host_openpilot_root must point to an existing openpilot checkout" >&2
  exit 1
fi

# Stop a previous bridge first: it may be using the openpilot build that gets refreshed below
docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

case "${IMAGE_SOURCE}" in
  build)
    docker build -t "${IMAGE_NAME}" -f "${REPO_ROOT}/docker/Dockerfile" "${REPO_ROOT}"
    ;;
  cyclone)
    # Published image + CycloneDDS + build tools; only built when missing
    if ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
      docker build -t "${IMAGE_NAME}" -f "${REPO_ROOT}/docker/Dockerfile.cyclone" "${REPO_ROOT}/docker"
    fi
    ;;
  *)
    docker pull "${IMAGE_NAME}"
    ;;
esac

# Check out the Comma's openpilot commit and build msgq + cereal bridge, each only when needed
prepare_openpilot() {
  local info="" url commit current head
  local stamp="${HOST_OPENPILOT_ROOT}/.bridge_build_commit"

  if [[ -n "${COMMA_TARGET}" ]]; then
    info="$(ssh -o BatchMode=yes -o ConnectTimeout=5 "${COMMA_TARGET}" \
      "cd '${COMMA_OPENPILOT_DIR}' && git remote get-url origin && git rev-parse HEAD" 2>/dev/null)" || info=""
  fi

  if [[ -z "${info}" ]]; then
    echo "Could not read the openpilot version from ${COMMA_TARGET:-the Comma} over SSH; using ${HOST_OPENPILOT_ROOT} as it is" >&2
  else
    url="$(sed -n 1p <<<"${info}")"
    commit="$(sed -n 2p <<<"${info}")"
    export GIT_LFS_SKIP_SMUDGE=1

    if [[ ! -d "${HOST_OPENPILOT_ROOT}/.git" ]]; then
      echo "Creating openpilot checkout in ${HOST_OPENPILOT_ROOT}"
      git init -q "${HOST_OPENPILOT_ROOT}"
      # submodule URLs are relative to origin
      git -C "${HOST_OPENPILOT_ROOT}" remote add origin "${url}"
    fi

    current="$(git -C "${HOST_OPENPILOT_ROOT}" rev-parse -q --verify HEAD || true)"
    if [[ "${current}" == "${commit}" ]]; then
      echo "openpilot checkout already at the Comma's commit ${commit:0:8}"
    elif [[ -n "${current}" && -n "$(git -C "${HOST_OPENPILOT_ROOT}" status --porcelain --untracked-files=no)" ]]; then
      echo "${HOST_OPENPILOT_ROOT} has local changes; not switching it to the Comma's commit ${commit:0:8}" >&2
    else
      echo "Checking out openpilot ${commit:0:8} from ${url} to match the Comma"
      git -C "${HOST_OPENPILOT_ROOT}" fetch -q --depth 1 "${url}" "${commit}"
      git -C "${HOST_OPENPILOT_ROOT}" checkout -q --detach FETCH_HEAD
      git -C "${HOST_OPENPILOT_ROOT}" submodule update -q --init --depth 1 msgq_repo opendbc_repo
    fi
  fi

  if [[ ! -d "${HOST_OPENPILOT_ROOT}/openpilot" ]]; then
    echo "Missing openpilot source tree under ${HOST_OPENPILOT_ROOT}/openpilot" >&2
    exit 1
  fi

  head="$(git -C "${HOST_OPENPILOT_ROOT}" rev-parse HEAD 2>/dev/null || echo unknown)"
  if [[ -f "${HOST_OPENPILOT_ROOT}/msgq_repo/msgq/ipc_pyx.so" \
        && -x "${HOST_OPENPILOT_ROOT}/openpilot/cereal/messaging/bridge" \
        && "$(cat "${stamp}" 2>/dev/null)" == "${head}" ]]; then
    echo "openpilot ${head:0:8} already built for the bridge"
    return
  fi

  echo "Building msgq and the cereal bridge for openpilot ${head:0:8}"
  docker run --rm --user "$(id -u):$(id -g)" \
    -v "${HOST_OPENPILOT_ROOT}:/workspace/openpilot" \
    -v "${REPO_ROOT}/scripts/build_openpilot_minimal.sh:/build_openpilot_minimal.sh:ro" \
    "${IMAGE_NAME}" bash /build_openpilot_minimal.sh
  echo "${head}" > "${stamp}"
}

if [[ "${PREPARE_OPENPILOT}" == "true" ]]; then
  prepare_openpilot
elif [[ ! -d "${HOST_OPENPILOT_ROOT}/openpilot" ]]; then
  echo "Missing openpilot source tree under ${HOST_OPENPILOT_ROOT}/openpilot" >&2
  exit 1
fi

DOCKER_ARGS=(
  --rm
  --name "${CONTAINER_NAME}"
  --network host
  --ipc host
  --add-host host.docker.internal:host-gateway
  -v "${HOST_OPENPILOT_ROOT}:/workspace/openpilot"
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}"
  -e ROS_LOCALHOST_ONLY=0
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION_VALUE}"
  -e ROS_DISTRO_NAME="${ROS_DISTRO_NAME}"
  -e OPENPILOT_ROOT=/workspace/openpilot
  -e ROS2_WS=/workspace/ros2_ws
  -e BRIDGE_VENV=/workspace/ros2_ws/src/openpilot_ros2_bridge/.venv
)

# Use the host's CycloneDDS config so the container joins the same DDS network
if [[ -n "${CYCLONEDDS_URI_VALUE}" ]]; then
  if [[ "${CYCLONEDDS_URI_VALUE}" == file://* ]]; then
    CYCLONEDDS_FILE="${CYCLONEDDS_URI_VALUE#file://}"
    DOCKER_ARGS+=(-v "${CYCLONEDDS_FILE}:${CYCLONEDDS_FILE}:ro")
  fi
  DOCKER_ARGS+=(-e CYCLONEDDS_URI="${CYCLONEDDS_URI_VALUE}")
fi

# Run this checkout's scripts and launch files instead of the ones baked into the image
if [[ "${MOUNT_SOURCE}" == "true" ]]; then
  if [[ ! -d "${REPO_ROOT}/openpilot" ]]; then
    echo "mount_source needs the bridge source tree; ${REPO_ROOT}/openpilot not found (set repo_root)" >&2
    exit 1
  fi
  CONTAINER_SHARE=/workspace/ros2_ws/install/openpilot_ros2_bridge/share/openpilot_ros2_bridge
  DOCKER_ARGS+=(
    -v "${REPO_ROOT}/openpilot:${CONTAINER_SHARE}/scripts:ro"
    -v "${REPO_ROOT}/launch:${CONTAINER_SHARE}/launch:ro"
    -v "${REPO_ROOT}/docker:/workspace/ros2_ws/src/openpilot_ros2_bridge/docker:ro"
  )
fi

exec docker run "${DOCKER_ARGS[@]}" \
  "${IMAGE_NAME}" \
  bash /workspace/ros2_ws/src/openpilot_ros2_bridge/docker/run_openpilot_bridge_container.sh "$@"
