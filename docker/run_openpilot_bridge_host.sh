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

PUBLISHED_IMAGE="ghcr.io/jkk-research/openpilot_ros2_bridge:latest"

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

# Stop a previous bridge first: it may be using the openpilot checkout that gets switched below
docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

pull_image() {
  if docker pull "${IMAGE_NAME}"; then
    return
  fi
  # Keep working offline (e.g. in the car) with the image pulled last time
  if docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
    echo "Could not pull ${IMAGE_NAME}; using the copy already on this machine" >&2
  else
    echo "Could not pull ${IMAGE_NAME}, and it is not on this machine yet" >&2
    exit 1
  fi
}

case "${IMAGE_SOURCE}" in
  build)
    docker build -t "${IMAGE_NAME}" -f "${REPO_ROOT}/docker/Dockerfile" "${REPO_ROOT}"
    ;;
  cyclone)
    # The published image now includes CycloneDDS, so there is nothing to build
    echo "image_source:=cyclone is no longer needed; using ${PUBLISHED_IMAGE} instead of ${IMAGE_NAME}" >&2
    IMAGE_NAME="${PUBLISHED_IMAGE}"
    pull_image
    ;;
  *)
    pull_image
    ;;
esac

# The typed message definitions are compiled into the image. ROS nodes on the host decode its topics correctly only
# with the same definitions; different ones give wrong values without any error. Each build lists the definitions it
# built in its ament index (rewritten by every build, unlike install/, which keeps files of removed messages), and
# the generated .idl files show what was built (with --symlink-install the installed .msg files link to the sources).
IMAGE_MSG_DIGESTS="$(docker run --rm --network none --entrypoint sh "${IMAGE_NAME}" -c \
  'cd /workspace/ros2_ws/install/openpilot_ros2_bridge/share/openpilot_ros2_bridge 2>/dev/null &&
   grep -E "\.(msg|idl)\$" ../ament_index/resource_index/rosidl_interfaces/openpilot_ros2_bridge | xargs -r sha256sum --' \
  2>/dev/null | LC_ALL=C sort -k2 || true)"
image_digests_of() {
  grep "\.$1\$" <<<"${IMAGE_MSG_DIGESTS}" || true
}
interface_digests_of() {
  # $1: install prefix that holds the package; the .idl files its last build registered
  (cd "$1/share/openpilot_ros2_bridge" 2>/dev/null &&
    grep '\.idl$' ../ament_index/resource_index/rosidl_interfaces/openpilot_ros2_bridge 2>/dev/null |
    xargs -r sha256sum -- 2>/dev/null | LC_ALL=C sort -k2) || true
}
host_package_prefix() {
  # The build host nodes use: the first on AMENT_PREFIX_PATH, like ament's own lookup, else the colcon workspace
  # this checkout sits in (<ws>/src/openpilot_ros2_bridge), with an isolated or a merged install
  local prefix ws
  local -a prefixes=()
  IFS=: read -r -a prefixes <<<"${AMENT_PREFIX_PATH:-}"
  ws="$(dirname "$(dirname "${REPO_ROOT}")")"
  for prefix in "${prefixes[@]}" "${ws}/install/openpilot_ros2_bridge" "${ws}/install"; do
    if [[ -n "${prefix}" && -e "${prefix}/share/ament_index/resource_index/packages/openpilot_ros2_bridge" ]]; then
      echo "${prefix}"
      return
    fi
  done
}

# mount_source swaps in this checkout's scripts and launch files, which need this checkout's definitions; check it
# before anything slow runs
if [[ "${MOUNT_SOURCE}" == "true" ]]; then
  if [[ ! -d "${REPO_ROOT}/openpilot" ]]; then
    echo "mount_source needs the bridge source tree; ${REPO_ROOT}/openpilot not found (set repo_root)" >&2
    exit 1
  fi
  if [[ "$(cd "${REPO_ROOT}" && sha256sum -- msg/*.msg 2>/dev/null | LC_ALL=C sort -k2)" != "$(image_digests_of msg)" ]]; then
    echo "mount_source:=true can't be used with ${IMAGE_NAME}: it was built with different message definitions than" \
      "${REPO_ROOT}/msg, and mount_source only swaps in scripts and launch files." \
      "Build the image from this checkout instead: image_name:=openpilot_ros2_bridge:local image_source:=build" >&2
    exit 1
  fi
  # Nor does it install the packages this checkout's image would have; the image holds a copy of the files that list them
  IMAGE_BUILD_DIGESTS="$(docker run --rm --network none --entrypoint sh "${IMAGE_NAME}" -c \
    'cd /workspace/ros2_ws/src/openpilot_ros2_bridge && sha256sum -- docker/Dockerfile requirements.txt' 2>/dev/null || true)"
  if [[ "$(cd "${REPO_ROOT}" && sha256sum -- docker/Dockerfile requirements.txt 2>/dev/null)" != "${IMAGE_BUILD_DIGESTS}" ]]; then
    echo "Warning: ${IMAGE_NAME} was built from a different docker/Dockerfile or requirements.txt than ${REPO_ROOT}," \
      "and mount_source:=true doesn't install what this checkout adds there, so nodes that need it fail" \
      "(the raw CAN frames need the can_msgs package, for example)." \
      "To get it, build the image from this checkout: image_name:=openpilot_ros2_bridge:local image_source:=build" >&2
  fi
fi

HOST_PACKAGE_PREFIX="$(host_package_prefix)"
if [[ -n "${HOST_PACKAGE_PREFIX}" && "$(interface_digests_of "${HOST_PACKAGE_PREFIX}")" != "$(image_digests_of idl)" ]]; then
  echo "Warning: the openpilot_ros2_bridge package built in ${HOST_PACKAGE_PREFIX} has different message definitions" \
    "than ${IMAGE_NAME}, so ROS nodes using it will read wrong values from the typed topics." \
    "Rebuild the package from the same commit as the image, or build the image from your checkout" \
    "(image_name:=openpilot_ros2_bridge:local image_source:=build)." >&2
fi

# Check out the Comma's openpilot commit, when it isn't already: the nodes decode its messages with that version's
# schema, service list and opendbc. Nothing in it has to be built.
prepare_openpilot() {
  local info="" url commit current path
  local -a submodules=()

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
    # Files that .gitattributes routes through git-lfs are left out of this check: the bridge doesn't use them, and
    # git can report them as modified when they aren't (openpilot commits some *.npz files there that aren't LFS
    # pointers, and LFS files can look changed to git without the LFS filter)
    elif [[ -n "${current}" && -n "$(git -C "${HOST_OPENPILOT_ROOT}" status --porcelain --untracked-files=no \
        -- . ':(exclude,attr:filter=lfs)')" ]]; then
      echo "${HOST_OPENPILOT_ROOT} has local changes; not switching it to the Comma's commit ${commit:0:8}" >&2
    else
      echo "Checking out openpilot ${commit:0:8} from ${url} to match the Comma"
      git -C "${HOST_OPENPILOT_ROOT}" fetch -q --depth 1 "${url}" "${commit}"
      git -C "${HOST_OPENPILOT_ROOT}" checkout -q --detach FETCH_HEAD
      current="${commit}"
    fi

    # opendbc, and cereal where it was a submodule (old openpilot); checked every time, in case an earlier launch
    # stopped before they were checked out
    if [[ "${current}" == "${commit}" ]]; then
      for path in opendbc_repo opendbc cereal; do
        if git -C "${HOST_OPENPILOT_ROOT}" config -f .gitmodules --get-regexp '^submodule\..*\.path$' 2>/dev/null |
            awk '{print $2}' | grep -qxF "${path}"; then
          submodules+=("${path}")
        fi
      done
      if (( ${#submodules[@]} )) && [[ -n "$(git -C "${HOST_OPENPILOT_ROOT}" submodule status -- "${submodules[@]}" |
          grep '^[-+U]' || true)" ]]; then
        echo "Checking out ${submodules[*]} in ${HOST_OPENPILOT_ROOT}"
        git -C "${HOST_OPENPILOT_ROOT}" submodule update -q --init --depth 1 "${submodules[@]}" ||
          echo "Could not check out ${submodules[*]} in ${HOST_OPENPILOT_ROOT}" >&2
      fi
    fi
  fi

  if [[ ! -d "${HOST_OPENPILOT_ROOT}/openpilot" ]]; then
    echo "Missing openpilot source tree under ${HOST_OPENPILOT_ROOT}/openpilot" >&2
    exit 1
  fi
}

if [[ "${PREPARE_OPENPILOT}" == "true" ]]; then
  prepare_openpilot
elif [[ ! -d "${HOST_OPENPILOT_ROOT}/openpilot" ]]; then
  echo "Missing openpilot source tree under ${HOST_OPENPILOT_ROOT}/openpilot" >&2
  exit 1
fi
# Every message definition imports opendbc's car.capnp, so none of the nodes could start without it
if [[ -d "${HOST_OPENPILOT_ROOT}/opendbc_repo" && -z "$(ls -A "${HOST_OPENPILOT_ROOT}/opendbc_repo")" ]]; then
  echo "${HOST_OPENPILOT_ROOT}/opendbc_repo is empty: check it out with" \
    "'git -C ${HOST_OPENPILOT_ROOT} submodule update --init opendbc_repo', or use prepare_openpilot:=true" >&2
  exit 1
fi

# Run as the host user: /dev/shm is shared, and a root container can't use the host user's
# Fast DDS shared memory (so host nodes get no data) and leaves root-owned files behind
DOCKER_ARGS=(
  --rm
  --name "${CONTAINER_NAME}"
  --user "$(id -u):$(id -g)"
  -e HOME=/tmp
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

# Run this checkout's scripts and launch files instead of the ones baked into the image (checked above)
if [[ "${MOUNT_SOURCE}" == "true" ]]; then
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
