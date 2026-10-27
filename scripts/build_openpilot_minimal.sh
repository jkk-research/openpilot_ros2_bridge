#!/usr/bin/env bash
# Minimal openpilot build for the bridge: msgq (lib + python ext) and cereal/messaging/bridge only.
# Run inside the openpilot_ros2_bridge:cyclone image with the openpilot checkout mounted at /workspace/openpilot:
#   docker run --rm --user "$(id -u):$(id -g)" -v ~/openpilot:/workspace/openpilot \
#     -v "$PWD/scripts/build_openpilot_minimal.sh:/build.sh:ro" openpilot_ros2_bridge:cyclone bash /build.sh
set -euo pipefail
cd "${OPENPILOT_ROOT:-/workspace/openpilot}"
VENV="${BRIDGE_VENV:-/workspace/ros2_ws/src/openpilot_ros2_bridge/.venv}"
OUT="$(mktemp -d)"
CXXFLAGS="-std=c++1z -O2 -fPIC -g -I. -Iopenpilot -Imsgq_repo"

echo "== msgq objects"
MSGQ_OBJS=()
for f in ipc event impl_msgq impl_fake msgq; do
  g++ $CXXFLAGS -c "msgq_repo/msgq/$f.cc" -o "$OUT/$f.o"
  MSGQ_OBJS+=("$OUT/$f.o")
done

echo "== msgq/ipc_pyx.so"
"$VENV/bin/cython" --cplus -3 msgq_repo/msgq/ipc_pyx.pyx -o "$OUT/ipc_pyx.cpp"
PYINC=$(python3 -c 'import sysconfig; print(sysconfig.get_paths()["include"])')
g++ $CXXFLAGS -I"$PYINC" -Wno-deprecated-declarations -shared -pthread \
  "$OUT/ipc_pyx.cpp" "${MSGQ_OBJS[@]}" -o msgq_repo/msgq/ipc_pyx.so

echo "== cereal/services.h"
python3 openpilot/cereal/services.py > openpilot/cereal/services.h

echo "== cereal/messaging/bridge"
g++ $CXXFLAGS \
  openpilot/cereal/messaging/bridge.cc \
  openpilot/cereal/messaging/msgq_to_zmq.cc \
  openpilot/cereal/messaging/bridge_zmq.cc \
  "${MSGQ_OBJS[@]}" -lzmq -lpthread -o openpilot/cereal/messaging/bridge

ls -la msgq_repo/msgq/ipc_pyx.so openpilot/cereal/messaging/bridge
