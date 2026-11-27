#!/usr/bin/env bash
# Starts the Comma's side of the bridge over SSH, for start_remote_bridge:=true. The session stays open while it runs;
# stopping the launch closes the session, which stops it.
# Usage: start_comma_bridge.sh <ssh target> [openpilot dir on the Comma] [python|cereal|auto] [command]
#   python  comma_bridge.py, run with openpilot's own Python (the default; works with every openpilot version)
#   cereal  openpilot's own cereal bridge (cereal/messaging/bridge, in openpilot 0.11.2 and older)
# Either way, a cereal bridge that already runs on the Comma (serving the PC) is used.
#   auto    openpilot's cereal bridge where the Comma's openpilot still has it, otherwise comma_bridge.py
#   command runs this on the Comma instead (the remote_bridge_command launch argument)
# The launch starts this again whenever it ends, so errors that another try won't fix make it wait for Ctrl+C instead.
set -euo pipefail

TARGET="$1"
OPENPILOT_DIR="${2:-/data/openpilot}"
MODE="${3:-python}"
COMMAND="${4:-}"
HERE="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
REMOTE_DIR=/tmp/openpilot_ros2_bridge
SSH_OPTS=(-o BatchMode=yes -o ConnectTimeout=5)
# Unpacked next to the directory and moved in, so a process starting there meanwhile (the bridge and the joystick relay
# start together and share comma_zmtp.py) never reads a half-written file
UNPACK="mkdir -p ${REMOTE_DIR} && T=\$(mktemp -d ${REMOTE_DIR}/.new.XXXXXX) && tar -C \"\$T\" -xf - && mv -f \"\$T\"/* ${REMOTE_DIR}/ && rmdir \"\$T\""

case "${MODE}" in
  python|cereal|auto) ;;
  *) echo "comma_bridge must be python, cereal or auto, not '${MODE}'" >&2; exec sleep infinity ;;
esac

# -tt: closing the session hangs up whatever runs on the Comma
if [[ -n "${COMMAND}" ]]; then
  exec ssh -tt "${SSH_OPTS[@]}" -o ServerAliveInterval=5 "${TARGET}" "${COMMAND}"
fi

# Copied also when openpilot's own bridge runs: it's small, and then the Comma always has the current one
tar -C "${HERE}" -cf - comma_bridge.py comma_zmtp.py |
  ssh "${SSH_OPTS[@]}" "${TARGET}" "${UNPACK}"

# Runs on the Comma. openpilot's cereal bridge only counts when its source is there too: a git update to an openpilot
# without it can leave the old binary behind, built for another msgq version, which would corrupt openpilot's queues.
read -r -d '' REMOTE <<'EOF' || true
C=
for d in "$OP/openpilot/cereal/messaging" "$OP/cereal/messaging"; do
  if [ -d "$d" ]; then C="$d"; break; fi
done
# openpilot's cereal bridge serves the PC unless it runs with two arguments, an address and services: then it runs the
# other way (ZMQ to msgq, e.g. for openpilot's joystick tools) and serves nothing. Its command line holds one NUL per
# word.
serving_bridge() {
  for p in $(pgrep -x bridge); do
    n=$(tr -cd '\000' < "/proc/$p/cmdline" 2>/dev/null | wc -c)
    [ "$((n))" -ge 1 ] && [ "$((n))" -le 2 ] && return 0
  done
  return 1
}
if serving_bridge; then
  echo "[comma bridge] openpilot's cereal bridge is already running on the Comma; using it"
  while serving_bridge; do sleep 5; done
  exit 0
fi
HAVE=no
[ -n "$C" ] && [ -f "$C/bridge.cc" ] && [ -x "$C/bridge" ] && HAVE=yes
if [ "$MODE" = cereal ] || { [ "$MODE" = auto ] && [ "$HAVE" = yes ]; }; then
  if [ "$HAVE" != yes ]; then
    echo "[comma bridge] No usable cereal bridge in $OP: openpilot 0.11.3 and newer don't have it (older versions need it" \
      "built). Use comma_bridge:=python." >&2
    exec sleep infinity
  fi
  pkill -f "$D/comma_bridge.py" 2>/dev/null && sleep 1
  echo "[comma bridge] Starting openpilot's cereal bridge"
  cd "$C" && exec ./bridge
fi
PY=/usr/local/venv/bin/python3
[ -x "$PY" ] || PY=python3
exec "$PY" "$D/comma_bridge.py" --openpilot-dir "$OP" --wait-on-error
EOF

exec ssh -tt "${SSH_OPTS[@]}" -o ServerAliveInterval=5 "${TARGET}" \
  "OP=$(printf %q "${OPENPILOT_DIR}"); MODE=${MODE}; D=${REMOTE_DIR}; ${REMOTE}"
