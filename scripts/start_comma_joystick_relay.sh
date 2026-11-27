#!/usr/bin/env bash
# Copies comma_joystick_relay.py to the Comma and runs it there over SSH, for start_control:=true.
# Usage: start_comma_joystick_relay.sh <ssh target> [openpilot dir on the Comma] [this PC's address as the Comma sees it]
set -euo pipefail

TARGET="$1"
OPENPILOT_DIR="${2:-/data/openpilot}"
HOST="${3:-}"
HERE="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
REMOTE_DIR=/tmp/openpilot_ros2_bridge
SSH_OPTS=(-o BatchMode=yes -o ConnectTimeout=5)
# Unpacked next to the directory and moved in, so a process starting there meanwhile (the bridge and the joystick relay
# start together and share comma_zmtp.py) never reads a half-written file
UNPACK="mkdir -p ${REMOTE_DIR} && T=\$(mktemp -d ${REMOTE_DIR}/.new.XXXXXX) && tar -C \"\$T\" -xf - && mv -f \"\$T\"/* ${REMOTE_DIR}/ && rmdir \"\$T\""

# comma_zmtp.py speaks ZeroMQ for it where openpilot's Python has no pyzmq
tar -C "${HERE}" -cf - comma_joystick_relay.py comma_zmtp.py |
  ssh "${SSH_OPTS[@]}" "${TARGET}" "${UNPACK}"

# -tt: stopping the launch closes the session, which hangs up the relay so it turns joystick mode off again.
# openpilot's own Python has its dependencies; the relay finds the PC through the SSH session unless HOST is set.
exec ssh -tt "${SSH_OPTS[@]}" -o ServerAliveInterval=5 "${TARGET}" \
  "PY=/usr/local/venv/bin/python3; [ -x \"\$PY\" ] || PY=python3; exec \"\$PY\" ${REMOTE_DIR}/comma_joystick_relay.py --openpilot-dir '${OPENPILOT_DIR}' --wait-on-error${HOST:+ --host '${HOST}'}"
