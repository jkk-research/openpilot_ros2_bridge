#!/usr/bin/env bash
# Copies comma_joystick_relay.py to the Comma and runs it there over SSH, for start_control:=true.
# Usage: start_comma_joystick_relay.sh <ssh target> [openpilot dir on the Comma] [this PC's address as the Comma sees it]
set -euo pipefail

TARGET="$1"
OPENPILOT_DIR="${2:-/data/openpilot}"
HOST="${3:-}"
RELAY="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/comma_joystick_relay.py"
REMOTE_RELAY=/tmp/openpilot_ros2_joystick_relay.py
SSH_OPTS=(-o BatchMode=yes -o ConnectTimeout=5)

ssh "${SSH_OPTS[@]}" "${TARGET}" "cat > ${REMOTE_RELAY}" < "${RELAY}"

# -tt: stopping the launch closes the session, which hangs up the relay so it turns joystick mode off again.
# openpilot's own Python has its dependencies; the relay finds the PC through the SSH session unless HOST is set.
exec ssh -tt "${SSH_OPTS[@]}" -o ServerAliveInterval=5 "${TARGET}" \
  "PY=/usr/local/venv/bin/python3; [ -x \"\$PY\" ] || PY=python3; exec \"\$PY\" ${REMOTE_RELAY} --openpilot-dir '${OPENPILOT_DIR}'${HOST:+ --host '${HOST}'}"
