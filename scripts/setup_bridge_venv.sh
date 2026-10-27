#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_PATH="${1:-${REPO_ROOT}/.venv}"

python3 -m venv --system-site-packages "${VENV_PATH}"
source "${VENV_PATH}/bin/activate"
python3 -m pip install --upgrade pip
python3 -m pip install -r "${REPO_ROOT}/requirements.txt"

echo "Bridge virtual environment created at ${VENV_PATH}"
