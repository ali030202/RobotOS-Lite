#!/usr/bin/env bash
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "${ROBOTOS_CONDA_SH:-$HOME/anaconda3/etc/profile.d/conda.sh}"
conda activate "${ROBOTOS_CONDA_ENV:-robotos}"
source /opt/ros/jazzy/setup.bash
source "${ROBOTOS_RUNTIME_DIR:-$HOME/robotos-runtime}/install/setup.bash"
export ROBOTOS_ROS_PYTHON=/usr/bin/python3
if [[ -z "${ROBOTOS_OPERATOR_TOKEN:-}" ]]; then
  export ROBOTOS_OPERATOR_TOKEN="$(cat "$PROJECT_DIR/.server-run/operator-token")"
fi
cd "$PROJECT_DIR"
exec python -m uvicorn app.main:app --app-dir backend --host 127.0.0.1 --port "${ROBOTOS_PORT:-8000}"
