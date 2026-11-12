#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME_DIR="${ROBOTOS_RUNTIME_DIR:-$HOME/robotos-runtime}"
exec env -i HOME="$HOME" USER="${USER:-user}" LANG="${LANG:-C.UTF-8}" PATH=/usr/bin:/bin \
  bash --noprofile --norc -ec '
    source /opt/ros/jazzy/setup.bash
    source "$1/install/setup.bash"
    exec ros2 launch robotos_bringup demo.launch.py enable_fault_injection:=true
  ' bash "$RUNTIME_DIR"
