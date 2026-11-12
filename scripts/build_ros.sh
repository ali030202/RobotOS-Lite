#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME_DIR="${ROBOTOS_RUNTIME_DIR:-$HOME/robotos-runtime}"
mkdir -p "$RUNTIME_DIR"
# ROS binary packages are built for Ubuntu's Python, not a Conda interpreter.
exec env -i HOME="$HOME" USER="${USER:-user}" LANG="${LANG:-C.UTF-8}" PATH=/usr/bin:/bin \
  bash --noprofile --norc -ec '
    source /opt/ros/jazzy/setup.bash
    cd "$1/ros2_ws"
    colcon --log-base "$2/log" build --build-base "$2/build" --install-base "$2/install" --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
    colcon --log-base "$2/log" test --build-base "$2/build" --install-base "$2/install" --event-handlers console_direct+
    colcon test-result --test-result-base "$2/build" --verbose
  ' bash "$PROJECT_DIR" "$RUNTIME_DIR"
