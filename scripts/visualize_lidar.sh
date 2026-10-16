#!/usr/bin/env bash
# Official Unitree URDF, live encoder states and LiDAR visualization.
set -eo pipefail
_doggy_view_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$_doggy_view_root"
source scripts/setup_ros.sh
mkdir -p reports/tf
exec 9>reports/tf/visualization.lock
if ! flock -n 9; then
    echo "Go2 visualization is already running. Close its RViz window before relaunching."
    exit 1
fi
exec ros2 launch "$_doggy_view_root/launch/go2_visualization.launch.py" "$@"
