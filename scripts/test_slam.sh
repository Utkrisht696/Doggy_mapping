#!/usr/bin/env bash
# Mapping only; optional RViz. No joystick, navigation or motion clients.
set -eo pipefail
_doggy_test_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$_doggy_test_root"
_doggy_algorithm="${1:-slam_toolbox}"
if [[ "$_doggy_algorithm" != "slam_toolbox" ]]; then
    echo "Usage: bash scripts/test_slam.sh slam_toolbox [rviz:=true]"
    echo "For standalone Point-LIO readiness, read POINT_LIO_SETUP.md. The CMU launch path has been removed."
    exit 2
fi
if [[ $# -gt 0 ]]; then shift; fi
source scripts/setup_ros.sh
mkdir -p reports/slam reports/tf
export ROS_LOG_DIR="${ROS_LOG_DIR:-$_doggy_test_root/reports/slam/ros_logs}"
# Share the visualization lock: one owner for odometry/robot-clock TF at a time.
exec 9>reports/tf/visualization.lock
if ! flock -n 9; then
    echo "Another Go2 visualization/mapping launch is running. Stop it before testing a mapper."
    exit 1
fi
exec ros2 launch "$_doggy_test_root/launch/slam/slam_test.launch.py" "algorithm:=$_doggy_algorithm" "$@"
