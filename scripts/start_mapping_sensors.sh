#!/usr/bin/env bash
# Only official model TF, reported odometry/clock, and cleaned sensor streams.
set -eo pipefail
_doggy_sensor_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$_doggy_sensor_root"
source scripts/setup_ros.sh
mkdir -p reports/tf reports/foundation
export ROS_LOG_DIR="${ROS_LOG_DIR:-$_doggy_sensor_root/reports/foundation/ros_logs}"
exec 9>reports/tf/visualization.lock
if ! flock -n 9; then
    echo "A Go2 TF/visualization/mapping launch is already running. Stop it before starting sensor preparation."
    exit 1
fi
exec ros2 launch "$_doggy_sensor_root/launch/mapping_sensors.launch.py" "$@"
