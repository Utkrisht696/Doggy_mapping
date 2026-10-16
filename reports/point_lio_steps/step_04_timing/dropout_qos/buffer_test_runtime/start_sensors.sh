#!/usr/bin/env bash
set -eo pipefail
cd /home/jain/Doggy
source scripts/setup_ros.sh
if (( $(cat /proc/sys/net/core/rmem_max) < 8388608 )); then
    echo "The approved temporary rmem_max=8388608 setting must be applied first." >&2
    exit 1
fi
export CYCLONEDDS_URI=/home/jain/Doggy/reports/point_lio_steps/step_04_timing/dropout_qos/buffer_test_runtime/cyclonedds.buffer_test.xml
export ROS_LOG_DIR=/home/jain/Doggy/reports/point_lio_steps/step_04_timing/dropout_qos/buffer_test_runtime/ros_logs
exec 9>reports/tf/visualization.lock
if ! flock -n 9; then
    echo "An existing sensor/model launch still owns the launch lock." >&2
    exit 1
fi
exec ros2 launch /home/jain/Doggy/launch/mapping_sensors.launch.py
