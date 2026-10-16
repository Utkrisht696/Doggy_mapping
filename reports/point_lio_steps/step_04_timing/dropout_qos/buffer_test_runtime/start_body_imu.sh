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
echo "$$" > reports/point_lio_steps/step_01_accelerometer/body_imu_adapter.pid
exec python3 -u scripts/publish_body_imu.py --calibration-file config/point_lio/body_imu_calibration.json
