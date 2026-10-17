#!/usr/bin/env bash
# Complete live pipeline. No robot-command publisher is started.
set -eo pipefail
_doggy_2d_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$_doggy_2d_root"
source scripts/setup_ros.sh
export CYCLONEDDS_URI="$_doggy_2d_root/config/point_lio/cyclonedds_live.xml"
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1
unset GTK_PATH GTK_EXE_PREFIX GTK_DATA_PREFIX GTK_IM_MODULE_FILE GIO_MODULE_DIR QT_PLUGIN_PATH QT_QPA_PLATFORM_PLUGIN_PATH LD_PRELOAD
_doggy_2d_rmem=$(cat /proc/sys/net/core/rmem_max)
if (( _doggy_2d_rmem < 8388608 )); then
    echo 'Run sudo sysctl -w net.core.rmem_max=8388608 before this launch.'
    exit 1
fi
mkdir -p reports/live_2d
export ROS_LOG_DIR="$_doggy_2d_root/reports/live_2d/ros_logs"
exec 9>reports/live_2d/launch.lock
flock -n 9 || { echo 'Live 2D mapping is already running.'; exit 1; }
python3 -B scripts/check_mapping_owners.py
exec ros2 launch "$_doggy_2d_root/launch/live_2d_mapping.launch.py" "$@"
