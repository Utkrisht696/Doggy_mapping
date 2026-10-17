#!/usr/bin/env bash
set -eo pipefail
_doggy_replay_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$_doggy_replay_root"
source scripts/setup_ros.sh
export ROS_DOMAIN_ID=74 ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="$_doggy_replay_root/config/cyclonedds_replay.xml"
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1
export ROS_LOG_DIR="$_doggy_replay_root/reports/live_2d/replay_ros_logs"
unset GTK_PATH GTK_EXE_PREFIX GTK_DATA_PREFIX GTK_IM_MODULE_FILE GIO_MODULE_DIR QT_PLUGIN_PATH QT_QPA_PLATFORM_PLUGIN_PATH LD_PRELOAD
mkdir -p reports/live_2d
exec 9>reports/live_2d/replay.lock
flock -n 9 || { echo '2D replay is already running.'; exit 1; }
python3 -B scripts/check_mapping_owners.py
exec ros2 launch "$_doggy_replay_root/launch/replay_2d_mapping.launch.py" "$@"
