#!/usr/bin/env bash
# Live preview by default. The estimator requires explicit start_mapping:=true.
set -eo pipefail
_doggy_lio_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$_doggy_lio_root"
source scripts/setup_ros.sh
if ! _doggy_lio_rmem=$(cat /proc/sys/net/core/rmem_max 2>/dev/null) || [[ ! $_doggy_lio_rmem =~ ^[0-9]+$ ]]; then
    echo 'Cannot read net.core.rmem_max in this environment. Run the live preview from the laptop terminal.'
    exit 1
fi
if (( _doggy_lio_rmem < 8388608 )); then
    echo 'The tested DDS profile needs net.core.rmem_max=8388608. Apply the approved setting before starting.'
    exit 1
fi
export CYCLONEDDS_URI="$_doggy_lio_root/config/point_lio/cyclonedds_live.xml"
export ROS_LOG_DIR="$_doggy_lio_root/reports/point_lio_steps/step_05_output_tf/live_logs"
mkdir -p "${ROS_LOG_DIR}"
exec 9>"$_doggy_lio_root/reports/point_lio_steps/step_05_output_tf/launch.lock"
if ! flock -n 9; then
    echo 'The Point-LIO preview/mapping launch is already running.'
    exit 1
fi
exec ros2 launch "$_doggy_lio_root/launch/point_lio_go2.launch.py" "$@"
