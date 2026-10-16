#!/usr/bin/env bash
# Source this file. Keep system ROS and the workspace-local dependencies together.
_doggy_setup_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
_doggy_setup_ros="$_doggy_setup_root/local/system/opt/ros/humble"
export AMENT_PREFIX_PATH="$_doggy_setup_ros:${AMENT_PREFIX_PATH:-}"
export CMAKE_PREFIX_PATH="$_doggy_setup_ros:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$_doggy_setup_ros/lib:$_doggy_setup_ros/lib/x86_64-linux-gnu:$_doggy_setup_root/local/system/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$_doggy_setup_root/local/python:$_doggy_setup_ros/local/lib/python3.10/dist-packages:$_doggy_setup_ros/lib/python3.10/site-packages:${PYTHONPATH:-}"
export PATH="$_doggy_setup_ros/bin:$PATH"
if [[ -f "$_doggy_setup_root/ros2_ws/install/local_setup.bash" ]]; then
    source "$_doggy_setup_root/ros2_ws/install/local_setup.bash"
fi
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
# Set DOGGY_RMW=rmw_fastrtps_cpp to use the middleware initially discovered here.
export RMW_IMPLEMENTATION="${DOGGY_RMW:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="$_doggy_setup_root/config/cyclonedds.xml"
unset _doggy_setup_root _doggy_setup_ros
