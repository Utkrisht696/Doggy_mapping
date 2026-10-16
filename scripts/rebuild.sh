#!/usr/bin/env bash
# Rebuild from the sources and package cache already in this workspace.
set -eo pipefail
_doggy_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$_doggy_root"
mkdir -p local/system reports
for _doggy_deb in downloads/debs/*.deb; do
    dpkg-deb -x "$_doggy_deb" local/system
done
if [[ ! -x .venv/bin/python ]]; then
    python3 -m venv .venv
fi
env -u PYTHONPATH .venv/bin/python -m pip install --no-index --find-links downloads/wheels -r requirements-sdk.txt setuptools wheel
env -u PYTHONPATH .venv/bin/python -m pip install --no-index --no-build-isolation --no-deps -e sdk/unitree_sdk2_python
source scripts/setup_ros.sh
colcon build --base-paths ros2_ws/src --build-base ros2_ws/build --install-base ros2_ws/install --executor sequential --cmake-force-configure --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
cmake -S sdk/unitree_sdk2 -B sdk/unitree_sdk2/build -DCMAKE_INSTALL_PREFIX="$_doggy_root/local" -DBUILD_EXAMPLES=ON
cmake --build sdk/unitree_sdk2/build --target test_subscriber -j2
cmake --install sdk/unitree_sdk2/build
cmake -S sdk/unitree_slam -B sdk/unitree_slam/build
cmake --build sdk/unitree_slam/build -j2
for _doggy_lidar in unilidar_sdk unilidar_sdk2; do
    cmake -S "sdk/$_doggy_lidar/unitree_lidar_sdk" -B "sdk/$_doggy_lidar/unitree_lidar_sdk/build"
    cmake --build "sdk/$_doggy_lidar/unitree_lidar_sdk/build" --target example_lidar_udp -j2
done
echo "Build complete. Source scripts/setup_ros.sh in your terminal."
