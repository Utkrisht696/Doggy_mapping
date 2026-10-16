# Restoring and running this workspace snapshot

This is a source/configuration and evidence snapshot of the existing laptop setup. It is not a portable system image or an installer for a fresh computer. Existing runtime scripts and calibration were preserved unchanged for this publication. Several launches and report utilities contain `/home/jain/Doggy` paths; the tested commands below assume that location. Calibration values belong to the tested Go2 Pro X.

## Source dependencies

Clone with `--recurse-submodules`, or run `git submodule update --init --recursive`. All 12 current SDK/third-party repositories are pinned by Git links and listed independently in `dependencies.repos`. The older CMU stack is retired and is not an active dependency. `reports/download_manifest.json` is historical provenance and contains retired entries.

The original Unitree ROS 2 checkout has one intentional local patch. After a fresh clone, apply it once:

```bash
git -C sdk/unitree_ros2 apply ../../reports/patches/unitree_ros2_voxel.patch
cp reports/patches/VoxelMapCompressed.msg sdk/unitree_ros2/cyclonedds_ws/src/unitree/unitree_go/msg/
```

On the original laptop that patch is already applied. Do not apply it twice. The custom message definitions and official description wrapper in `ros2_ws/src` are tracked, including relative symlinks to their pinned dependencies. Upstream SDK code is retained under its original licenses.

## Original laptop prerequisites

- Ubuntu 22.04, ROS 2 Humble, system Python 3.10 and colcon.
- Locally built Unitree message overlay, developer description package and Point-LIO.
- Original `local/system` extracted ROS dependencies, `.venv`, compiled SDK libraries and cached packages. These are not in Git. Package versions/hashes are recorded in `reports/download_manifest.json`, with Python SDK pins in `requirements-sdk.txt`.
- `scripts/rebuild.sh` is the original cache-based rebuild script: it needs the excluded `downloads/debs` and `downloads/wheels` caches. It is not an online bootstrap installer.
- Ethernet interface `enp7s0`, DDS domain 0, the tested CycloneDDS profile, and Performance power mode.
- Before the tested large-buffer profile, the operator applies `sudo sysctl -w net.core.rmem_max=8388608`. That kernel setting is temporary until reboot.

Run only one sensor/TF bringup and one calibrated-IMU adapter. The commands below are for restarting a stopped setup; do not duplicate already-running processes.

## Sensor bringup and calibrated body IMU

In a terminal on the original laptop:

```bash
cd /home/jain/Doggy
source scripts/setup_ros.sh
export CYCLONEDDS_URI="$PWD/config/point_lio/cyclonedds_live.xml"
ros2 launch "$PWD/launch/mapping_sensors.launch.py"
```

In another terminal:

```bash
cd /home/jain/Doggy
source scripts/setup_ros.sh
export CYCLONEDDS_URI="$PWD/config/point_lio/cyclonedds_live.xml"
python3 -u scripts/publish_body_imu.py --calibration-file config/point_lio/body_imu_calibration.json
```

This uses the body IMU from SportModeState, preserves source timestamps, subtracts the approved fixed accelerometer/gyro biases, and publishes `/doggy/imu/body_calibrated` in developer frame `imu`. It does not publish TF or send robot commands. The original LiDAR IMU is not repaired or selected.

## Exact successful mapping trial

Keep the robot stationary for initialization. The successful test uses the retained 4000-point override, rather than the base profile's older value of 10:

```bash
cd /home/jain/Doggy
source scripts/setup_ros.sh
export CYCLONEDDS_URI="$PWD/config/point_lio/cyclonedds_live.xml"
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1
unset GTK_PATH GTK_EXE_PREFIX GTK_DATA_PREFIX GTK_IM_MODULE_FILE GIO_MODULE_DIR QT_PLUGIN_PATH QT_QPA_PLATFORM_PLUGIN_PATH LD_PRELOAD
ros2 launch "$PWD/reports/point_lio_steps/step_07_live_test/accuracy_20261016T224027+1100/live_mapping.launch.py" start_mapping:=true rviz:=true
```

This launch starts Point-LIO, its map/odom TF bridge and RViz. Closing one launch component closes the launch. It does not start sensors, the IMU adapter, a rosbag recorder or any robot motion controller. The retained supervisor scripts document the recorded tests; they use per-run pointers/PID files and are not general launch wrappers.

`scripts/start_point_lio.sh` remains the original preview/mapping wrapper. Its default is preview, and its base profile does not automatically select the successful trial override. No default application parameter was changed solely for publication.

## What was calibrated and checked

- Fixed body accelerometer bias from five held poses, gyro bias from a separate quiet baseline; not full scale, misalignment, temperature or dynamic calibration.
- LiDAR-to-body-IMU extrinsics from vendor raw/body point correspondences composed with the developer mount. This is topic-derived geometry, not a surveyed physical mount.
- Original source clock and per-point times preserved; sample coverage, adapter delivery and TF composition checked. Physical LiDAR/cloud acquisition lag was not conclusively calibrated; configured offset remains zero.
- Reliable adapter subscription, larger DDS receive buffers and user-selected Performance mode removed the tested delivery backlog. Larger startup map improved initialization.
- Incoming covariance remains unknown; Point-LIO uses its configured noise values. Upstream source remains unchanged.

## Data and evidence

Bags remain under their original `reports/.../bag` paths on the laptop. `reports/recordings_manifest.json` records file sizes, SHA-256 hashes and topic counts. Raw recordings, NPZ arrays, full accumulated PCDs and build/download caches are excluded from Git. Some historical analysis scripts require these excluded captures to run.

Included map artifacts are in `reports/point_lio_steps/step_07_live_test/saved_maps`: the interactive HTML files, PNG previews, trajectory CSVs and the drive's 5 cm voxel PCD. The latest two-distance test report is under `reports/point_lio_steps/step_07_live_test/accuracy_20261016T224027+1100/analysis`.

Nav2 dependencies are present in the original laptop cache, but robot-specific costmaps, saved-map localization, a Point-LIO local-odom adapter and a tested velocity-command bridge still need integration. This snapshot does not enable autonomous motion.
