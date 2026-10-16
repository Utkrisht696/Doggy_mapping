# Go2 Pro X mapping workspace

Validated on Ubuntu 22.04 / ROS 2 Humble with a Unitree Go2 Pro X over Ethernet.
This repository preserves the standalone Point-LIO integration, calibrated body-IMU adapter, developer-URDF TF setup, diagnostic tools and mapping results from 16 October 2026. No Nav2 movement bridge or autonomous mission controller is implemented yet.

The selected inputs are raw `/utlidar/cloud` and `/doggy/imu/body_calibrated`, derived from `/sportmodestate`. The internal LiDAR accelerometer remains unused. The robot uses DDS domain 0 through `enp7s0`; scripts and profiles are specific to the original laptop.

## Latest verified result

| Operator-reported outbound distance | Point-LIO displacement | Vendor displacement |
|---|---:|---:|
| 3 m | 2.934 m | 2.443 m |
| 5 m | 5.028 m | 4.226 m |

After both out-and-back trips, the estimated horizontal offset from the original starting hold was **3.9 cm for Point-LIO**, versus **19.0 cm for vendor odometry**. Starting-floor cells repeated with median absolute height differences of approximately 3–5 mm. Distances are operator-reported, paths were curved, and endpoint placement was not surveyed; these are test results, not absolute accuracy specifications.

Read the [latest test report](reports/point_lio_steps/step_07_live_test/accuracy_20261016T224027+1100/analysis/REPORT.md), [comparison plot](reports/point_lio_steps/step_07_live_test/accuracy_20261016T224027+1100/analysis/distance_comparison.png), and [longer-drive drift review](reports/point_lio_steps/step_07_live_test/drift_review/REPORT.md).

The successful trials used **`init_map_size: 4000`**, supplied by the retained trial launch; the base YAML still contains the earlier value of 10. The original gyro measurement variance of 0.1 was retained. Acquisition-time offset remains provisionally zero, and noise values are not a full measured noise calibration.

## Repository contents and restoration

See [REPRODUCING.md](REPRODUCING.md) for pinned dependencies, the Unitree message patch, tested launch commands and laptop prerequisites. SDKs and third-party sources are Git submodules at the recorded commits; [dependencies.repos](dependencies.repos) records the same revisions.

```bash
git clone --recurse-submodules https://github.com/Utkrisht696/Doggy_mapping.git
cd Doggy_mapping
```

The full bags, NumPy sensor captures, build trees, virtual environment, extracted packages and download caches remain on the original laptop and are excluded from Git. Report metrics, analysis scripts, plots, saved interactive map views and one 5 cm voxel point cloud are included. [Recording manifest](reports/recordings_manifest.json).

Historical setup documents preserve the investigation and may describe earlier paused states. For the current verified configuration, start with this README and REPRODUCING.md. Calibration is specific to this robot.

The current TF chain during mapping is `map -> odom -> base_footprint -> base_link`, then the developer links including `imu` and `radar -> utlidar_lidar`. The source developer URDF is unchanged. Vendor odometry still owns the local odom/body chain; the Point-LIO bridge supplies `map -> odom`. Moving Point-LIO into Nav2's local odometry role remains future work.

## Earlier setup notes

For the official Unitree URDF, live joint states and RViz setup, see [LIDAR_VISUALIZATION.md](LIDAR_VISUALIZATION.md):

```bash
cd /home/jain/Doggy
bash scripts/visualize_lidar.sh
```

## Start a development terminal

```bash
cd /home/jain/Doggy
source scripts/setup_ros.sh
ros2 topic list -t --no-daemon --spin-time 5
ros2 topic echo /lf/sportmodestate --once --qos-reliability best_effort
```

`setup_ros.sh` loads the existing system Humble installation, locally extracted ROS
packages, custom message overlay, and Cyclone DDS bound to `enp7s0`. It does not start
a controller. Use `DOGGY_RMW=rmw_fastrtps_cpp source scripts/setup_ros.sh` to select the
initially working Fast DDS implementation. For SDK Python code, use `.venv/bin/python`;
for ROS tools and the scan, use system `python3`. The SDK environment is isolated and
its runtime dependencies are pinned in `requirements-sdk.txt`.

## Repeat the passive scan

```bash
python3 scripts/scan_topics.py --duration 45 --output reports/new_scan
```

The script inventories types, endpoint counts and QoS, then receives data without
command publishers or service calls. It saves JSON samples and CSV rates. Large
arrays are summarized, and at most three samples per topic are retained. Rates are
observed receipt rates. Use repeatable `--only /topic/name` arguments for a focused
measurement; topics outside that selection are inventoried but not sampled.

## What is here

| Path | Contents and readiness |
|---|---|
| `sdk/unitree_sdk2` | Official C++ SDK; libraries/headers installed to `local`; subscriber example compiled |
| `sdk/unitree_sdk2_python` | Official Python SDK; installed in `.venv`; native state reception verified |
| `sdk/unitree_ros2` | Official ROS 2 source; Go2 and API messages built in `ros2_ws` |
| `sdk/unitree_slam` | Official SLAM/waypoint examples; C++ executables compiled, navigation commands not executed |
| `sdk/unilidar_sdk`, `sdk/unilidar_sdk2` | Official L1/L2 SDK sources and compiled receiver examples; exact installed LiDAR model not established |
| `sdk/unitree_model/Go2` | Official Go2 USD assets |
| `sdk/unitree_ros/robots/go2_description` | Official Go2 developer URDF, DAE/STL meshes and xacros; unchanged source |
| `third_party` | Humble sources for Nav2, pointcloud_to_laserscan, robot_localization, and standalone ROS 2 Point-LIO |
| `local/system` | Extracted dependency packages including Nav2, Cyclone DDS, PCL ROS, scan conversion and localization |
| `downloads` | Debian archives and Python dependency wheels for rebuilding on this laptop |
| `ros2_ws/src` | Unitree messages, official description wrapper, standalone Point-LIO (package `point_lio`) |
| `reports` | Scan evidence, source commits, package hashes, Python lock, build logs and local patches |

No sudo installation was performed. The local ROS package setup was checked for
package discovery, shared-library dependencies and successful Cyclone DDS discovery.
The environment still relies on this laptop's existing Ubuntu/ROS installation and
is not a portable image for arbitrary computers. Upstream source is recorded at exact
commits in `reports/download_manifest.json`; shallow clones omit older Git history.

`unitree_ros2` has one local addition: `VoxelMapCompressed.msg`, reconstructed from
the official SDK's C++ header, and its CMake registration. The message decoded live
voxel data successfully. Other SLAM messages are copied from the official source;
`PcToImage` is reconstructed from its generated header. See
`ros2_ws/src/MESSAGE_PROVENANCE.md` and `reports/patches`.

To rebuild from the local cache on this laptop:

```bash
bash scripts/rebuild.sh
```

The repositories include examples that move the robot or reconfigure sensors.
Compilation, discovery, passive subscribers and headless mapping startup tests were used during setup.
No velocity bridge or mission controller was started. The optional
visualization launcher starts the official URDF robot_state_publisher, read-only joint/odometry adapter, a dedicated robot-time clock, and RViz.
