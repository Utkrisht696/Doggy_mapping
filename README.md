# Go2 Pro X mapping workspace

Validated on Ubuntu 22.04 / ROS 2 Humble with a Unitree Go2 Pro X over Ethernet.
This repository preserves the standalone Point-LIO integration, calibrated body-IMU adapter, developer-URDF TF setup, diagnostic tools and mapping results. The heading calibration/profile was updated on 18 October 2026. No Nav2 movement bridge or autonomous mission controller is implemented yet.

The selected inputs are raw `/utlidar/cloud` and `/doggy/imu/body_calibrated`, derived from `/sportmodestate`. The internal LiDAR accelerometer remains unused. The robot uses DDS domain 0 through `enp7s0`; scripts and profiles are specific to the original laptop.

## Live 2D terrain mapping

The new [live 2D mapping pipeline](LIVE_2D_MAPPING.md) publishes 5 cm occupancy and
terrain grids and shows a top-down map in RViz. Run `bash scripts/replay_2d.sh` to
view the recorded walking session, or `bash scripts/start_live_2d.sh` for the
complete live setup. The current solution permanently fuses vendor height with
Point-LIO X/Y and orientation for `/odom`, registered clouds and local TF.
Sensor-time clocking remains independent of vendor odometry. Terrain thresholds are
provisional; no Nav2 movement controller is started. The older mapping launch
described below also uses height-fused Point-LIO output, with the historical
vendor-odom/map-correction TF arrangement.

## Permanent height fusion — 20 October 2026

Both current mapping launches start `scripts/publish_height_fusion.py` as an
essential component; no enable flag is needed. Point-LIO publishes diagnostic
outputs under `/point_lio/raw/` in `lio_local`. The height filter publishes the
canonical `/point_lio/odometry`, `/point_lio/cloud_registered` and `/point_lio/path`.
The 2D mapper, `/odom` adapter and TF consume those corrected outputs. The complete
2D pipeline uses `odom`; the legacy 3D launch uses `map`.

The unchanged scalar filter uses past `/utlidar/robot_odom` height, converted to
the developer IMU origin, with at most 50 ms source age and one startup alignment.
X/Y, quaternion and source stamps are preserved. Every cloud receives exactly
the same Z shift as its matching pose. Missing/rejected vendor samples propagate
Point-LIO height increments and increase uncertainty; status reports the fallback.
The path display is sampled at no more than 5 Hz, published at most twice per
second and bounded to 6,000 poses. Odometry/clouds retain their input rate.

Settings are in `config/point_lio/height_fusion.yaml`; diagnostics are on
`/point_lio/height_status`. The filter runs outside Point-LIO and does not correct
its internal scan-matching map, horizontal drift or heading. Upstream twist is
retained; the configured height uncertainty is provisional and full fused
velocity/covariance is not a validated navigation-state estimator. Vendor height
has not been shown to measure world elevation over ramps/drains. Making this the
current mapping solution does not establish that terrain-height limitation as fixed.

Recorded-output replay also runs height fusion and requires recorded vendor
odometry. New fused recordings must retain `/point_lio/raw/odometry` and
`/point_lio/raw/cloud_registered`, preventing repeated correction of fused data.
Use normal-speed replay for accuracy checks: accelerated tests exposed scan loss
after receipt pauses and inconsistent trajectories. See the
[multi-bag findings](reports/bag_comparisons/return_drift_20261018_112353/height_fusion_test/batch_20261019/REPORT.md)
and [production integration verification](reports/height_fusion_20261020/REPORT.md).

## Heading calibration update — 18 October 2026

The current adapter applies measured body gyro Z gain **0.9785884535708254** after the existing fixed bias subtraction. Independent quiet-wall checks on a separate CW/CCW recording reduced angular-rate integration error from 7–8 degrees per turn to less than 0.5 degrees on three accepted turns. The vendor fused quaternion is retained for inspection but marked unavailable; Point-LIO uses calibrated acceleration and angular velocity.

The shared Point-LIO profile now uses gyro measurement variance `0.0001`, zero additional gyro-bias prior/process variance and `init_map_size: 4000`. A minimal local upstream patch clears initialization batches correctly, sizes indexed point caches and exposes the bias-prior parameter. See the [heading report](reports/point_lio_diagnosis_20261018/FINAL_REPORT.md) and [replay comparison](reports/point_lio_diagnosis_20261018/final_validation/route_comparison.png).

With one fixed profile, the measured 5/10 m plus rectangle recording's estimated return offset improved from **4.26 m to 2.0 cm**. The soft-turn and small-wander recordings still have approximately **0.74 m and 0.62 m** offsets, respectively. These are offline replay results; approximate physical returns, cloud-registration reference uncertainty and residual position drift limit accuracy claims. No loop closure or vendor position scaling is applied. Physical acquisition-time offset remains uncalibrated at zero.

## Earlier short-distance live result — 16 October 2026

| Operator-reported outbound distance | Point-LIO displacement | Vendor displacement |
|---|---:|---:|
| 3 m | 2.934 m | 2.443 m |
| 5 m | 5.028 m | 4.226 m |

After both out-and-back trips, the estimated horizontal offset from the original starting hold was **3.9 cm for Point-LIO**, versus **19.0 cm for vendor odometry**. Starting-floor cells repeated with median absolute height differences of approximately 3–5 mm. Distances are operator-reported, paths were curved, and endpoint placement was not surveyed; these are test results, not absolute accuracy specifications.

Read the [latest test report](reports/point_lio_steps/step_07_live_test/accuracy_20261016T224027+1100/analysis/REPORT.md), [comparison plot](reports/point_lio_steps/step_07_live_test/accuracy_20261016T224027+1100/analysis/distance_comparison.png), and [longer-drive drift review](reports/point_lio_steps/step_07_live_test/drift_review/REPORT.md).

Those historical trials used **`init_map_size: 4000`**, supplied by their retained trial launch, and gyro measurement variance 0.1. The current shared YAML uses the heading-calibrated profile described above, so a new replay will differ from the historical outputs. Noise values are not a full measured noise calibration.

## Repository contents and restoration

See [REPRODUCING.md](REPRODUCING.md) for pinned dependencies, the Unitree message patch, tested launch commands and laptop prerequisites. SDKs and third-party sources are Git submodules at the recorded commits; [dependencies.repos](dependencies.repos) records the same revisions.

```bash
git clone --recurse-submodules https://github.com/Utkrisht696/Doggy_mapping.git
cd Doggy_mapping
```

The full bags, NumPy sensor captures, build trees, virtual environment, extracted packages and download caches remain on the original laptop and are excluded from Git. Report metrics, analysis scripts, plots, saved interactive map views and one 5 cm voxel point cloud are included. [Recording manifest](reports/recordings_manifest.json).

Historical setup documents preserve the investigation and may describe earlier paused states. For the current verified configuration, start with this README and REPRODUCING.md. Calibration is specific to this robot.

The historical mapping launch used for these recorded results has TF `map -> odom -> base_footprint -> base_link`, then the developer links including `imu` and `radar -> utlidar_lidar`. The source developer URDF is unchanged. In that older launch, vendor odometry owns the local odom/body chain and the Point-LIO bridge supplies `map -> odom`. The current live 2D pipeline derives `/odom` and local-body TF from Point-LIO X/Y/orientation and the height-fused Z output.

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
