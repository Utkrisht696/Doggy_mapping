# Point-LIO and SLAM Toolbox test readiness

**Current status:** mapping is paused after poor map quality and shifting pose.
Read [MAPPING_PREREQUISITES.md](MAPPING_PREREQUISITES.md) for the subsequent
sensor-only audit and prepared clean streams. The startup results below do not
establish mapping quality; the SLAM launch has not been retested with the clean
streams. Do not use this earlier startup check as proof that mapping inputs are
fully ready.

Checked on the connected Go2 Pro X over Ethernet on 12 October 2026, including a
fresh check after the battery swap. Ubuntu 22.04 / ROS 2 Humble, DDS domain 0.

| Item | Result |
|---|---|
| Raw LiDAR, body cloud, deskewed cloud | Live, approximately 15.4 Hz |
| LiDAR IMU | Live, approximately 248 Hz |
| Robot odometry | Live, approximately 149 Hz; `odom` → `base_link` |
| Official Unitree URDF and meshes | Installed; URDF matches the developer source byte for byte |
| SLAM Toolbox and cloud-to-scan conversion | Installed; live stationary startup check passed |
| Standalone ROS 2 Point-LIO | See [POINT_LIO_SETUP.md](POINT_LIO_SETUP.md); earlier CMU startup evidence is historical |
| Point-LIO sensor measurements | **Acceleration semantics/calibration unresolved; standalone sensor profile not yet configured** |
| Physical LiDAR model | **L1 likely; not conclusively identified** |

All temporary tests have been stopped. RViz is optional for SLAM Toolbox; the
mapping launchers start no motion controller, navigation stack or IMU calibration.

## Test SLAM Toolbox

```bash
cd /home/jain/Doggy
bash scripts/test_slam.sh slam_toolbox
```

Leave that terminal running; Ctrl+C stops the launch.

To open the map, live scan and official robot model in RViz with the mapper:

```bash
bash scripts/test_slam.sh slam_toolbox rviz:=true
```

Use this instead of the headless command. The view uses fixed frame `map`, the
robot-derived clock and the test topics. Closing RViz leaves the mapper running;
Ctrl+C in the launch terminal stops everything. Move the dog with your normal
manual controller to collect observations; the script does not drive it.

To check outputs in another terminal:

```bash
cd /home/jain/Doggy
source scripts/setup_ros.sh
python3 scripts/check_slam_inputs.py --seconds 15 --with-mapping --output reports/slam/manual_toolbox_check.json
```

Input is `/utlidar/cloud_deskewed`, expressed in `odom`. The scan converter uses
the reported robot odometry to transform it into `base_link`, filters heights
from -0.15 to 1.5 m relative to the body, and produces a 360-bin scan over 0.4–12 m.
The lower height threshold may need adjustment for the robot's posture and room.

Outputs are `/tests/slam_toolbox/scan`, `/tests/slam_toolbox/map`, and TF
`map` → `odom` → `base_footprint` → `base_link` → the developer URDF links.
The shared TF adapter now preserves reported odometry through a dynamic footprint;
the runtime developer root name is normalized to `base_link`. See
[TF_CONSISTENCY.md](TF_CONSISTENCY.md). This shared TF change has not been tested
by running SLAM; all robot geometry comes from Unitree.
The removed fitted sensor TF is absent; there is no guessed raw-LiDAR frame alias.

The final 15-second stationary check received 232 scans and 16 map messages.
The last map was 106 × 158 cells at 0.05 m resolution, with 214 known cells and
22 occupied cells. TF lookup at the scan timestamp passed. Valid scan returns
ranged from 14 to 37 out of 360 bins, so coverage is sparse. This establishes
startup readiness; a walk with room features is still needed to assess map
quality, drift, loop closure, and obstacle coverage. The profile lets the first
observation enter the map (`min_pass_through: 1`).

## Save the SLAM Toolbox map

While the mapper is still running, use another terminal:

```bash
cd /home/jain/Doggy
source scripts/setup_ros.sh
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -t /tests/slam_toolbox/map -f maps/my_map
```

This writes a map image and YAML for later localization/navigation. Choose a new
name for each map. It saves an occupancy map, not the SLAM pose graph required
for continuing the same mapping session.

## Standalone Point-LIO

The previous CMU Point-LIO packages, preprocessing, calibration tools and launch
branch were removed on 16 October 2026 at the user's request. The old calibration
file and commands no longer apply. Read [POINT_LIO_SETUP.md](POINT_LIO_SETUP.md)
for the standalone package state and the prerequisites before its first run.

## Timing and reproducibility

Robot headers are about 787 seconds behind the laptop clock. This is a clock
offset observation, not a measured network delay. Mapping, scan conversion and
robot-state publishing use the robot-derived `/doggy/robot_clock`, with their
`/clock` remapped to it. Source sensor timestamps and the laptop clock are retained.

Run one launcher at a time. Mapping and official visualization share a lock so
they cannot both own the robot odometry/clock publisher. For a fair comparison,
use the same route and environment; SLAM Toolbox relies on the dog's existing
odometry/deskewing, whereas Point-LIO estimates motion from the raw sensor stream.

`bash scripts/rebuild.sh` uses the cached archives and sources on this laptop,
including the added PCL dependency and local Python transform library. No sudo
system installation was needed. This still depends on the existing Humble/PCL
installation and is not a standalone image for another machine.

Evidence: `reports/slam/post_battery_sensors.json`, `software_check.json`,
`point_time_fields.json`, `point_lio_build.log`, `point_lio_startup_check.log`,
`point_lio_calibration_gate.log`, and `slam_toolbox_smoke_check.json`.
