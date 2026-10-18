# Live terrain-aware 2D mapping

The new pipeline uses Point-LIO as the only source of `/odom` and local-body TF.
It builds 5 cm XY occupancy and terrain grids from registered Point-LIO scans,
and opens RViz in a top-down view with no 3D cloud display enabled. The default
view is the user-selected higher-return wall layer; terrain assessment is an
optional display.

## Replay the saved walking session

```bash
cd /home/jain/Doggy
bash scripts/replay_2d.sh
```

This replays the retained 3 m/5 m out-and-back session at its original rate in
localhost-only ROS domain 74. The map grows during playback and remains visible
after playback ends. Close RViz or press Ctrl+C in the launcher to end the
session and save its maps. `rate:=2.0` selects twice the playback speed.

This is replay of **recorded Point-LIO output and measured joint states**, not a rerun of the estimator.
The old recording called its local estimator frame `map`; the replay consistently
calls those same coordinates `odom`. Vendor odometry, vendor world-frame clouds
and the historical `map -> odom` TF are not replayed. The new body adapter builds
the local TF from the recorded IMU poses and the unchanged URDF IMU mount.
Recorded joint states retain their original headers so the self-filter can
interpolate measured leg positions at each scan timestamp.

## Start actual live mapping

Stop older sensor, mapping, IMU and visualization launches first. The new command
starts all required components itself; its preflight refuses existing publishers
on the main mapping topics, and a lock prevents duplicate launches.

Keep the robot stationary for Point-LIO initialization. Use the tested Ethernet
connection on `enp7s0`, DDS domain 0 and laptop Performance power mode.

```bash
cd /home/jain/Doggy
sudo sysctl -w net.core.rmem_max=8388608
bash scripts/start_live_2d.sh
```

Add `rviz:=false` for headless operation. This launch starts no robot movement
controller or command publisher. The shared body-IMU calibration/profile now
includes the independently checked 18 October gyro Z gain and fixed residual
gyro bias, with `init_map_size: 4000`; LiDAR extrinsics remain unchanged. See the
[heading validation report](reports/point_lio_diagnosis_20261018/FINAL_REPORT.md).
The sensor clock comes from calibrated IMU timestamps and does not depend on
vendor odometry or Point-LIO outputs.

`/odom` is `odom / base_link`, with twist expressed at the body origin in body
axes. The adapter accounts for the IMU-to-body pose and velocity lever arms.
TF is `odom -> base_footprint -> base_link -> developer sensor links`. The
footprint's Z=0 is an odometry reference plane, not a measured terrain surface.
No `map -> odom` transform is introduced. The existing upstream mapping mode has
unknown/zero covariance; the adapter does not invent uncertainty estimates.

Older visualization/SLAM launch files remain available as historical tools and
still use vendor odometry. Do not run them alongside the new pipeline.

## Published maps and display

| Topic | Contents |
|---|---|
| `/map` | Occupancy: observed ground free, observed obstacles occupied, unsupported ground unknown |
| `/terrain/costmap` | Separate slope/roughness/height-discontinuity costs; unknown where ground lacks support |
| `/terrain/traversability` | Combined display: obstacles and terrain hazards blocked; ordinary ground free; moderate terrain costs shaded |
| `/walls/map` | Separate 5 cm wall view: selected higher returns occupied, observed ground white, unobserved cells unknown |
| `/terrain/status` | Counts, source age, unmatched scans, bounds/capacity diagnostics and provisional-threshold flag |
| `/terrain/replay_status` | Replay progress and completion; present only during output replay |

All grids have 0.05 m resolution in `odom`, using reliable transient-local
publication. RViz now defaults to `/walls/map`. This layer selects points with
body-frame Z above -0.05 m, matching the approved third diagnostic panel.
The selection follows the measured body orientation and the URDF IMU offset;
it is not a fixed global height cut. Wall cells need two scan observations.
Only observed ground supplies the white background; unknown cells remain unknown.
This view does not include the lower-return obstacle/terrain band and does not
resolve the raw near-field return issue or replace full navigation occupancy.
The optional terrain display retains `/terrain/traversability`. Use
`rviz_config:=/home/jain/Doggy/config/live_2d.rviz` to open the earlier terrain view.
Black means blocked, white ordinary observed
ground, intermediate shades terrain cost, and gray unknown. RViz's built-in
background grid and 3D clouds are disabled. The axes marker shows the robot pose.

## Terrain interpretation and limits

Implementation is NumPy-only; no additional external packages are installed.
Before accumulation, points inside the official URDF collision shapes (body,
head, legs and feet) plus 2.5 cm padding are removed. Shapes are posed using the
matching Point-LIO scan pose and timestamped measured joints. Joint interpolation
requires a bracket no wider than 150 ms and samples no farther than 75 ms from
the scan. After a short wait, a past sample may be held for at most 75 ms at a
recording tail or temporary transport delay. Missing model/joint data causes a
scan to be skipped and reported, rather than accumulated without filtering.
The underlying LowState-to-JointState adapter associates encoder receipt with
the latest available robot sensor timestamp; that association is approximate,
not a calibrated physical encoder acquisition time. Filtering does not change
the raw clouds or the inputs consumed by Point-LIO, and it does not erase a
circular area around the robot. Filter counts and joint-sampling modes appear
in `/terrain/status` and saved map metadata. Returns outside the modeled robot
shapes and residual ground classification can still produce speckles.

Distinct scans must observe a voxel twice before it establishes evidence.
Nearby observed floor returns seed a connected lower surface on a 25 cm terrain
grid. Local plane fits use neighboring terrain cells, with residual rejection.
The 5 cm output cells retain observed ground height, slope, roughness and support
confidence. Obstacles are classified relative to that local surface.

The initial limits in `config/terrain_mapping.yaml` are **provisional**, not
validated Go2 capabilities:

- Slope costs start at 8 degrees; slopes at/above 20 degrees are blocked.
- Roughness costs start at 2 cm plane-fit RMS; 6 cm is blocked.
- Measured neighboring height changes above 12 cm are flagged.
- Obstacle returns are considered from 15 cm to 1 m above local ground.
- Clouds are integrated within 10 m of the IMU, inside an 80 m square centered
  on the estimator origin, with a 600,000-voxel limit. Clipping/capacity loss is
  explicit in `/terrain/status`.

Missing returns do not establish a drop or supported ground. Only observed floor
cells become free; the mapper does not clear a corridor just because a ray passed
above it. This intentionally leaves gaps in sparse observations. Height change
and ground classification are heuristics: this implementation is not foothold
planning, multi-level terrain reconstruction or a validated drop detector.
The accumulated map has no loop closure or dynamic-obstacle clearing. Thin/low
objects below the selected obstacle threshold and unobserved terrain remain
limitations. Point-LIO pose/vertical drift also affects terrain classification.

## Save maps

Each launch defaults to a fresh timestamped directory under `maps/`:

- `map.yaml` and `map.pgm`: occupancy map.
- `map_traversability.yaml` and `.pgm`: thresholded combined display map.
- `map_walls.yaml` and `.pgm`: selected higher-return wall view.
- `map.npz`: full occupancy, terrain costs, traversability, elevation, slope,
  wall view, roughness, confidence, origin, resolution, frame and configuration.
- `map.json`: observation counts, clipping and configuration metadata.

Maps save on orderly shutdown. Save a snapshot while the launch stays open:

```bash
cd /home/jain/Doggy
source scripts/setup_ros.sh
# For replay only (omit this block for live robot mapping):
export ROS_DOMAIN_ID=74 ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="$PWD/config/cyclonedds_replay.xml"
ros2 service call /terrain/save_map std_srvs/srv/Trigger '{}'
```

The service response gives the saved path. `output_map:=/absolute/path/map`
overrides the file stem. Repeated snapshot saves within one run replace that
run's files. PGM/YAML use trinary thresholds; use the NPZ to retain exact terrain
costs and heights. A saved map's coordinate frame remains the run's local odom
frame; localization for a subsequent run is separate integration work.

## Verification

```bash
python3 -B -m unittest discover -s tests -p test_terrain_grid.py -v
python3 -B -m unittest discover -s tests -p test_robot_self_filter.py -v
source scripts/setup_ros.sh
OPENBLAS_NUM_THREADS=1 python3 -B scripts/validate_terrain_recording.py \
  --bag reports/point_lio_steps/step_07_live_test/accuracy_20261016T224027+1100/bag \
  --output reports/live_2d/offline_validation/map
```

The synthetic cases check flat/gentle/steep terrain, walls versus overhead returns,
measured steps, unknown gaps, isolated outliers, saved layers and body-twist lever
arms. Full-session offline results are in
`reports/live_2d/offline_validation/validation.json`. Live ROS replay transport,
map updates, exclusive odometry publisher and TF parents are recorded in
`reports/live_2d/replay_verification.json`. Synthetic and replay checks do not
establish ground-truth terrain accuracy or autonomous navigation readiness.

The self-filter comparison for the longer drive is in
`reports/live_2d/self_filter_longer/validation.json`. All 2,074 scans had bounded
joint interpolation; 2,071 returns inside robot collision volumes were removed.
In the first 27 seconds, occupied cells within 1 m of the starting origin changed
from 26 to 24. This modest change shows that self-returns explain only part of the
reported nearby markings; it is not evidence that all ground speckles are fixed.
