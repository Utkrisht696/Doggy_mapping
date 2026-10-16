# TF contract — 16 October 2026

The user approved normalization to one `base_link` and addition of a dynamic
`base_footprint`. The active body/model tree is:

```text
odom
└── base_footprint
    └── base_link
        ├── the developer leg links
        ├── front_camera
        ├── imu       (the body IMU mount)
        └── radar     (the developer LiDAR mount)
            └── utlidar_lidar (approved DDS measurement-frame conversion)
```

There is no `base` alias. The user approved the topic-derived raw-cloud edge
`radar -> utlidar_lidar`; it is applied and live-verified. The invalid, unused
internal LiDAR-IMU frame `utlidar_imu` remains unconnected. The selected calibrated
BODY IMU publishes in the existing developer `imu` frame.

## Geometry and ownership

`scripts/go2_frames.py` generates the ROS description from the installed developer
URDF. It renames only root link `base` and joint references to that root to
`base_link`. Mesh paths, joint names, joint origins, sensor origins, inertias and
all original numeric geometry remain unchanged. It then appends the approved
measurement link/joint from `config/point_lio/sensor_geometry.json`. This additional
edge expresses vendor stream coordinates; it is not surveyed physical mount data. The downloaded and installed original
URDF files remain unchanged. Original SHA256:

```text
7d19fe48e2e689ee1a032ab99f2a4a8b671d87e73de48d3e65811682a5b48b9e
```

All three local launchers use this same helper: visualization, sensor preparation
and the previously prepared mapping test launch. No mapping algorithm was started
or tuned as part of this TF change.

| Edge / output | Owner |
|---|---|
| `odom → base_footprint` | `doggy_go2_state`, dynamic |
| `base_footprint → base_link` | `doggy_go2_state`, dynamic, same source stamp |
| Body → all developer model links | `go2_robot_state_publisher`, from normalized-root developer description and joint states |
| `radar → utlidar_lidar` | `go2_robot_state_publisher`, fixed runtime measurement joint |
| `/odom` | `doggy_go2_state`, unchanged vendor odometry message |
| `/joint_states` | `doggy_go2_state`, 12 measured encoder joints |
| `/doggy/robot_clock` | `doggy_go2_state`, robot odometry time |

Two nodes legitimately publish `/tf`: they own different edges. Only the model
publisher publishes `/tf_static`. No child has multiple parents. There is one
robot-clock owner. No placeholder `map` transform is added.

## Footprint definition

The footprint is the body XY position and yaw projected onto the horizontal
reference plane `odom.z = footprint_ground_z`. The current value is **0.0 m**.
Its roll and pitch are zero. This is a planar navigation reference, not a terrain
height estimator. On uneven terrain or a different odometry datum, its ground
reference must be reviewed.

The user confirmed standing on level floor for verification. Nearby deskewed
returns clustered close to odom Z=0, with a dominant 1 cm bin centered at -0.025 m
and substantial centimeter-scale spread. We retained the existing zero reference
rather than fitting a new height correction. This check does not claim a surveyed
ground height; its observations are saved in `reports/tf/ground_reference_check.json`.

Body height, roll and pitch remain in `base_footprint → base_link`. The product of
the two transforms equals the original full vendor body pose. A fixed nominal
robot height is not used, and the pose is not flattened or filtered. `/odom`
continues to describe `base_link` in `odom`; the intermediate TF frame does not
change the odometry message's meaning.

All launchers expose `footprint_ground_z` and pass it to the same state adapter.
For example, the current setup is:

```bash
bash scripts/visualize_lidar.sh footprint_ground_z:=0.0
```

Sensor timestamps are retained. RSP and RViz follow `/doggy/robot_clock` through
their `/clock` remap. Encoder timing remains visualization-level association with
the latest odometry stamp because LowState has no epoch header.

## Validation

`tests/test_go2_frames.py` checks 300 arbitrary 3D body poses, heading wrap,
non-unit quaternions, reference-plane heights and invalid inputs. Independent
rotation-matrix composition confirms full physical pose preservation. A structural
URDF comparison confirms that removing the added measurement edge and reversing
the root rename recovers every original developer element. The stored Point-LIO
extrinsics and runtime TF chain also agree numerically.

`scripts/check_lidar_tf.py` now checks the canonical description, all 43 runtime model
links, the 12 real joints, the expected footprint/body topology, original-stamp
raw/body/deskewed-cloud transforms and composed TF against matched vendor odometry.
Graph inspections are throttled so validation does not starve high-rate callbacks.

Live evidence:

- `reports/tf/canonical_tf_check.json`: all links/joints and cloud transforms;
  five matched source-pose comparisons, maximum position error ~1.1e-16 m and
  quaternion error ~3.3e-16 in the initial passing check.
- `reports/tf/canonical_tree_audit.json`: live ownership, parent uniqueness,
  planar footprint and absence of the legacy body alias or mapping/calibration nodes.
- `reports/tf/canonical_launch.log`: running visualization/model/state launch.

To repeat the TF check:

```bash
source scripts/setup_ros.sh
python3 scripts/check_lidar_tf.py --output reports/tf/current_tf_check.json
```

## Approved raw sensor connection

`config/point_lio/sensor_geometry.json` stores the exact reviewed transform and
its provenance. The developer `radar` origin is unchanged; the appended measurement
edge has XYZ `[-0.005152688451, 0.000000019947, -0.047108095577]` m and RPY
`[-0.001508482628, -0.000981514429, -2.146758634683]` rad. Consequently
`base_link -> utlidar_lidar` has translation approximately `[0.282160, 0, 0]` m.

The actual live TF reproduced 19,012 fresh corresponding raw/body cloud points
within 2.339e-6 m (float32 rounding). This is numerical coordinate agreement,
not physical placement accuracy. Body-cloud coordinates are assumed to correspond
to the developer body frame; no independent physical survey is available.

Current evidence is in `reports/point_lio_steps/step_02_geometry/`:
`applied_tf_check.json`, `applied_geometry_validation.json` and `REPORT.md`.
All 45 frames including odom/footprint are present; observed parents are unique.
The user independently confirmed both requested sensor TF lookups; step 2 is complete.
The estimator numeric extrinsics are stored but not yet wired into a Go2 launch.
