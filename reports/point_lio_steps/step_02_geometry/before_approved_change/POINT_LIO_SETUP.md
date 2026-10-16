# Standalone Point-LIO readiness — 16 October 2026

The user selected standalone Point-LIO and authorized removal of CMU and its
associated tooling. Mapping remains paused until the input profile is reviewed.

**Step 1 update:** the user approved a calibrated body-IMU route. The passive
adapter is running and its offline/live stationary checks passed. The user
checked standing output and a subsequent standing capture passed; step 1 is closed.
Step 2 geometry is under review, with developer TF restored and raw LiDAR TF
proposal pending approval. Output is `/doggy/imu/body_calibrated` in developer
frame `imu`, sourced from timestamped `/sportmodestate`. The raw LiDAR IMU remains
unchanged and unvalidated. See [step 1 report](reports/point_lio_steps/step_01_accelerometer/REPORT.md)
and [workflow state](reports/point_lio_steps/STATE.json).

## Selected implementation

- Source: https://github.com/dfloreaa/point_lio_ros2
- Commit: `a8e2d0d5090af97ead8dd4fac3d37cf3dbb33ff7`.
- Checkout: `third_party/point_lio_ros2`, linked as `ros2_ws/src/point_lio`.
- ROS package: `point_lio`; executable: `pointlio_mapping`.
- This is a community ROS 2 port with UniLiDAR L1/L2 support. The original
  HKU-MARS Point-LIO repository targets ROS 1; it is not a drop-in Humble package.
- Upstream algorithm, launch files and sensor configurations are unmodified.
- Build and discovery results are recorded in
  `reports/point_lio_migration_20261016/verification.json` and `build.log`.

**Software checks passed:** the package builds and installs on this laptop, all
shared libraries resolve, declared ROS dependencies are discoverable, and both
the standalone and retained Toolbox launch files pass argument introspection.
No mapper, RViz or bag replay was started by these checks. Sensor readiness is
separate from build success.

## Removed from active use

The complete `third_party/autonomy_stack_go2` checkout and source/build/install
entries for `point_lio_unilidar`, `transform_sensors`, `calibrate_imu`, and
`go2_sport_api` were removed. The calibration wrappers, unaccepted candidate file
and obsolete calibration resume document were removed. The old Point-LIO branch
and CMU calibration gate were removed from the mapping test launcher; its SLAM
Toolbox branch is retained. `scripts/test_slam.sh point_lio` now exits with an
explanation before launching nodes.

The CMU-only transforms3d installation step was removed from the rebuild script.
Existing generic dependency libraries are retained; nothing was uninstalled
from system ROS or other SDKs. A recovery archive and removal manifest are in
`reports/point_lio_migration_20261016/`. Historical reports remain evidence, not
current run instructions.

Official SDKs, Unitree message definitions, the unchanged developer URDF, body TF
helpers, SLAM Toolbox, bags and sensor analysis reports are preserved. The existing
visualization was running during inspection and was not restarted by this task.

## What is already available

The passive scan in `reports/point_lio_migration_20261016/live_scan.json` received
raw clouds, LiDAR IMU, robot odometry and LiDAR status with no decoding errors.
Both sensor topics had one reliable, volatile DDS publisher, compatible with the
package's best-effort cloud subscription and reliable IMU subscription.

Cloud fields match the package's UniLiDAR point declaration: float32 XYZ,
intensity and time, plus uint16 ring. Previous recordings establish per-point
time values in seconds spanning roughly 0.062 seconds per cloud. The UniLiDAR
parser (`lidar_type: 5`, `timestamp_unit: 0`) converts seconds to its internal
millisecond representation. Matching fields alone is not a completed mapping test.

The fresh short scan received 132 clouds and 1,911 IMU messages over eight seconds;
receipt rates were about 16.4 and 242 Hz. Discovery/catch-up and host load can
affect these short receipt rates. LiDAR status reported about 15.4 Hz cloud and
243–246 Hz IMU. No sensor reconfiguration was performed.

The package consumes standard ROS messages. An additional USB/serial/UDP LiDAR
driver is not needed to consume the already available robot DDS topics. The
current source compiles its UniLiDAR parser without a Livox driver dependency;
older generic README instructions about installing that driver do not establish
a requirement for this build.

## Required before the first mapping test

1. **Body-IMU static input validity: implemented and user-checked.** The
   approved adapter and fixed-bias calibration pass independent held-pose checks
   and live stationary integrity/magnitude checks. Source timestamps are retained
   and identical duplicates discarded. This provides an alternative to the
   unvalidated LiDAR IMU; it does not repair it. Full walking calibration and
   LiDAR-relative acquisition latency are not established by these checks.

2. **Define LiDAR-to-selected-IMU extrinsics once.** The package uses
   `p_IMU = extrinsic_R * p_LiDAR + extrinsic_T`; it does not look up sensor TF
   to obtain these values. For the manufacturer SDK frames, nominal R is identity
   and T is `[0.007698, 0.014655, -0.006670]` metres. The raw gyro/cloud axis
   agreement is supported by the posture bag, but the DDS cloud origin matching
   the SDK origin remains an explicit assumption. The old topic-derived mount
   proposal remains unapplied. Preserve the developer `radar` and body `imu`
   geometry; they are not interchangeable with the raw sensor frames. The internal
   SDK LiDAR-IMU offset above does NOT describe the selected body IMU. Its
   LiDAR-to-body-IMU geometry needs separate review in step 2.

3. **Prepare a separate reviewed Go2 launch/profile.** Upstream L1 defaults select
   `/unilidar/cloud` and `/unilidar/imu`. The planned Go2 inputs are `/utlidar/cloud`
   and the approved `/doggy/imu/body_calibrated`; the existing Point-LIO profile
   has not been changed to select them yet. Use raw clouds with their original point times, not the vendor
   deskewed/body cloud as an unexplained replacement. Review IMU interval,
   acceleration units, saturation and configured process/measurement noise. The
   L1 hardware identification remains provisional. Upstream profiles are examples,
   not measured calibration for this robot. No replacement Go2 profile is applied.

4. **Check relative timing and set one clock policy.** Preserve source stamps and
   check that IMU samples cover every scan's point-time interval. A common-looking
   clock does not prove calibrated LiDAR/IMU latency. The source exposes
   `common.time_lag_imu_to_lidar`; do not estimate it from laptop-minus-header age.
   During this fresh scan header ages were around 12 seconds, different from an
   older session's roughly 787 seconds; neither value should be hardcoded as an
   IMU/LiDAR offset. Use robot-derived time for a future live setup, or one bag
   clock for replay, and avoid competing clock publishers.

5. **Review output TF and visualization ownership.** Defaults publish
   `camera_init -> aft_mapped`; those names are configurable. The pose is centred
   at the estimator's IMU frame. Merely renaming `aft_mapped` to `base_link` would
   lose the sensor-to-body mounting transform. Plan its connection to the existing
   `odom -> base_footprint -> base_link` tree so no child gets two TF parents or
   competing pose publishers. The optional body-cloud publisher still hardcodes
   `body`, which needs review if enabled. Raw sensor mounting TF is needed for
   coherent robot visualization/integration; it is separate from the estimator's
   numeric extrinsic parameters. No sensor TF was added in this migration.

6. **Then validate on the saved bag before a live mapping walk.** First settle
   stationary for initialization, then check stable pose, cloud registration,
   timestamps, TF and resource use. A bag replay/mapper has not been started by
   this task. Upstream L1 launch enables RViz and accumulated PCD saving by
   default; review those settings and output paths before launch.

There is no requirement for the old eight-value CMU calibration file or its
automatic spinning routine. The standard package's startup initialization does
not command robot motion and is not a substitute for resolving the acceleration
inconsistency.

The incoming orientation quaternion and Imu covariance arrays are not read by
this estimator's measurement path. Its configured noise values still need review.
Correcting quaternion field order or filling message covariance alone would not
resolve the accelerometer problem; `use_imu_as_input: false` also does not disable
the use of acceleration measurements.

## Evidence

- [Manual IMU posture analysis](reports/tf/imu_axes_20261016_162621_analysis/REPORT.md).
- [Offline geometry audit](reports/tf/geometry_review_20261016/REPORT.md).
- [Migration manifest](reports/point_lio_migration_20261016/migration.json).
- [Official L1 coordinates](https://github.com/unitreerobotics/unilidar_sdk#coordinate-system-definition).
- [Standalone ROS 2 source](https://github.com/dfloreaa/point_lio_ros2).
