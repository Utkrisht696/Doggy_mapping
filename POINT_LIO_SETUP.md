# Standalone Point-LIO readiness — 16 October 2026

The user selected standalone Point-LIO and authorized removal of CMU and its
associated tooling. Mapping remains paused for user review of the implemented
output integration and RViz preview before any estimator test.

**Step 1 update:** the user approved a calibrated body-IMU route. The passive
adapter is running and its offline/live stationary checks passed. The user
checked standing output and a subsequent standing capture passed; step 1 is closed.
Step 2 geometry is approved, applied, live-verified and independently checked by
the user. Step 3’s approved Go2 input YAML is applied and all 52 parameters passed ROS
loading/type/value checks; the user accepted the result and requested step 4. Passive timing/coverage/clock
checks now pass; a repeat motion test found approximately 1–2 ms relative IMU response alignment,
while precise cloud acquisition lag remains uncalibrated and source gaps were observed. The shared runtime model adds `radar -> utlidar_lidar`
using the validated topic-derived measurement conversion. Developer mounts remain
unchanged. See [step 2 report](reports/point_lio_steps/step_02_geometry/REPORT.md). Output is `/doggy/imu/body_calibrated` in developer
frame `imu`, sourced from timestamped `/sportmodestate`. The raw LiDAR IMU remains
unchanged and unvalidated. See [step 1 report](reports/point_lio_steps/step_01_accelerometer/REPORT.md)
and [workflow state](reports/point_lio_steps/STATE.json).

**Output integration update:** `launch/point_lio_go2.launch.py` now references
the approved input profile and defaults to preview with mapping disabled.
`scripts/start_point_lio.sh` opens the raw-cloud/model preview; the separate
mapping view and exact-time `map -> odom` bridge are implemented and tested.
Seven geometry/bridge tests and an isolated ROS transport check passed, without
starting Point-LIO. User GUI verification remains pending. See
[preview commands and verification](reports/point_lio_steps/step_05_output_tf/VERIFY.md).


**Saved-bag mapping update:** the original seed-10 profile failed stationary
initialization. A user-approved temporary seed-4000 trial, retaining original gyro
variance 0.1, completed two full replays and reduced the first 13.75-second pose
excursion to 4.58 cm. Complete source-stamp observation and identical trajectories
were verified on the repeat. A later 34.6 cm stationary transient and substantial
motion disagreement remain, so mapping is not accepted for live use. The active
profile has not adopted the trial. The live calibrated-IMU adapter has separately
stopped on its source-quaternion guard. Review the [latest mapping results](reports/point_lio_steps/step_06_saved_bag/20261016T215933+1100/REPORT.md)
before proceeding. All test mapper/replay/RViz processes are stopped.

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
visualization was running during the migration inspection. Step 2 now runs the
headless sensor launch and the calibrated body-IMU adapter, without RViz or mapping.

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

2. **LiDAR-to-body-IMU geometry: complete and user-checked.**
   `config/point_lio/sensor_geometry.json` stores the approved measurement TF and
   matching `p_IMU = extrinsic_R * p_LiDAR + extrinsic_T` values. The latter map
   `utlidar_lidar` into developer body-IMU frame `imu`, with translation
   `[0.307730028668, 0.000000019947, -0.042320015374]` m. The new runtime edge
   is topic-derived coordinate conversion, not a manufacturer-specified mount.
   All original developer joints and source URDF files remain unchanged. The
   nominal internal LiDAR-IMU offset must not be used for this selected body IMU.
   The approved numeric estimator values are now in the step-3 Go2 profile; TF
   alone does not configure the estimator.

3. **Go2 input profile applied, validated and accepted by the user.**
   `config/point_lio/go2_body_imu.yaml` selects `/utlidar/cloud` and
   `/doggy/imu/body_calibrated`, copies the exact approved extrinsics, carries
   explicit upstream L1 algorithm/filter settings and disables PCD saving. All
   52 parameter names/types/values passed the actual ROS parameter-file loader.
   The upstream checkout remains unchanged; the new Go2 launch references the
   profile with mapping disabled by default. L1 identification is provisional. Noise/saturation settings remain
   upstream starting values, and timing is pending step 4. Evidence is in
   `reports/point_lio_steps/step_03_input_profile/`.

4. **Passive timestamp/coverage/clock checks passed; acquisition lag remains open.**
   A 45-second capture covered all 694 scan point intervals with the selected body
   IMU, without backwards timestamps. An independent reliable clock audit matched
   all 759 interior clock samples exactly to source/relayed odometry. See
   `reports/point_lio_steps/step_04_timing/REPORT.md`. No correction has been applied;
   the repeat posture bag shows ~1–2 ms relative IMU response offset, but direct
   cloud acquisition lag remains uncalibrated and body gaps up to 34 ms were observed.
   No active offset was changed. See `posture_timing_result.json` in the same directory.
   The approved temporary laptop receive-buffer fix and reliable/depth-2000 body
   source subscription now eliminate observed receive overflows and missing
   calibrated samples in baseline/recording trials. Intermittent recording-load
   delay previously reached roughly 0.6 s during heavy audit workloads in Power
   Saver. After the user selected Performance mode, the same unchanged application
   passed a repeated sensor/recording/subscriber comparison: no observed output
   loss or UDP overflow, phase p99 below 1.22 ms, maximum relative delay 4.98 ms.
   Step 4 remains open for cloud acquisition timing. See
   `reports/point_lio_steps/step_04_timing/delivery_delay/REPORT.md`.
   The kernel buffer increase resets after a laptop reboot; persistence is pending.
   Preserve source stamps and
   check that IMU samples cover every scan's point-time interval. A common-looking
   clock does not prove calibrated LiDAR/IMU latency. The source exposes
   `common.time_lag_imu_to_lidar`; do not estimate it from laptop-minus-header age.
   During this fresh scan header ages were around 12 seconds, different from an
   older session's roughly 787 seconds; neither value should be hardcoded as an
   IMU/LiDAR offset. Use robot-derived time for a future live setup, or one bag
   clock for replay, and avoid competing clock publishers.

5. **Output TF and visualization implemented; user verification pending.**
   The Go2 launch configures estimator messages as `map` / `imu`, moves its
   direct TF to `/point_lio/raw_tf`, and uses a bridge to publish only
   `map -> odom`. Composition uses existing `odom -> imu` at the exact estimator
   stamp, preserving the body-IMU lever arm and full attitude. Missing TF causes
   wait/drop diagnostics, never a latest-time or identity fallback. Existing
   odom/body/model TF owners remain unchanged. Preview mode displays the raw
   cloud/model in odom without starting Point-LIO; mapping mode selects map,
   registered scans, a labelled 60-second display accumulation and trajectory.
   The upstream hardcoded body-cloud publisher remains disabled. See
   [implementation and verification](reports/point_lio_steps/step_05_output_tf/VERIFY.md).

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
