# Step 2 — selected body IMU and raw LiDAR geometry

Status: **COMPLETE. User-approved geometry applied, live checks passed, and the
user independently confirmed both requested TF lookups.**

The review-stage investigation below is retained as provenance. The implementation
and current runtime results are recorded at the end of this report.

Step 1 was closed after the user's standing readings and a fresh 4,282-message
standing check passed. The selected IMU topic is `/doggy/imu/body_calibrated`,
with developer frame `imu`. The original `/utlidar/imu` is not the selected input.

## Current runtime

The earlier visualization/model launch had stopped: no TF, clock or joint-state
publisher was present. The existing sensor-only launcher was restored without
code changes, RViz, mapping, calibration or robot commands. It publishes the
unchanged developer model with the previously approved base-name normalization,
the existing odometry/footprint tree and cleaned clouds. The calibrated body-IMU
adapter remains running independently.

The current 44-frame developer/body tree includes:

```text
odom -> base_footprint -> base_link
                           +-> imu
                           +-> radar
                           +-> other developer links
```

Both sensor links have `base_link` as parent. There is no legacy `base` alias.
Observed TF parents are unique. Publisher endpoints match the expected RSP/state
adapter contract. The installed Python ROS interface does not expose publisher
GID on received messages; ownership checking therefore combines endpoint counts
and verified source behavior rather than claiming per-message attribution.

## Preserved developer geometry

| Parent | Child | XYZ, metres | RPY, radians |
|---|---|---|---|
| base_link | imu | [-0.02557, 0, 0.04232] | [0, 0, 0] |
| base_link | radar | [0.28945, 0, -0.046825] | [0, 2.8782, 0] |

These are the original developer mount values. The source URDF remains unchanged,
SHA256 `7d19fe48e2e689ee1a032ab99f2a4a8b671d87e73de48d3e65811682a5b48b9e`.
The calibrated body-IMU topic therefore already has a valid developer sensor frame
when this model publisher is running. No new body-IMU TF is needed.

## Proposed DDS measurement-frame connection

The raw cloud header says `utlidar_lidar`, which is absent from the developer URDF.
The vendor's raw/body cloud conversion was recovered previously and validated
unchanged against independent captures, including manual pitch, roll and yaw.
The fresh standing capture also agrees: 12,836 corresponding points across ten
cloud pairs, maximum numerical residual 2.521e-6 m. Points were matched by exact
header stamp plus unique ring, float-time bits and intensity bits. No refit was
performed on the fresh capture.

Proposed additional edge beneath the preserved developer `radar` mount:

```text
radar -> utlidar_lidar
XYZ metres: [-0.00515268845, 0.00000001995, -0.04710809558]
RPY radians: [-0.001508482628, -0.000981514429, -2.146758634683]
RPY degrees: [-0.086429688, -0.056236634, -123.000209401]
```

**This is a topic-derived measurement-frame conversion, not a new manufacturer
physical-mount value.** Its tiny numerical residual is coordinate-stream agreement,
not micrometre placement accuracy. Absolute correspondence between the vendor
body-cloud frame and developer body frame is assumed from their documented/model
roles and message labels; it has not been physically surveyed. An identity alias
between `radar` and this DDS frame fails the point correspondence checks.

The proposed implementation would add only this explicitly labelled measurement
frame beneath `radar` in the runtime model/bringup shared by sensor launchers,
keeping original developer mount joints and source URDF unchanged. One owner
would publish the edge. Before implementation, the user must approve this
additional topic-derived frame rather than mistake it for developer geometry.
After application, verify the TF lookup and cloud alignment at source stamps,
then request the user's own TF/visualization double-check before step 3.

## Point-LIO extrinsic for the selected BODY IMU

The estimator uses:

```text
p_imu = extrinsic_R * p_utlidar_lidar + extrinsic_T
T_imu_lidar = inverse(T_base_link_imu) * T_base_link_lidar
```

Composing the unchanged developer body-IMU mount with the verified vendor cloud
conversion gives:

```text
extrinsic_T metres = [0.307730028668, 0.000000019947, -0.042320015374]
extrinsic_R =
[ 0.526113901735, -0.810135815726,  0.258619648286,
 -0.838668173457, -0.544642722184,  0.000001581407,
  0.140854028099, -0.216896900048, -0.965979232446 ]
```

These values would be stored as a reviewed Go2 geometry profile for the later
input-profile step; the standalone repository/default YAML remains unmodified.
TF alone does not configure these estimator values. Conversely, estimator
extrinsics do not publish the physical/model TF needed for visualization.
No sensor coordinates should also be pre-rotated, which would double-apply geometry.

The small nominal SDK LiDAR-internal IMU offset is **not** the body-IMU offset and
must not be used with the selected body topic. The unused, invalid `utlidar_imu`
stream is not being attached through an assumed SDK origin just to populate TF.

All five transform algebra checks passed, including forward composition through
both the body-IMU and radar chains on 2,000 arbitrary points. Maximum composition
error was 1.78e-15 m. These validate equations, not unknown physical surveying.

## Evidence and current decision

- `geometry_proposal.json`: exact proposed transforms, provenance and assumptions.
- `live_geometry_audit.json` and `live_geometry_points.npz`: fresh raw/body stream correspondence. Its initial
  missing-TF flags record the stopped bringup state before restoration.
- `restored_developer_tf.json`: current developer links, endpoints and checks.
- `sensor_bringup.log`: existing headless sensor launch, running in session 80250.

No step-2 application code/configuration or raw-sensor TF has been changed.
Approval requested: add the reviewed measurement-frame edge and store the matching
LiDAR-to-body-IMU geometry without changing developer mounts. Keep mapping paused.


## Approved implementation and live verification

User approval: **“Apply and verify the reviewed geometry.”** The exact approved
values are now stored in `config/point_lio/sensor_geometry.json` (`approved: true`).
The original `geometry_proposal.json` remains an unchanged review artifact.

The shared runtime-description helper appends only one empty measurement link and
one fixed joint, `radar -> utlidar_lidar`. All three local launchers explicitly
load the same profile. The existing robot_state_publisher is its sole owner;
there is no additional static publisher. Original developer URDF files and all
original physical mount/joint geometry are unchanged. The installed runtime model
has 43 links; including `odom` and `base_footprint`, the observed tree has 45 frames.
The invalid internal LiDAR IMU remains unused and unconnected. The selected body
IMU uses the existing developer `imu` link.

The matching LiDAR-to-body-IMU numeric extrinsics are stored for step 3. They have
not been injected into upstream Point-LIO files, and no estimator was started.

Verification results:

- Six frame tests passed, including unchanged developer geometry, pose composition,
  matching TF/extrinsic chains and rejection of unapproved/stale/invalid profiles.
- All changed Python files compile, and all three launchers pass `--show-args`.
- `applied_tf_check.json`: 43/43 runtime model links resolved, 12 measured joints,
  matching runtime description, five source-pose comparisons and five source-stamp
  lookups for EACH raw/body/deskewed cloud. Maximum body position error 5.56e-17 m;
  maximum quaternion error 1.12e-16. No remaining lookup errors.
- `applied_geometry_validation.json`: 15 fresh exact-stamp cloud pairs, **19,012**
  corresponding points. Applying the actual LIVE TF (no refit) reproduces vendor
  body coordinates with maximum numerical residual **2.339e-6 m**. This is stream
  equivalence at float32 precision, not surveyed physical placement accuracy.
- Live `imu -> utlidar_lidar` matches stored Point-LIO extrinsics to <9e-16 rotation
  coefficient error and <1.7e-16 m translation error.
- All 45 frames present; unique observed parents; no legacy `base`; `/tf` endpoints
  are RSP/state adapter, `/tf_static` has only RSP, robot clock has one owner.
  Ownership evidence uses endpoints plus source contract, not per-message GIDs.
- 1,043 calibrated body-IMU messages received in frame `imu`, one publisher,
  acceleration norm mean 9.80967 m/s². This is a live integrity check, not new calibration.

An independent runtime issue was discovered before restart: the previous body-IMU
adapter exited on an out-of-range source orientation quaternion after a telemetry
gap. The failing sample itself was not recorded, so its cause is unknown. A fresh
1,495-message source check contained no invalid quaternions. The previously approved
adapter was restarted **without code or calibration changes** and the output check
above passed. Its validation guards remain active; recurrence needs investigation,
not bypassing. Historical step-1 calibration evidence remains valid, but continuous
operation across source interruptions is not guaranteed by this short check.

Current sessions: headless sensor launch 12713; body-IMU adapter 44297. Logs are
`sensor_bringup_approved.log` and `body_imu_adapter_restart.log`. Earlier launch/log
entries above describe the review stage. RViz, mapping, bag replay and motion/control
routines were not started. Pre-change files are in `before_approved_change/`.

User double-check (Ctrl+C stops only each echo command):

```bash
cd ~/Doggy
source scripts/setup_ros.sh
ros2 run tf2_ros tf2_echo base_link utlidar_lidar
ros2 run tf2_ros tf2_echo imu utlidar_lidar
```

Expected translations, metres: first approximately `[0.282160, 0, 0]`, second
`[0.307730, 0, -0.042320]`. A brief initial discovery warning may appear; repeated
missing-frame messages after discovery would need investigation. Confirm these
resolve before moving to step 3. No mapper or additional code change is authorized
by this checkpoint.


## User double-check received

The user pasted repeated successful `base_link -> utlidar_lidar` and
`imu -> utlidar_lidar` lookups. Translation and rotation matrices match the
reviewed values at the displayed precision. Each initial discovery warning clears;
zero timestamps are expected for a chain containing only fixed transforms.
Step 2 is complete. Evidence: `user_tf_doublecheck.json`.
