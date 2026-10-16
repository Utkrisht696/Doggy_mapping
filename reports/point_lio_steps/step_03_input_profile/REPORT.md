# Step 3 input profile — applied and validated

Status: **User-approved input profile applied; parameter validation passed.
Awaiting user double-check before step 4.**

Step 2 is closed by the user’s independent TF output. The proposal-stage draft
remains preserved outside the active config/launch directories.

Applied active file: `config/point_lio/go2_body_imu.yaml`.
Reviewable draft: `go2_body_imu.review.yaml` in this report directory.

Changes from the checked-out upstream L1 YAML and launch settings:

- Select `/utlidar/cloud` and `/doggy/imu/body_calibrated`.
- Copy the exact approved body-IMU extrinsic R/T from sensor_geometry.json.
- Keep UniLiDAR parser 5, point-time seconds (timestamp_unit 0), scan_line 18,
  no cloud combining/cutting, stationary initialization, fixed extrinsics and
  acceleration units m/s² (acc_norm 9.81), as in the upstream example.
- Carry upstream launch overrides explicitly into this YAML so algorithm defaults
  cannot silently change the intended variant or filtering parameters.
- Disable accumulated PCD saving. Leave body-frame cloud publication disabled.
- Retain the remaining upstream noise/filter/saturation starting values, explicitly
  marked as unmeasured. They are not physical sensor calibration or covariance.

Unresolved prerequisites remain visible: exact L1 model identification is provisional;
nominal 0.004-second IMU interval and zero inter-sensor lag require step 4; process
and measurement noise are upstream starting values; estimator output/TF ownership
and its launch are step 5. No runnable launch is prepared by this profile-only
proposal, and applying it does not authorize starting Point-LIO. It retains upstream
output-frame defaults because final frame integration has not yet been reviewed.

The source C++ declares every proposed parameter, consumes these topic names and
uses p_IMU = R * p_LiDAR + T. The code does not load Imu covariance arrays. Input
QoS is sensor-data for clouds and reliable for IMU; the approved adapter publishes
reliably. Source files inspected: src/parameters.cpp, src/laserMapping.cpp,
config/unilidar_l1.yaml and launch/mapping_unilidar_l1.launch.py.

## Application and validation

The user approved applying the reviewed profile and validating it without starting
Point-LIO (“yeah do it”). `config/point_lio/go2_body_imu.yaml` is now present.
All 52 parameter values are identical to the review draft; only the introductory
comments were changed to reflect approval. No launcher references this profile yet.

`applied_profile_validation.json` records a passing check using ROS’s actual
parameter-file loader in a temporary validator node, isolated to loopback on domain
197. All 52 loaded names, types and values match the estimator C++ declarations
and reviewed YAML. Approved R/T match exactly, source topics are correct, finite
numeric values pass, stationary initialization/fixed extrinsics are retained, and
PCD/body-frame cloud output are disabled. Upstream Point-LIO remains clean.

Two initial validator attempts failed due to validation-environment compatibility:
CycloneDDS selected loopback twice with ROS_LOCALHOST_ONLY enabled, then the
installed rclpy enum required `.value` rather than `int()`. These were fixed only
in the temporary validation invocation/helper; the approved profile was unchanged.
The completed validator exited successfully and left no persistent validator node.

The headless sensor/IMU processes were not restarted. No Point-LIO, mapping,
RViz or bag replay was started. This validates parameter loading and configuration
consistency; it does not establish sensor timing, calibrated noise or map quality.

For the requested user double-check, inspect the applied file and verify the
`common` topic names plus `mapping.extrinsic_T` and `mapping.extrinsic_R`. Do not
start the upstream launch. Once confirmed, proceed to step 4 timing checks.

