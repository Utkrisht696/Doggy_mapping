# Manual IMU axis test — 16 October 2026

Source bag: `/home/jain/Doggy/ros2_ws/reports/tf/imu_axes_20261016_162621`.

The bag is readable and SQLite quick-check passed. It contains approximately 87.3 seconds and 92,149 messages across ten requested topics. The LiDAR IMU has 21,408 samples at approximately 245.6 Hz. The operator reported pitch, a pause, roll and then yaw, holding each pose about five seconds. All three sections are present. No repeated test is needed to establish these observations.

## Observed sequence

| Section | Approximate seconds from bag start |
|---|---|
| Baseline | 0–4 |
| Pitch | 5–29 |
| Pause before roll | 32–40 |
| Roll | 41–60 |
| Yaw | 63–81 |
| Final standing | 83–87 |

Body attitude spans approximately 11.2° in pitch, 13.4° in roll and 56.0° in yaw across the recording. These are measured peak-to-peak ranges, not assumed commands. Phase windows were identified from body telemetry rather than equal-duration timing.

## Quaternion-order evidence

Angular-rate correlation between quaternion increments and the same LiDAR IMU gyro, XYZ: as-published ROS interpretation `0.187, -0.614, -0.710`; scalar-first interpretation `0.817, 0.827, 0.998`.

The scalar-first interpretation is strongly supported on all three axes. It interprets incoming named fields `(orientation.x, orientation.y, orientation.z, orientation.w)` as the scalar-first components `(qw, qx, qy, qz)` and would output ROS XYZW as incoming `(y, z, w, x)`, then normalize. No such correction has been applied. The quaternion norm is about 0.998312 rather than exactly one; analytical rotations were normalized.

This is empirical evidence, not source-level confirmation of the inaccessible DDS firmware publisher. The standalone manufacturer SDK documents XYZW, so that source cannot by itself confirm the runtime DDS encoding. Reordering improves internal gyro/gravity agreement but does not validate the fused orientation against the physical body mounting: the acceleration used as an independent-looking reference is itself inconsistent. Do not treat reordering alone as mapping readiness.

## Raw gyro and cloud axes

Using the earlier independently recovered cloud rotation unchanged, LiDAR/body gyro correlations are `0.939, 0.904, 0.999`. The comparison uses a 2 ms receipt-time alignment, 50 ms smoothing and constant comparison offsets; it does not prove hardware synchronization or determine a lever arm. RMS differences after those offsets are `0.016, 0.016, 0.007` rad/s.

This supports aligned raw LiDAR gyro and raw cloud axes. The physical IMU position remains distinct from the body IMU mount. No mounting rotation was fitted to this bag for this cross-check. A separate exploratory unconstrained gyro fit is saved as diagnostic evidence only and is not a proposed TF.

The gyro match is not a complete calibration. Some residuals and offsets remain, including posture-related differences; correlations are not proof that bias and noise have been calibrated. The comparison offsets are not installed calibration values.

The same earlier cloud conversion was also applied unchanged to raw/body clouds during this test: 11838 unique point correspondences across ten cloud pairs, maximum numerical conversion residual 2.45e-06 metres. This verifies published coordinate equivalence across postures, not surveyed physical placement.

## Acceleration is still inconsistent

| Held section | Mean acceleration norm, m/s² |
|---|---|
| baseline | 10.248 |
| pitch_negative | 10.571 |
| pitch_positive | 9.976 |
| pause | 10.122 |
| roll_negative | 10.918 |
| roll_positive | 9.893 |
| yaw_positive | 10.225 |
| yaw_negative | 10.177 |

Expected stationary specific-force magnitude is approximately 9.81 m/s² regardless of mounting angle. Held-pose norms here vary roughly 9.89–10.92 m/s². Raw Y stays near zero and Z near +9.81 during several distinct held pitch/roll postures, while X changes substantially. Applying the gyro-supported cloud rotation does not align this acceleration with body gravity. Negating all acceleration components does not resolve the posture-dependent directional disagreement either.

A fixed TF rotation preserves magnitude, so adding TF cannot repair these values. The observations establish a measurement/convention/calibration inconsistency, not a hardware-failure diagnosis. One cannot responsibly fix this by subtracting gravity, flipping one axis, normalizing the acceleration, or selecting arbitrary covariance values without validating the input contract. Dynamic spikes during transitions were excluded from the held-pose comparisons.

## Covariance and timestamp limits

All recorded IMU orientation, gyro and acceleration covariance matrices remain all zero: unknown uncertainty in the ROS IMU convention. No invented covariance has been added. Body LowState has no epoch header; cross-sensor matching uses bag receipt time, while the quaternion-vs-own-gyro check uses the LiDAR source timestamps. The observed source intervals have no duplicate/backwards steps, but do contain jitter/gaps. See `analysis.json` for the interval distribution.

## Next review before changes

The available evidence now supports a common raw gyro/cloud axis convention, while acceleration and fused orientation require separate handling. Before running Point-LIO, review one coherent sensor-input path with explicit TF/extrinsics and acceleration/orientation policy. The existing CMU path performs additional rotations and uses a gyro-focused processed IMU; its assumptions must agree with these measured frames. Preserve raw topics and developer geometry. Any actual TF, preprocessing, calibration or estimator-parameter change still requires user confirmation.

No robot commands were sent, no application code/configuration was changed, and no new TF was published. The source bag was opened read-only and left unchanged.

## Artifacts

- [Overview plot](axis_test_overview.png), [PDF](axis_test_overview.pdf).
- [Gyro comparison](gyro_frame_comparison.png), [PDF](gyro_frame_comparison.pdf).
- `analysis.json`: full metrics, held-pose statistics and conclusions.
- `cloud_frame_crosscheck.json`: independent raw/body cloud validation.
- `decoded_samples.npz`: decoded numerical telemetry for reproducibility.
