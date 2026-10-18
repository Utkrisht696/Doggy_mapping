# Soft-turn heading instrumentation result

The approved logging-only replay reproduced all 2,176 control pose values exactly. The heading failure is unchanged, and the instrumentation can be used to locate which filter operations altered the estimate without introducing a new trajectory. No remedy has been applied to active code.

## Measured entry points

At source time 55.8992 s, Point-LIO heading differs from the aligned body orientation by −18.5299°. The heading accounting at this instant is:

| Component | Contribution |
|---|---:|
| Prediction minus body orientation change | -4.308192° |
| Accumulated direct IMU update yaw change | +0.108844° |
| Accumulated LiDAR update yaw change | -14.261883° |
| Total disagreement | -18.529876° |

Each contribution is the sum of local yaw changes caused by that operation, zeroed over the same initial quiet window. Medians introduce a small constant offset if component medians are summed separately; the complete per-scan accounting closes to approximately 5.1e−14° and its integrated yaw matches the output to 1.5e−13°. This is an algebraic attribution of changes in the estimator. The operations are coupled: LiDAR corrections also affect gyro bias and future propagation. The attribution does not establish which physical signal is correct.

At 40.009 s, prediction follows the body heading to about 0.04°, while accumulated LiDAR corrections already contribute −5.42° and direct IMU updates −0.67°. By the peak discrepancy, estimated residual Z gyro bias reaches about +0.00916 rad/s (+0.525°/s), after the fixed adapter bias correction. Its physical validity has not been established. Near the end, opposing contributions partly cancel, masking the larger error during the turns.

Body orientation and gyro integration agree to within about 0.31° over this route, but both share the body IMU. They are diagnostic references rather than independently surveyed ground truth. Wrong scan matching, acquisition alignment, dynamic extrinsics or noise/bias modeling remain possible causes; the trace identifies the mechanism of state change, not a uniquely proved root cause.

## Verification

- 2176 CSV rows match the published scan-rate poses. All pose values are exactly equal to the cache-fixed control.
- 1,075,934 state predictions, 41,299 direct IMU updates and 1,034,635 LiDAR update attempts were traced.
- Only 2 negative prediction steps occurred; minimum -0.203133 ms. There is no widespread timestamp reversal. These two small intervals are not demonstrated to cause the drift.
- 2176 finite poses, no nonincreasing stamps, no output gaps >150 ms, no missing exact-stamp TF and no recovery reseeds.
- All 2,188 raw clouds and 41,521 corrected IMU messages reached the replay observer with the expected stamps. Observer delivery is distinct from internal subscription counts.
- Protected active/baseline hashes unchanged: True. Original source MCAP sizes/mtimes unchanged: True.
- Localhost DDS domain 223, full bag at 1×, no control topics, no robot commands, RViz disabled. Playback, estimator and bridge stopped successfully.

## Proposed next isolated test — not run

Change only `mapping.imu_meas_omg_cov` from 0.1 to 0.001 using the prepared [override](proposed_next_test/gyro_weight_override.yaml). This would increase gyro measurement weight, leaving bias parameters, LiDAR weighting, calibration, timestamps and extrinsics unchanged. The value is a controlled hypothesis test, not a calibrated covariance. Short stationary variance measurements are not a sufficient dynamic noise characterization.

Use the same trace and soft-turn input to compare the complete heading curve, return position/height, bias evolution and update contributions. A smaller endpoint error alone is not enough. Reject a candidate that improves yaw by worsening position, height or stability. Any apparent improvement must later survive other existing route bags without fitting a heading scale. This test is awaiting confirmation under the user’s approval-before-changes instruction.

[Trace plot](run_01/analysis/trace_analysis.png) · [Numeric trace analysis](run_01/analysis/trace_analysis.json) · [Peak breakdown](run_01/analysis/peak_disagreement.json) · [Applied diagnostic diff](applied.patch) · [Next test scope](proposed_next_test/scope.json)
