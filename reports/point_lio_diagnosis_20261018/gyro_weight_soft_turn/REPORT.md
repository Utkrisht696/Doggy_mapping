# Single-parameter gyro-weight experiment

The approved variance change improved the full-route heading comparison, but it is not a complete drift fix. RMS heading disagreement fell from 7.62° to 5.59° and peak disagreement from 18.53° to 11.46°. Horizontal return offset barely improved (3.51 m → 3.37 m). Final height change improved (−43.3 cm → −33.7 cm), while the deepest intermediate height estimate worsened (−47.6 cm → −52.0 cm). The candidate has not been promoted to the active setup.

The only estimator setting changed was `mapping.imu_meas_omg_cov: 0.1 → 0.001`, a 100× reduction in measurement variance. The running isolated node returned `Double value is: 0.001`. Estimator source is byte-identical to the previously approved logging-only control. The diagnostic build has its own runtime log directory; all earlier outputs and active files are protected.

## Comparison

| Metric | Control, variance 0.1 | Trial, variance 0.001 |
|---|---:|---:|
| RMS heading disagreement | 7.617 ° | 5.588 ° |
| Peak heading disagreement | 18.530 ° | 11.457 ° |
| Horizontal return offset | 3.509 m | 3.370 m |
| Final height change | -43.258 cm | -33.746 cm |
| Final heading change | 0.336 ° | -6.638 ° |

Heading disagreement is against the body orientation after a common initial hold alignment; that reference and gyro integration share the body IMU and are not independent surveyed truth. Physical return to start is user reported rather than surveyed. Quiet analysis windows are identical: 2–6 s and 137.057–141.557 s. No heading scale, per-bag temporal correction or extrinsic fit was introduced.

## Remaining heading mechanism

Stronger gyro weighting greatly reduces direct yaw changes from LiDAR updates, but the filter can still alter its estimated angular velocity through the additional estimated gyro bias. At the new peak near 61.929 s, prediction differs from the body orientation change by −10.54°, while accumulated direct LiDAR corrections contribute only −0.89° and direct IMU corrections −0.03°. These are coupled filter operations; their local yaw contributions do not establish independent physical accuracy.

A read-only reconstruction integrates the recorded calibrated body gyro after subtracting the per-scan estimated residual bias, interpolated to IMU timestamps. For the variance-0.001 trial, it reproduces the predicted heading contribution with RMS difference 0.0975° and maximum difference 0.3049° over the analyzed route. At 61.929 s, plain gyro integration gives about −66.95°, gyro-minus-estimated-bias integration gives −77.68°, and filter prediction contributes −77.58°. Thus approximately 10.7° of the departure can be reconstructed from the filter’s additional bias estimate. This is a stronger diagnostic lead than continuing blind covariance tuning.

The adapter’s fixed gyro correction is already applied to the recorded IMU. The additional state is the residual bias estimated inside Point-LIO, not the fixed calibration itself. Real residual or thermal bias is possible. Close gyro/quaternion agreement alone does not prove the physically correct bias, and reconstruction from final per-scan bias omits transient changes within each scan. A controlled hold-zero test is still needed before claiming that this mechanism is the fix.

## Validation

- control: 2176 finite poses, 15.40 Hz median; no nonincreasing stamps or output gaps >150 ms; no missing exact-stamp TF or recovery reseeds. Observer raw/IMU delivery exact: True/True.
- gyro_weight: 2176 finite poses, 15.40 Hz median; no nonincreasing stamps or output gaps >150 ms; no missing exact-stamp TF or recovery reseeds. Observer raw/IMU delivery exact: True/True.

Both cases use the same full input recording, corrected IMU samples, point times, profile, geometry and 4,000-point startup override. The new run uses localhost DDS domain 224, replay rate 1× and no robot control topics or RViz. All 2,188 expected clouds and 41,521 corrected IMU messages reached the replay observer. Trace operation counts and the two small negative prediction intervals match the control. Observer checks do not separately instrument subscription delivery inside the estimator.

All protected active and previous baseline files retain their SHA-256 hashes. Original source MCAP sizes/mtimes are unchanged. The replay and estimator/bridge stopped successfully.

## Prepared next test — awaiting review

A [small optional diagnostic patch](proposed_bias_test/proposed.patch) would hold only the additional residual gyro-bias state at zero in a new isolated build, retaining variance 0.001 and the approved fixed adapter calibration. It zeros that state’s initial value, prior covariance and process noise. Accelerometer bias remains estimated. The option defaults to false and the patch is not applied or compiled. This tests the bias-feedback hypothesis; it is not a deployment recommendation to disable all gyro-bias estimation.

Run the same bag once, verify exactly zero logged residual gyro bias, and compare full-route heading, horizontal/vertical return, intermediate height and output continuity against both prior cases. Review the result before another change or bag. No active setup changes are authorized by this proposal.

## Spin bags

Existing `classic_mode_spins` (127.76 s) and `running_mode_spins` (66.33 s) contain raw clouds, body state, lowstate and LiDAR IMU topics according to their metadata. They have only been inventoried here, not replayed or integrity-validated. They may provide a useful later check; no new recording is needed for the proposed bias experiment.

[Comparison plot](comparison.png) · [Numeric comparison](comparison.json) · [Bias reconstruction plot](run_01/analysis/bias_reconstruction.png) · [Bias reconstruction data](run_01/analysis/bias_reconstruction.json) · [Next test scope](proposed_bias_test/scope.json)
