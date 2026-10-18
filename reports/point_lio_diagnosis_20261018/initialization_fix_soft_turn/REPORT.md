# Startup IMU batch fix: soft-turn replay

The approved startup-batch correction did not solve the failure. In this replay, horizontal return offset increased from 3.51 m to 4.20 m, RMS heading disagreement against the body orientation increased from 7.62° to 9.58°, and the final height change stayed near −43 cm. The peak heading disagreement decreased slightly, from 18.53° to 17.65°, but the overall result is worse. No change has been promoted to the active workspace.

Only two estimator calls changed relative to the cache-fixed control: `meas.imu.shrink_to_fit()` became `meas.imu.clear()` in the startup branches. The previously approved cache resize correction remains in both cases. IMU bias, noise, timestamps, acquisition-lag correction, extrinsics and TF logic are unchanged. The exact diff is [applied.patch](applied.patch).

## Results

| Metric | Cache-fixed control | Startup batch correction |
|---|---:|---:|
| Horizontal return offset | 3.509 m | 4.199 m |
| Height change | -43.258 cm | -43.063 cm |
| Final heading change | 0.336 ° | -14.610 ° |
| RMS heading disagreement | 7.617° | 9.577° |
| Peak heading disagreement | 18.530° | 17.650° |
| 95th-percentile absolute disagreement | 14.816° | 15.063° |

Heading disagreement is against the timestamp-interpolated body orientation, after zeroing each trace at the same initial hold. The body orientation and gyro integration share the body IMU and are diagnostic references, not independent surveyed truth. They agree within about 0.31° throughout this route, while both Point-LIO cases separate from them. This supports investigating heading corrections, without proving the body IMU heading is physically exact. A small endpoint heading change in the control concealed a much larger disagreement during the route.

The largest heading disagreement in both cases occurs near source time 55.90 s. Both first cross 10° disagreement near 52.4–52.7 s. The control later returns close to its initial heading; the startup-corrected run retains roughly −14.6° heading change. The two trajectories differ by up to 1.05 m and 16.05° at their 2,173 shared timestamps. Startup/map evolution sensitivity is present; this single changed run does not uniquely establish the mechanism.

The source begins quietly. The corrected startup takes longer, with first output at 1.036 s versus 0.840 s for the control. Three missing startup poses are expected from this delay; there are no later output gaps. Logs show the initializer accumulating new batches rather than repeated earlier samples. All endpoint and heading comparisons use the same quiet windows: 2.000–6.000 s and 137.057–141.557 s. The old control report used a slightly different initial window (3.510 m); recomputation here gives 3.509 m. No scale or offset was fitted to the trajectory.

## Validation

- control: 2176 finite poses, about 15.40 Hz, no nonincreasing timestamps, no output gaps >150 ms, no missing exact-stamp map→odom TF, no conflicting TF parents, and no recovery reseeds. Observer received exactly all 2188 clouds and 41521 calibrated IMU messages.
- initialization_fix: 2173 finite poses, about 15.40 Hz, no nonincreasing timestamps, no output gaps >150 ms, no missing exact-stamp map→odom TF, no conflicting TF parents, and no recovery reseeds. Observer received exactly all 2188 clouds and 41521 calibrated IMU messages.

The new run uses the full recorded bag at 1×, isolated on localhost DDS domain 222. The saved control was previously reproduced exactly in three control/baseline replays; its arrays and all prior results are unchanged. Observer delivery checks do not separately instrument every estimator callback. The original source MCAP sizes and mtimes are unchanged, and all protected active code/configuration and baseline binaries match their recorded SHA-256 hashes. No robot commands were sent. The bag player, estimator and TF bridge stopped successfully.

## Interpretation and next check

The cache resize bug is real but had no numerical effect on this recording. Clearing startup batches fixes another real defect but did not provide a drift remedy. Heading is a significant observed problem; it is not yet established as the sole cause of the position and height errors. About 43 cm of final height change remains in both cases. Physical body-IMU/LiDAR acquisition lag remains uncalibrated.

The next useful experiment is to trace the predicted orientation and the change applied by each LiDAR/IMU filter update, along with innovation, bias and effective prediction times, around the first divergence. That would show whether the unwanted yaw accumulates during propagation or is introduced during scan updates. It needs a separately reviewed instrumentation change; no further estimator edits have been made. Do not tune a yaw scale to this one route.

[Comparison plot](comparison.png) · [Numeric results](comparison.json) · [Heading analysis](run_01/analysis/heading_comparison.json) · [Source audit](source_change_audit.json)
