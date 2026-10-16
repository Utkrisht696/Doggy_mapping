# Fresh motion timing check — 16 October 2026

The recording saved cleanly: 207.61 s, all 11 requested topics, no decoding errors,
SQLite integrity ok, metadata/database counts agree. The user manually performed
two pitch-then-roll sequences with stationary holds. The agent sent no movement
commands, ran no mapper/replay and changed no active application code/config/TF.

## Timestamp transport and coverage

- All 3,170 interior raw scan intervals have selected body-IMU bracketing.
- All 60,193 unique source timestamps in the shared interior have calibrated output;
  no extras, no backwards source/output/cloud headers, zero fixed-bias math error.
- All 8,704 checked private-clock stamps occur exactly in both the vendor odometry
  and its standard /odom relay. One clock owner and no global /clock publisher were
  verified in the preceding fresh passive capture.
- Point times are finite, nonnegative seconds within the scan period; retained scan
  intervals do not overlap. Within-cloud record order can reverse, and Point-LIO
  sorts retained points; no published timing was rewritten.
- Relative bag callback receipt delay (body output minus matching source): median
  0.238 ms, p99 0.484 ms, max 8.247 ms. These callback differences are transport
  diagnostics, not physical acquisition latency.
- LiDAR status error codes and both packet-loss fields remained zero in this bag.

The preceding 45 s passive audit independently passed every check: 662 interior
scans covered, 13,396 exact source/output value matches, and maximum scan-end IMU
callback wait 5.037 ms. The laptop remained in user-selected Performance mode.

## Independent response comparison

Apply the previously reviewed axis rotation without refitting extrinsics. Compare
the two gyro streams using original source times, zero-phase low-pass filtering and
a +/-200 ms shift search. Positive delta means body-event stamp minus LiDAR-gyro
event stamp, and is the value Point-LIO would subtract only if independently
validated for the cloud. Repeated blocks were identified from actual gyro excitation.

| Block | Measured source interval | Best delta at 5 Hz | Correlation |
|---|---:|---:|---:|
| pitch_1 | 40.63–69.90 s | -1.0 ms | 0.999669 |
| roll_3 | 80.62–103.96 s | -2.0 ms | 0.999977 |
| pitch_2 | 114.56–148.71 s | -2.0 ms | 0.998461 |
| roll_5 | 155.27–182.60 s | -1.5 ms | 0.989686 |

The short-window estimates range from approximately -3 to +1 ms. This reproduces
the earlier close IMU response alignment; it does not prove that cloud acquisition
shares exactly the LiDAR gyro timestamp model. Some last-roll gyro residuals are
not explained by a constant shift. The internal LiDAR accelerometer remains
unvalidated and is not selected for Point-LIO.

## Direct cloud timing check

A new per-point diagnostic rotates each selected floor point at its own point
timestamp plus a candidate offset, using integrated calibrated body gyro and the
unchanged approved extrinsics. It refits a plane per scan and tests how constant
the floor normal is across the block. This improves on treating a whole scan as
one acquisition instant. A non-saturating Huber loss avoids hiding large residuals.
The fused body orientation is only an independent diagnostic.

| Block | Gyro-reference optimum | Improvement over zero | Near-minimum sensitivity range |
|---|---:|---:|---:|
| pitch_1 | +19 ms | 0.302% | -50 to +80 ms |
| roll_3 | +4 ms | 0.074% | -30 to +40 ms |
| pitch_2 | -5 ms | 0.024% | -90 to +80 ms |
| roll_5 | -8 ms | 0.099% | -50 to +50 ms |

The preferred shift changes between repeats and barely improves the objective.
All these broad sensitivity ranges include zero. They are not confidence intervals.
Sparse/noisy returns, residual mounting/calibration error, gyro drift and omitted
within-scan body translation can confound this floor diagnostic. This result does
not justify a nonzero correction and does not certify precise physical cloud
acquisition synchronization. Leave common.time_lag_imu_to_lidar=0.0 unchanged
and explicitly provisional. More repetitions of the same posture test alone are
unlikely to establish a precise offset.

## Remaining source gaps and next step

The maximum body timestamp gap is 35.96 ms. All 36 motion-window gaps above 12 ms
have no intervening source packet in the bag either. This locates them outside
the calibrated conversion for the observed packet set, but cannot distinguish
robot publication from common transport/recording behavior. Bracketing passes;
a saved-bag estimator trial must assess how it handles these gaps.

The transport/coverage and repeat-response checks support a controlled saved-bag
Point-LIO experiment after output-TF integration and user review. They do not
establish production/live autonomous-mapping readiness or precise temporal
calibration. No acquisition offset is guessed and no mapper has started.

Please check that the motion overview shows the two pitch-then-roll sequences
you performed before advancing to the TF/launch integration checkpoint.

Evidence:
- [Motion overview and gyro comparison](gyro_timing_comparison.png)
- [Per-point cloud timing](per_point_cloud_timing.png)
- [Input timing integrity](input_timing_integrity.json)
- [Structured result](timing_result.json)
- [Recording verification](recording_verification.json)
