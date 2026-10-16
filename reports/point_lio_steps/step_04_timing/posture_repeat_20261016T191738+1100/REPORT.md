# Repeat posture-test timing result — 16 October 2026

Status: **Recording saved; clock/source-time/scan-coverage checks pass. Relative
IMU response timing checked. Absolute cloud acquisition lag remains uncalibrated.**
No application code, active YAML, TF, sensor setting or system clock was changed.
No mapper or ROS bag replay was run. The user performed the movements manually.

## Recording integrity

The repeat recorded 181.2106 seconds and 205,057 messages across all ten requested
topics. It includes the verified pre-cue baseline and the full pitch/roll sequence.
SQLite quick_check returned ok, metadata/database counts match, and the recorder
logged cache flush and Recording stopped. The CLI ended on SIGINT (return -2);
that interrupt status is not a failed recording. Original bag bytes are preserved.

| Stream | Saved messages |
|---|---:|
| Calibrated body IMU | 53,394 |
| Source SportModeState | 54,090 |
| Original LiDAR IMU (gyro diagnostics only) | 44,776 |
| Raw clouds | 2,792 |
| Body clouds | 2,793 |
| Vendor odometry | 27,346 |
| Private robot clock | 7,664 |
| TF / static TF | 11,295 / 1 |
| LiDAR status | 906 |

No decoding errors. Selected body IMU and raw cloud headers are strictly increasing,
with no duplicates or backward stamps. All 2,763 interior scans have body-IMU
samples bracketing the first/last retained point time; this includes 1,156 scans
inside the movement windows. Matching 53,350 source/output packets preserve source
time and fixed-bias-corrected six-channel measurements exactly (zero value error).
Bracketing is not a guarantee that every physical IMU sample was captured.

Of 7,579 interior clock samples, 7,578 match the stored vendor odometry directly.
The remaining stamp matches both odom->footprint and footprint->body TF messages,
which are emitted by the same source callback as the clock. This supports a missed
source-odometry record, rather than an invented/different clock timestamp. Earlier
independent reliable clock capture matched all its 759 interior source/relay stamps.

## Observed movements

The source attitude spans pitch -17.44 to +17.45 degrees, and roll -34.20 to +34.59
degrees. The user clarified that the previous “more than 5” referred to tilt angle.
The analysis uses actual source measurements and does not assume five-degree motion
or a particular repetition count. Major pitch and roll windows are 90–126 s and
129–165 s respectively, measured from the first recorded body-IMU source stamp.
These were identified from the measured attitude/rate activity after the cue.

## Independent IMU response-time comparison

Apply the previously reviewed cloud/raw-gyro-to-body rotation unchanged. Angular
velocity needs no lever-arm translation. This raw gyro/cloud axis correspondence
is supported by the earlier posture bag; it is not a newly fitted sensor rotation.

Convert integer source timestamps to relative seconds before interpolation. Resample
at 2 ms, apply a zero-phase third-order low-pass to both signals, search shifts over
+/-200 ms with 0.5 ms spacing, and remove a constant comparison offset in each axis.
No gain, orientation or extrinsic is adjusted in the fit. Repeat with 3/5/10/20 Hz
cutoffs and independently compare ten-second windows.

Sign convention: delta = body event timestamp minus LiDAR-IMU event timestamp.
Compare body_gyro(t+delta) with rotated_lidar_gyro(t). If an offset were independently
validated for the cloud, Point-LIO would subtract delta from the selected body stamps.

| Excited axis | Best delta at 5 Hz | Correlation after alignment | RMS at zero / best, rad/s |
|---|---:|---:|---:|
| Pitch | -1.5 ms | 0.999956 | 0.003672 / 0.002477 |
| Roll | -1.0 ms | 0.994183 | 0.040850 / 0.040755 |

Across the four smoothing cutoffs, pitch remains -1.5 ms, roll -1 to -1.5 ms.
Across the ten-second windows, fitted deltas range -2 to -0.5 ms. Most windows have
very high correlation; some roll windows contain larger residual mismatch,
including a LiDAR-gyro plateau not present in the body gyro during a held posture.
A constant time shift does not explain that mismatch; the raw LiDAR IMU remains
a diagnostic reference, not a validated input or absolute time reference.
This establishes close response timing at millisecond scale for these IMU streams.
It is NOT proof that either published IMU timestamp is its exact hardware acquisition
instant, nor that the cloud uses exactly the LiDAR gyro's clock/latency model.
The typical body sample interval is about 3.97 ms; the reported fit grid is not a
hardware synchronization accuracy specification.

## Direct cloud-time diagnostic

To avoid equating gyro alignment with cloud synchronization, fit a stationary floor
plane to each raw scan after the approved body-coordinate conversion. Use broad
floor-height/normal candidate selection, RANSAC and unconstrained SVD plane fitting;
do not force normals to the body attitude. Set each plane's effective source time
to its mean inlier point timestamp. 1,450 floor fits were available, about 553/555
quality scans in the pitch/roll windows.

Integrate the approved calibrated body gyro using actual source intervals, then
search for the time shift that makes the rotated floor normal constant. Unknown
world-floor normal is allowed. Fused body orientation is a separate diagnostic.
No physical mount transform or gyro calibration is refitted or published.

The direct-floor objective is too broad and noisy to calibrate a small offset.
Its nominal global minima are +24/+23 ms using integrated gyro, but independent
windows range approximately -24 to +25 ms and near-minimum ranges span much of the
+/-150 ms search. The world-normal residual remains about 1.38/1.46 degrees, and the
nonzero shifts barely improve it. Fused-attitude checks are also insufficiently
precise. These +23/+24 ms minima are **not valid corrections** and must not be applied.
Cloud-plane fitting noise, intra-scan motion distortion, residual gyro drift and
physical frame assumptions limit this diagnostic. Objective-width ranges are
sensitivity measures, not confidence intervals.

## Data gaps and delivery limits

The largest selected body-IMU timestamp gap was 33.94767 ms. Within 85–170 s, all
44 gaps exceeding 12 ms also have no intervening SportModeState packet in the bag.
This points upstream of the adapter for those gaps, but cannot distinguish robot
publication from common DDS transport or recording loss. Some smaller source/output
capture differences also exist; this bag is not certified loss-free. No samples
are fabricated, interpolated into the published stream or shifted to hide gaps.

LiDAR error-state codes are zero, but the vendor cloud_packet_loss_rate field is
occasionally nonzero (maximum 0.552486 in vendor field units); IMU loss field is zero.
No unit conversion or unsupported percentage interpretation is assumed. These
status fields do not independently certify completeness of the body-IMU topic.
A fixed offset cannot repair missing data. Gap/freshness behavior remains a concern
for later saved-bag validation and future mapper launch monitoring.

## Decision and checkpoint

- Preserve original sensor timestamps and the existing single-owner robot clock.
- Keep nominal imu_time_inte=0.004; the enabled IMU path uses actual source dt.
- Leave common.time_lag_imu_to_lidar=0.0 unchanged as a **provisional starting value**.
  Neither -1.5 ms from the gyro comparison nor +23 ms from the noisy floor fit is
  a validated cloud/body acquisition correction.
- Relative clock/units/coverage and IMU response checks have useful positive evidence;
  exact cloud acquisition lag and loss-free operation are not established.
- Request the user's check that the overview plot reflects their pitch-then-roll
  test. Step 4 remains the current checkpoint; no step-5 code is approved/applied.

Evidence: `input_timing_integrity.json`, `clock_capture_crosscheck.json`,
`gyro_lag_analysis.json`, `cloud_lag_analysis.json`, the standalone PNG/PDF figures,
`decoded_samples.npz`, and the unchanged `bag/`.
