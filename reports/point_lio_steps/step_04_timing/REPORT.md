# Step 4 — timing and clocks (measured; acquisition calibration remains open)

The user accepted step 3 and requested timing checks. No application code,
configuration, sensor settings or system clock has been changed in step 4.
No mapper, replay, RViz or control routine has been started.

## Fresh passive capture

45 seconds, 694 raw clouds, 13,362 calibrated body-IMU messages, 13,327 SportModeState
messages, 11,088 original LiDAR IMU messages, 6,817 vendor odometry messages and
1,909 private clock messages. No decoding errors. Measurements are saved with exact
integer source, host and monotonic timestamps in `live_timing_capture.npz`.
The original invalid LiDAR accelerometer is observed only for diagnostics, never
selected as Point-LIO input.

Selected body-IMU stamps and cloud headers are strictly increasing, with no duplicate
or backwards output stamps. 13,231 source/output packets match at EXACT source
stamps and fixed-bias corrected six-channel values (zero comparison error).

All 694 retained scan point intervals have body-IMU samples bracketing both ends.
There are 12–20 body-IMU samples inside each interval (median 18). The median scan
point span is 59.30 ms. No retained point-time interval overlaps/reverses into the
next scan. Point timestamps are finite, nonnegative and in seconds; the parser’s
float32 seconds-to-milliseconds conversion changes endpoint time by at most 2 ns.

Raw point records sometimes arrive out of time order (182/694 scans). This port
sorts retained/downsampled points by time before integrating them, so this is not
a reason to reorder the published raw stream or change acquisition timestamps.

## Clock and transport observations

The laptop receives source-stamped body IMU roughly 13.025 seconds after its header
time, while clouds include their scan assembly time (median header age 13.091 s;
median age relative to last retained point 13.031 s). This difference contains the
laptop/robot epoch difference and transport/assembly latency. It does not determine
an IMU-to-LiDAR physical acquisition lag. Do not subtract 13 seconds from one sensor,
retimestamp on receipt, or change the laptop clock to make RViz look current.

There is exactly one `/doggy/robot_clock` publisher (`doggy_go2_state`) and no
observed global `/clock` publisher. Live parameter reads confirmed use_sim_time=true
on both model and cloud-preparation nodes. Their existing launch remaps `/clock`
to the dedicated robot clock. The body-IMU adapter preserves source stamps directly;
its health timer need not supply measurement time. Future Point-LIO/RViz launch
must use the same clock policy, with one separate bag clock during replay.

One clock stamp was absent from the initial BEST_EFFORT odometry subscription;
1,908/1,909 matched exactly. This requires an independent reliable-clock capture
rather than treating subscriber loss as a clock defect. The independent 20-second
RELIABLE high-depth capture passed: all 759 interior clock samples match BOTH
vendor and relayed odometry stamps exactly, with no missing stamps or clock
backsteps. See `clock_reference_validation.json`. The initial best-effort result
is retained as capture-loss evidence; it is not silently rewritten.

Median body-IMU timestamp interval is 3.965 ms; mean distinct-stamp rate is 296.82 Hz.
Intervals vary, so replacing 0.004 seconds by 1/296.82 is not justified. The port’s
IMU-enabled propagation uses actual source timestamp differences. The current
0.004-second parameter is nominal and is not used to stamp or resample data.
Maximum observed body-IMU gap was 15.20 ms. This capture does not establish a
hardware rate or guarantee completeness under every host load/motion condition.

99/694 clouds arrived before the audit callback received the IMU sample bracketing
the end; 95th percentile extra wait across all scans was 15.13 ms, maximum 156.0 ms.
All eventually had coverage. The port explicitly waits until last_timestamp_imu
covers the scan end. This is transport/callback behavior, not evidence for applying
that delay as a constant acquisition-time correction.

## Remaining acquisition-lag check

No source-stamp or point-time-unit fix is justified by the observed input data.
The configured inter-sensor lag remains 0.0, UNCALIBRATED. Timestamp overlap alone
cannot certify physical acquisition synchronization. A controlled posture capture
with sufficient angular excitation and source-stamped body/LiDAR streams is needed
to estimate relative gyro response timing; any estimate must be repeatable across
axes/windows and distinguish processing/filter delays from the cloud time origin.
It cannot by itself prove the hidden DDS producer’s hardware acquisition clock.

The old manual-motion bag includes raw LiDAR IMU/cloud/odom and low-level body IMU
with a boot tick, but no `/sportmodestate` TimeSpec or calibrated output. Its prior
axis comparison used receipt-time alignment. It is useful diagnostic evidence but
cannot directly establish an acquisition-time correction for the selected stamped
body route without another timebase assumption. The user was asked whether to
perform a short manually controlled posture test or defer it; no motion command
has been sent by this work.

References checked: official Unitree L1 SDK PointUnitree time field definition
https://github.com/unitreerobotics/unilidar_sdk/blob/main/unitree_lidar_sdk/include/unitree_lidar_sdk.h
and ROS clock design https://design.ros2.org/articles/clock_and_time.html.
The installed Point-LIO parser, synchronization and propagation code were inspected
directly at the pinned checkout; upstream online configuration was cross-checked.


## Checkpoint

**Source-stamp, point-time, IMU-coverage and clock-reference checks PASS.**
**Acquisition lag is NOT yet calibrated; step 4 remains open.**
There is no evidence-based timestamp correction to apply at this checkpoint.
The active Go2 profile hash still matches the approved step-3 hash exactly.

For a subsequent manually controlled posture capture, record raw clouds, both
IMUs, SportModeState, vendor odometry and the existing clock. Allow stationary
baselines, then gentle alternating pitch/roll and return to neutral with feet
planted. No automated motion routine is authorized. Start recording and give the
user a clear cue before requesting posture changes. Review timing identifiability,
repeatability and source/transport effects before proposing any offset change.


## First controlled-motion recording failed

The first posture recorder aborted with a SQLite database-lock exception before
the movement cue. Its database is intact but contains only 6.032 seconds, ending
before the cue. All requested topics were initially subscribed and the file had
grown, but that was insufficient evidence that it stayed alive. An attempted
read-only inspection of the active database may have interfered with the writer;
the exact contention was not independently reproduced. The failure was discovered
when the user said done. The user was informed promptly, and no movement-timing
result is claimed. The original short database and failure manifest are preserved
in `posture_20261016T191129+1100/`.

The user clarified that “more than 5” meant more than 5 degrees of tilt, not more
than five repetitions. Larger comfortable angles do not themselves invalidate
timing estimation; actual motion and signal quality must be measured from data.

Recovery: qualify the recorder for 30 seconds, stop it cleanly, then inspect its
closed database and metadata. Never open a live recording database for inspection.
For future captures, monitor only process liveness, recorder log and file-size
progress while writing; inspect topic counts/data only after the recorder exits.


Recorder recovery verification PASSED: clean exit, SQLite quick_check=ok, all ten
topics saved with dynamic-topic spans over 25 seconds, metadata/database message
counts match, no recorder errors. Saved 31,331 messages in
87,281,664 bytes. Evidence: `reports/point_lio_steps/step_04_timing/recorder_validation_20261016T191547+1100/verification.json`.
The missing controlled-motion test still needs repeating; no timing correction
has been applied.


## Completed repeat posture analysis

The repeat recording saved successfully and has now been analysed. Current result:
`posture_timing_result.json`. Full report:
`posture_repeat_20261016T191738+1100/REPORT.md`. The relative gyro response offset is
about 1–2 ms; all 2,763 interior scan intervals have IMU bracketing. The direct
cloud-floor check cannot calibrate a precise acquisition offset. Upstream/capture
gaps up to 33.95 ms were observed. No active settings changed; zero lag remains a
provisional starting value. Awaiting user plot double-check before further work.


## Receive-buffer fix approved and tested

The temporary laptop receive-buffer change is applied and verified: host UDP
overflows 1182/6708 -> 0/0, adapter socket drops 107/456 -> 0/0. Calibrated-output
misses still remain (79 baseline, 302 under recording load); exact cloud acquisition
lag is not calibrated. Full result: `dropout_qos/buffer_test_runtime/REPORT.md`.
The subsequent user-approved RELIABLE/depth-2000 source patch is now applied.
Both comparison phases have zero observed missing calibrated samples and zero
receive-buffer drops, but recording-load latency still reaches about 0.6 s.
Step 4 remains open. See `dropout_qos/adapter_qos_runtime/REPORT.md`.


## Delivery-delay investigation: user-selected Performance mode

The lightweight A/B/A audit reproduced CPU backlog in Power Saver when extra
Python audit subscribers were present. The user selected Performance mode; the
unchanged application passed the repeated workload with zero missing observed
output stamps/UDP receive overflows, phase p99 below 1.22 ms and max 4.98 ms.
No app/TF/bias/stamp change or sensor restart was performed. Historical larger
delays above describe earlier workloads, not current Performance-mode results.
See `delivery_delay/REPORT.md` and `delivery_delay/result.json`. Cloud acquisition
lag and estimator-load validation remain separate; step 4 is not fully complete.


## Fresh two-block manual motion test

The 207.61 s fresh recording passed transport/integrity checks: 3,170 interior
scans bracketed, all 60,193 interior source stamps matched, all 8,704 checked
clock stamps matched source/relay. Repeated gyro fits are approximately -1 to
-2 ms. A per-point floor diagnostic remains too insensitive to establish precise
cloud acquisition lag; zero is unchanged and provisional. Source/capture gaps
up to 35.96 ms remain. This supports an experimental saved-bag estimator check,
not a claim of exact calibration. See `final_timing_check/result.json` and the
report it references. User result double-check and output-TF integration precede
the mapper trial; no mapper/replay has started.
