# nromal-np bag: Point-LIO and vendor odometry

Source: `/home/jain/go2_bags/nromal-np/robot_20261018_060254_0.mcap`. Duration **83.641 s**, 163,737 messages.

Operator confirmed the same route as classic: 3 m forward, wait, 3 m back, wait; then 5 m forward, wait, 5 m back, wait.

| Check | Point-LIO | Vendor odometry |
|---|---:|---:|
| 3 m outbound horizontal displacement | 3.0956 m | 2.6567 m |
| 3 m inbound horizontal displacement | 3.0752 m | 2.5619 m |
| 3 m leg return offset | 2.16 cm | 9.80 cm |
| 5 m outbound horizontal displacement | 5.2413 m | 4.4989 m |
| 5 m inbound horizontal displacement | 5.2283 m | 4.3625 m |
| 5 m leg return offset | 10.97 cm | 15.72 cm |
| Final offset from original start, horizontal | 10.75 cm | 23.77 cm |
| Final height change from original start | 1.00 cm | 1.30 cm |

![Comparison plot](analysis/comparison.png)

## Recording integrity

All **1,000 zstd chunks decompress**. Full container traversal, MCAP statistics and metadata message counts agree. Summary CRC matches. No chunk payload CRCs are present, so this is not a per-payload CRC guarantee. All mapping inputs were deserialized. Unrelated message schemas were not fully validated.

Approved raw/body conversion agrees across 10,530 matched points in 12 scans, maximum residual 2.793 micrometres. This validates the recorded topic conversion, not a surveyed mount.

Source MCAP was read-only; source.sha256 identifies it. Original Jazzy metadata was preserved. Separate Humble-compatible SQLite replay includes only raw cloud, calibrated body IMU, vendor odometry and derived clock/TF. No robot command topics replayed.

## Recomputed Point-LIO

No Point-LIO output is present in the source. This replay uses the existing binary, unchanged bias calibration/extrinsics, zero acquisition-lag correction and the same temporary **init_map_size=4000** setting as classic. Headless, 0.5x playback, isolated localhost-only DDS domain 209. Application hashes unchanged: **True**.

Observer cloud delivery matched all 1,289 source stamps exactly: **True**. Calibrated IMU delivery matched all 24,564 stamps exactly: **True**. Duplicate SportModeState samples dropped: 325; converter errors: 0.

Stop reason `playback_complete`, player exit 0, launch exit 0. Point-LIO: **1,279 poses at 15.422 Hz**. Nonfinite pose values 0; nonincreasing stamps 0. Largest single translation step 4.74 cm, rotation 2.50 degrees. Steps >15 cm: 0; rotations >5 degrees: 0.

TF parent conflicts: {}. Missing pose TF stamps 0; extra stamps 0; exact full sequence match True. End-boundary check: `{"missing_pose_count": 0, "missing_only_at_end": false, "all_earlier_pose_tf_sequences_exact": true, "last_pose_beyond_last_derived_vendor_tf_ms": -16.414642}`. Deferred exact-time lookup retries are distinct from dropped outputs; selected endpoint holds precede the recording boundary.

## Timing and endpoint selection

Raw cloud median period 64.903 ms, no duplicate/backward stamps, no gaps exceeding three median periods. Calibrated IMU: 0 backwards stamps, 102 gaps exceeding three median periods, maximum 28.147 ms. Gaps are retained, not filled. Detailed largest gaps and nearby pose steps appear in comparison.json.

IMU recording brackets 1289 scans; 0 scans extend outside its boundaries. Point-LIO first output 0.711 s after first raw cloud header.

Recorder minus source-header time includes a host/robot clock offset; it is not direct network latency. Header times are unchanged. Zero acquisition lag remains uncalibrated; replay success does not establish exact LiDAR/IMU acquisition synchronization.

Endpoint holds were selected independently for this recording from its quiet waits. All times are seconds after the first raw-cloud header. Values are median positions; both estimates refer to the IMU using the developer body-to-IMU lever arm. No scale fit or per-leg pose alignment was used.

- start: 1.5–4.7 s, 49 poses; Point-LIO p95 distance to median 3.01 cm, body gyro RMS 0.0132 rad/s.
- far_3m: 19–22.5 s, 54 poses; Point-LIO p95 distance to median 1.49 cm, body gyro RMS 0.0129 rad/s.
- return_3m: 36.5–40.5 s, 62 poses; Point-LIO p95 distance to median 1.25 cm, body gyro RMS 0.0125 rad/s.
- far_5m: 58.15–59.8 s, 26 poses; Point-LIO p95 distance to median 1.45 cm, body gyro RMS 0.0133 rad/s.
- return_5m: 78.5–82.5 s, 62 poses; Point-LIO p95 distance to median 1.32 cm, body gyro RMS 0.0127 rad/s.

## Vertical consistency

- 3 m leg: Point-LIO far-hold height change -4.44 cm; returned height change 1.36 cm.
- 5 m leg: Point-LIO far-hold height change -10.37 cm; returned height change -0.35 cm.
- return_3m: 558 common 15 cm floor-candidate cells; median absolute difference 2.29 mm, p90 20.78 mm.
- return_5m: 544 common 15 cm floor-candidate cells; median absolute difference 4.63 mm, p90 23.49 mm.

Map XY grid 15 cm, >=5 points per cell, within 3 m of initial IMU and .15-.9 m below it; no ICP/relevelling. Same XY assumes adequate horizontal registration.

An outward height slope that reverses on return can include actual floor slope or gravity/map tilt. Return-height and repeated-cell agreement measure consistency, not absolute vertical accuracy.

## Limits

- Reported 3 m and 5 m are nominal references; physical measurement method and uncertainty were not supplied.
- Curved walking paths: reported hold-to-hold horizontal displacement, not integrated travelled path length.
- No independent endpoint survey. Return offsets include real placement differences and estimator error; they are not pure drift measurements.
- Vendor odometry shares body IMU information and is not ground truth. Its poses include the developer body-to-IMU lever arm.
- Repeated floor-cell agreement measures internal consistency, not surveyed absolute vertical accuracy.
- Zero LiDAR/IMU acquisition-lag correction remains uncalibrated. Replay success does not prove exact sensor synchronization.

Quiet-hold pose variation is included above. Small median return offsets must not be interpreted as an equally precise absolute accuracy guarantee.

Details: [comparison.json](analysis/comparison.json), [input_audit.json](input_audit.json), [source_timing.json](source_timing.json), [result.json](result.json).
