# Point-LIO acquisition-gap recovery: lab bag trial

The isolated prototype recovered LiDAR tracking after both acquisition gaps in the existing lab recording. Horizontal start/finish offset fell from **10.39 m to 58.6 cm**. It did not eliminate accumulated drift or add global loop closure. The active package, launch, TF, IMU adapter and calibration files remain unchanged; see `active_hashes_after.json`.

## Results

Same source bag: `/home/jain/go2_bags/go2_20261018_173219`. The replay preserves the original acquisition timestamps and both gaps, excluding only the first 1.5 seconds of initial body motion, as in the original baseline. Quiet-pose windows are 2.6–3.6 s and 210–216 s relative to the first original raw-cloud stamp.

| Estimate | Horizontal return offset | Height change at return | Heading change at return |
|---|---:|---:|---:|
| Original standalone Point-LIO | 10.388 m | −49.0 cm | +62.37° |
| Point-LIO with this gap recovery | **0.586 m** | **−3.22 cm** | **−6.70°** |
| Raw vendor odometry | 1.083 m | −0.12 cm | −7.11° |

These are estimated start/finish differences. The physical return was approximate, so they are not surveyed drift measurements. Vendor height is not a height reference. The earlier repository yaw-corrected vendor result was 15.5 cm on this bag, smaller than this trial's 58.6 cm; that correction is not part of this trial.

See [route and height plot](run_01/analysis/lab_comparison.png), [detailed comparison](run_01/recovery_comparison.json), and [full analysis](run_01/analysis/comparison.json).

## What changed

The tested standalone mapper is copied under `src/point_lio`; its executable is installed only under this directory. The new logic is disabled by default and enabled explicitly in `offline.launch.py`.

1. Detect forward gaps over 0.15 s using IMU acquisition timestamps.
2. Discard scans overlapping a gap rather than propagating stale motion through the missing interval.
3. Seed the next complete scan using vendor relative motion since the most recent accepted LiDAR pose. Convert vendor body pose to the developer URDF's IMU origin using the existing mount `[-0.02557, 0, 0.04232]` m. Vendor aid is used for reseeding, not as a continuous replacement for Point-LIO output.
4. Reinitialize dynamic state and its covariance; retain the LiDAR map, gravity, bias and extrinsics. Covariance priors are trial assumptions, not newly measured sensor calibration.
5. Withhold public poses, registered clouds and map insertion until three consecutive scans meet the local old-map agreement gate. The gate requires at least 25 point-to-plane matches and a 25% match fraction, with absolute residual at most 0.15 m.

Recovery after the 0.19985 s IMU gap resumed at source time 28.5722 s. Recovery after the 3.85634 s IMU gap resumed at 48.4647 s. Both passed the three-scan gate immediately. The second event's three matched fractions were 302/312, 238/242, and 215/237, with inlier RMS residuals 3.09, 4.25 and 4.44 cm.

The original gap caused roughly 158° of estimated orientation change; the recovered gap-to-gap public poses changed by 50.60°, close to the vendor's roughly 53° motion. There were exactly two reseeds, followed by continued LiDAR estimation to the end of the bag. Missing observations remain missing.

## Validation

- Isolated CMake Release build completed; the active installed executable was not replaced.
- C++ helper tests passed: transformed relative pose, velocity, gap interpolation rejection, stale/duplicate samples and missing aid.
- All **396** compared pose messages before source time 28 s were exactly identical to the original baseline, including orientation.
- Full replay at 1× on localhost DDS domain 211 completed. The observer received the exact expected sequences of **3,267 clouds** and **61,864 calibrated IMUs**. These are observer delivery checks, not internal per-callback instrumentation in the estimator.
- **3,249** finite poses and registered clouds; no backward pose timestamps; every pose had an exact-stamp `map → odom` transform. No conflicting TF parents. Median contiguous output rate 15.41 Hz; largest contiguous position step 7.34 cm.
- The five fewer outputs than the baseline correspond to one discarded overlapping scan and two unpublished probation scans per gap.
- No TF bridge expiry or overflow. Its temporary lookup failures were retries waiting for exact-time recorded TF; all output poses eventually received a transform.
- Negative test on domain 212 removed vendor odometry from replay. Output stopped at source 28.1187 s while raw input continued to 42.0520 s; zero poses were published after the first gap and no reseed was applied. See `missing_aid_test/validation.json`. This test was intentionally stopped early, so full-bag delivery checks in its generic runner result are false by design.
- Original recordings were not modified. Only test-owned processes were stopped, and no robot commands were sent.

## Remaining work and limits

This repairs the observed acquisition-gap failure for this recording. It is not global relocalization, place recognition or a pose-graph/loop-closure backend. A robot can still accumulate drift while continuously tracking, or lose tracking for reasons other than a detected timestamp gap. Local planar agreement is not a proof of unique global pose. The remaining 58.6 cm offset is not automatically corrected.

Revisited floor candidates shared 267 map cells: median absolute height difference 2.53 cm, 90th percentile 5.20 cm. Horizontal correspondence is approximate; this is not absolute vertical accuracy. There is still a roughly 22 cm height excursion before the long gap, so endpoint height alone does not validate height throughout the route.

The zero LiDAR/body-IMU acquisition-lag correction remains uncalibrated. This trial did not change sensor timing or establish exact synchronization.

Before active deployment, review this trial and add explicit recovery/health status for consumers, a bounded retry policy, and further recordings with vendor discontinuities or silent scan-matching failure. Correcting accumulated drift requires a separate validated loop-closure/localization backend. The trial currently exposes recovery events in logs and withholds output when it cannot recover; it does not command the robot or implement navigation stopping behavior.

## Reproduce the isolated build and replay

From `/home/jain/Doggy`:

```bash
source scripts/setup_ros.sh
cmake -S reports/point_lio_recovery/20261018/src/point_lio \
  -B reports/point_lio_recovery/20261018/build/direct \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/home/jain/Doggy/reports/point_lio_recovery/20261018/install/point_lio
cmake --build reports/point_lio_recovery/20261018/build/direct -j2
cmake --install reports/point_lio_recovery/20261018/build/direct
export ROS_DOMAIN_ID=211 ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI=/home/jain/Doggy/reports/bag_comparisons/go2_lab_20261018/cyclonedds.xml
export ROS_LOG_DIR=/home/jain/Doggy/reports/point_lio_recovery/20261018/repeat_ros_logs
python3 -u reports/point_lio_recovery/20261018/run_replay.py --run-name repeat_01
```

Use a fresh run name each time. The analysis scripts default to the completed `run_01`. `trial.patch` includes the mapper diff and the new helper header for review; applying it to the active package is a separate step.
