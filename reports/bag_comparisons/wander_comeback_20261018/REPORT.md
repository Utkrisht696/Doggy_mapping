# Wandering and comeback recordings — 18 October 2026

Both comeback recordings show metre-scale return error in the current Point-LIO setup. Raw vendor odometry has a smaller horizontal return offset on both. The soft-turn recording returns close to its initial estimated heading while retaining a large position error, so final heading alone cannot validate tracking.

Two new bags evaluated at normal playback speed using the existing isolated standalone Point-LIO recovery build, unchanged approved body-IMU calibration and geometry, and the same profile/startup override as previous comparisons. No per-bag scale, lag, extrinsic or noise tuning. No active application changes.

The user previously specified that the recordings return to their starting pose. This is used as the physical return reference, consistent with both filenames. Endpoint placement and heading uncertainty are not surveyed; return offsets include that uncertainty. Vendor odometry shares body IMU information and is not ground truth.

## Results

| Bag | Duration | Point-LIO return XY | Vendor return XY | Point-LIO return height | Vendor return height | Point-LIO / vendor heading change |
|---|---:|---:|---:|---:|---:|---:|
| `small_wander_comeback` | 71.4 s | **7.92 m** | 0.68 m | -73.4 cm | +0.0 cm | -22.81° / +0.50° |
| `wander_soft_turn_comeback` | 141.9 s | **3.51 m** | 0.71 m | -43.3 cm | +0.0 cm | +0.28° / -2.84° |

These are differences between median quiet-hold poses at the developer body-IMU origin. Each estimate is centered at its own initial pose for plotting, using only initial heading and translation, with no scale fitting or end alignment. Estimated travelled distances are derived from odometry and are not measured route lengths.

## Replay and input checks

### `small_wander_comeback`

- All 2 source segments read through their footers with available CRCs validated. Message counts match both per-file statistics and bag metadata.
- 1,101 raw clouds and 20,954 deduplicated calibrated IMU samples. Largest acquisition gaps: cloud 69.8 ms, IMU 19.9 ms. No backward acquisition stamps.
- Full 1× playback completed. Observer raw-cloud and IMU sequence matches: True / True.
- 1,090 finite output poses; 0 backward/duplicate pose timestamps; 0 output gaps above 150 ms; 0 output stamps missing exact-time map-to-odom TF.
- Recovery reseeds/resumes: 0 / 0. No vendor pose reseeding is used when these counts are zero.
- Quiet analysis windows: [0.82319169, 1.5643396569999999] s and [68.164339657, 71.064339657] s from the first original raw-cloud header.
- Start/finish Point-LIO hold samples: 12 / 45; position 95th-percentile radius within each hold: 2.80 / 3.71 cm.
- Estimated observed travel (not surveyed): Point-LIO 27.0 m; vendor 23.3 m.

[Trajectory, height and heading](small_wander_comeback/run_01/analysis/comparison.png) · [Heading diagnostic](small_wander_comeback/run_01/analysis/heading_comparison.png) · [Detailed numeric results](small_wander_comeback/run_01/analysis/comparison.json)

### `wander_soft_turn_comeback`

- All 3 source segments read through their footers with available CRCs validated. Message counts match both per-file statistics and bag metadata.
- 2,188 raw clouds and 41,521 deduplicated calibrated IMU samples. Largest acquisition gaps: cloud 75.9 ms, IMU 22.4 ms. No backward acquisition stamps.
- Full 1× playback completed. Observer raw-cloud and IMU sequence matches: True / True.
- 2,176 finite output poses; 0 backward/duplicate pose timestamps; 0 output gaps above 150 ms; 0 output stamps missing exact-time map-to-odom TF.
- Recovery reseeds/resumes: 0 / 0. No vendor pose reseeding is used when these counts are zero.
- Quiet analysis windows: [0.89030509, 6.057172143000001] s and [137.05717214300003, 141.557172143] s from the first original raw-cloud header.
- Start/finish Point-LIO hold samples: 80 / 69; position 95th-percentile radius within each hold: 3.09 / 3.18 cm.
- Estimated observed travel (not surveyed): Point-LIO 36.7 m; vendor 29.8 m.

[Trajectory, height and heading](wander_soft_turn_comeback/run_01/analysis/comparison.png) · [Heading diagnostic](wander_soft_turn_comeback/run_01/analysis/heading_comparison.png) · [Detailed numeric results](wander_soft_turn_comeback/run_01/analysis/comparison.json)

## Interpretation and limits

These recordings include turns in both directions with little net body heading change, in contrast to the large accumulated heading changes in the earlier repeated-rectangle recordings. Neither recording establishes usable return consistency. A near-zero final heading change in the soft-turn bag does not undo its position or height drift. These findings extend the earlier repeated-rectangle failures to wandering routes. A source-clock continuity check does not prove exact LiDAR/body-IMU acquisition synchronization; the zero acquisition-lag setting remains uncalibrated.

The gyro diagnostic integrates calibrated angular velocity with source timestamps and compares it with body orientation and Point-LIO. These signals share IMU information; none is an externally surveyed orientation reference.

The small recording has a short initial quiet interval. Its analysis uses the available initialized Point-LIO output inside that interval with a 0.05 s margin, rather than the longer margin used in the previous batch. At least eight pose samples are required. The selected window and within-hold spread are saved above; the estimator itself is unchanged.

Passive replay uses localhost-only DDS domains 218 and 219. Only raw cloud, calibrated body IMU, vendor odometry, source-derived clock and canonical TF are replayed. Recorded control topics are excluded, no robot commands are sent, and RViz is not started. Every test-owned replay process is stopped after saving outputs.

Observer delivery audits do not independently instrument every internal estimator callback. Output continuity is checked separately. Original recordings remain unchanged. The mapping/TF bridge and calibration hashes are checked against their values before this task.

## Reproduce

Input preparation, quiet-window detection, timing audit and passive replay use the saved utilities in `../new_routes_20261018/`. This directory contains the resulting inputs, manifests, logs, output arrays, dedicated analysis script and summary. The replay manifest records the full source path and estimator settings.
