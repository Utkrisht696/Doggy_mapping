# New-route Point-LIO validation — 18 October 2026

The new recordings expose substantial failure on routes with repeated turns. Short straight-distance scale is close to the nominal distances, but the present configuration is not validated for longer autonomous missions. No active source, calibration, TF or launch settings were changed.

The user confirmed that all recordings should finish at the original starting pose. The newest 5 m/10 m out-and-back, two 1×3 m rectangles and one 1×1 m rectangle sequence was confirmed roughly correct. The earlier measured route was confirmed to walk 5 m, pause, then continue another 5 m to 10 m total, followed by rectangles and wandering. Physical endpoint uncertainty was not surveyed.

## Return consistency on recordings with continuous sensor data

| Bag | Replay | Point-LIO horizontal return offset | Raw vendor offset | Point-LIO height change | Point-LIO / vendor heading changes |
|---|---|---:|---:|---:|---:|
| `doggy_real1` | 315 s source; 2× | **14.03 m** | 4.94 m | -19.6 cm | +46.5° / -28.7° |
| `5m_10m_1x1m_1x3m_1x3m_wander_return_start` | 353 s source; 1× | **6.80 m** | 3.55 m | -31.1 cm | +19.3° / -42.3° |
| `5m_10m_1x3m_1x3m_1x1m_return_start` | 276 s source; 2× | **4.26 m** | 4.84 m | +0.1 cm | +23.3° / -28.5° |

These are quiet-hold start/finish differences at the developer IMU origin, not independently surveyed errors. Metre-scale offsets are inconsistent with the stated return-to-start route. Heading changes are wrapped endpoint differences; body posture and heading were approximate. Vendor is a comparison estimate, not ground truth.

## Straight distances

| Bag | Nominal | Point-LIO | Raw vendor | Interpretation |
|---|---:|---:|---:|---|
| `doggy_real1` | 5 m | 5.004 m | 4.438 m | Out-and-back far-end displacement; intermediate route inferred |
| `doggy_real1` | 10 m | 10.178 m | 9.003 m | Out-and-back far-end displacement; intermediate route inferred |
| `measured_route_1` | 5 m | 5.033 m | 4.455 m | Cumulative checkpoint on same straight leg |
| `measured_route_1` | 10 m | 10.175 m | 8.972 m | Cumulative checkpoint on same straight leg |
| `measured_route_2` | 5 m | 5.057 m | 4.470 m | Out-and-back far-end displacement |
| `measured_route_2` | 10 m | 10.229 m | 9.021 m | Out-and-back far-end displacement |

The newest measured route returned within about 9 cm after each straight out-and-back in Point-LIO, before the repeated rectangles. Nominal straight distances are user references; these are horizontal endpoint displacements rather than integrated curved travel. No distance-scale fit was applied.

## Heading during turns

On the completed clean recordings, Point-LIO separates substantially from the body orientation during repeated turns. A separate diagnostic integrates calibrated body gyro measurements as full 3D quaternion increments, using source acquisition timestamps with trapezoidal angular velocities. No scale or time-offset fit was made. The integrated gyro finishes roughly 4–6° from the body quaternion; Point-LIO finishes tens of degrees away. This localizes the disagreement to the turning portion of the estimator pipeline; it does not prove a particular timing, calibration or scan-matching cause.

Body quaternion and vendor orientation agree almost exactly and share sensor information. They are not two independent references. Both their endpoint heading and Point-LIO’s endpoint heading differ from the stated original heading, so the body quaternion must not be treated as surveyed yaw truth.

The normal-speed repeat of the earlier measured route also fails: 6.80 m return offset versus 12.55 m at 2×. Two shorter output pauses remain near source 331–332 s (about 0.26 s each, versus 0.91/0.97 s at 2×), with continuous recorded sensor input. The pause is therefore not solely an accelerated-playback artifact. The final trajectory also depends substantially on replay scheduling and/or evolving map/filter state; the cause has not been instrumented. The 1× result is primary, and the accelerated endpoint must not be presented as a reproducible accuracy result. Consult `replay_speed_comparison.json`.

## Dropout stress recording

`doggy_real2` has 18 IMU acquisition gaps over 0.15 s, the largest about 9.12 s. It also starts during motion; replay omits the first 14.38 receipt seconds and initializes at the first detected low-motion interval. That interval is less stationary than the other bags. Its first processed hold is therefore not the original physical recording-start pose, and no full-route return-error number is claimed for it.

The recovery trial’s reseed/resume counts, original gaps and quarantined outputs are saved in its detailed analysis. Passing a local map-agreement gate after a gap does not validate global tracking or loop closure. One exact-time map-to-odom transform was withheld/expired during this stress replay; the bridge did not invent a replacement.

## Integrity and replay checks

All 21 source MCAP segments were read through to their footers with available chunk and summary CRCs validated. File message counts agree with per-file statistics and aggregate bag metadata. Original source sizes and modification times were checked unchanged. Required point clouds and body states decode; the existing converter rejects inconsistent timestamps and drops identical duplicate body samples. Sparse raw/body cloud point matches still agree with the approved geometry within micrometres.

The three clean recordings have maximum IMU gaps of about 22–28 ms and no backward raw-cloud, calibrated-IMU or vendor acquisition timestamps. Gap recovery is not triggered in those recordings. Their failures therefore cannot be attributed to the old four-second acquisition gap.

Full replays use only raw cloud, calibrated body IMU, recorded vendor odometry, canonical derived TF and a source-header-derived clock. Each uses a distinct localhost-only DDS domain (213–217). No recorded control topics are replayed, no robot commands are issued, and RViz is not launched. The same existing standalone recovery binary and unchanged profile are used throughout, with the previously tested 4,000-point startup override and scan-rate output. No per-bag tuning is performed.

Observer delivery matches the exact expected raw-cloud and calibrated-IMU header sequences in every full replay. This verifies observer delivery, not every internal callback in the estimator. Pose timestamp continuity is checked separately. The accelerated earlier measured-route replay has two approximately one-second output gaps despite continuous source data, which motivated a full 1× repeat. At 1×, those gaps shrink to about 0.26 s each, but remain. Internal callback delivery/backpressure and estimator scheduling need diagnosis. The accelerated run must not be used alone to characterize ordinary tracking.

The newest clean bag has one final pose without a map-to-odom transform: the requested time is about 0.44 ms newer than the last recorded-derived TF sample after TF throttling. It is confined to the end of the replay; the quiet endpoint window ends earlier. This is a diagnostic TF coverage limitation, not a fabricated late transform or an explanation of metre-scale drift.

Original bag receipt clocks differ from robot header clocks by about 27.6 s. That offset is not a measured network delivery latency. The zero LiDAR/IMU acquisition-lag correction remains uncalibrated; successful file playback is not proof of exact acquisition synchronization.

## Artifacts

- [Newest route confirmation plot](measured_route_2/route_confirmation.png).
- `doggy_real1`: [trajectory/height/turns](doggy_real1/run_01/analysis/comparison.png), [detailed results](doggy_real1/run_01/analysis/comparison.json), [input audit](doggy_real1/input_audit.json).
  [Heading signals](doggy_real1/run_01/analysis/heading_comparison.png).
- `measured_route_1`: [trajectory/height/turns](measured_route_1/run_02_1x/analysis/comparison.png), [detailed results](measured_route_1/run_02_1x/analysis/comparison.json), [input audit](measured_route_1/input_audit.json).
  [Heading signals](measured_route_1/run_02_1x/analysis/heading_comparison.png).
- `measured_route_2`: [trajectory/height/turns](measured_route_2/run_01/analysis/comparison.png), [detailed results](measured_route_2/run_01/analysis/comparison.json), [input audit](measured_route_2/input_audit.json).
  [Heading signals](measured_route_2/run_01/analysis/heading_comparison.png).
- `doggy_real2`: [trajectory/height/turns](doggy_real2/run_01/analysis/comparison.png), [detailed results](doggy_real2/run_01/analysis/comparison.json), [input audit](doggy_real2/input_audit.json).

The next investigation should identify the turning-motion disagreement using LiDAR geometry, IMU measurements and acquisition timing. These data do not justify a calibration change or an arbitrary heading/distance multiplier. The existing gap recovery is insufficient to establish long-route reliability.
