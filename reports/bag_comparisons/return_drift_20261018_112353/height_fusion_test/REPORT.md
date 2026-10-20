# Causal height-fusion investigation — 19 October 2026

**Historical status:** on 20 October, the user approved making this filter the
permanent mapping output. It is now integrated into the current launches; see
[production verification](../../../height_fusion_20261020/REPORT.md).
The statements below about an uninstalled prototype describe the 19 October tests.

A separate, causal height filter successfully combines the saved Point-LIO pose with vendor height on two recordings. It preserves Point-LIO X/Y and quaternion orientation exactly and applies the same height correction to the registered clouds and canonical TF. It is **experimental, not installed in the live robot launch**. World elevation on real slopes/drains has not been validated.

The expanded comparison is in [batch_20261019/REPORT.md](batch_20261019/REPORT.md). It finds substantial horizontal drift on both wander recordings, real source-data gaps, accelerated-replay output backlog, missing processed scans after a recording-delay burst, and unresolved replay repeatability. The successful examples below do not establish general reliability. Active launch/configuration/calibration remain unchanged; no multi-bag result justifies calling this fully fixed.

## Accepted implementation

`height_output_core.py` and `fuse_height_output.py` implement scalar position-height fusion. Prediction uses Point-LIO's change in Z. The measurement is `/utlidar/robot_odom` body-origin Z plus the unchanged developer body-to-IMU mount rotated by the vendor quaternion. A single initial height offset anchors the two coordinates. Vendor X/Y/yaw do not enter the height filter. It never feeds back into Point-LIO.

Only a previously arrived measurement with source stamp at or before the LIO stamp and age at most 50 ms is eligible. Missing, stale, backward, duplicate, invalid and implausibly jumping vendor measurements are rejected or bypassed. Missing height causes prediction using LIO increments, with increasing uncertainty. It does not freeze altitude or pretend height remains measured.

Assumed measurement standard deviation is 0.01 m; random-walk process variance is 0.0025 m²/s. The source jump bound is 0.02 m plus 1 m/s times source time difference. The innovation gate is eight standard deviations and 0.5 m maximum. These are explicit experimental assumptions, **not manufacturer-calibrated uncertainty**. The sources share body-IMU information. This is a pose/cloud/TF experiment; the upstream twist is retained, and controller-facing twist frame semantics and full covariance still need separate review.

Raw estimator topics use `lio_local`. Fused poses/clouds use `map`. The existing TF bridge subscribes to fused odometry and is the sole publisher of `map → odom`; recorded canonical body/sensor TF remains unchanged. The clock and all observation source stamps are preserved. No future loop constraint, final-return fit, trajectory scale fit or offline Z substitution is used.

## Recorded-data results

Return errors use the existing independent quiet-cloud registrations. Those references are LiDAR-derived, not surveyed external ground truth; millimetre numerical differences do not establish millimetre absolute accuracy. The position comparison is expressed in the initial IMU coordinates, so tilt and horizontal errors can also affect its Z component.

| Run / return | Height discrepancy | Horizontal error | Heading error |
|---|---:|---:|---:|
| Latest original Point-LIO, first return | −34.52 cm | 13.94 cm | −0.44° |
| Latest causal output fusion, first return | −0.25 cm | 14.10 cm | −0.44° |
| Latest original Point-LIO, final return | −34.95 cm | 13.91 cm | −0.37° |
| Latest causal output fusion, final return | −0.09 cm | 14.07 cm | −0.37° |
| Other bag, raw Point-LIO final return | −2.35 cm | 73.66 cm | −0.92° |
| Other bag, fused final return | +1.34 cm | 73.59 cm | −0.92° |

The small differences in the horizontal error scalar despite exact map-frame X/Y preservation arise from expressing translation in the initially tilted IMU coordinates after changing Z. `output_causal_1x/raw_outputs.npz` is exactly equal to the saved baseline estimator trajectory. The fusion does not fix the other bag's underlying ~0.74 m horizontal drift or Point-LIO tilt errors.

The other bag is `wander_soft_turn_comeback`. Its separate replay copy was regenerated from the original SportModeState using the current approved body-IMU calibration. Sensor stamps, receipt order, raw LiDAR and vendor measurements are unchanged. Its independent endpoint registration passed multi-start, overlap/geometry and reverse-cycle gates (2.9 mm reverse translation disagreement, 0.052° rotation disagreement). It was not used to tune the filter settings.

## Delivery and consistency checks

- Latest bag at 1×: 5,024 raw scans, 95,186 calibrated IMU messages, 48,804 vendor odometry messages; 5,010 matched raw/fused pose/cloud outputs and TF stamps, about 15.4 Hz.
- Other bag at 2×: 2,188 raw scans, 41,521 calibrated IMU messages, 21,248 vendor odometry messages; 2,173 matched output pairs and TF stamps.
- Both: exact observed input delivery, no output gaps over 150 ms, no missing cloud/TF stamps, no conflicting TF parents, no future measurements used, exact preserved X/Y/quaternion and cloud X/Y. Maximum cloud Z correction discrepancy is 0.24 micrometres from float32 rounding.
- Missing-vendor integration control at 2×: full latest bag, zero vendor messages, all 5,010 output pairs and TF stamps. Maximum pose difference from raw Point-LIO is 5.6e-17. No stale height is invented.
- Focused unit checks passed for missing/stale/future/duplicate/NaN/outlier inputs, prediction fallback and a synthetic ramp **when the vendor reference itself tracks the ramp**. That synthetic case does not establish actual vendor elevation behaviour.
- Protected application/configuration/calibration hashes match before and after every accepted replay. Original source bags are unchanged. No robot commands were replayed or sent. ROS domains are localhost-only with multicast disabled.

## Earlier failures and a corrected map diagnosis

The private, fully coupled height update inside the Point-LIO ESKF was rejected. With 3 cm assumed noise it still had ~24.7 cm final height error; with stronger weight the horizontal estimate diverged catastrophically. The stronger test was stopped. This is a failure of the tested prototype, not proof that all estimator-level fusion designs are unsuitable. These builds/launches must not be promoted.

The previous offline pose graph aligned the quiet returns but substantially distorted intermediate heights. The huge dense patch was primarily introduced by that correction. In the same selected 5 cm XY region:

| Map | Occupied cells | Median absolute floor-like repeat-pass difference |
|---|---:|---:|
| Original Point-LIO without loop correction | 785 | 1.44 cm |
| Previous offline loop correction | 5,027 | 21.65 cm |
| New causal height fusion without loop correction | 583 | 2.96 cm |

The causal filter improves the bad return-height discrepancy but is **not uniformly more accurate everywhere**. Its local floor consistency in that checked region is worse than original Point-LIO, despite fewer occupied cells. Occupied-cell counts are processing diagnostics, not independent obstacle truth. The earlier explanation that the loop correction merely left residual original height error was incomplete.

## Real terrain limitation

The operator reports drains and lower areas, and thinks the starting areas of the latest and wander-comeback bags are the lowest. The elevation differences and exact drain locations are unmeasured. This overrides any interpretation of the route as perfectly level.

Vendor odometry Z stays roughly 27.6–32.4 cm in the latest recording and is strongly correlated with the separate vendor body-height field. Neither that correlation, a stable value nor successful balancing proves world elevation accuracy. A robot can keep the same clearance above the local floor while its elevation relative to the starting floor changes. We cannot yet tell whether this published Z preserves the real rises/dips.

Before promotion, measure a real elevation change (for example a safe existing ramp with measured top/bottom height), record pauses at the bottom/top/bottom with unchanged standing posture, and compare vendor Z change, raw LIO, fused pose and LiDAR-measured local terrain. It must preserve the measured rise and return height, not merely produce a flat-looking map. Do not use an improvised loose platform or command automated robot motion for this test.

## Reviewable artifacts

- `VALIDATION.json`: accepted recorded-data health, geometry and limits.
- `output_causal_1x/analysis.json`, `comparison.png`: latest-bag metrics.
- `output_heldout_2x/reference_comparison.json`: other-bag comparison.
- `output_missing_vendor_2x/result.json`: full missing-height control.
- `output_causal_1x/three_way_floor_consistency.json`, `patch_counts.json`: map diagnosis.
- `maps/robot_20261018_112353_causal_height_fusion/`: 5 cm ROS occupancy map, wall/traversability exports and previews.
- `output_fusion.launch.py`: private localhost replay integration; normal live launch remains untouched.

RViz displayed both accepted recorded-data replays live. Playback and mapper processes stopped after completion; the viewer may remain open. No physical robot run has been performed for this fusion layer.
