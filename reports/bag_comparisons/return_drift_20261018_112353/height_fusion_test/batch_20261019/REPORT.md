# Fixed-setting multi-bag comparison — 19 October 2026

Nine additional bags were replayed with the same approved body-IMU calibration, unchanged raw Point-LIO estimator and fixed experimental output-height fusion. Replays were localhost-only at 3×, sequential, and headless. No robot commands were sent. Original recordings and active application/configuration files were unchanged.

Errors below are discrepancies against independent quiet start/end LiDAR registrations, not surveyed accuracy. Each reference uses the same multi-start, reverse-cycle, overlap and geometric-information gates. Failed gates are not relaxed to obtain a score. X/Y and quaternion are checked for exact preservation by height fusion; the metric uses initial IMU axes, so a Z change can slightly change the reported XY scalar.

| Bag | Result | Raw LIO XY | Fused XY | Vendor XY | Raw / fused Z discrepancy | Raw/fused heading error |
|---|---|---:|---:|---:|---:|---:|
| small_wander | Reference passed; outputs complete | 64.8 cm | 64.9 cm | 73.8 cm | -3.6 / -0.0 cm | -1.60° |
| classic | Reference passed; 1 missing TF | 2.4 cm | 2.4 cm | 45.0 cm | +0.7 / +0.0 cm | +0.60° |
| default | Endpoint reference failed gates; unscored | — | — | — | — | — |
| normal | Reference passed; outputs complete | 0.9 cm | 0.9 cm | 22.2 cm | +0.5 / +0.4 cm | +0.72° |
| lab | Stopped: impossible_pose_step | — | — | 106.6 cm | — | — |
| real1 | Incomplete fused output at 3× | 44.2 cm | — | 494.3 cm | — | — |
| real2 | Stopped: impossible_pose_step | — | — | — | — | — |
| measured1 | Incomplete fused output at 3× | 212.1 cm | — | 353.4 cm | — | — |
| measured2 | Reference passed; 1 missing TF | 3.9 cm | 3.8 cm | 489.5 cm | +0.1 / +0.2 cm | -0.70° |
| real1_drained | Reference passed; outputs complete | 81.3 cm | 81.3 cm | 494.3 cm | -2.6 / +0.1 cm | -2.20° |
| measured1_drained | Reference passed; missing processed scans | 1.8 cm | 1.8 cm | 353.4 cm | -0.4 / +0.1 cm | -0.42° |

The earlier latest-bag test had about 14 cm horizontal return discrepancy and reduced its roughly 35 cm return-height discrepancy to under 1 cm. The separate soft-wander test had about 74 cm horizontal discrepancy, unchanged by height fusion. Together with the additional bags, the wander result cannot be called an isolated exception. Good short-route closure does not establish reliable longer-route accuracy.

## Gaps and timing

| Bag | Largest LiDAR source interval | Largest body-IMU source interval | LiDAR source intervals >150 ms |
|---|---:|---:|---:|
| small_wander | 69.8 ms | 19.9 ms | 0 |
| soft_wander | 75.9 ms | 22.4 ms | 0 |
| classic | 72.2 ms | 32.2 ms | 0 |
| default | 71.6 ms | 24.0 ms | 0 |
| normal | 75.3 ms | 28.1 ms | 0 |
| lab | 3953.0 ms | 3856.3 ms | 2 |
| real1 | 73.5 ms | 28.3 ms | 0 |
| real2 | 6542.7 ms | 9115.3 ms | 18 |
| measured1 | 78.8 ms | 21.9 ms | 0 |
| measured2 | 71.6 ms | 25.0 ms | 0 |
| latest | 71.4 ms | 23.9 ms | 0 |

- **Lab dropout:** at source time 44.38–48.33 s, Point-LIO jumped 10.43 m across a 3.95 s interval. Vendor displacement over the same interval was 0.83 m (a comparison estimate, not ground truth). The test stopped on the >2 m pose-step guard. Its complete-stream delivery flags are therefore false by design; the observed prefix is checked separately. No endpoint LIO accuracy is claimed for this interrupted run.
- **Real2:** repeated missing source intervals reach 6.54 s in LiDAR and 9.12 s in body IMU. This is a compromised recording for continuous inertial integration. Any estimator result must be interpreted with these holes visible.
- **Wander bags:** largest LiDAR intervals are about 70/76 ms, and body-IMU intervals about 20/22 ms. No large source-data hole explains their return discrepancies. This does not rule out acquisition lag or other calibration/estimation errors.
- **Receipt delays:** soft-wander contains common recording/arrival pauses of about 0.2 s near source times 64 and 128 s, despite continuous header stamps. Measured1 has a roughly 3.7 s receipt-offset step near 329 s across all streams with continuous sensor stamps. These are different from missing acquisition intervals. Host clock adjustment and recording/delivery batching cannot be separated from header/receipt timestamps alone.
- **Physical synchronisation remains uncalibrated.** A common timestamp epoch, continuous source stamps and exact playback delivery do not prove that LiDAR and body IMU measure the same physical instant. No new offset has been fitted or silently installed.

## Validation and limitations

The first measured1 replay has 45 recorded scans with no corresponding raw estimator output, immediately after the ~3.7 s receipt pause. All observed output stamps match scan-end acquisition times within 1 ms; this checks processing coverage separately from the deep-queue observer receiving every input. The mapper uses best-effort SensorDataQoS for LiDAR, making burst loss a plausible explanation. The missing outputs are measured; callback-level loss is inferred, not directly instrumented. This is not missing data in the original bag, and its 3× endpoint result must not be treated as an uncontaminated accuracy measurement.

The first real1 and measured1 attempts stopped before all fused outputs had drained. Matching raw/fused poses preserve X/Y/quaternion exactly, but unmatched tails are an integration-test failure. Repeats allow up to 60 seconds of drain time; this changes the diagnostic observer, not estimator, fusion settings, calibration or recording timestamps. All original failed attempts remain saved.

**Replay repeatability is unresolved:** real1 raw output stamps match exactly between attempts and show no missing interior scans, but horizontal endpoint discrepancy changes from about 44 cm to 81 cm at 3×. Estimator code and approved calibration were unchanged. This is evidence of replay/estimator sensitivity; scheduling, initialisation, internal buffering and algorithmic nondeterminism have not been isolated. It cannot be presented as a robust accuracy pass or blamed on the one-way height filter. See `repeatability.json`. A controlled repeated 1× replay is required before using accelerated results as a reliability claim.

Measured1 is even more sensitive: the first replay has 45 missing processed scans and a 2.12 m raw horizontal return discrepancy; the repeat has 36 missing processed scans and about 1.8 cm discrepancy. Both loss intervals follow the receipt pause near 329.6 s. The good-looking repeat does not erase the failed run or prove the bag reliable at 3×. These results establish a replay-delivery confound, not a clean measurement of physical robot drift.

Measured2 has one missing canonical TF at the tail: the requested scan-end time was about 0.44 ms later than the final available vendor-derived TF. The bridge correctly refused extrapolation. It is retained as a TF completeness limitation, rather than fabricating a transform or claiming a clean pass.

- small_wander: full observer input delivery exact=True; protected application unchanged=True; matched-pose XY/quaternion preserved=True; unmatched raw/fused pose counts=0/0; missing fused output cloud/TF stamps=0/0.
- classic: full observer input delivery exact=True; protected application unchanged=True; matched-pose XY/quaternion preserved=True; unmatched raw/fused pose counts=0/0; missing fused output cloud/TF stamps=0/1.
- default: full observer input delivery exact=True; protected application unchanged=True; matched-pose XY/quaternion preserved=True; unmatched raw/fused pose counts=0/0; missing fused output cloud/TF stamps=0/0.
- normal: full observer input delivery exact=True; protected application unchanged=True; matched-pose XY/quaternion preserved=True; unmatched raw/fused pose counts=0/0; missing fused output cloud/TF stamps=0/0.
- lab: early-stop prefix observer input delivery exact=True; protected application unchanged=True; matched-pose XY/quaternion preserved=True; unmatched raw/fused pose counts=0/0; missing fused output cloud/TF stamps=1/1.
- real1: full observer input delivery exact=True; protected application unchanged=True; matched-pose XY/quaternion preserved=True; unmatched raw/fused pose counts=176/0; missing fused output cloud/TF stamps=2/0.
- real2: early-stop prefix observer input delivery exact=True; protected application unchanged=True; matched-pose XY/quaternion preserved=True; unmatched raw/fused pose counts=0/0; missing fused output cloud/TF stamps=1/1.
- measured1: full observer input delivery exact=True; protected application unchanged=True; matched-pose XY/quaternion preserved=True; unmatched raw/fused pose counts=218/0; missing fused output cloud/TF stamps=2/0.
- measured2: full observer input delivery exact=True; protected application unchanged=True; matched-pose XY/quaternion preserved=True; unmatched raw/fused pose counts=0/0; missing fused output cloud/TF stamps=0/1.
- real1_drained: full observer input delivery exact=True; protected application unchanged=True; matched-pose XY/quaternion preserved=True; unmatched raw/fused pose counts=0/0; missing fused output cloud/TF stamps=0/0.
- measured1_drained: full observer input delivery exact=True; protected application unchanged=True; matched-pose XY/quaternion preserved=True; unmatched raw/fused pose counts=0/0; missing fused output cloud/TF stamps=0/0.

The output-height fusion cannot repair horizontal or heading drift, lost geometry, or integration through a source-data gap. Vendor height may represent clearance above local terrain; these tests do not validate absolute elevation over the lab drains. Some local floor consistency was already worse with fusion than raw LIO. Settings remain experimental and have not been promoted to the live launch.

Next recordings should use a marked start position/heading, unchanged standing posture during 10-second initial/final pauses, repeated straight and loop routes, and named intermediate return pauses. Record a measured elevation change separately if world-height accuracy is required. The bag should be checked immediately for source gaps; retaining a readable, CRC-valid file does not guarantee uninterrupted sensor acquisition.

Raw evidence: `source_timing.json`, `summary.json`, per-run `comparison.json` and plots, original/observed stamp arrays, replay logs, calibration provenance and protected-file hashes. A dropout recovery/validity policy still needs a separately reviewed implementation.
