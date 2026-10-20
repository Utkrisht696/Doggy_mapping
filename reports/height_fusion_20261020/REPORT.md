# Permanent height-fusion integration — 20 October 2026

The current live 2D, legacy Point-LIO and recorded-output replay launches now start height fusion unconditionally. Public Point-LIO odometry/cloud/path topics are corrected; diagnostic estimator outputs are under `/point_lio/raw/` in `lio_local`. Current 2D odometry, body TF and terrain maps all consume the same corrected pose/cloud pair. No optional enable flag was added.

The scalar filter is byte-identical to the reviewed causal prototype. Its settings and the approved body-IMU calibration, sensor geometry and internal Point-LIO estimator are unchanged. Vendor body Z is converted to the developer IMU origin, aligned once at startup, and accepted only from the past within 50 ms. Missing/rejected measurements propagate raw LIO increments and growing height variance. No horizontal/yaw vendor scaling, future fit or feedback into the internal estimator is introduced.

The path display is decimated, bounded to 6,000 poses and published at most 2 Hz; the estimator's unused raw path is disabled. Odometry and cloud rates remain unchanged. Losing the height-fusion process stops the launch, preventing an unnoticed switch to unfused outputs. Vendor input QoS has a deeper bounded best-effort queue.

## Verification

- Full suite: **34 tests passed**, including physical frame composition, exact-time TF, causal/missing-height behavior, synthetic measured ramp, padded big-endian clouds, unchanged XY/quaternion/source stamps, bounded path publication and terrain/self-filter checks.
- Full original sensor recording at **1×**, through `launch/live_2d_mapping.launch.py`, headless, localhost-only: **passed=True**.
- Inputs: 5,024 raw scans; 95,186 derived body IMU samples; 48,804 vendor odometry samples. Observed source stamp sequences match exactly: {'cloud': True, 'imu': True, 'vendor': True}.
- Outputs: 5,010 raw/fused/body poses; 5,010 registered clouds; median 15.40 Hz. Interior scans missing from estimator outputs: 0. Output intervals over 150 ms: 0.
- Raw/fused XY/quaternion and cloud XY are exact; maximum cloud-Z shift error 1.79e-07 m. Body mount composition error 1.78e-15 m. Missing exact body-TF stamps: 0; conflicting TF parents: {}.
- Recorded-output replay smoke test: 219 matched poses and 219 clouds; fusion applied, XY/quaternion unchanged, public frame `odom`.
- All three launch descriptions load successfully. Original recordings were read-only; no robot command topics were replayed.

| Return | Raw horizontal | Fused horizontal | Heading error (both) | Raw height | Fused height |
|---|---:|---:|---:|---:|---:|
| first_return_reference | 13.9 cm | 14.1 cm | -0.44° | -34.52 cm | -0.25 cm |
| endpoint_reference | 13.9 cm | 14.1 cm | -0.37° | -34.95 cm | -0.09 cm |

These are discrepancies against independent quiet-cloud registrations, not surveyed ground truth. Raw and fused X/Y are exact; the slightly different horizontal-error scalar comes from projecting translation into the initially tilted IMU axes after changing Z.

## Remaining limits

The complete 2D mapper produced maps throughout the test. Its existing capacity bound reached 600000 voxels; the last status reports 277359 capacity-dropped points and 3 scans skipped for missing model/joint alignment. This is a terrain-map completeness limit, separate from exact raw/fused pose/cloud transport. Capacity is explicit in `/terrain/status`; this integration does not claim an unlimited map.
The filter corrects output Z only. Horizontal/heading drift, large source-data gaps and accelerated-replay sensitivity remain. Vendor Z has not been validated as world elevation over drains/ramps. The configured uncertainty is provisional; upstream twist is retained and full fused velocity/covariance is not a validated navigation state. No autonomous controller is started.
The GitHub push precedes the requested physical-robot test. This report covers recorded-data verification; a live test result is recorded separately after the push.

Evidence: `production_1x/validation.json`, `output_replay_validation.json`, `unit_tests.txt`, passive input provenance and the included validation scripts. Raw bags/build artifacts are excluded from Git.
