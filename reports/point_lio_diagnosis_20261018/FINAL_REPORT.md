# Heading calibration and fixed-profile validation — 18 October 2026

The body gyro Z measurements overcount rotation by approximately 2.2% on these recordings. A gain of **0.9785884535708254**, applied after the existing fixed gyro-bias subtraction, corrects that rate error. The gain was fitted on `running_mode_spins` using independent quiet-wall registration, then held fixed on the new `spinny` bag and all mapping replays. It was not fitted to the new bag, vendor odometry positions, desired endpoints or commanded rotations.

The resulting clean Point-LIO profile strongly reduces heading/trajectory failure. Residual position drift remains on the wander recordings; this is not a claim that local odometry is globally drift-free. All validation was offline in localhost-only ROS domains, with no commands replayed and no RViz or live robot motion started.

## New bag and independent angular check

`/home/jain/go2_bags/spinny` contains 78.78 seconds, 154,054 messages and one complete MCAP file. Full-stream decoding, footer/summary CRC and metadata counts pass. Its chunks do not contain CRC values, so no chunk-level CRC guarantee is claimed. Raw point layout, source-time continuity and exact replay delivery also pass. Original files remain unchanged.

The recording shows two CW turns, two CCW turns and extra turns, separated by short pauses. Those pauses let stationary walls provide a rotation reference without using either IMU orientation or odometry to initialize the match. Checks include disjoint self-tests, reverse-cycle agreement and split-cloud agreement. Three turn increments pass the quantitative reference gates:

| Turn | Wall-derived rotation | Gyro error before gain | Gyro error after fixed gain |
|---|---:|---:|---:|
| CW 1 | −357.30° | −7.42° | +0.37° |
| CW 2 | −365.88° | −7.51° | +0.47° |
| CCW 1 | +368.96° | +7.76° | −0.30° |

The second CCW stop fails the wall-reference quality gates and is excluded; it is not counted as a successful independent calibration check. The recorded turns are approximately 360°, as the operator described, rather than assumed to be exact full turns. The final wall reference places the robot about 7.1 cm from the initial position with −2.89° heading change. Point-LIO estimates 2.8 cm and −2.36°, giving about 0.54° endpoint heading disagreement. An estimated return offset is not the same as surveyed position error.

Evidence: [held-out angular validation](spinny_gain_validation.json), [angular plot](spinny_gain_validation.png), [training fit](gyro_gain_running_fit.json), [wall references](endpoint_truth/spinny_end/wall_reference.json), and [bag audit](spin_inputs/spinny/input_audit.json).

## Same clean profile across recordings

These are the final clean-source results, rather than the earlier instrumented prototypes. Initial and final quiet holds are compared. Position offsets below are estimated horizontal displacement from the initial hold, not absolute ground-truth drift.

| Recording | Previous Point-LIO return offset | Updated return offset | Updated height change | Endpoint heading difference from independent clouds/walls |
|---|---:|---:|---:|---:|
| Soft-turn wander | 3.51 m | **0.735 m** | −3.71 cm | about −1.01° |
| Small wander | 7.92 m | **0.624 m** | −3.59 cm | about −1.47° |
| 5/10 m + rectangles | 4.26 m | **0.020 m** | −0.10 cm | about −0.68° |
| New `spinny` | — | **0.028 m** | −1.16 cm | about +0.54° |

The measured route's nominal 5 m displacement is 5.072 m and nominal 10 m displacement is 10.222 m. Segment return offsets are 8.6 cm and 9.2 cm. Vendor odometry gives 4.470 m and 9.021 m for those outbound legs, and 4.839 m final offset. Operator distances and endpoint placement are approximate; curved paths differ from hold-to-hold displacement.

Independent start/end cloud registration on the measured route estimates an actual horizontal offset of about 5.85 cm and heading change +1.48°; Point-LIO estimates +0.80°. Thus the 2.0 cm estimated closure is not itself a claim of 2.0 cm absolute accuracy. Quiet-cloud registration is an additional sensor-based reference, not an external surveyed tracker.

The older spin regression estimates 1.200 m displacement and −1.26° heading change. Independent walls show approximately 1.225 m displacement and −0.97°: the robot actually translated during this recording, so its metre-scale displacement must not be labeled metre-scale drift. The original profile already had a good final heading on that earlier bag; the new profile preserves it within about 0.3° of the reference.

All five final runs have finite, increasing output at approximately 15.4 Hz, exact observer cloud/IMU delivery and no output gaps above 150 ms, reseeds or multiple TF parents. The measured route has one missing final map→odom transform because the final LIO timestamp is 0.441 ms beyond the final recorded vendor TF sample; the bridge correctly refuses extrapolation. This is at the recording boundary, after the endpoint analysis window. Other final runs have exact-stamp TF for every pose. Bridge lookup retries while waiting for recorded TF are distinct from dropped output.

See [before/after trajectories](final_validation/route_comparison.png), [machine-readable summary](final_validation/summary.json), and each `final_validation/<case>/run_01` directory for output arrays, manifests, logs and analysis. Soft-turn endpoints use common 2–6 s and 137.057–141.557 s windows. The short initial small-wander hold uses a 0.4 s minimum window and at least eight actual poses; clean initialization starts slightly later than the old implementation.

## Installed changes

- `scripts/publish_body_imu.py` applies `(raw_gyro - fixed_bias) * gyro_scale_xyz`; only Z has a non-unit gain. It preserves source timestamps, axes, gravity and accelerometer bias. Identical duplicate source stamps are still dropped; invalid/backward data still fail rather than being retimed.
- The vendor fused quaternion shares the uncorrected angular behavior. Its numeric values are retained for inspection but marked unavailable with `orientation_covariance[0] = -1`. Point-LIO consumes angular velocity and acceleration, not this quaternion. Gyro/acceleration covariance stays unknown rather than fabricated.
- Shared YAML: output-IMU model retained; gyro measurement variance `0.0001`; additional gyro-bias initial/process variance both zero; initialization map 4000 points; scan-rate odometry. The adapter's existing fixed gyro-bias calibration is retained; the filter cannot invent an extra residual gyro bias. Accelerometer-bias estimation remains enabled.
- Minimal upstream patch: clear each initialization IMU batch, resize indexed point caches, and expose gyro-bias prior variance with finite/nonnegative validation. Its default remains the previous `0.001` for other configurations. No trace logger, experimental bias seeding, vendor aid or gap-recovery code is promoted.
- Developer URDF, TF geometry, extrinsics, raw topics and acquisition-lag configuration remain unchanged. Both current mapping launches load this shared profile. No loop closure, world-trajectory scaling or endpoint alignment is applied to improve outputs.

The adapter was checked against all **22,828** unique Sport IMU samples in `spinny`: numerical measurements exactly match the independently prepared calibrated replay and all timestamps are preserved; 389 identical duplicates are dropped. [Validation](clean_candidate/adapter_validation.json).

Source originals and before/after hashes are saved in [deployment](deployment/source_changes.json). The minimal submodule patch is preserved at `reports/patches/point_lio_heading_20261018.patch`; rebuilding the patched Point-LIO package is required. The installed launch/parameter check is recorded in `deployment/installed_profile_validation.json`.

## Limits and remaining work

The dominant angular gain issue is independently supported and substantially improved across these bags. Remaining 0.6–0.75 m wander offsets still matter for navigation. Endpoint agreement alone cannot bound every intermediate pose error, and no loop closure/global localization is present. Dynamic gyro scale under different temperature, gait and speed is not fully calibrated; only body gyro Z gain was measured. Noise weighting was replay-validated, not derived from a complete sensor noise/Allan characterization.

Physical LiDAR/IMU acquisition lag is still uncalibrated, with zero configured correction. Correct delivery, common source clock and good mapping do not establish hardware synchronization. Recording gaps and vendor-aided recovery experiments are separate; the clean mapper has no such recovery. This work does not establish Nav2 readiness or enable an autonomous movement controller. Mapping and visualization remain stopped after verification.
