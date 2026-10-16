# First Point-LIO map test — stationary failure, stopped

The user approved starting mapping after preview review. The test used the saved
253-second manual-motion bag in localhost-only ROS domain 197, with only raw
LiDAR, calibrated SportModeState body IMU, recorded robot clock, TF, robot model
and joints replayed. Playback began at half speed. No robot commands were sent,
no live walking was requested, and active code/calibration/settings were unchanged.

The initial GUI attempt stopped before playback because inherited VS Code Snap
GTK plugins loaded an incompatible libpthread. A second attempt removed those
GUI environment variables from the test process only. RViz opened successfully,
Point-LIO initialized, and recorded inputs were consumed.

**The stationary test failed.** Playback was stopped at 13.681 source seconds,
before the first walking section at roughly 69 seconds. Relative to the first
Point-LIO output, estimated displacement reached 2.327 m and orientation differed
by up to 179.51 degrees. The automatic gate, using its post-startup reference,
reported 2.104 m displacement and approximately 180 degrees orientation range.
This is estimator instability, not a successful map test.

The independent recorded reference streams were stationary: vendor body position
changed by at most 0.000336 m and fused body orientation by 0.0616 degrees. The
reference streams are robot estimates, not external metrology, but clearly do
not support the large Point-LIO motion.

All 209 observed estimator odometry stamps matched the 209 published map->odom
TF stamps exactly. No child had multiple observed TF parents. There were no
missing cloud or calibrated-IMU stamps within the observed replay stamp ranges.
The bridge's transient lookup attempts resolved; no expired, invalid, backwards
or overflow counts were observed in its periodic reports. Point-LIO CPU was about
20.5% and RSS 162 MB in the early half-speed snapshot; RViz was about 16.8% and
295 MB. This is not a complete live-load benchmark.

The approved raw-to-body cloud conversion was checked again across all 212 raw
clouds in the replay interval: 241,279 uniquely matched ring/time/intensity point
signatures had median residual 6.43e-8 m and maximum 5.34e-6 m. This establishes
stream-coordinate consistency, not surveyed mount accuracy. An initial analysis
that compared array indices produced no pairs because body clouds are a filtered
subset; the final comparison uses measurement signatures as intended.

The UniLiDAR parser reads float point times and scales seconds to milliseconds;
the active normal path sorts those times. The fixed-extrinsics branch uses the
approved rotation/translation. No source edit was made. The known frame-combine
path remains off. The startup map threshold `init_map_size: 10` counts points,
not scans; a denser seed is another possible investigation, not a proven fix.

A concrete candidate issue is the stock gyro measurement variance. This source
assigns `mapping.imu_meas_omg_cov: 0.1` directly into R_IMU, corresponding to a
0.316 rad/s standard deviation. The recorded stationary per-axis standard
deviations were approximately 0.0069–0.00825 rad/s. The generic value weights the
body gyro much less strongly than these stationary samples suggest. This does
not prove it caused the divergence; scan geometry, initialization and other
filter assumptions remain possible contributors.

A **review-only**, one-parameter trial is prepared: use a temporary override of
`mapping.imu_meas_omg_cov` from 0.1 to 0.0001, keep all other settings and input
stamps/extrinsics unchanged, and repeat the same isolated bag/gates. The candidate
variance exceeds the measured per-axis stationary variances but is not a factory
noise calibration or validated dynamic model. User approval is required before
loading it. Active profile and launch files have not been changed.

[Failure plot](stationary_failure.png) · [Measured crosschecks](failure_crosschecks.json) ·
[Run result](result.json) · [Review-only trial](review/proposal.json)

Point-LIO, its TF bridge, replay player and test RViz all stopped cleanly. The
pre-existing live sensor and calibrated-IMU processes were left running. The
source bag, timing offset, calibration and application code remain unchanged.
No timing correction can be inferred from this failed mapping experiment.
