# Gyro-weighting trial — failed; temporary override not adopted

The user approved the proposed isolated replay with only gyro measurement
variance changed from 0.1 to 0.0001. The original active input profile and launch
were not edited. A copy of the launch under this report directory loaded one
additional temporary parameter file. Resolved parameter comparison found exactly
one difference. A live parameter-service read confirmed the running estimator
used 0.0001, the existing SportModeState-based IMU, provisional zero time lag,
map/imu output frames and per-scan odometry.

The same bag, selected sensor/model/TF topics, recorded private clock, localhost
ROS domain 197, half-speed playback, RViz configuration and drift gates were used
as in the original baseline. Existing live sensor and body-IMU processes were
left running. Snap GUI plugin overrides were removed from this test's environment
only, as in the successful baseline GUI startup. No robot commands were sent.

**The change did not fix stationary divergence.** Both tests were stopped after
13.681 seconds of input cloud-header time, before the recorded walking began.

| Metric | Original variance 0.1 | Trial variance 0.0001 |
| --- | ---: | ---: |
| Maximum displacement from first estimate | 2.327 m | 1.656 m |
| Continuous false yaw at final output | approximately -293 degrees | approximately -296 degrees |
| Estimator outputs / matching map->odom TF | 209 / 209 | 209 / 209 |
| Stationary drift gate | Failed | Failed |

The trial's automatic gate uses a reference after startup and reported 1.668 m
and approximately 180 degrees geodesic rotation range. That reference differs
from the first-output displacement above. Continuous yaw unwraps the rotation
instead of folding it into the 0–180 degree shortest rotation distance.

The common input-header cutoff comparison has 208 outputs per run because the
last scan-end output is later than its input cloud header. The full trial had
209 outputs, all exactly matched by map->odom TF source stamps. The output stamp
sequences also matched between runs. There were no missing observed replay IMU
or raw-cloud stamps inside their captured stamp intervals and no multiple TF
parents. No expired/invalid/backward/overflow count was observed in the bridge
status snapshots. Both runs used a stationary segment independently checked in
the baseline report against vendor odometry/body orientation.

[Comparison plot](baseline_comparison.png) · [Measured comparison](baseline_comparison.json) ·
[Actual runtime parameters](live_parameter_audit.json) · [Only parameter difference](parameter_diff.json)

The gyro-only change therefore does not justify adopting 0.0001 into the active
profile. It also does not establish a timing fault, sensor-source failure or
single root cause. All test-owned processes stopped cleanly and file hashes
confirm active code/configuration stayed unchanged. Test launch/parameter files
remain evidence only; the default gyro variance is still 0.1.

## Read-only startup geometry check

Source inspection shows `init_map_size` counts accumulated downsampled points,
not frames. Its current value 10 allows one normal sparse scan to initialize the
map. A NumPy approximation of nearest-five plane tests at zero motion used the
same later held-out stationary cloud for each seed. A one-scan seed had 481
points and accepted 216/357 query points; a ten-scan seed had 4039 points and
accepted 357/357. This suggests denser startup geometry is worth a controlled
test. It does not prove correspondence correctness, the exact live map seed or
a fix; PCL voxel processing and running-filter differences remain.

[Screening method and results](seed_geometry_screening.json)

A **review-only, unapproved** next trial is prepared: start from the original
profile (gyro variance 0.1), change only `init_map_size` from 10 to 4000, and repeat
the same isolated replay with stationary startup and stop gates. Do not load the
failed gyro override as part of that test. This changes initial-map density;
normal raw-scan combination remains off, so the known frame-combine issue is not
introduced. No active code change is needed. The candidate is not applied.

[Next trial for user review](review/proposal.json)

## Independent live adapter status at final check

The final host process check found the original live body-IMU PID 1155223 no
longer running. Its existing session completed with exit code 1, and the adapter
log ends with `ValueError: Body orientation quaternion is outside the reviewed
normalization range`. The isolated test shutdown targets only its own player and
launch, not the live adapter. No restart or guard relaxation was performed.
This does not affect the replayed calibrated IMU measurements or explain the
saved-bag divergence. Before a live test, inspect fresh SportModeState validity
and restore the unchanged adapter only after its input is valid. Headless sensor
TF/model preparation remains running.
