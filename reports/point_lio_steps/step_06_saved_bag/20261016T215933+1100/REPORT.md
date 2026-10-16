# Initial-map threshold trial — startup improved; mapping not yet validated

The user approved testing `init_map_size: 4000` against the original threshold
10, with gyro measurement variance kept at its original 0.1. The failed gyro-only
override was not loaded. Exactly this single estimator parameter differed from
the original baseline. No active profile, production launch, calibration, numeric
extrinsics or timing offset changed. All tests were localhost-only ROS domain
197 and replayed the same closed bag; no robot commands were sent.

Two full 253-second replays completed. The first passive observer fell behind
because it converted its growing trajectory to an array on every callback and
used a small best-effort IMU queue. This was corrected in the diagnostic harness
only: constant-time last-pose checks and a reliable depth-10000 observer queue.
The second full run observed **all 73,859 recorded calibrated IMU stamps and all
3,906 raw cloud stamps, in their original sequence**. Both full runs produced
3,894 estimator poses and registered clouds with exactly matching map->odom TF
stamps. No multiple TF parents, expired transforms, backwards poses or queue
overflows were observed. The two estimator pose arrays were bit-identical despite
the observer correction, so the observed mapping behavior was reproducible.

[Delivery and repeatability](repeatability_and_delivery.json) · [Run result](result.json)

A preparation-script error briefly restarted the old observer before the proper
repeat. It was stopped at 11.48 source seconds, preserved separately, and the
completed full-run outputs were restored from backup. The first full run's launch
log was recovered from ROS launch logging. See the preceding run's
`restart_incident.json`; no robot controls or active application edits occurred.

## What improved

The threshold counts accumulated downsampled **points**, not scans. A value of 10
allowed initialization from one sparse scan. The larger seed delayed regular
outputs by nine additional scans, providing denser startup geometry.

Maximum displacement from each run's first estimate during the same initial
13.75-second stationary segment:

| Configuration | Maximum displacement |
| --- | ---: |
| Original seed 10, original gyro variance 0.1 | 2.327 m |
| Seed 10, trial gyro variance 0.0001 | 1.656 m |
| Seed 4000, original gyro variance 0.1 | **0.0458 m** |

The catastrophic startup spin was avoided and the complete recorded walking,
turning and body-tilt sequence ran without the original early-stop condition.
This supports the denser seed as a useful candidate, not a complete mapping fix.

## What remains unresolved

During the initial stationary window (source seconds 5–55), the estimate had a
**34.6 cm transient excursion at source second 42.01**, and orientation changed
by as much as 4.37 degrees. The source vendor position changed by less than 1 mm
and fused body orientation by about 0.31 degree in that window. The LIO position
returned near its initial stationary value: start-to-end change was 1.48 cm.
The peak is therefore a transient tracking problem, not steady drift alone.

During motion, the initially aligned LIO trajectory separated from the vendor
estimate, with maximum position difference 1.92 m and orientation difference
13.17 degrees. That alignment uses one initial transform and the real body-IMU
lever arm; it does not optimize/fix the trajectory. Vendor odometry is not
external ground truth, so these differences identify a validation concern rather
than proving which estimate is correct. The large change around source seconds
130–145 warrants inspection. The accumulated map remains a diagnostic result,
not an accepted autonomous-navigation map.

The final stationary interval (220–250 seconds) had 3.54 cm excursion and 0.61
degree orientation variation. Acquisition latency remains uncalibrated; zero lag
is still provisional. Neither this improvement nor exact message delivery proves
LiDAR acquisition synchronization.

[Trajectory/map comparison](mapping_comparison.png) · [PDF](mapping_comparison.pdf) ·
[Detailed metrics](analysis.json)

`registered_scans_sampled.ply` contains 112,836 voxel-selected points from sampled
registered output clouds. It is a diagnostic accumulated point cloud, **not** a
full-resolution export of the filter's internal map or a 2D navigation map.

## Saved state and review checkpoint

All test-owned mapper, bridge, player and RViz processes stopped cleanly. The
active profile still has seed 10 and gyro variance 0.1; the tested override remains
in this report directory only. The live body-IMU adapter remains stopped on its
source-quaternion normalization guard and was not restarted. Existing headless
sensor/model preparation was left untouched.

Please review the startup improvement and the remaining trajectory/map problems
before another change. A useful next diagnostic would enable the existing
`runtime_pos_log_enable` flag for a separate replay, preserving these estimator
settings, to inspect velocity, angular velocity and estimated biases around the
stationary and turning anomalies. That flag has not been enabled or run.
