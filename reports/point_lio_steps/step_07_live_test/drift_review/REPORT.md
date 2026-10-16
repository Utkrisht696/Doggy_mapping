# Live driving bag: drift review

The map is continuous and useful for further testing. This recording does not establish absolute navigation accuracy. The remaining concern is vertical/attitude consistency near the final section; the drain landmark is repeatable, while a later region appears lower than that known low point. No application files, estimator settings, TF or calibration were changed. The analysis reads the closed bags and recorded outputs; no estimator was rerun.

## Scope and terrain reference

Primary capture: `../20261016T220926+1100/bag`, with 2,074 Point-LIO poses and registered scans over 134.417 seconds of estimator output. Times below are relative to its first Point-LIO pose. The user confirmed actual manual motion; travel is not classified as stationary drift. The separate short restart is supplementary and has its own map origin.

The user identified a dip/drain at the first roughly 90-degree right turn as the lowest part of the route. That turn occurs around 65–71 seconds (approximately 83 degrees of yaw over that interval). It is near map XY `[6.18, -1.14]`. This is a qualitative terrain reference; its physical depth and exact coordinates were not surveyed. The route must not be assumed flat.

## What passed

- **Continuity:** no nonfinite Point-LIO values, backward output timestamps, or position steps above 15 cm. Largest step was 5.12 cm over 65 ms during motion; no multi-metre teleport is present.
- **Initial quiet period, 0.5–27.72 seconds:** maximum excursion from the segment first pose was 4.23 cm, with only 2.06 mm start-to-end translation. Attitude changed up to 1.72 degrees; physical sway and estimator jitter are not separable from this reference alone. Quiet intervals were screened from vendor position velocity, SportModeState velocity and calibrated gyro, rather than assumed from the launch phase label.
- **Stationary surfaces:** early 2–4 second and late 23–25 second clouds overlay without additional registration. Median nearest-point spacing was 2.30 cm; 96.6% of sampled later points have an early neighbour within 10 cm. Sparse sampling affects this metric.
- **Drain revisit:** outward 61–75 seconds and return 113–119 seconds share 467 floor cells (15 cm map-XY cells, at least five points per epoch, within 2 m of the landmark). Median signed height difference is +1.46 mm; median absolute difference is 1.08 cm and 90th percentile is 3.61 cm. This supports local vertical repeatability at that landmark. It depends on correct horizontal cell associations and is not an independent accuracy measurement.
- **TF:** reconstructed the recorded global chain `map -> odom -> base_footprint -> base_link -> imu` at every original LIO stamp. All 2,074 lookups succeeded. Maximum disagreement with the recorded estimator pose was approximately `6e-15 m` and `9e-14 degrees`, i.e. numerical roundoff. No conflicting TF parents. Temporary future-TF waits in the live log resolved: the final bridge status has no expired, invalid or overflowed poses. This verifies TF composition, not the physical extrinsic calibration or estimator accuracy.

## Remaining concerns

### Vertical consistency relative to the drain

Local ground-plane fits with radii of 1, 1.5, 2 and 3 m all put the final section (125–133 seconds) about **13.8–15.0 cm below** the first drain/turn area (65–75 seconds) in raw map Z. That conflicts with the user observation that the drain is the lowest area. The estimate is robust to this particular radius choice, but floor selection and real uneven ground remain limitations.

The initial floor patch is tilted approximately **0.76 degrees** in map coordinates. A diagnostic re-levelling using that initial plane reduces the final-below-drain mismatch to **7.2–8.3 cm**. This sensitivity check assumes the initial patch was physically level. It does not change the saved map or TF. Constant global tilt, real terrain and accumulating estimator error are therefore not fully separated. Treat this as evidence of a vertical/attitude accuracy issue to investigate, not a claim of exactly 14 cm of measured drift.

The full route's roughly 19 cm robot-height drop and 23 cm fitted-floor drop must not be assigned entirely to drift: the user confirmed a real decline.

### Horizontal accuracy remains unresolved

After one initial rigid alignment and the developer body-to-IMU lever arm, Point-LIO and `/utlidar/robot_odom` finish **1.93 m apart**, with a maximum orientation difference of 4.47 degrees. Aligning over the quiet 3–25 second window instead changes the endpoint disagreement only to 1.88 m. This is meaningful estimator disagreement, not proof that Point-LIO drifted by that amount: vendor odometry is also an estimate, shares body IMU information, and no external reference was recorded. The robot did not finish at its starting position.

Smoothed path lengths also differ: with a 0.3 second Gaussian smoothing scale, Point-LIO travels 22.48 m versus vendor 17.87 m. Varying smoothing from 0.15–0.5 seconds gives a length ratio of 1.23–1.32. Some raw path-length inflation comes from stationary jitter. Neither this ratio nor the endpoint disagreement justifies applying a scale correction.

Two exploratory full-pose ICP revisit fits were rejected as drift measurements: opposite views expose different walls, overlap is partial, and the fits exhausted their iteration limit. Their fitted yaw/translation corrections must not be presented as measured drift. The more restricted shared-floor comparison above is retained with its limitations.

### IMU sampling is irregular

There are 230 calibrated-IMU timestamp intervals above 10 ms and five above 20 ms, maximum **24.26 ms**. No unique recorded SportModeState source stamps are missing from calibrated IMU output within their overlapping recording interval. The same gaps exist in recorded SportModeState source timestamps. Thus no adapter stamp losses were found, but the bag cannot distinguish robot scheduling from upstream DDS loss. This does not prove the mapper consumed every sample or that acquisition lag is calibrated.

Raw LiDAR has no backward or duplicate timestamps and no gap over three median scan intervals; every recorded scan interval lies within the outer IMU recording range. This is an interval coverage check, not proof of precise sensor synchronization.

## Suggested next verification

Before changing tuning, a repeatable closed route back to a marked physical starting position and heading would give an independent endpoint constraint. Include the same drain in both directions, hold still 10 seconds at start/end, and record known-level floor patches before and after it. Measuring the floor-height difference or level at those patches would distinguish global map tilt from vertical drift. No new test or code change is started by this review.

Please verify the circled turn/drain location in the plot before relying on that terrain reference.

## Artifacts

- [Drain landmark and height checks](drive/drain_reference.png)
- [Trajectory, disagreement, height and continuity](drive/drift_overview.png)
- [Stationary scan overlay and floor profile](drive/stationary_and_floor.png)
- [Detailed motion metrics](drive/metrics.json)
- [Terrain and radius/tilt sensitivity](drive/drain_reference_check.json)
- [Exact canonical TF verification](drive/tf_check.json)
- [Review summary](review_summary.json)
