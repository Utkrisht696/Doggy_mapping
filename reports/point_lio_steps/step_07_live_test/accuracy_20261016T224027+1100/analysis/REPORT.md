# 3 m and 5 m out-and-back test

Point-LIO agrees much more closely with the user-reported distances than vendor odometry in this test. It also has smaller estimated return offsets. Starting-floor repeatability is good. These results support retaining the current Point-LIO setup; they do not establish surveyed absolute accuracy. No estimator, TF, calibration or application settings were changed.

## Recording and motion

The rosbag and mapper/RViz test stopped cleanly after the user said done. Recorder and launch return codes were both zero. The bag is `../bag`, with 2,879 recorded LIO poses/scans over 186.596 seconds of output. Passive sensor bringup and the calibrated IMU adapter were left running. No robot control commands were sent.

The user walked approximately 3 m forward/back, then 5 m forward/back, reversing without turning around and attempting to keep heading. Paths were not perfectly straight. Small heading variations are present in the data; no 180-degree reversal was assumed. Motion began roughly 13 seconds after the first output, before the formal cue. The large early 'startup excursion' field includes real movement and must not be interpreted as stationary drift.

## Outbound distance comparison

Positions below use median poses during the stationary holds. Each leg is measured from its own preceding start hold. Vendor body odometry is converted to the developer IMU origin using its fixed lever arm. One initial rigid alignment is used for plots, with no scale adjustment or trajectory fitting.

| User-reported distance | Point-LIO horizontal displacement | Difference from nominal | Vendor horizontal displacement | Difference from nominal |
|---|---:|---:|---:|---:|
| 3 m | 2.934 m | -0.066 m (-2.2%) | 2.443 m | -0.557 m (-18.6%) |
| 5 m | 5.028 m | +0.028 m (+0.6%) | 4.226 m | -0.774 m (-15.5%) |

The comparison uses horizontal endpoint displacement, not integrated speed or total curved path length. The user has not supplied how the 3 m/5 m reference was measured or its uncertainty, so percentage differences are agreement with nominal distances, not calibrated accuracy specifications. Slightly curved paths can have travelled lengths longer than endpoint separation.

## Return offsets

| Check | Point-LIO horizontal offset | Vendor horizontal offset |
|---|---:|---:|
| End of 3 m trip vs its start | 6.29 cm | 10.13 cm |
| End of 5 m trip vs its own start | 9.23 cm | 14.99 cm |
| Final return vs original starting hold | 3.91 cm | 19.04 cm |

The two per-trip offsets need not add: they are vectors and partly cancel. Returning to a physically different spot also produces an estimated offset. No independent endpoint survey was recorded, so these are estimated return offsets, not exact drift values.

## Vertical repeatability

Point-LIO final estimated IMU height is 3.53 mm below its initial held median. The per-trip vertical return differences are -8.09 mm after the first trip and +4.56 mm after the second, relative to that trip's start. The additional change between the final 110–120 second hold and late 165–175 second hold is only 2.89 mm in 3D.

More directly, compare the **same starting-area floor cells** as recorded, with no ICP, re-levelling or flattening:

| Return hold | Common floor cells | Median signed height difference | Median absolute height difference | 90th percentile absolute difference |
|---|---:|---:|---:|---:|
| After 3 m trip | 413 | -2.71 mm | 5.08 mm | 25.90 mm |
| After 5 m trip | 433 | -2.22 mm | 3.24 mm | 17.23 mm |
| Late stationary hold | 433 | -1.45 mm | 3.53 mm | 16.34 mm |

Floor cells are 15 cm in map XY, with at least five returns per epoch, within 3 m of the original start and 0.15–0.9 m below its estimated IMU. Matching cells assumes adequate horizontal registration; sampling, sloped surfaces and outliers explain part of the spread. The median differences demonstrate repeatability of a sampled floor surface, not millimetre absolute sensor accuracy.

The outbound estimated IMU height decreases by about 5.2 cm at the 3 m hold and 6.8 cm at the 5 m hold, then recovers. Actual floor height and posture were not measured at those points. This recording therefore does not resolve all vertical map tilt/accuracy questions from the longer drain route. Its return repeatability is substantially more reassuring than claiming those outbound changes are entirely drift.

## Transport and TF checks

- No nonfinite LIO values, backward LIO stamps or position steps over 15 cm. Largest output step was 5.48 cm during motion.
- Calibrated IMU contains every unique recorded SportModeState source stamp in its overlapping recording interval. This checks the recorded conversion route; it does not prove all robot samples reached the laptop or mapper.
- No conflicting TF parents. Exact recorded canonical `map -> odom -> base_footprint -> base_link -> imu` reconstruction succeeded for 2,878 poses, matching LIO to numerical precision (maximum position difference `2.7e-15 m`).
- The **last** LIO pose alone lacks its corresponding `map -> odom` transform, 65 ms beyond the last recorded map transform, at mapper shutdown. There is no internal missing-TF gap. This is a shutdown-tail limitation rather than evidence of tracking drift during the route.
- Original application file hashes still match the capture. No parameter changes or estimator rerun were used for this analysis.

## Hold windows and artifacts

Seconds are relative to first recorded LIO pose: start 3–12 s; 3 m far hold 27–32 s; first return 59–64 s; 5 m far hold 81.5–84.5 s; final return 110–120 s; late stationary 165–175 s. Please double-check these correspond to your two trips.

- [Distance, path and vertical comparison](distance_comparison.png)
- [Exact values and hold windows](distance_checks.json)
- [General motion and timestamp metrics](metrics.json)
- [Recorded TF checks](tf_check.json)
- [Summary](summary.json)
