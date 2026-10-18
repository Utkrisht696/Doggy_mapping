# Point-LIO drift diagnosis — 18 October 2026

**Current result:** the later independent wall checks identified body gyro Z gain error, and the validated calibration/filter profile is now installed. See [the final heading report](FINAL_REPORT.md) and [before/after routes](final_validation/route_comparison.png). The notes below are the historical investigation and describe earlier paused states; their statements that active code is unchanged are superseded. Residual wander position drift and uncalibrated acquisition lag remain explicitly documented.

Two concrete implementation defects were found during read-only inspection. Neither has yet been demonstrated to cause the observed metre-scale trajectory errors. Active estimator code, calibration, TF and launch configuration remain unchanged. With user approval, the first patch was subsequently tested in an isolated copy on the soft-turn bag; all 2,176 poses matched the unchanged control and the earlier saved baseline exactly. It did not improve drift. See [the completed cache-fix trial](cache_fix_soft_turn/REPORT.md). The startup-initialization patch was then approved and tested only in a second isolated copy; it worsened horizontal return offset from 3.51 m to 4.20 m and RMS heading disagreement from 7.62° to 9.58°. See [the startup-fix trial](initialization_fix_soft_turn/REPORT.md). Neither change has been promoted to active code.

## Confirmed: point caches are indexed without elements

`third_party/point_lio_ros2/src/laserMapping.cpp:936–937` calls `reserve(feats_down_size)` on the global `crossmat_list` and `pbody_list` vectors. Their sizes remain zero: no resize or insertion occurs elsewhere. The subsequent loop assigns through `pbody_list[i]` and `crossmat_list[i]`; the measurement functions read those indexed entries.

`reserve()` allocates capacity without creating elements; indexing outside size violates the vector contract. See the [C++ working draft](https://eel.is/c++draft/vector.capacity) and the [upstream port](https://github.com/dfloreaa/point_lio_ros2/blob/main/src/laserMapping.cpp). The same pattern is also present in the locally inspected Unitree/CMU-derived source, so this finding does not establish that another implementation is unaffected.

A minimal reproduction with the same Eigen vector/matrix types was compiled with `-D_GLIBCXX_ASSERTIONS`. The reserve version reports size 0/capacity 2 and aborts on indexed assignment (return −6). The resize version reports size 2/capacity 2 and succeeds (return 0). Files: `check_vector_access.cpp`, `vector_access_results.json`.

Proposed first change: `01_resize_caches.patch`, exactly two reserve-to-resize replacements. It has now been applied and replay-tested only in the isolated cache-fix trial, with no change to the measured trajectory. A valid resize may leave the optimized build's numerical trajectory unchanged, since writing within allocated capacity can appear to work in an unchecked build. The defect is real; its contribution to this failure is unproven.

## Confirmed: startup IMU samples are counted repeatedly

In `sync_packages`, lines 417 and 429 use `meas.imu.shrink_to_fit()` before appending a new batch. That does not clear the existing samples. `ImuProcess::IMU_init` then iterates over the whole retained vector and increments its running sample count for old samples again. Startup gravity averaging therefore weights early measurements repeatedly and can hit its nominal initialization threshold with fewer distinct measurements.

The illustrative `initialization_reuse_example.json` shows 60 distinct samples being applied 120 times with batches of 20. This is not a claim of the exact runtime batch counts. Repeated sampling could affect initial gravity alignment; the data do not yet establish that it explains the trajectory drift.

Proposed separate change: `02_unique_initialization_samples.patch`, replacing those two calls with `clear()`. This should be tested separately from the cache fix to retain an interpretable comparison.

## Other checks and unresolved hypotheses

- Raw point times span about 62.22 ms per cloud, consistent with seconds-to-milliseconds conversion in the current UniLiDAR preprocessing. The [L1 SDK definition](https://github.com/unitreerobotics/unilidar_sdk/blob/main/unitree_lidar_sdk/include/unitree_lidar_sdk.h) describes point time relative to cloud stamp. No gross unit-factor error was found. This does not verify the onboard DDS publisher's physical acquisition reference.
- The declared raw point intervals overlap twice in each wandering recording, by at most 0.56 ms and 0.22 ms. Prediction-step time checks would be useful, but these small overlaps alone have not been linked to the failure.
- Physical acquisition alignment of the body IMU and LiDAR remains uncalibrated. Continuity and delivery checks do not determine that offset. The [upstream Point-LIO documentation](https://github.com/dfloreaa/point_lio_ros2#point-lio-robust-high-bandwidth-lidar-inertial-odometry) requires LiDAR/IMU synchronization and correct per-point timing.
- The YAML noise values are generic starting values for the sensor profile. Standing corrected-gyro sample variance is about 4–6×10⁻⁵ per axis in these short windows, while the configured gyro measurement covariance is 0.1. These are different in scale, but standing variance includes sway/vibration and is not a validated dynamic noise model. Do not replace the parameters with those numbers without a controlled test.
- This mapper supplies IMU measurement covariance from YAML in `h_model_IMU_output`; it does not use the ROS Imu covariance fields there. Zero/unknown covariance in the messages is therefore not itself a demonstrated direct cause.
- The clean wandering bags have continuous source data and continuous output, without recovery reseeds. The old recording-gap failure is a separate issue. Adding loop closure would not diagnose the present local estimation error.

## Proposed test order

1. Completed with user approval, using `wander_soft_turn_comeback` as requested. The two-line cache correction produced exactly the same 2,176 poses as a fresh unchanged control and the saved 3.51 m baseline. Both fresh replays had exact observer input delivery and continuous output. Internal estimator callback counts have not been instrumented.
2. Completed with separate approval on the same full soft-turn bag. The startup fix changed the trajectory but did not improve the result: 4.20 m horizontal return offset and about −43 cm final height change. Full-route heading disagreement increased overall. Both isolated trials leave the active workspace untouched.
3. If those defects do not account for the failure, instrument prediction times, IMU innovations, estimated bias/tilt and LiDAR update corrections to identify where the state separates from the measurements. Then measure acquisition lag and dynamic extrinsic consistency using the existing motion recordings before tuning the noise parameters.
4. Validate any demonstrated improvement on the soft-turn and repeated-rectangle recordings without per-bag tuning.

There is no validated complete fix yet. No new robot motion or recording is needed for the first isolated test.


The approved [heading instrumentation replay](heading_trace_soft_turn/REPORT.md) reproduced all 2,176 baseline poses exactly. At the peak −18.53° disagreement, accumulated LiDAR yaw corrections contribute −14.26° and prediction relative to the body reference contributes −4.31°. This localizes the estimate changes but does not establish a unique physical root cause. A single-parameter stronger-gyro-weight test is prepared and awaits approval.


The approved [gyro measurement weighting trial](gyro_weight_soft_turn/REPORT.md) changed only variance 0.1 to 0.001. RMS heading disagreement improved from 7.62° to 5.59°, but horizontal return remains 3.37 m. Read-only gyro integration with the filter’s logged residual bias reproduces the remaining prediction heading to 0.10° RMS. An isolated residual-bias hold-zero diagnostic is prepared and awaits review; no active settings have changed.
