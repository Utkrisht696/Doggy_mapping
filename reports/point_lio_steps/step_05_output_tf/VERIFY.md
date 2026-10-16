# Point-LIO launch and RViz — ready for user verification

Implemented with the user's authorization to do steps 1 and 2, then wait before
beginning map tests. No estimator, bag replay or live mapping was started.

The selected input remains `/utlidar/cloud` and the SportModeState-based
`/doggy/imu/body_calibrated`. No numeric input settings, fixed calibration,
extrinsics, raw sensor topic or developer URDF changed. Only the input YAML's
historical header comments were updated to reflect its launch integration.
Acquisition delay is still uncalibrated; zero lag remains provisional.

## Preview now — does not start mapping

With the existing headless sensor launch and body-IMU adapter running:

```bash
cd ~/Doggy
./scripts/start_point_lio.sh
```

This opens RViz in `odom`, with the developer robot model, raw LiDAR and grid.
It starts the passive output bridge, which waits for estimator odometry and
publishes no `map -> odom` transform while the estimator is off. It does not
start duplicate sensor adapters, robot_state_publisher, clocks or robot controls.
Closing RViz or pressing Ctrl+C stops the preview launch, leaving the existing
sensor/IMU processes alone. The script rejects a second copy of itself.

Please double-check the robot and raw cloud are visible, there is one
`base_link`, and RViz reports no missing-transform error for the raw cloud.
There is intentionally no `map` frame, registered map or trajectory before the
estimator produces a pose. No identity/static `map -> odom` placeholder is used.

Read the bridge status in a second terminal:

```bash
cd ~/Doggy
source scripts/setup_ros.sh
export CYCLONEDDS_URI="$PWD/config/point_lio/cyclonedds_live.xml"
ros2 topic echo /point_lio/tf_status --once --qos-durability transient_local
```

Before mapping, expected status includes `received: 0`, `published: 0`,
`waiting: 0`, and `last_published_stamp_ns: null`. Stop and inspect an unexpected
existing `/point_lio/odometry` publisher; do not start a second estimator.

## What the mapping launch is prepared to do

After user review and separate approval to begin testing, the same launch can
enable its `start_mapping` argument explicitly. That has not been run.

- The existing Go2 input YAML supplies the selected cloud/IMU and extrinsics.
- Launch overrides set estimator odometry frames to `map` / `imu` and publish
  odometry once per scan instead of once per point, reducing output load.
- Estimator odometry is `/point_lio/odometry`. Its raw `map -> imu` TF is isolated
  on `/point_lio/raw_tf`; it never competes with the developer `imu` link.
- The bridge alone publishes `map -> odom`, using
  `T_map_imu * inverse(T_odom_imu)` at the exact estimator source stamp. Real tf2
  interpolation is permitted; latest-time or identity fallback is not.
- Existing ownership remains `odom -> base_footprint -> base_link`, followed by
  developer model joints including `imu` and `radar -> utlidar_lidar`.
- The bridge waits up to two wall-clock seconds for matching TF, then drops the
  unmatched pose and reports it. Invalid frames/poses and duplicate/backward
  estimator stamps are rejected. Restart after resetting a replay/robot clock.
- RViz automatically selects `map` for mapping, showing latest registered scans,
  a height-coloured **60-second accumulation** of registered scans, trajectory
  and the robot. This accumulation is a visualization setting; it does not
  combine raw scans or alter the Point-LIO input. It is not a saved full map.
- Point-LIO's `/Laser_map` output is only the initial map seed in this source;
  the view does not mislabel it as a continuously published full map. PCD saving
  remains off. The internal LIO map does not provide global loop closure.
- All three launched processes use `/doggy/robot_clock` with ROS simulated time.

The live wrapper selects the same tested 8 MiB DDS receive-buffer settings from
`config/point_lio/cyclonedds_live.xml`, requires the existing kernel buffer
increase and does not modify it. This new file is not substituted into the
already-running sensor processes. Performance mode remains selected by the user.

## Checks completed without mapping

- Seven tests: 500 random full 3-D composition cases, explicit body-IMU lever arm,
  invalid poses, exact-stamp tf2 interpolation, missing-TF expiry, clock/order
  rejection and bounded pending queue. ROS tests used localhost/domain 197.
- [Resolved launch contracts](launch_contract_validation.json): mapping off by
  default, correct source profile, frame/topic remappings, per-scan output,
  unchanged zero lag and disabled PCD saving. Both RViz views parse as YAML.
- [Isolated preview/transport check](preview_smoke.json): real launch with
  `rviz:=false`, no estimator, no TF output before synthetic poses; synthetic
  odometry and TF then produced exactly one correctly stamped `map -> odom`
  output over real ROS transport. It stopped cleanly. No robot contact occurred.
- Python/shell syntax and ROS launch argument introspection passed.

The first isolated test setup duplicated CycloneDDS's automatic localhost
interface and then lacked explicit unicast discovery; those test-environment
issues were corrected. Final bridge and interprocess checks passed. The
environment's existing SciPy/NumPy version warning did not prevent test success;
the runtime bridge uses standard-library math and does not depend on SciPy.

The RViz window has not been opened by this work. GUI rendering and live
estimator output remain for user review and the separately approved map test.
The isolated saved-bag runner and actual estimator tests remain the next stage.
