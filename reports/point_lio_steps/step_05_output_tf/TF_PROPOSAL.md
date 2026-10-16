# Proposed Point-LIO output integration — review only

**Implementation update:** the user authorized launch/output-TF and RViz work.
Those portions are implemented and isolated checks passed. See [VERIFY.md](VERIFY.md).
The saved-bag runner below is still a proposal, and no estimator/replay or live
mapping was started. User verification is the next checkpoint. The remainder
records the reviewed design, rather than current execution instructions.

The canonical tree would be:

`map -> odom -> base_footprint -> base_link -> developer URDF -> imu / radar -> utlidar_lidar`

The existing state adapter remains the sole owner of odom -> base_footprint ->
base_link; robot_state_publisher remains the sole owner of all URDF edges.
A new output adapter would own only map -> odom. No second base_link or imu link,
physical mount change, static odometry shortcut or invented sensor alias is needed.

Point-LIO estimates the selected body IMU pose, not the base_link pose. Configure
its odometry message as map / imu, remap its raw TF to a private topic, and obtain
odom -> imu from the existing canonical TF at the same source timestamp. Publish:

`T_map_odom = T_map_imu(Point-LIO) * inverse(T_odom_imu(existing TF))`.

This composes the real IMU lever arm and body attitude, preserving the canonical
model. It uses vendor odometry only to express the LIO pose through the existing
tree; it is not an additional fused measurement in Point-LIO. Every global child
has one TF parent. Missing matching TF must cause a wait/drop with diagnostics,
not use of a latest-time or identity fallback.

Proposed implementation scope, not yet written into active application paths:

1. A small output-TF adapter with tested rigid-transform composition and strict
   map/imu/odom contracts, preserving estimator stamps and publishing map -> odom.
2. A Go2 Point-LIO launch using the already-approved input profile, private output
   topics and raw TF, with initial odometry publication once per scan rather than
   every point. This reduces ROS/TF output load; it does not change sensor timing.
3. A matching RViz configuration using map, registered clouds and trajectory.
4. An isolated localhost/domain saved-bag runner so recorded inputs cannot mix
   with live robot topics or clock/TF owners. Use the bag's robot-derived private
   clock and do not run another clock source or sensor adapter during replay.

The first test would replay the newly saved posture bag slowly, keep source stamps
and the zero provisional lag, verify initialization/still-period drift/cloud
alignment and TF ownership, and save diagnostics. No live walking or robot control
commands are part of that test. Point-LIO's internal map is an odometry map and
does not by itself provide SLAM Toolbox-style global loop closure.

The launch/output-TF and RViz portions now have user implementation approval.
User GUI review and subsequent mapping-test approval remain required. Estimator
geometry/output has not been verified with a running live mapper.
