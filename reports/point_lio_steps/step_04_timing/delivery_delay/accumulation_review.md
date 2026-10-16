# LiDAR accumulation: review only, no implementation

The existing `launch/slam/slam_test.launch.py` feeds each vendor
`/utlidar/cloud_deskewed` message to pointcloud_to_laserscan with `base_link` as
its target frame, then feeds that LaserScan to SLAM Toolbox. No multi-message
cloud accumulation stage is present in this launch. Vendor internal aggregation
is not established by inspecting this launch. RViz cloud decay is only display
history; it does not densify the mapper input.

Combining a short history of sparse LiDAR scans can improve angular coverage for
2D projection. This is a plausible explanation for the user's friend's result,
not a measured result from our setup. Points from different acquisition times
must be transformed consistently into a fixed odometry/local frame using poses
at their acquisition times, then projected from a common reference pose. For
accurate combination, retain per-point timing/motion compensation, remove invalid
and out-of-height/range points, bound retention time and point count, and discard
stale observations. Robot sway means concatenating points directly in base_link
can smear otherwise stationary walls. Dynamic obstacles can leave trails.

A future review should compare angular-bin coverage and wall thickness on the
same saved motion bag, and measure added CPU use and latency. No window size or
mapping improvement is established yet. Increasing pointcloud_to_laserscan
`scan_time` does not accumulate clouds: upstream documents it as output metadata
only. Increasing its `queue_size` does not combine clouds either.

For Point-LIO, preserve original timestamps and point time fields. A separate
accumulated output can support visualization or later 2D projection. The installed
community fork also contains `common.con_frame`/`con_frame_num`, currently disabled.
Do not enable it without review: in the checked-out standard_pcl_cbk,
`last_timestamp_lidar` is assigned only `header.stamp.sec`, and its combine path
uses that truncated value for frame timing. The normal non-combine path uses
get_time_sec(header.stamp). This is a source-code observation, not a tested repair
or a reason to modify the current unchanged checkout.

Sources:
- Local launch: launch/slam/slam_test.launch.py
- Local profile: config/point_lio/go2_body_imu.yaml
- Local source: third_party/point_lio_ros2/src/laserMapping.cpp, standard_pcl_cbk
- https://raw.githubusercontent.com/ros-perception/pointcloud_to_laserscan/humble/README.md
- https://github.com/hku-mars/Point-LIO (Important notes A and C: synchronization and per-point timestamps)

No application code, parameter, topic publisher or mapper was changed/started by
this review. Accumulation is a future option after timing and TF integration.
