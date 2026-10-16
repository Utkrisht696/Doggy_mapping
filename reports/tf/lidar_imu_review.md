# LiDAR IMU check — 16 October 2026

## Outcome

No TF or sensor-processing correction has been applied. The proposed raw LiDAR edge is also unapplied, following the user's instruction to check the IMU first.

The earlier 20-second capture included a sit-to-stand transition, confirmed by the user. Its impacts are actual posture-transition observations, not stationary sensor noise. The subsequent 12-second capture contains 2937 LiDAR IMU messages, approximately 244.6 Hz.

## Quaternion interpretation

As normal ROS XYZW, the standing orientation gives a median gravity-direction disagreement of 161.55 degrees under a Z-up, gravity-referenced world assumption. Reinterpreting the incoming fields as scalar-first gives 0.62 degrees. During the earlier transition, angular-rate correlation from quaternion differences also improves with that hypothesis: approximately 0.87/0.73/0.95 across axes, versus -0.65/0.60/0.11 as published.

This is strong evidence for an ordering problem, not verified manufacturer documentation of this DDS firmware. The hypothetical mapping is corrected ROS XYZW = incoming `(orientation.y, orientation.z, orientation.w, orientation.x)`, followed by quaternion normalization. This mapping has NOT been installed or published. The large roll previously reported from direct ROS interpretation may be a decoding artifact, not physical tilt.

The standalone Unitree SDK explicitly documents XYZW, and its ROS driver copies those array components into ROS fields in that order:

- https://github.com/unitreerobotics/unilidar_sdk/blob/main/unitree_lidar_sdk/include/unitree_lidar_sdk.h
- https://github.com/unitreerobotics/unilidar_sdk/blob/main/unitree_lidar_ros2/src/unitree_lidar_ros2/include/unitree_lidar_ros2.h
- ROS quaternion definition: https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Quaternion.msg

These sources establish the expected convention, not the implementation of the inaccessible onboard DDS publisher. The live LidarState RPY follows the message's as-published interpretation, so that status field is not an independent physical-orientation validation.

## Acceleration and mounting

Standing mean acceleration: `[3.434641312124131, -0.0006709086988882311, 9.80826876104223]` m/s². Mean magnitude: 10.3934 m/s², approximately 6.0% above standard gravity. A coordinate rotation preserves magnitude, so TF cannot remove that discrepancy. Bias, scale, firmware processing and residual vibration require separate investigation; one standing orientation cannot identify all of them.

Mean gyro: `[-0.0002579516071560371, 0.0005726672016327925, 0.0004155339897306584]` rad/s. The standing body roll/pitch is about -1.76/-0.54 degrees. A receipt-time comparison of LiDAR and body gyros was saved, but it does not validate a full mounting rotation: time alignment, filtering and frame conventions remain uncertain. No new mount transform is justified from that exploratory comparison.

The SDK's internal IMU offset cannot be attached blindly to DDS measurement frames whose axes/origins have not yet been established. Keep the original developer body `imu` and `radar` geometry unchanged.

## Next validation

A deliberate, small one-axis-at-a-time orientation check using the operator's normal supported controller mode would provide independent confirmation of pitch, roll and yaw behavior. It must be described and agreed before any movement; no automatic motion or calibration spin is started by this audit. It would validate conventions, not by itself solve full accelerometer calibration or lever-arm position.

No covariance values were fabricated. Sample variation includes mechanical vibration and is not a full IMU noise calibration. Point-LIO/SLAM remain deferred.

## Saved evidence

- `lidar_imu_review.json`: compact findings and remaining prerequisites.
- `lidar_imu_check_samples.npz`, `lidar_imu_check_metadata.json`: transition capture, explicitly labelled with user context.
- `lidar_imu_consistency_check.json`: quaternion, gravity and angular-rate checks.
- `lidar_imu_gyro_axis_check.json`: exploratory body-gyro axis/receipt-lag comparison; not a TF calibration.
- `lidar_imu_standing_samples.npz`, `lidar_imu_standing_metadata.json`, `lidar_imu_standing_check.json`: separate standing capture and analysis.
