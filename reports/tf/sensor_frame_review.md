# Sensor-frame review — 16 October 2026

Read-only source and topic investigation. No sensor TF has been added, no code/configuration changed, and no robot motion commanded.

## Manufacturer sources

- Unitree developer Go2 URDF: https://github.com/unitreerobotics/unitree_ros/blob/master/robots/go2_description/urdf/go2_description.urdf
- Unitree L1 coordinate definitions: https://github.com/unitreerobotics/unilidar_sdk#coordinate-system-definition
- Unitree L2 coordinate definitions: https://github.com/unitreerobotics/unilidar_sdk2#2-coordinate-system-definition
- CMU integration: https://github.com/jizhang-cmu/autonomy_stack_go2/blob/foxy-humble/src/utilities/transform_sensors/transform_sensors/transform_everything.py

The developer `radar` link is the LiDAR mount: XYZ `[0.28945, 0.0, -0.046825]` metres, RPY `[0.0, 2.8782, 0.0]` radians. Keep this geometry unchanged. The SDK documents parallel LiDAR/IMU axes with the IMU origin at `[-0.007698, -0.014655, 0.00667]` metres in the SDK LiDAR frame. The CMU DDS integration instead flips axes in its IMU vector conversion. Those source conventions must not be mixed without validation.

## Live cloud comparison

Five raw/body clouds were paired using identical integer header stamps. Within each pair, point correspondence required matching ring, exact float time bits, and exact intensity bits, with the key unique in both clouds. The full developer mount transform produced median matched-point errors from approximately 0.59 to 0.93 metres. This rejects an identity `radar -> utlidar_lidar` alias for this stream.

A rigid conversion was recovered offline from the first pair solely to diagnose the vendor coordinate relationship. It was then tested unchanged against 12 later clouds, approximately 219.8 seconds later, comprising 5706 unique correspondences. Maximum numerical residual: 2.10283203e-06 metres. These tiny residuals describe equivalence of published coordinates, not physical calibration accuracy.

The recovered effective body-to-raw origin is `[0.28216002866813134, 1.994733730970033e-08, -1.5374389983335845e-08]` metres. It differs from the physical mount definition. A separate measurement-frame edge under the original radar frame would preserve the developer geometry. Exact proposed values are in `lidar_tf_proposal.json`.

## Proposed next change — requires user confirmation

Add only the static measurement-frame edge `radar -> utlidar_lidar`, using the topic-derived conversion expressed relative to the existing mount. No manufacturer document found in this audit specifies these particular DDS raw-frame values; their basis is the independently checked vendor streams. The source URDF remains untouched.

Parent: `radar`; child: `utlidar_lidar`.

- XYZ metres: `[-0.005152688450810778, 1.9947337309700327e-08, -0.04710809557747969]`
- Quaternion XYZW: `[0.0007911776879826205, -0.00042867244871045253, 0.8788178054238481, -0.4771566358626999]`
- RPY degrees, for review: `[-0.08642968804679273, -0.056236634339976395, -123.00020940062508]`

After approval, verify parent uniqueness, source-timestamp transforms and raw/body cloud alignment in RViz. Stationary agreement does not establish restart or moving-dataset stability; those remain verification limits. No SLAM starts as part of this change.

## IMU and uncertainty follow separately

The SDK sensor offset alone does not verify this firmware's `/utlidar/imu` axis convention. A single stationary orientation cannot establish the full mapping of all gyro axes. Do not connect this frame to body `imu`, or silently treat it as parallel to the DDS raw cloud, without further evidence. The exposed `LidarState` provides status and orientation, not a documented mounting extrinsic; its model/firmware identifiers are still blank.

Vendor IMU covariances remain unknown (all-zero arrays). Point-LIO in this checkout constructs estimator noise from parameters. No invented covariance or parameter tuning is included in this proposal.

## Evidence

- `raw_mount_samples.npz`, `raw_mount_samples.json`: original observations.
- `raw_mount_validation.json`: direct developer-transform and axis-hypothesis checks.
- `raw_mount_holdout.json`, `raw_mount_holdout_samples.npz`: independent later validation.
- `lidar_metadata_audit.json`: exposed status fields.
- `lidar_tf_proposal.json`: precise proposal and limitations.
