# Official Unitree Go2 URDF and live LiDAR

The active setup uses the **developer geometry and meshes** from
[Unitree's go2_description package](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/go2_description).
The downloaded URDF stays unchanged. The ROS description normalizes its root
name to `base_link` and the state adapter adds dynamic `base_footprint`.
See [TF_CONSISTENCY.md](TF_CONSISTENCY.md) for the current frame contract and checks.

```bash
cd /home/jain/Doggy
bash scripts/visualize_lidar.sh
```

Closing RViz stops this launch, including robot_state_publisher and the read-only
telemetry adapter. Close the current RViz before relaunching.

## Model and transforms

The original file is:
`sdk/unitree_ros/robots/go2_description/urdf/go2_description.urdf`.
All meshes, xacros and developer files are in the same source package. A small
ament packaging wrapper in `ros2_ws/src/go2_description` installs these assets for
ROS 2; it does not edit the URDF or its joint/sensor transforms. The installed URDF
was verified byte-for-byte against the upstream checkout.

`robot_state_publisher` reads the root-normalized description and publishes its complete developer geometry: 42
links, plus the approved `utlidar_lidar` measurement link, including all legs, the body IMU, radar and camera. The 12 movable joints
receive real encoder positions from `/lf/lowstate`, using the official SDK order
FR, FL, RR, RL, each hip/thigh/calf. No synthetic standing pose is supplied.

The tree starts with:

```text
odom -> base_footprint -> base_link -> official developer URDF links
```

The adapter decomposes live body odometry through a dynamic footprint and
preserves its full composed pose. The runtime description uses `base_link`
as its root; the former `base` naming bridge is removed. All sensor mount and
leg geometry comes from the URDF. In particular, `base_link -> radar` uses the
published developer origin `[0.28945, 0, -0.046825]` m and RPY `[0, 2.8782, 0]` rad.
Original developer mount rotations remain unchanged. A separately approved,
topic-derived measurement edge connects `radar` to the DDS raw-cloud frame; see
`TF_CONSISTENCY.md` for provenance and limits.

The source LowState message contains a boot tick, not an epoch/header timestamp.
For visualization its encoder sample is associated with the latest received
odometry stamp. That is sufficient for an articulated display, but does not
establish tightly synchronized joint/IMU data for research sensor fusion.

## LiDAR displays

RViz uses `odom` as its fixed frame and displays:

- **Unitree Go2 - official URDF:** the complete developer robot mesh model.
- **Body-frame LiDAR - green:** `/utlidar/cloud_base`, already in `base_link`.
- **Deskewed LiDAR - cyan:** `/utlidar/cloud_deskewed`, already in `odom`.
- **Developer URDF frames:** optional TF axes, disabled initially to keep the view clear.

The raw topic uses `utlidar_lidar`, which is appended to the runtime model beneath
`radar` using the approved vendor raw/body coordinate conversion. This is not a
manufacturer-specified physical mount. It now resolves into `odom` at cloud source
timestamps, so `/utlidar/cloud` can be added to the same RViz view. The existing
RViz presets are unchanged and do not automatically enable this display. The
raw-only preset `config/lidar_raw.rviz` remains available.

## Time and verification

Sensor and odometry timestamps remain unchanged. RViz and robot_state_publisher
follow `/doggy/robot_clock`, derived from the robot odometry, using the ROS
`use_sim_time` parameter and a `/clock` remap. This accommodates the observed
approximately 786-second laptop/robot offset for visualization; neither system
clock is changed. [ROS 2 time design](https://design.ros2.org/articles/clock_and_time.html).

The adapter publishes `/odom` (an unchanged relay of vendor odometry), `/joint_states`, TF and this dedicated clock. It does
not publish motion commands or alter robot settings. The launcher clears the
incompatible GTK/GIO paths inherited from the VS Code Snap for RViz only.

To check the running setup:

```bash
source scripts/setup_ros.sh
python3 scripts/check_lidar_tf.py
```

The check verifies the published description matches the root-normalized developer
model, resolves all 42 links, checks all 12 joint names, checks footprint/body
composition against source odometry, and transforms registered clouds at original
timestamps. Current evidence is in `reports/tf/canonical_tf_check.json` and
`reports/tf/canonical_tree_audit.json`. The current launch log is
`reports/tf/canonical_launch.log`. Older official_urdf reports/previews are historical.

To run the model and state adapter without opening RViz:

```bash
bash scripts/visualize_lidar.sh rviz:=false
```

This provides local odometry and the official robot geometry. Global mapping,
localization and autonomous motion control remain separate work.

For cleaned clouds and the standard odometry relay without RViz or SLAM, use
`bash scripts/start_mapping_sensors.sh`. See
[MAPPING_PREREQUISITES.md](MAPPING_PREREQUISITES.md) for its verification and limits.
