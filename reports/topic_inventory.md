# Complete DDS/ROS topic inventory

Full scan: 2026-10-12T17:13:06.686921+11:00 (45.0s). Focused scan: 2026-10-12T17:16:07.401173+11:00 (35.0s).

Publisher/subscriber counts are from the full scan before scanner subscriptions. Robot DDS endpoints are not generally named ROS nodes. Focused rates replace full-scan rates only where explicitly sampled. Traffic means receipt at this laptop; no traffic means none in the observation window, not proof of unsupported functionality. Role descriptions combine official source inspection and topic-name inference; unverified semantics are labeled.

| Topic | Type | Pub/Sub | Observed Hz | Traffic/decoding | Navigation relevance |
|---|---|---:|---:|---|---|
| `/api/assistant_recorder/request` | `unitree_api/msg/Request` | 1/1 | — | No samples observed | Auxiliary assistant recorder API; request |
| `/api/assistant_recorder/response` | `unitree_api/msg/Response` | 1/1 | — | No samples observed | Auxiliary assistant recorder API; response |
| `/api/audiohub/request` | `unitree_api/msg/Request` | 3/1 | — | No samples observed | Auxiliary audiohub API; request |
| `/api/audiohub/response` | `unitree_api/msg/Response` | 1/2 | — | No samples observed | Auxiliary audiohub API; response |
| `/api/bashrunner/request` | `unitree_api/msg/Request` | 1/1 | — | No samples observed | Remote command execution; unnecessary for navigation; request |
| `/api/bashrunner/response` | `unitree_api/msg/Response` | 1/1 | — | No samples observed | Remote command execution; unnecessary for navigation; response |
| `/api/config/request` | `unitree_api/msg/Request` | 3/1 | — | No samples observed | Robot configuration; request |
| `/api/config/response` | `unitree_api/msg/Response` | 1/3 | — | No samples observed | Robot configuration; response |
| `/api/fourg_agent/request` | `unitree_api/msg/Request` | 1/1 | — | No samples observed | Auxiliary fourg agent API; request |
| `/api/fourg_agent/response` | `unitree_api/msg/Response` | 1/1 | — | No samples observed | Auxiliary fourg agent API; response |
| `/api/gas_sensor/request` | `unitree_api/msg/Request` | 1/0 | — | No samples observed | Auxiliary gas sensor API; request |
| `/api/gas_sensor/response` | `unitree_api/msg/Response` | 0/1 | — | No samples observed | Auxiliary gas sensor API; response |
| `/api/gesture/request` | `unitree_api/msg/Request` | 1/1 | — | No samples observed | Auxiliary gesture API; request |
| `/api/gpt/request` | `unitree_api/msg/Request` | 1/1 | — | No samples observed | Auxiliary gpt API; request |
| `/api/gpt/response` | `unitree_api/msg/Response` | 1/1 | — | No samples observed | Auxiliary gpt API; response |
| `/api/motion_switcher/request` | `unitree_api/msg/Request` | 2/1 | — | No samples observed | Controller mode management; request |
| `/api/motion_switcher/response` | `unitree_api/msg/Response` | 1/2 | — | No samples observed | Controller mode management; response |
| `/api/obstacles_avoid/request` | `unitree_api/msg/Request` | 6/1 | 12.79 | Decoded samples | Avoidance-aware control/settings; request |
| `/api/obstacles_avoid/response` | `unitree_api/msg/Response` | 1/5 | 12.79 | Decoded samples | Avoidance-aware control/settings; response |
| `/api/pet/request` | `unitree_api/msg/Request` | 1/1 | — | No samples observed | Auxiliary pet API; request |
| `/api/pet/response` | `unitree_api/msg/Response` | 1/1 | — | No samples observed | Auxiliary pet API; response |
| `/api/programming_actuator/request` | `unitree_api/msg/Request` | 1/1 | — | No samples observed | Auxiliary programming actuator API; request |
| `/api/programming_actuator/response` | `unitree_api/msg/Response` | 1/1 | — | No samples observed | Auxiliary programming actuator API; response |
| `/api/robot_state/request` | `unitree_api/msg/Request` | 1/1 | — | No samples observed | Robot service/state management; request |
| `/api/robot_state/response` | `unitree_api/msg/Response` | 1/1 | — | No samples observed | Robot service/state management; response |
| `/api/sport/request` | `unitree_api/msg/Request` | 9/1 | — | No samples observed | High-level locomotion; request |
| `/api/sport/response` | `unitree_api/msg/Response` | 1/8 | — | No samples observed | High-level locomotion; response |
| `/api/sport_lease/request` | `unitree_api/msg/Request` | 0/1 | — | No samples observed | Sport control lease management; request |
| `/api/sport_lease/response` | `unitree_api/msg/Response` | 1/0 | — | No samples observed | Sport control lease management; response |
| `/api/uwbswitch/request` | `unitree_api/msg/Request` | 3/1 | 2.69 | Decoded samples | Auxiliary uwbswitch API; request |
| `/api/uwbswitch/response` | `unitree_api/msg/Response` | 1/3 | 2.69 | Decoded samples | Auxiliary uwbswitch API; response |
| `/api/videohub/request` | `unitree_api/msg/Request` | 4/1 | — | No samples observed | Auxiliary videohub API; request |
| `/api/videohub/response` | `unitree_api/msg/Response` | 1/4 | — | No samples observed | Auxiliary videohub API; response |
| `/api/vui/request` | `unitree_api/msg/Request` | 7/1 | 2.02 | Decoded samples | Auxiliary vui API; request |
| `/api/vui/response` | `unitree_api/msg/Response` | 1/6 | 2.02 | Decoded samples | Auxiliary vui API; response |
| `/arm_Command` | `unitree_arm/msg/ArmString` | 1/0 | — | Schema unavailable | Arm interface; schema unavailable; optional payload hardware |
| `/arm_Feedback` | `unitree_arm/msg/ArmString` | 0/1 | — | Schema unavailable | Arm interface; schema unavailable; optional payload hardware |
| `/audiohub/player/state` | `std_msgs/msg/String` | 1/1 | 4.01 | Decoded samples | Auxiliary interaction/sensor/connectivity; not essential for core LiDAR navigation |
| `/audioreceiver` | `unitree_go/msg/AudioData` | 0/1 | — | No samples observed | Auxiliary interaction/sensor/connectivity; not essential for core LiDAR navigation |
| `/audiosender` | `unitree_go/msg/AudioData` | 1/1 | 50.09 | Decoded samples | Auxiliary interaction/sensor/connectivity; not essential for core LiDAR navigation |
| `/config_change_status` | `unitree_go/msg/ConfigChangeStatus` | 1/1 | — | Schema unavailable | Configuration changes; message schema not built |
| `/frontvideostream` | `unitree_go/msg/Go2FrontVideoData` | 1/1 | 28.73 | Raw traffic; decode failed | Live raw video; ROS deserialization failed |
| `/gas_sensor` | `std_msgs/msg/String` | 0/1 | — | No samples observed | Auxiliary interaction/sensor/connectivity; not essential for core LiDAR navigation |
| `/gesture/result` | `std_msgs/msg/String` | 1/1 | — | No samples observed | Auxiliary interaction/sensor/connectivity; not essential for core LiDAR navigation |
| `/gnss` | `std_msgs/msg/String` | 1/1 | 1.01 | Decoded samples | JSON GPS-like data; fix and timestamp unverified |
| `/gpt_cmd` | `std_msgs/msg/String` | 2/0 | — | No samples observed | Auxiliary interaction/sensor/connectivity; not essential for core LiDAR navigation |
| `/gptflowfeedback` | `std_msgs/msg/String` | 1/1 | — | No samples observed | Auxiliary interaction/sensor/connectivity; not essential for core LiDAR navigation |
| `/lf/lowstate` | `unitree_go/msg/LowState` | 1/6 | 20.25 | Decoded samples | Recommended battery/power/motor health supervision |
| `/lf/sportmodestate` | `unitree_go/msg/SportModeState` | 1/7 | 20.26 | Decoded samples | Recommended motion/posture/gait supervision |
| `/lio_sam_ros2/mapping/odometry` | `nav_msgs/msg/Odometry` | 0/1 | — | No samples observed | Advertised alternative SLAM odometry; no publisher |
| `/lowcmd` | `unitree_go/msg/LowCmd` | 1/1 | 158.67 | Decoded samples | Direct motor command stream; avoid for ordinary navigation |
| `/lowstate` | `unitree_go/msg/LowState` | 1/1 | 158.68 | Decoded samples | Higher-rate battery/power/motor/IMU data |
| `/multiplestate` | `std_msgs/msg/String` | 1/1 | 1.00 | Decoded samples | Robot settings summary; avoidance switch observed enabled |
| `/parameter_events` | `rcl_interfaces/msg/ParameterEvent` | 2/0 | — | No samples observed | ROS infrastructure/diagnostics |
| `/pctoimage_local` | `unitree_interfaces/msg/PcToImage` | 0/1 | — | No samples observed | Custom local raster map; no publisher |
| `/pet/flowfeedback` | `std_msgs/msg/String` | 1/1 | — | No samples observed | Auxiliary interaction/sensor/connectivity; not essential for core LiDAR navigation |
| `/programming_actuator/command` | `std_msgs/msg/String` | 1/0 | — | No samples observed | Auxiliary interface; semantics not established |
| `/programming_actuator/feedback` | `std_msgs/msg/String` | 0/1 | — | No samples observed | Auxiliary interface; semantics not established |
| `/public_network_status` | `std_msgs/msg/String` | 1/2 | — | No samples observed | Auxiliary interaction/sensor/connectivity; not essential for core LiDAR navigation |
| `/qt_add_edge` | `unitree_interfaces/msg/QtEdge` | 1/0 | — | No samples observed | Topological waypoint-edge creation; no receiver found |
| `/qt_add_node` | `unitree_interfaces/msg/QtNode` | 1/0 | — | No samples observed | Topological waypoint-node creation; no receiver found |
| `/qt_command` | `unitree_interfaces/msg/QtCommand` | 1/0 | — | No samples observed | Vendor waypoint commands; no command receiver found |
| `/qt_notice` | `std_msgs/msg/String` | 0/1 | — | No samples observed | Vendor execution/arrival feedback; no publisher found |
| `/query_result_edge` | `unitree_interfaces/msg/QtEdge` | 0/1 | — | No samples observed | Vendor waypoint-edge query result; no publisher |
| `/query_result_node` | `unitree_interfaces/msg/QtNode` | 0/1 | — | No samples observed | Vendor waypoint-node query result; no publisher |
| `/rosout` | `rcl_interfaces/msg/Log` | 1/0 | — | No samples observed | ROS infrastructure/diagnostics |
| `/rtc/state` | `std_msgs/msg/String` | 1/1 | 1.00 | Decoded samples | Auxiliary interaction/sensor/connectivity; not essential for core LiDAR navigation |
| `/rtc_status` | `std_msgs/msg/String` | 1/0 | — | No samples observed | Auxiliary interaction/sensor/connectivity; not essential for core LiDAR navigation |
| `/selftest` | `std_msgs/msg/String` | 1/1 | — | No samples observed | Potential health diagnostics; payload unverified |
| `/servicestate` | `std_msgs/msg/String` | 1/1 | — | No samples observed | Potential service state diagnostics; payload unverified |
| `/servicestateactivate` | `std_msgs/msg/String` | 1/0 | — | No samples observed | Potential service activation interface; not exercised |
| `/sportmodestate` | `unitree_go/msg/SportModeState` | 1/1 | 158.69 | Decoded samples | Higher-rate motion/posture/gait state |
| `/uslam/client_command` | `std_msgs/msg/String` | 3/1 | — | No samples observed | Onboard SLAM command input; protocol not validated |
| `/uslam/cloud_map` | `sensor_msgs/msg/PointCloud2` | 1/0 | — | No samples observed | Vendor 3D point-cloud map; no live map observed |
| `/uslam/frontend/cloud_world_ds` | `sensor_msgs/msg/PointCloud2` | 0/1 | — | No samples observed | Vendor mapping cloud; no publisher found |
| `/uslam/frontend/odom` | `nav_msgs/msg/Odometry` | 0/1 | — | No samples observed | Vendor mapping odometry; no publisher found |
| `/uslam/localization/cloud_world` | `sensor_msgs/msg/PointCloud2` | 0/1 | — | No samples observed | Vendor localization cloud; no publisher found |
| `/uslam/localization/odom` | `nav_msgs/msg/Odometry` | 0/1 | — | No samples observed | Vendor global localization; no publisher found |
| `/uslam/navigation/global_path` | `sensor_msgs/msg/PointCloud2` | 0/1 | — | No samples observed | Vendor route as PointCloud2; no publisher found |
| `/uslam/server_log` | `std_msgs/msg/String` | 2/2 | — | No samples observed | Onboard SLAM diagnostics; no traffic observed |
| `/utlidar/client_cmd` | `std_msgs/msg/String` | 0/1 | — | No samples observed | LiDAR/vendor control; no commands sent |
| `/utlidar/cloud` | `sensor_msgs/msg/PointCloud2` | 1/1 | 15.43 | Decoded samples | Raw LiDAR for external mapping; sensor-frame extrinsics needed |
| `/utlidar/cloud_base` | `sensor_msgs/msg/PointCloud2` | 1/0 | 15.49 | Decoded samples | Preferred starting obstacle cloud in base_link |
| `/utlidar/cloud_deskewed` | `sensor_msgs/msg/PointCloud2` | 1/1 | 15.08 | Decoded samples | Cloud in odom; motion-compensated mapping/debug input |
| `/utlidar/grid_map` | `sensor_msgs/msg/PointCloud2` | 1/0 | — | No samples observed | Advertised PointCloud2 map; not a Nav2 occupancy grid |
| `/utlidar/height_map` | `sensor_msgs/msg/PointCloud2` | 1/0 | — | No samples observed | Advertised height cloud; no live samples |
| `/utlidar/height_map_array` | `unitree_go/msg/HeightMap` | 1/2 | 15.45 | Decoded samples | Custom terrain heights; adapter and invalid-cell handling needed |
| `/utlidar/imu` | `sensor_msgs/msg/Imu` | 1/0 | 248.27 | Decoded samples | High-rate LiDAR IMU; useful for LIO |
| `/utlidar/lidar_state` | `unitree_go/msg/LidarState` | 1/1 | 5.01 | Decoded samples | LiDAR health, rates and packet loss |
| `/utlidar/mapping_cmd` | `std_msgs/msg/String` | 1/0 | — | No samples observed | Vendor mapping control; no commands sent |
| `/utlidar/range_info` | `geometry_msgs/msg/PointStamped` | 1/2 | 15.44 | Decoded samples | Coarse obstacle/range metadata; not a LaserScan |
| `/utlidar/range_map` | `sensor_msgs/msg/PointCloud2` | 1/0 | — | No samples observed | Advertised range cloud; no live samples |
| `/utlidar/robot_odom` | `nav_msgs/msg/Odometry` | 1/0 | 149.41 | Decoded samples | Primary live local odometry pose and velocity |
| `/utlidar/robot_pose` | `geometry_msgs/msg/PoseStamped` | 1/2 | 18.77 | Decoded samples | Lightweight pose monitoring in odom |
| `/utlidar/server_log` | `std_msgs/msg/String` | 1/0 | — | No samples observed | LiDAR/vendor diagnostics; no traffic observed |
| `/utlidar/switch` | `std_msgs/msg/String` | 1/1 | — | No samples observed | Vendor LiDAR switch/control; no commands sent |
| `/utlidar/voxel_map` | `sensor_msgs/msg/PointCloud2` | 1/0 | — | No samples observed | Advertised uncompressed voxel cloud; no live samples |
| `/utlidar/voxel_map_compressed` | `unitree_go/msg/VoxelMapCompressed` | 1/1 | 7.72 | Decoded samples | Live compressed 3D voxel map; decoding adapter needed |
| `/uwbstate` | `unitree_go/msg/UwbState` | 1/1 | — | No samples observed | Auxiliary interaction/sensor/connectivity; not essential for core LiDAR navigation |
| `/uwbswitch` | `unitree_go/msg/UwbSwitch` | 1/0 | — | No samples observed | Auxiliary interaction/sensor/connectivity; not essential for core LiDAR navigation |
| `/videohub/inner` | `std_msgs/msg/String` | 1/1 | — | No samples observed | Auxiliary interaction/sensor/connectivity; not essential for core LiDAR navigation |
| `/webrtcreq` | `std_msgs/msg/String` | 1/1 | — | No samples observed | Auxiliary interaction/sensor/connectivity; not essential for core LiDAR navigation |
| `/webrtcres` | `std_msgs/msg/String` | 1/1 | — | No samples observed | Auxiliary interaction/sensor/connectivity; not essential for core LiDAR navigation |
| `/wirelesscontroller` | `unitree_go/msg/WirelessController` | 2/3 | — | No samples observed | Potential operator override; no traffic observed |
| `/wirelesscontroller_unprocessed` | `unitree_go/msg/WirelessController` | 1/1 | — | No samples observed | Raw remote input; no traffic observed |
| `/xfk_webrtcreq` | `std_msgs/msg/String` | 1/1 | — | No samples observed | Auxiliary interaction/sensor/connectivity; not essential for core LiDAR navigation |
| `/xfk_webrtcres` | `std_msgs/msg/String` | 1/1 | — | No samples observed | Auxiliary interaction/sensor/connectivity; not essential for core LiDAR navigation |

## Absent standard navigation interfaces

`/tf`, `/tf_static`, `/map`, `/scan`, `/odom`, `/cmd_vel`, `/joint_states`, `/battery_state`, `/navigate_to_pose/_action/status`, `/follow_waypoints/_action/status`.

The initial full scan lacked VoxelMapCompressed support. That message was added from the official SDK C++ header, rebuilt, and decoded in the focused scan. Other unresolved schemas were not guessed. Camera raw reception is counted separately from successful decoding.

Source evidence: [Unitree ROS 2](https://github.com/unitreerobotics/unitree_ros2), [Unitree SLAM](https://github.com/unitreerobotics/unitree_slam), and the exact downloaded source commits in [download_manifest.json](download_manifest.json). Full endpoint QoS and samples: [live_scan.json](live_scan.json); focused data: [focused_scan.json](focused_scan.json).
