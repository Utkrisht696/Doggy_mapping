# Autonomous navigation assessment — Go2 Pro X

The Ethernet connection exposes usable LiDAR, IMU, odometry and robot health data.
It is a good starting point for building autonomous missions, but a working mission
system is not currently demonstrated. The onboard waypoint receiver was absent,
and the ROS transform tree, global map and velocity bridge are missing.

**Follow-up:** the official developer Go2 URDF, live joint-state TF tree and robot-time RViz setup are now prepared
in [LIDAR_VISUALIZATION.md](LIDAR_VISUALIZATION.md). The absence of TF reported below
describes the original scan. Global mapping/localization and a control bridge remain
separate integration work.

Measured on 12 October 2026 in Australia/Melbourne. The laptop uses Ubuntu 22.04,
ROS 2 Humble, `enp7s0` at `192.168.123.20/24`, DDS domain 0, and a 1 Gbit/s Ethernet
link. `192.168.123.161` answered ping. Model/edition is user-reported Go2 Pro X;
the exact firmware entitlement and installed LiDAR generation were not established.

## Evidence and scan coverage

The complete graph contained **107 topics**. A 45-second scan received traffic on
26 topics; a subsequent 35-second focused scan decoded the additional compressed
voxel stream. Thus **27 topics produced traffic across the scans**. The first
35-second scan used the existing Fast DDS installation; later scans used the locally
prepared Cyclone DDS middleware. Both discovered the robot.

The scanner created read-only subscribers, used best-effort/volatile QoS to accept
live samples from reliable and best-effort publishers, and retained bounded samples.
It did not send motion commands, change modes, start SLAM, call robot services, or
change network settings. Receipt rates vary with laptop load and subscriptions:
the focused measurement is preferable for the key streams below. No historical
transient-local data was requested. A silent topic is not proof that its feature is
unsupported, and an advertised topic is not proof that a receiver exists.

Most robot endpoints identify as bare DDS applications, so `ros2 node list` does
not enumerate the robot's internal processes. No ROS services were exposed in the
scan. Unitree's `/api/...` interfaces use request/response **topics**, not ROS
services. Endpoint QoS and counts are preserved in the JSON evidence.

## Topics to use first

| Topic | Measured receipt rate | Purpose for missions | Observed detail |
|---|---:|---|---|
| `/utlidar/cloud_base` | 15.49 Hz | Local obstacle detection; practical input for a 2D scan/costmap adapter | `sensor_msgs/PointCloud2`, already in `base_link`; ground/body filtering and height selection still needed |
| `/utlidar/robot_odom` | 149.41 Hz | Local pose and velocity estimate | `nav_msgs/Odometry`, parent `odom`, child `base_link`; this message does not itself publish TF |
| `/utlidar/imu` | 248.27 Hz | LiDAR-inertial mapping and attitude data | `sensor_msgs/Imu`, `utlidar_imu`; covariance arrays are zero, and sensor mounting axes must be accounted for |
| `/utlidar/cloud` | 15.43 Hz | Raw point cloud for an external LiDAR SLAM pipeline | `utlidar_lidar`; includes intensity, ring and per-point `time` fields; verify their units/layout before configuring LIO |
| `/utlidar/cloud_deskewed` | 15.08 Hz | Motion-compensated cloud for mapping/debugging | Already in `odom`; sampled clouds include zero XYZ points, so inspect/filter those before consuming |
| `/lf/sportmodestate` | 20.26 Hz | Mission monitoring: posture, gait, body height, position and velocity | Mode 0 and `error_code=100` observed; code meaning is not established for this firmware |
| `/lf/lowstate` | 20.25 Hz | Battery, power and motor/IMU health | Battery state of charge 67%; power approximately 29.45 V during the full scan |
| `/utlidar/lidar_state` | 5.01 Hz | Detect sensor faults and degraded data | `error_state=0`, reported cloud/IMU packet loss 0; software version `1.0.0.38`; hardware firmware field empty |
| `/utlidar/height_map_array` | 15.45 Hz | Terrain-height/traversability processing | Custom `HeightMap`, `odom`, 128×128 cells, 0.06 m resolution; sampled entries include `1e9`, which needs an explicit invalid-cell policy |
| `/utlidar/voxel_map_compressed` | 7.72 Hz | Local 3D obstacle representation | Custom compressed format, `odom`, 0.05 m resolution; sampled dimensions 128×128×38; requires a validated decoder/adapter |
| `/utlidar/robot_pose` | 18.77 Hz | Lightweight pose monitoring | `PoseStamped` in `odom`; odometry is more useful for Nav2 because it also carries velocity |
| `/wirelesscontroller` | No samples | Potential operator override/mission cancellation input | Endpoints advertised, but remote telemetry was not demonstrated |

The full scan also received `/sportmodestate` and `/lowstate` at about 159 Hz.
Their `/lf/` variants are sufficient for ordinary mission supervision and reduce
processing load. Native Python SDK subscribers independently received both `/lf/`
streams at approximately 20 Hz, confirming the SDK works against this connection.

For ordinary planar indoor missions, start with `cloud_base`, `robot_odom`,
`lf/sportmodestate`, `lf/lowstate` and `lidar_state`. Add the raw cloud and IMU when
building an external LIO/SLAM pipeline. Height/voxel maps are useful for terrain
research but are not standard Nav2 occupancy grids.

## Command interfaces

| Interface | Use | What the scan establishes |
|---|---|---|
| `/api/sport/request` and `/api/sport/response` | High-level locomotion through the SDK; destination for a future Nav2 velocity bridge | A request receiver and response publisher exist. Movement was not tested. The SDK's `Move` method uses API ID 1008; use its implementation to construct the request |
| `/api/obstacles_avoid/request` and `/api/obstacles_avoid/response` | Built-in avoidance-aware control path | Existing traffic successfully queried the switch: response code 0, `enable=true`. This verifies that query, not mission or movement behavior |
| `/api/motion_switcher/request` and `/api/motion_switcher/response` | Controller/mode management | Advertised; no calls made. Autonomous navigation should normally retain the high-level locomotion controller |
| `/api/robot_state/request` and `/api/robot_state/response` | Robot service/status management | Advertised; operations not exercised |
| `/lowcmd` | Direct motor commands | Existing internal traffic observed at approximately 159 Hz. It is not the appropriate interface for ordinary autonomous missions |

The official SDK supports high-level status and motion APIs. Its ROS 2 integration
describes request/response messaging through the sport topic pair.
[Unitree Python SDK](https://github.com/unitreerobotics/unitree_sdk2_python),
[Unitree ROS 2 integration](https://github.com/unitreerobotics/unitree_ros2).

The SDK's sport and obstacle-avoidance velocity requests have different parameter
schemas. Build against the selected client rather than reusing hand-written JSON.
Avoidance being enabled does not establish collision protection for every API path.
A mission bridge must handle stale commands, stopping, operator override and robot
health, and its actual stop behavior must be checked before unattended operation.

## Onboard waypoint missions

The following topics are relevant if using Unitree's onboard mapping/navigation:

| Topics | Intended role from source/names | Observed condition |
|---|---|---|
| `/qt_command`, `/qt_notice` | Waypoint/navigation commands and execution feedback | `/qt_command` has a publisher but **zero receivers**; `/qt_notice` has a subscriber but **zero publishers** |
| `/qt_add_node`, `/qt_add_edge` | Define the topological waypoint graph | Advertised publishers, no receivers and no data during scan |
| `/query_result_node`, `/query_result_edge` | Graph query results | No publishers or samples |
| `/uslam/client_command`, `/uslam/server_log` | SLAM lifecycle commands and diagnostics | Command receiver exists; no messages observed; command protocol not validated |
| `/uslam/frontend/odom`, `/uslam/localization/odom` | Mapping/localization pose | No publishers or samples; these cannot currently provide localization |
| `/uslam/cloud_map` | 3D map output | Publisher advertised, no live samples |
| `/uslam/frontend/cloud_world_ds`, `/uslam/localization/cloud_world` | SLAM/localization clouds | No publishers or samples |
| `/uslam/navigation/global_path` | Vendor path output | No publisher or samples; type is `PointCloud2`, not `nav_msgs/Path` |
| `/lio_sam_ros2/mapping/odometry` | Alternative advertised mapping odometry | No publisher or samples |

The downloaded [official Unitree SLAM repository](https://github.com/unitreerobotics/unitree_slam)
contains source examples for mapping, graph editing, relocation, single/multiple
waypoints, pausing, resuming and returning to origin. These examples compiled.
Their operation on this Go2 Pro X firmware is unverified. The endpoint evidence
suggests that the corresponding waypoint backend is currently inactive or unavailable;
the scan cannot establish why. Do not assume that a topic listed by an app-side
publisher provides a functional onboard command handler.

## Readiness gaps for a laptop-hosted Nav2 mission stack

1. **Time alignment:** receipt time is consistently about **785.8 seconds ahead of
   sensor header time**. Timestamps advance while data arrives, suggesting a clock
   offset rather than an old static sample. This difference is not a network-latency
   measurement. Align the robot/laptop clocks where supported, or use a carefully
   validated adapter preserving synchronization across clouds, IMU, odometry and TF.
   Independently stamping each stream with arrival time can break sensor fusion.
2. **Transforms:** neither `/tf` nor `/tf_static` was discovered. Supply
   `map → odom → base_link → sensor frames`, with a single authority per transform.
   The live odometry already identifies `odom` and `base_link`, but a corresponding
   transform broadcaster is still needed. Calibrate sensor mounting geometry;
   `cloud_base` simplifies local perception because it is already in the body frame.
3. **Global mapping/localization:** no `/map` exists and no vendor localization
   odometry is active. For 2D indoor navigation, convert a suitable cloud slice into
   `/scan`, then configure SLAM Toolbox/AMCL and an occupancy grid. For 3D terrain,
   choose a validated LIO/global-localization and traversability pipeline.
4. **Robot control bridge:** no `/cmd_vel` subscriber or Nav2 actions exist. Implement
   a bounded velocity bridge to the chosen high-level SDK API, with a command timeout,
   explicit stop handling and operator override. Its motion behavior remains untested.
5. **Robot-specific configuration:** measure the footprint including legs/body,
   tune speed/acceleration limits and obstacle height filters, and validate how
   odometry drifts during walking. Zero covariance arrays are not measured certainty;
   establish credible noise settings before fusion. The stationary scan cannot
   establish motion accuracy, slip, dynamic obstacle avoidance or route success.

Nav2 needs a consistent TF tree and odometry pose/velocity data; standard sensor,
map and command adapters are the missing integration work here.
[Nav2 transform requirements](https://github.com/ros-navigation/docs.nav2.org/blob/rolling/docs/configuration_and_development/first_time_robot_setup_guide/transformation/setup_transforms.md),
[Nav2 odometry requirements](https://github.com/ros-navigation/docs.nav2.org/blob/rolling/docs/configuration_and_development/first_time_robot_setup_guide/odom/setup_odom.md).
The locally downloaded runtime/source targets **Humble**; these links explain
general architecture and do not imply a newer ROS distribution is installed.

For flexible missions such as patrol routes, inspection waypoints and custom task
logic, a laptop-hosted Nav2 stack is the practical development path from the verified
streams. That is an engineering recommendation based on the scan, not a claim that
Nav2 is already configured. Once mapping, localization and the control bridge are
validated, use NavigateToPose/NavigateThroughPoses or FollowWaypoints and a task
manager for retries, pause/resume and low-battery policies.

## Other topics and limitations

`/frontvideostream` produces raw traffic at about 28.7 Hz, but all three bounded
samples failed deserialization with the downloaded message definition/middleware.
The cause is unresolved, so it is not ready for a vision navigation pipeline. It is
also a custom video transport, not a standard ROS image plus calibrated CameraInfo.
`/arm_Command`, `/arm_Feedback` and `/config_change_status` lacked built message
definitions in the full scan; their payloads were not inspected. The voxel definition
was subsequently added from the official C++ header and decoded successfully.

`/gnss` produces JSON at about 1 Hz, but reports `fixed=0` and an embedded timestamp
that disagrees with the current session. Treat the coordinates as unverified;
neither fix status nor timestamp semantics were established. It is not a validated
`sensor_msgs/NavSatFix` source for outdoor mission localization.

Audio, lights, gesture, GPT, pet, UWB and connectivity topics may support mission
interaction or diagnostics but are not needed for core LiDAR navigation. UWB may be
relevant with known anchors, which were not established here. A nonzero sport
`error_code=100` was observed consistently; its meaning must be checked against the
specific firmware rather than guessed from generic error tables.

See [the complete topic inventory](reports/topic_inventory.md),
[full scan JSON](reports/live_scan.json), [full scan CSV](reports/live_scan.csv),
[focused scan JSON](reports/focused_scan.json) and
[download manifest](reports/download_manifest.json) for the evidence.
