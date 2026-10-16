# Step 1 — accelerometer validity

Status: **STEP 1 COMPLETE for the approved calibrated body-IMU route.
User double-check and subsequent standing validation passed.**

The original LiDAR IMU has not been repaired. Following the source investigation,
the user explicitly approved using a calibrated body-IMU adapter and its passive
check. Current implementation and results are appended below. Earlier statements
about no correction/code changes describe the historical investigation stages.
No later workflow requirement has been started.
The user requested sequential fixes and a user double-check of each result.

## Fresh baseline

The operator reported the dog sitting or lying down and was asked to leave it
untouched. A 20-second passive subscription captured 4,836 LiDAR IMU messages and
7,880 body LowState messages. All decoded numbers were finite. Body attitude
peak-to-peak roll/pitch/yaw variation was about 0.081/0.079/0.072 degrees. This
supports treating the capture as a quiet baseline; it does not exclude vibration.

| Quantity | LiDAR IMU | Body IMU |
|---|---:|---:|
| Mean X acceleration, m/s² | 3.91648 | 0.84262 |
| Mean Y acceleration, m/s² | -0.00232 | -0.07552 |
| Mean Z acceleration, m/s² | 9.80493 | 9.49067 |
| Mean acceleration magnitude, m/s² | 10.55930 | 9.52837 |

The body IMU also has a magnitude discrepancy and is not being treated as a
validated drop-in replacement. Its reported RPY is used only as a comparison
reference with that limitation. No source topics were modified. The covariance
arrays on every received LiDAR IMU message were zero, meaning unknown uncertainty.

## Fixed-bias check on the existing posture bag

Use the previous independent raw/body cloud rotation, which was also supported
by gyro correspondence, and the recorded body attitude to predict quasistatic
specific force. Compare baseline and the held positive/negative pitch and roll
poses. This depends on those reference assumptions and is not factory calibration.

| Axis | Predicted held-mean span, m/s² | Reported held-mean span, m/s² |
|---|---:|---:|
| X | 1.7442 | 3.4935 |
| Y | 1.7233 | 0.00565 |
| Z | 0.5033 | 0.00614 |

A single fitted additive bias leaves per-axis held-mean RMS discrepancies of
0.645/0.659/0.164 m/s² under this reference. Merely reversing the full acceleration
sign and fitting a bias also fails. An exploratory per-axis fit gives an X gain
near 1.99 and near-zero Y/Z response across these poses. These are diagnostic
observations, **not calibration values to install**. The limited angular range,
reference uncertainty and nearly absent Y/Z response do not justify a reliable
inverse gain correction. Dividing X by two or synthesizing missing gravity
components would not establish valid dynamic acceleration measurements.

## Published interface review

The official Unitree standalone L1 ROS 2 driver copies all three acceleration
components directly into the ROS message. Its public source does not document a
special correction matching the observed DDS pattern. The checked official SDK
C++ Imu/Vector3 DDS declarations use the standard field order and float64 vector
components, consistent with ROS sensor_msgs/Imu. This is a source-schema check;
it does not independently validate the inaccessible onboard publisher or its data.

- [Official driver](https://github.com/unitreerobotics/unilidar_sdk/blob/main/unitree_lidar_ros2/src/unitree_lidar_ros2/include/unitree_lidar_ros2.h).
- [Official SDK DDS Imu schema](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/idl/ros2/Imu_.hpp).
- [ROS Humble Imu definition](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Imu.msg).
- [Manufacturer-repository report of invalid Go2 LiDAR IMU values](https://github.com/unitreerobotics/unilidar_sdk/issues/34).
  That reporter later replaced a sensor. Their enormous invalid readings differ
  from this robot's finite, repeatable pattern; their diagnosis cannot be assigned
  to this robot. No applicable software correction was established from that issue.

## User double-check completed

The user supplied a fresh direct ROS echo with acceleration
`[3.90864634513855, 0.035757191479206085, 9.731149673461914]` m/s².
Its magnitude is 10.486852 m/s², consistent with the earlier baseline pattern.
This independently confirms the topic readings. It does not establish physical
stationarity during the earlier capture; that explicit confirmation was not given.
The submitted values are saved in `user_crosscheck.json`.

## Independent official native DDS decoding check

A temporary read-only C++ subscriber used the official SDK's generated
`sensor_msgs::msg::dds_::Imu_` type on `rt/utlidar/imu`, outside ROS. In parallel,
a ROS subscriber decoded `/utlidar/imu` as standard `sensor_msgs/msg/Imu`.
Both subscribed in domain 0 on Ethernet. No data writer, service client,
control client, calibration, mapping, TF or configuration update was used.

| Check | Result |
|---|---:|
| Native SDK messages | 3,423 |
| ROS messages in longer capture window | 4,662 |
| Paired by identical integer seconds/nanoseconds | 3,423 |
| Compared numeric fields per pair | 37 |
| Frame ID differences | 0 |
| Maximum numeric difference | **0.0** |
| Exact decoded floating-point agreement | **Yes, every paired field** |
| Native mean acceleration XYZ, m/s² | [3.91549, -0.00110, 9.80374] |
| Native mean acceleration magnitude, m/s² | 10.55785 |

The 37 fields comprise quaternion (4), gyro (3), acceleration (3) and covariance
matrices (27). All compared values were finite. The subscribers' different capture
windows explain their different total message counts; comparison uses only exact
matching source timestamps, not receipt-time interpolation.

**Conclusion:** the unusual acceleration is present in the DDS publication.
A discrepancy between ROS decoding and the official SDK decoding is ruled out
for these samples. This does not identify whether the values originate in sensor
hardware, sensor firmware or the onboard conversion/publishing service. Native
DDS still reads the robot's publication, not a separate direct hardware connection.
It also does not prove the published frame/measurement semantics are correct.

`native_dds_crosscheck.json` contains full metrics. Both numeric capture CSVs,
`native_imu_reader_diagnostic.cpp` and `native_diagnostic_provenance.json` are
saved for inspection. The source was compiled as a temporary diagnostic in /tmp;
no SDK, ROS package, Point-LIO source or application configuration was changed.

A six-second passive `/utlidar/server_log` subscription returned no messages;
it therefore supplied no firmware/calibration diagnostic. See `lidar_server_log.json`.

## Next decision within step 1

The user has only topic access, so an onboard driver inspection or direct internal
sensor connection is not currently available. The user also confirmed that app access is unavailable. Robot firmware and
LiDAR calibration entries cannot currently be inspected through that route.
Do not start an update, reset or calibration without a documented applicable
procedure and explicit user agreement. No calibration or change is proposed yet.

If no supported route restores a physically valid LiDAR IMU stream, alternatives
would require a separate decision: supplier investigation of that stream, or
validation of a different IMU input. The body IMU has not passed full calibration
or LiDAR synchronization checks and is not being substituted automatically.
Later Point-LIO requirements remain queued while this input issue is unresolved.

Pass criterion: physically consistent specific-force response across independent
stationary poses and a justified measurement/input contract. Receiving messages,
matching the expected data type, or forcing the magnitude to 9.81 is insufficient.

Artifacts: `sitting_baseline.npz`, `sitting_baseline.json`,
`held_pose_diagnostics.json`. Application code, runtime sensor configurations,
TF and the standalone upstream checkout were unchanged.


## Topic-only options after access clarification

The user confirmed no access to the Unitree app. Only robot topics are available;
SSH/factory files were already unavailable. The checked Go2 SDK configuration
interface exposes generic Get/Set/Meta operations on named configuration entries,
but does not establish which LiDAR calibration entry exists or its semantics.
The checked public L1 sensor SDK has IMU reads and version access for a direct
sensor connection; that is different from the laptop's existing robot DDS access.
No supported calibration command for the observed DDS acceleration behavior was
established in the inspected sources. This is not proof that no private or newer
vendor procedure exists. No guessed configuration names or command IDs were sent.

### Offline body-IMU screening

The existing posture bag supplies another available IMU stream inside `/lowstate`.
Five held poses (baseline, positive/negative pitch and roll) were compared with
quasistatic specific force predicted from body RPY. Both physical excitation
axes produce body acceleration changes, unlike the near-flat Y/Z LiDAR channels.

| Quantity | Reference from body attitude | Body IMU |
|---|---:|---:|
| Held-mean X span, m/s² | 1.88852 | 1.85607 |
| Held-mean Y span, m/s² | 2.23882 | 2.20948 |
| Held-mean Z span, m/s² | 0.09008 | 0.09127 |

A constant comparison offset leaves held-mean RMS discrepancies of about
0.012/0.010/0.006 m/s² in XYZ. The comparison offset is approximately
`[-0.12955, -0.01941, -0.27702]` m/s². These are **diagnostic comparison values,
not calibration to install**. Body RPY is provided by the robot's estimator and
may depend on the same accelerometer; it is not an independent external attitude
reference. Limited posture excitation and stable mean agreement do not validate
full dynamic behavior, noise, bias calibration or LiDAR synchronization.

This makes the body IMU a candidate worth validating, not a validated replacement.
LowState has a boot tick rather than a ROS epoch header; any later adapter needs
an explicit timestamp model, not an arbitrary receipt timestamp. It also needs
body-IMU-to-LiDAR extrinsics. Those are additional requirements if the user
chooses this route; later workflow steps have not been marked passed or started.

The source choice needs user review before an application change:

1. Validate body IMU as a candidate Point-LIO input while retaining raw LiDAR
   clouds. No input switch yet. This is the practical software route to assess
   with currently available topics; it is not a guarantee of successful fusion.
2. Retain the LiDAR IMU requirement and seek a documented supplier/Unitree remedy
   or access to direct sensor output/onboard configuration. The existing DDS
   acceleration has not passed and cannot be repaired by a justified local
   correction from the current evidence. No vendor message has been sent.

No application code, runtime sensor configuration, TF or estimator setting was
changed by this screening. Step 1 remains unresolved pending source choice and
validation. Full metrics are in `body_imu_suitability.json`.


## Approved solution for step 1: calibrated, timestamped body IMU

The user approved applying the adapter and running a passive output check.

- Active adapter: `scripts/publish_body_imu.py`.
- Approved fixed-bias file: `config/point_lio/body_imu_calibration.json`.
- Source: `/sportmodestate`, using its body accelerometer and gyro fields.
- Output: `/doggy/imu/body_calibrated`, `sensor_msgs/Imu`, developer frame `imu`.
- Header time: original source TimeSpec seconds/nanoseconds, preserved exactly.
- No source topics, TF, Point-LIO source/configuration or robot settings changed.
- No robot commands, calibration motion, mapping or ROS bag playback started.

### Calibration model and independent checks

Accelerometer bias was fitted from gravity magnitudes in five held poses, assuming
SI units and unit gain. Body orientation was NOT used to fit the bias. One fixed
bias is subtracted from every sample; there is no per-sample normalization,
gravity removal/synthesis or replacement of dynamic acceleration.

Estimated accelerometer bias XYZ in m/s²:
`[-0.0366806006, 0.0049033384, -0.2773451302]`.

Gyro bias XYZ in rad/s from a separate quiet capture:
`[-0.0035657965, -0.0039591873, -0.0114517265]`.

Five-pose leave-one-pose-out gravity-magnitude errors stayed below 0.00155 m/s².
The three held sections not used in the accelerometer fit stayed below 0.00334
m/s² mean-vector magnitude error. A fresh, later body capture gave a corrected
mean-vector magnitude of 9.80677 m/s². These checks use held poses and a later
capture; they do not establish complete scale/misalignment or walking calibration.

The fresh SportModeState capture contained 3,216 exact six-channel acc/gyro
matches to LowState within 20 ms receipt proximity. Its fused heading differs
slightly from LowState, so identical complete IMU messages are not claimed.
SportModeState supplies its own timestamp; no boot-tick-to-epoch guess is needed.
This is not proof of acquisition-time synchronization with the LiDAR.

### Adapter verification

Offline processing checked all 6,190 recorded SportModeState inputs: 6,161
strictly timed outputs and 29 identical duplicate stamps discarded. Changed data
at one timestamp, backward clocks and unapproved calibration were rejected.
Source timestamps, quaternion ordering/normalization and unknown covariance
semantics were checked. The output preserves the body fused quaternion in ROS
XYZW order; this Point-LIO implementation does not consume that quaternion.

The live 20-second check paired 5,819 source/output messages by exact source
stamp. Maximum numeric differences for correction and quaternion conversion were
zero. All ten live checks passed, including finite fields, strictly increasing
timestamps, developer `imu` frame, unknown covariance, one output publisher and
absence of mapping nodes. Corrected acceleration magnitude averaged 9.805754
m/s², standard deviation 0.027640 m/s². Mean corrected gyro XYZ was approximately
`[-0.0001781, 0.0000493, 0.0000802]` rad/s. Body attitude was quiet during the
check, with reported RPY spans near 0.12/0.12/0.11 degrees.

### User double-check before step 2

The adapter is running. In the current sitting/lying posture, expect acceleration
roughly `[0.88, -0.08, 9.77]` m/s², with small fluctuations. Nonzero X/Y while
tilted are gravity projections; the relevant stationary magnitude is near 9.81.

```bash
cd /home/jain/Doggy
source scripts/setup_ros.sh
ros2 topic echo /doggy/imu/body_calibrated --field linear_acceleration --once --qos-reliability best_effort
```

After the user checks this result, close step 1 and move to extrinsics/TF. At this
save point all later steps remain queued. The new topic is not yet selected by a
Point-LIO launch; that belongs to its separate input-profile step.

Evidence: `body_bias_validation.json`, `body_imu_proposal_offline_checks.json`,
`body_imu_live_validation.json/.npz`, `body_imu_implementation.json` and
`body_imu_adapter.log`. The earlier proposal remains archived for review; the
active calibration file is the explicitly approved version.


## Step 1 closure after user standing confirmation

The user supplied four direct corrected-topic readings and confirmed the dog was
standing. Their magnitudes were 9.7932, 9.8063, 9.8491 and 9.8709 m/s². A fresh
15-second standing capture then received 4,282 messages, with mean acceleration
magnitude 9.803781 m/s² and standard deviation 0.040954 m/s². Mean gyro XYZ was
approximately `[-0.000243, 0.000281, 0.000070]` rad/s. All six standing checks
passed, including one publisher and strictly increasing timestamps. The existing
calibration was not changed for standing.

This closes the static input-validity requirement for the user-approved body
IMU route. The LiDAR IMU remains unvalidated and unused by this adapter. Full
intrinsic/dynamic accuracy and sensor-relative timing remain outside this static
check; mapping readiness is not claimed. The workflow can now enter the separate
extrinsics/TF review (step 2). See `standing_user_confirmation.json/.npz`.
