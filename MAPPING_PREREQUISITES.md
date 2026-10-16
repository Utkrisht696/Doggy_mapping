# Mapping prerequisites: sensor-only verification

**16 October standalone migration:** CMU packages, preprocessing and calibration
tools have been removed. See [POINT_LIO_SETUP.md](POINT_LIO_SETUP.md) for current
Point-LIO requirements; the older sensor audit below remains historical evidence.

**16 October TF update:** the approved current tree is
`odom → base_footprint → base_link → developer links`, with no `base` alias.
The runtime description normalizes only the developer root name. See
[TF_CONSISTENCY.md](TF_CONSISTENCY.md) for the current contract and live evidence.
The original sensor audit below is dated 12 October.

Checked on 12 October 2026, Australia/Melbourne. Mapping is paused at the user's
request after poor map quality and shifting pose. No SLAM, scan converter, RViz,
navigation controller, calibration motion routine or robot commands were run
during this audit. Temporary sensor-only tests are stopped after verification.

## Current result

The Ethernet connection, sensor reception, official robot geometry, body/cloud
TF alignment and common robot time base passed. Cleaned point-cloud outputs and
standard `/odom` are prepared and verified. This does **not** establish that all
conditions for reliable mapping are satisfied: sensor calibration, physical
LiDAR identification and sufficient geometric observations still need attention.

| Requirement | Evidence and status |
|---|---|
| Ethernet / DDS | 1 Gbit/s full duplex, no interface receive errors/drops, no ping loss; one publisher per source sensor topic |
| LiDAR streams | Raw, body and deskewed streams around 15.4 Hz during the 30 s audit; headers advance without resets |
| IMU / odometry | Around 248 Hz / 149 Hz; finite measurements and normalized odometry quaternion |
| Official developer geometry | Unchanged URDF, all 42 links and 12 encoder joints verified live |
| Odometry TF | Full reported pose preserved through dynamic `odom` → `base_footprint` → `base_link`; former `base` alias removed in the 16 October update |
| TF ownership | Only the state adapter and official robot-state publisher; no child frame has conflicting parents; one robot-clock publisher |
| Cloud alignment | Deskewed cloud transformed into body coordinates agrees with body cloud: median nearest-point distance ~0.042 mm, maximum ~1.26 mm in the sampled pair |
| Time | Source stamps retained; consumers use robot time through `/doggy/robot_clock`; no global `/clock` publisher |
| Standard odometry | `/odom` relays the complete vendor message; live serialized-message equality checked |
| Invalid points | Cleaned outputs remove exactly-zero or nonfinite XYZ point records, retaining source coordinates, timestamps and fields |
| ROS software | Existing dependency checks passed; See POINT_LIO_SETUP.md for the standalone build; the previous mapper was removed |
| IMU calibration | Raw acceleration semantics/calibration unresolved; no biases or covariance fabricated |
| LiDAR generation | User says L1 is most likely; DDS identification fields are blank, so identification is provisional |
| Mapping feature coverage | Sparse at the previous height selection; mapping quality remains unverified |

## Findings relevant to the bad map

Every sampled `/utlidar/cloud_deskewed` message contained **10,000 exactly-zero
XYZ records**, marked `is_dense: true`, plus roughly 1,200 usable points. Zero
points must be removed before transforming a cloud into another frame: an origin
point can otherwise become a plausible nonzero point after transformation. These
placeholders are now removed in the prepared clean stream. The vendor topics
remain untouched. The raw and body cloud samples contained no such placeholders.

At the previous selection of -0.15 to 1.5 m above the body-frame origin, the
sampled body clouds occupied only 19–36 distinct one-degree angular bins
(median 27). Lowering the cutoff to -0.3 m increased bins to 80–115, but may admit
ground points. No scan/SLAM parameters were changed during this audit. These
counts measure available observations; removing placeholders does not create
missing wall/obstacle measurements.

Over the 30-second capture, reported position spanned about 0.48 mm in X,
0.47 mm in Y and 0.015 mm in Z. Heading spanned 0.505 degrees. The user confirmed
the dog stayed in place but vibrates/sways while standing. This is **reported
pose variation during standing**, not proven estimator drift: balancing may cause
real rotation. TF matches the input odometry; it does not introduce another
pose estimate. An independent orientation reference or a suitably still capture
is needed to separate physical sway from estimator drift. No heading clamp or
synthetic stationary pose was introduced.

The LiDAR IMU acceleration norm had a median of 10.36 m/s², spanning 9.92–10.86
m/s², and its reported orientation quaternion norm was about 0.9983. Standing
vibration complicates bias estimation. IMU and odometry covariance arrays are
all zero; this is not evidence of zero uncertainty. The raw IMU is retained
unchanged and robot-specific calibration remains outstanding before fusion.

Laptop-minus-sensor time is about 787 seconds. This is a clock-offset observation,
not a measured one-way network delay. Earlier paired samples had deskewed-cloud
headers roughly 0.062 s after raw-cloud headers, while raw per-point times also
spanned about 0.062 s. These clouds represent different processing stages.
Each processing chain must keep its documented timestamp convention. The robot
clock setup avoids comparing robot sensor stamps directly against laptop wall
time; it does not calibrate sensor time offsets or measure transport latency.

The cause of the earlier large shifts in the `map` frame is not established by
this audit. There is no mapping node running to attribute those corrections.
The measured sparse observations and placeholder points are input issues to
address before interpreting a future mapping result.

## Run only the prepared sensors and TF

```bash
cd /home/jain/Doggy
bash scripts/start_mapping_sensors.sh
```

This starts three processes: the read-only state adapter, the official URDF
publisher, and the point-record filter. It opens no RViz and performs no SLAM.
Ctrl+C stops the launch. The shared lock prevents simultaneous visualization or
mapping launches from creating duplicate state/clock owners.

| Topic | Coordinates / purpose |
|---|---|
| `/odom` | Original vendor `nav_msgs/Odometry`, parent `odom`, child `base_link`, original stamps/twist/covariance |
| `/doggy/lidar/cloud_raw` | Cleaned raw point records in `utlidar_lidar`; intensity, ring and time retained |
| `/doggy/lidar/cloud_body` | Cleaned vendor body cloud in `base_link` |
| `/doggy/lidar/cloud_deskewed` | Cleaned vendor motion-compensated cloud in `odom` |
| `/utlidar/imu` | Original LiDAR IMU, unchanged; calibration still needed |
| `/doggy/robot_clock` | Robot-derived time used by the model and sensor preparation |
| `/joint_states`, `/tf`, `/tf_static` | Live joint angles, reported odometry and official developer geometry |

The raw `utlidar_lidar` and `utlidar_imu` names are not developer URDF link names.
No guessed link alias was added. They are not interchangeable with the URDF's
`radar`/body `imu` links. Registered body/deskewed streams can already use the
verified TF; raw LiDAR/IMU fusion needs the correct sensor profile and calibration.

The existing SLAM test launcher still selects its previously prepared inputs and
parameters. It has not been rewired or retested with these new outputs because
the current task excludes SLAM work. The earlier startup result is not a mapping
quality approval.

In another terminal, check the sensor-only setup:

```bash
cd /home/jain/Doggy
source scripts/setup_ros.sh
python3 scripts/check_lidar_tf.py --output reports/foundation/manual_tf_check.json
python3 scripts/check_clean_lidar.py --seconds 12 --output reports/foundation/manual_clean_check.json
python3 scripts/audit_mapping_sensors.py --seconds 30 --output reports/foundation/manual_sensor_audit.json
```

The cloud checker matches source and output by timestamp, compares retained point
records byte for byte, checks original frame/field definitions, measures relative
receipt times, and verifies that SLAM nodes are absent. The filter uses NumPy and
typed byte arrays to avoid Humble's slow per-byte validation path.

The final 12-second check matched 187 raw, 187 body and 184 deskewed cloud pairs,
with zero integrity failures. Source and output receipt rates agreed at roughly
15.4 Hz. Median source-to-clean receipt differences were 3.5 ms, 5.3 ms and 3.8 ms,
respectively. These relative receipt measurements are not end-to-end LiDAR
latency. The `/odom` relay matched 1,772 original messages with zero differences.

## Confirming L1 versus L2

Check the sensor housing/label for an L1/L2 marking or part number, or the supplier
specification/delivery sheet for this specific unit. The model name "Go2 Pro X"
alone is not a hardware identification. The current `/utlidar/lidar_state` reports
software `1.0.0.38` but blank firmware/SDK version strings. Point counts and stream
rates alone are not conclusive model evidence.

Unitree provides separate official [L1 SDK](https://github.com/unitreerobotics/unilidar_sdk)
and [L2 SDK](https://github.com/unitreerobotics/unilidar_sdk2) documentation. Both
describe parallel LiDAR/IMU axes and the same sensor-internal displacement;
that does not establish the robot DDS frame convention or validate every option
in a standalone L1 profile. The earlier audit did not reconfigure the sensor.
The later motion-calibration attempt is historical evidence only; its tools and
unaccepted candidate configuration have now been removed from active use.

Evidence is saved in `reports/foundation/`: `sensor_audit.json`,
`clean_cloud_check.json`, `final_urdf_check.json`, and `readiness.json`.
Software/URDF integrity evidence is in `reports/slam/software_check.json`.
