# Jetson recording integrity check — 18 October 2026

**The MCAP container is intact and the essential Point-LIO input messages decode successfully.** One camera topic has a schema/wire-format incompatibility. The recording is only **12.4757 seconds**, not a long-distance test capture. Original files were opened read-only and left unchanged. No recorded messages were played or published.

Bag: `/home/jain/go2_bags/robot_20261018_055326`

File: `robot_20261018_055326_0.mcap`, **49,889,371 bytes**.

SHA-256: `fb83782c8c34f34ec15977390efedc6e5c8391a4b7b9815ddda807683b2331fb`. This identifies the laptop copy; an original Jetson hash was not provided for transfer comparison.

## Container and count checks

- Valid opening magic, complete footer, closing magic and no trailing unread bytes.
- All **148 zstd chunks** decompress and parse.
- Summary/footer CRC is present and matches an independently calculated checksum.
- There are no stored chunk CRCs or data-section CRC, so a payload-wide checksum verification cannot be claimed.
- Sequentially read **24,109 messages**. Total, every per-topic count, start time and duration match `metadata.yaml` and the MCAP summary.
- Metadata lists 83 topics; 27 have messages. Zero-message listed topics are not evidence of file corruption.
- File size and modification timestamp remained unchanged throughout inspection.

## Essential mapping inputs

| Topic | Messages | Result |
|---|---:|---|
| `/utlidar/cloud` | 191 | Every message decodes; point-cloud buffer/layout checks pass |
| `/sportmodestate` | 3,665 | Every message decodes; selected IMU fields finite |
| `/utlidar/robot_odom` | 1,844 | Every message decodes |
| `/utlidar/imu` | 3,045 | Every message decodes; finite fields do not establish correct physical acceleration semantics |

Selected sensor streams have no backward source timestamp steps. SportModeState contains 63 duplicate source stamps, which alone do not indicate file corruption. Body and LiDAR timestamps retain the robot's own epoch. The bag does not contain our calibrated body-IMU output, Point-LIO outputs, canonical TF or robot clock; these would need to be reconstructed for offline Point-LIO analysis. This is a raw recording, not an existing Point-LIO run.

## Camera schema exception

**23,753 messages decode using their embedded ROS message schemas.** The remaining **356 `/frontvideostream` messages** cannot be decoded using their advertised `unitree_go/msg/Go2FrontVideoData` schema (`uint64 time_frame` followed by three byte sequences).

Every one of those 356 records instead has a consistent bounded layout of one uint64, one uint32 and one byte sequence, followed by exactly the padding declared by its CDR header. The uint32 values are 360 and 720, 178 occurrences each. All sequence payloads begin with an Annex-B start code. Field semantics are not assumed or repaired. This consistent layout supports a camera publisher/interface-schema compatibility issue rather than random file truncation. The mapping inputs are unaffected. Camera playback should not be declared validated.

## ROS version compatibility

Metadata identifies ROS 2 **Jazzy**, version 9, with the newer QoS representation. The original laptop's Humble `ros2 bag info` fails parsing that metadata (`bad conversion` at the first QoS entry). An independent MCAP reader successfully read the file directly, using embedded schemas. The CLI metadata error therefore is not proof of a corrupt MCAP. No metadata conversion, reindexing, repair or playback was attempted.

Temporary inspection readers were installed under `/tmp/doggy_mcap_integrity_tools`; the ROS workspace and runtime configuration were not changed.

## Artifacts

- [Detailed integrity and message-count results](integrity_report.json)
- [Camera layout audit](camera_schema_audit.json)
- [Summary](summary.json)
- [Read-only checker source](check_mcap.py)
