# Additional temporal-calibration recording

Capture manual short forward/back passes, small left/right turns, body tilts,
stationary holds and a repeat at a different gentle pace, in a scene with fixed
walls/corners. The user confirmed 0.5-1 m clear space and Ethernet slack. The
agent sends no movement commands. Automatic recording limit is 300 seconds.

Keep original raw per-point time fields and calibrated body-IMU source stamps.
Record vendor body cloud, invalid/raw LiDAR IMU for diagnostics, source sport
telemetry, vendor/relayed odometry, private clock, status, TF, joint states and
robot description. The latter make the saved input/model self-contained for a
future isolated trial, without adding a second robot-state publisher.

After user done: stop and flush recorder, verify closed SQLite integrity/topic
counts/metadata, then decode. Never open the active recording database. Preserve
all earlier captures and active application hashes.

First assess actual yaw/translation excitation, status and timestamp/IMU coverage.
Then attempt LiDAR-only relative scan registration using the existing Open3D
installation, independently of body/vendor pose and IMU motion. Raw full-scan
registration can suffer intra-scan distortion: test short point-time subframes,
geometry conditioning and registration quality before using its motion as a
reference. Reject unobservable or poor fits. The selected body IMU and reviewed
extrinsics stay unchanged.

Only infer an acquisition offset if independently repeated sections provide a
well-defined, consistent optimum and a held-out section improves without refit.
Do not equate successful registration or a visually good map with proof of exact
hardware synchronization. No calibration correction is authorized by recording
alone. No Point-LIO or other mapper is started during this capture.
