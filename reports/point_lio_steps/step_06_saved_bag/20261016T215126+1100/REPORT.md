# Denser initial-map trial — completed, residual drift

The user approved a temporary `init_map_size` change from 10 to 4000, relative to
the original input profile. Gyro measurement variance stayed at the original 0.1;
the previous failed gyro-only override was not loaded. Actual estimator parameter
reads confirmed these settings, zero provisional lag and unchanged source/frames.
No active application/configuration files were edited and no robot commands sent.

The full 253-second saved motion bag completed in isolated localhost ROS domain
197. Playback used half speed initially and normal speed after roughly 34 source
seconds. All test-owned processes and RViz stopped cleanly after completion.
There were 3894 estimator odometry, registered-cloud and map->odom TF messages,
with exact matching odometry/TF stamps and no multiple TF parents or expired TF.
All 3906 raw clouds were observed; twelve scans were used before regular outputs
began, consistent with the larger startup seed.

The first 13.75-second stationary comparison improved substantially: displacement
from the first estimate was 4.58 cm, compared with 2.327 m for the original seed
and 1.656 m for the gyro-only trial. This supports improving startup geometry,
not declaring the whole mapper fixed.

During source seconds 5–55, the post-startup reference excursion reached 34.6 cm
and orientation changed by 4.37 degrees, despite the vendor position changing by
less than 1 mm. The full trajectory also separated from the initially aligned
vendor estimate, reaching 1.92 m position difference and 13.17 degrees orientation
difference. Vendor odometry is a comparison, not external ground truth. The final
stationary source interval 220–250 seconds had 3.54 cm excursion and 0.61 degree
orientation change. Large registration discrepancies remain unresolved.

[Comparison plot](mapping_comparison.png) · [Metrics](analysis.json) ·
[Runtime parameter read](live_parameter_audit.json)

The passive observer used best-effort depth 5 for IMU and repeatedly converted
its growing trajectory to an array on every callback. Its CPU cost grew during
the run. It missed 16,887 IMU stamps within its observed interval, starting with
small losses around source second 70 and increasing later. The mapper itself
has a reliable/depth-200000 IMU subscription; observer losses do not establish
mapper input loss. This run cannot certify full IMU sample delivery. A repeat
with only the passive observer corrected was announced; mapping settings remain
the same. See [observer audit](observer_imu_audit.json).

A test-preparation syntax error caused the shell to restart the old observer.
That run was interrupted at 11.48 source seconds and saved separately in
`unintended_restart`. The completed 3894-pose output/result were backed up and
restored; the full original launch log was recovered from its ROS launch log.
See `restart_incident.json`. No active application change or robot command
occurred. The proper repeat uses a new directory and syntax-checked preparation.

The exported `registered_scans_sampled.ply` is a diagnostic accumulation of
subsampled registered clouds, not the full internal map or a navigation map.
The default profile remains unchanged. The live body-IMU adapter remains stopped
on its source-quaternion guard; only recorded IMU data was used here.
