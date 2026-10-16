# Saved live Point-LIO maps

Live mapping, RViz and both recorders are stopped. Both bags closed cleanly (return code 0). Passive sensor bringup and the unchanged IMU adapter were left running. No robot movement commands were sent.

- `drive/map_3d.html`: standalone interactive map from the first live drive; drag to rotate and scroll to zoom.
- `drive/map_preview.png`: top and oblique views with estimated robot path.
- `drive/registered_scans_full.pcd`: all finite points from all 2,074 recorded registered scans (945,470 points), with intensity.
- `drive/map_5cm.pcd`: same accumulation voxel sampled at 5 cm (191,159 points).
- `drive/trajectory.csv`: estimated IMU path in the map frame.
- `restart/`: separate short restarted capture, with its own map origin.

These maps accumulate already registered, map-frame scans from the bags. The estimator was not rerun and the application code/configuration was not changed. This is a 3D point cloud export, not a 2D occupancy grid or dump of the estimator internal map. The original bags retain the inputs and outputs. Preview-only outer-tail clipping is documented in each export.json; the full PCD retains all finite recorded points.

The user confirmed that the travel in the first run was real manual motion. The assistant stopped that run before waiting for this confirmation. Its travel must not be described as stationary drift. The two maps are intentionally kept separate rather than merged with an assumed alignment.
