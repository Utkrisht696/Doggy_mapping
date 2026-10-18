# Isolated Point-LIO acquisition-gap recovery trial

This directory contains an opt-in C++ prototype and offline replay tools. It does not modify the active workspace package.

- `src/point_lio/src/gap_recovery.hpp`: timestamp-checked vendor relative-pose aid.
- `trial.patch`: estimator integration against the active standalone source.
- `test_recovery_aid.cpp`: transform and invalid/stale/missing-aid tests.
- `offline.launch.py`: private mapper binary with existing calibrated inputs and exact-time TF bridge.
- `run_replay.py`: localhost/domain-211 replay of the lab bag, preserving original acquisition gaps. Outputs checkpoint every 15 seconds.
- `run_01/`: logs, input-delivery checks, poses, registered cloud samples and final analysis.

When a forward IMU acquisition discontinuity exceeds 0.15 s, scans overlapping the interval are discarded. For the next complete scan, vendor relative motion since the last tracking anchor seeds position, rotation and velocity in the existing LiDAR map. Bias, gravity and extrinsics are retained. Dynamic-state covariance is reinitialized with documented trial priors. Publication and map insertion are withheld until three scans pass a local point-to-plane agreement check.

The aid is used at the reseed, rather than continuously replacing Point-LIO odometry. This is local recovery near the prior map using vendor motion, not global place recognition, loop closure, calibrated acquisition-lag compensation, or recovery from every form of silent tracking loss. Missing or stale aid blocks reseeding. The probation plane check alone does not prove a globally unique pose.

Status: full replay and missing-aid negative test completed. See [REPORT.md](REPORT.md). Gap recovery succeeded; horizontal return offset improved from 10.39 m to 58.6 cm, with residual drift still present. Active workspace files are unchanged.
