# Additional motion capture and independent timing test

The user confirmed the motion overview with “yeah looks about right”, including
curved forward/back excursions and the recorded turns. The 253.38-second bag
closed cleanly and passed SQLite integrity, metadata/count and all 13 topic
checks. Host UDP receive-buffer overflows and Ethernet errors/drops increased
by zero. Performance mode remained selected. No active code/configuration,
TF, calibration or lag was changed; no ROS mapping node or replay was started.

[Motion overview](motion_overview.png) · [Input integrity](input_integrity_summary.json)

All 73,250 unique interior source IMU stamps had corresponding calibrated output
stamps, with no extras. All 3,876 interior raw clouds were bracketed by IMU.
There were no body/cloud timestamp backsteps. Maximum body input gap was
25.350 ms; this is not uniformly spaced acquisition. Vendor LiDAR error code
was zero. The cloud packet-loss field reached 0.552486 in vendor units, while
the IMU loss field remained zero; its units have not been established.

Independent raw-cloud registration did not yield a reliable acquisition lag.
Adjacent sparse scans had poor geometric overlap. A stationary accumulated
reference improved correspondence, but accepted matches did not provide an
accurate enough motion reference across turns and later repeats. No body IMU
or vendor pose was used to guide the cloud registrations.

The full-scan reference test had 295 initially geometry-gated poses out of
2,375 evaluated. Comparing accepted endpoint rotations against integrated body
gyro gave about 6.76 degrees relative rotation RMSE; fitting delay reduced the
objective by only 0.099%. Its entire -100 to +100 ms search range remained
within 5% of the minimum, with no usable held-out repeat pairs. Restricting
scans to a central 30 ms window gave approximately 10.9 degrees RMSE and also
an insensitive objective. The two methods' nominal minima disagreed. Those
minima are not sensor-delay measurements and must not be applied.

[dense_full_timing.json](dense_full_timing.json) · [Full-scan fit plot](dense_full_timing.png) ·
[dense_short_timing.json](dense_short_timing.json) · [Short-window fit plot](dense_short_timing.png)

Conclusion: transport and source/output completeness passed for this capture;
independent acquisition-delay calibration is inconclusive. Configured zero lag
remains explicitly provisional. A purpose-built calibrator requires a separate
compatibility review and approval before installation/execution. The next
Point-LIO trial also needs the proposed output-TF integration and user approval.
Do not request more of the same motion or tune fit gates merely to obtain an
offset from this insensitive diagnostic.
