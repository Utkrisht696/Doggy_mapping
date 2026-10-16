# Delivery-delay check: power-profile comparison

The repeated passive test supports CPU/power-profile contention as the cause of
large delivery backlog. The previous diagnostic subscribers amplified this load.
The same unchanged adapter PID 1155223 was used for both trials. The user changed
the laptop from Power Saver to Performance between them; no ROS application code,
TF, calibration, timestamp or source QoS change was needed for this check.

## Before and after

| Workload | Power Saver p99 / max delay | Performance p99 / max delay |
|---|---:|---:|
| Sensors only | 23.38 / 56.37 ms | 0.63 / 1.17 ms |
| Recording, before extra probes | 3.15 / 16.24 ms | 0.65 / 3.50 ms |
| Recording + four Python probes | 209.36 / 309.18 ms | 1.21 / 3.77 ms |
| Recording, after removing probes | 10.65 / 71.90 ms | 0.65 / 4.98 ms |
| Sensors only, recovery | 39.99 / 62.65 ms | 0.64 / 2.33 ms |

Performance mode matched all **38,778** observed source timestamps in the analysed
phase interiors, with zero missing calibrated-output stamps, zero backwards source
stamps and zero UDP receive-buffer overflows. The largest relative delay was
**4.98 ms**; all phase p99 values were below **1.22 ms**. Record-only p99 was about
**0.65 ms**, maximum **4.98 ms**. Both recordings closed cleanly, with all ten topics,
metadata/count agreement and SQLite quick_check=ok. Diagnostic readers/probes are
stopped; the original headless sensors and body adapter remain running.

Power Saver: sampled CPU frequencies approximately 800-900 MHz. Adapter median
CPU was about 68% of one logical core for sensors only, 76% with recording and
84% with the extra Python probes. With those probes the laptop median CPU busy
was about 79%, peaking near 99%, and the adapter spent longer runnable but waiting
for a CPU. The corresponding delay spike supports a scheduling/backlog explanation.

Performance: sampled frequency about 4.2 GHz. Adapter median CPU was roughly
15% for sensors only, 16% with recording and 21% with the probes. Whole-laptop
median busy was about 10% recording-only and 20% with probes. These are successive
live observations; they do not isolate every factor behind the historical 0.6 s
event, but show that the same workload no longer creates that backlog.

## What this validates

The local delivery check passes for the tested passive sensor and recording
workloads in Performance mode with the approved larger DDS buffers. It does not
calibrate physical LiDAR/IMU acquisition lag or exercise Point-LIO/RViz CPU load.
Do not subtract the earlier 0.6 s delay from sensor timestamps: it was variable
delivery backlog, not a measured constant acquisition offset.

The observer uses raw serialized subscriptions, verifies initial stamp parsing
against ROS deserialization, then records only timestamps and DDS metadata.
Phase interiors omit the first 5 and last 3 seconds; process transition pauses
are excluded. Polling/order can bias sub-millisecond values, so tiny medians
must not be interpreted as precise adapter compute time. The source reference
is observed packets, not a guarantee about every sample produced inside the robot.

The user-selected Performance mode is retained. No power-profile change was made
by the agent. Larger DDS buffers are still a temporary setup requiring reapplication
after laptop reboot. Step 4 remains open for cloud acquisition timing; user review
of this delivery result is pending before advancing.

## Accumulation option

The existing SLAM Toolbox launch projects individual vendor deskewed cloud messages;
it has no multi-message cloud accumulation stage. A bounded, motion-compensated
history before 2D projection may improve coverage. RViz decay and scan_time are
not such a stage. Preserve acquisition timing for Point-LIO and review the fork’s
disabled combine implementation before considering it. No accumulation was enabled.
See [accumulation review](accumulation_review.md).

## Evidence

- [Structured result](result.json)
- [Power Saver trace](20261016T202514+1100/analysis.json)
- [Power Saver plot](20261016T202514+1100/delivery_delay.png)
- [Performance trace](20261016T203011+1100/analysis.json)
- [Performance plot](20261016T203011+1100/delivery_delay.png)
- [Closed Performance bag verification](20261016T203011+1100/bag_verification.json)
