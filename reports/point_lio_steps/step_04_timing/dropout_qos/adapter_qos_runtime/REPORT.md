# Approved adapter source QoS: applied and verified

The `/sportmodestate` subscription in `scripts/publish_body_imu.py` now uses
RELIABLE delivery and depth 2000 instead of sensor-data BEST_EFFORT/depth 5.
This exactly matches the candidate approved by the user. Conversion, calibration,
source timestamps, validation guards and TF were preserved. The normal adapter
was restored after temporary profiling; its final session is 55738, PID 1155223.

## Comparison result

| Check | Baseline | With ten-topic recording |
|---|---:|---:|
| Common source interval | 65.434 s | 65.342 s |
| Unique source timestamps observed | 19,427 | 19,494 |
| Calibrated output timestamps missing | 0 | 0 |
| Fixed-bias comparison maximum error | 0 | 0 |
| Host UDP receive-buffer overflows | 0 | 0 |
| Adapter socket drops | 0 | 0 |
| Median output delay relative to source observers | 1.69 ms | 2.82 ms |
| 99th percentile relative delay | 61.60 ms | 554.20 ms |
| Maximum relative delay | 172.25 ms | 606.71 ms |

Counts refer to the union of independently observed source samples in a shared
interior interval. They cannot establish losslessness inside the robot. The
previous large-buffer/old-QoS trial missed 79 and 302 output stamps respectively;
the two trials have different source samples and are not a same-packet replay.

## Remaining timing issue

Sample completeness passed these trials. The recording-load delivery delay is
still unresolved, so step 4 is not complete. A separate unprofiled metadata trace
matched all 19,516 observed source timestamps. Its maximum total relative delay
was 628.59 ms; much of the delay preceded adapter DDS write-time metadata
(pre-publication p99 458.07 ms, max 554.46 ms). This localizes delay more narrowly
than an output-only observation, but does not isolate input service, conversion,
publication blocking or host scheduling. There is no basis to apply a constant
timestamp offset for these variable delays.

A further cProfile trace materially slowed the adapter (roughly 280–285 samples/s
while the source supplied roughly 297/s), producing multi-second backlog. That
trace is saved only as instrumented diagnostic evidence; its latency does not
represent ordinary operation. Aggregate profiling does not establish a unique
root cause or justify another application patch. Profiling is stopped.

The final eight-second passive check received 2,373 samples, all in `imu` with
strictly increasing source stamps; it found exactly one adapter publisher, the
approved RELIABLE/depth-2000 source subscription and four correctly sized DDS
sockets. The ordinary adapter and headless sensor launch remain running. Recording,
profiling, mapping and replay are stopped. No robot commands were sent.

The 8 MiB kernel limit and DDS profile are temporary; they are not installed as a
persistent laptop configuration. Physical LiDAR-cloud/body-IMU acquisition lag
also remains uncalibrated. User supplied three independent seated topic echoes; see `user_seated_doublecheck.json`. Their acceleration norms are 9.81–9.86 m/s² and reported pitch is approximately −5.9°. This supports basic output sanity but does not validate delivery latency or precise calibration.

Evidence: `result.json`, `final_runtime_verification.json`, `implementation.json`,
`profile_summary.txt`; comparison `../20261016T195913+1100/comparison.json`;
unprofiled latency `../latency_metadata_20261016T200446+1100/latency_analysis.json`;
profiled trial `../latency_metadata_20261016T200921+1100/latency_analysis.json`.
