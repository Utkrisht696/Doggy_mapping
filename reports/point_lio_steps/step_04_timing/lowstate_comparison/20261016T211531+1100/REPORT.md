# LowState body-IMU source comparison

A 15-second passive live capture compared raw DDS messages. No application,
calibration, TF, source selection or robot-control changes were made.

| Topic | Observed message rate | Distinct consecutive gyro/acc updates |
| --- | ---: | ---: |
| `/lowstate` | 499.99 Hz | 499.24 Hz |
| `/sportmodestate` | 295.21 Hz | 286.87 Hz |
| `/lf/lowstate` | 20.00 Hz | 20.00 Hz |
| `/hf/lowstate` | No publisher/messages | — |

Rates use the common interior interval after startup discovery. Each raw parser
was crosschecked against standard deserialization on five actual messages.
All 4,428 captured SportModeState gyro/accelerometer six-value tuples matched
bit-identical float32 values in LowState, choosing a nearby DDS source time
(within 100 ms) for repeated tuples. This strongly supports the same raw body
IMU stream with additional samples through LowState, rather than a different
sensor or only repeated messages.

LowState has a tick counter, no TimeSpec/header. Its nonzero DDS source timestamp
is a publication timestamp; it has not been proved to represent acquisition.
A centred linear fit of matched SportModeState epoch stamps to LowState tick
increments gave 0.999995 ms/tick over this short capture. Using a fixed 1 ms/tick
scale, fitting only the first half's median epoch offset, second-half residuals
had median -0.007 ms, p99 2.948 ms, maximum 7.174 ms. These are agreement with
SportModeState stamps, not measured LiDAR acquisition errors. Tick units,
long-term clock stability, wrap/reboot handling and moving-data timing still
need validation before implementing a timestamp adapter.

Matching raw channels do not make the orientation estimates interchangeable:
matched fused quaternions differ by approximately 0.870086 degrees. Do not copy
orientation conventions or its covariance without review. Raw units/axes match
for these samples; existing fixed raw-channel bias corrections are therefore
promising, but a replacement output still requires validation.

Recommendation: LowState is the stronger candidate for Point-LIO input because
it supplies roughly 500 fresh body-IMU samples/s. Keep the working SportModeState
adapter until a reviewed timestamp conversion and passive equivalence/coverage
check pass. The higher rate alone does not establish sensor synchronization or
repair the inconclusive independent cloud timing test. Nothing was switched.

[Capture metrics](comparison.json) · [Exact-sample and clock comparison](sample_correspondence.json)
