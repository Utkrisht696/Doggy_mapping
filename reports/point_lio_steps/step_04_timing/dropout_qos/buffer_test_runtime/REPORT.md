# Approved temporary laptop receive-buffer fix — 16 October 2026

Status: **Applied and verified for UDP receive-buffer overflow. Remaining calibrated
IMU delivery gaps and exact cloud acquisition calibration are NOT fixed.**

The user explicitly approved the laptop-only temporary change. `sudo -n` required a
password, so the user ran the exact sysctl command in their own terminal. The agent
verified net.core.rmem_max=8,388,608; rmem_default remained 212,992. No persistent
sysctl file was written. All active code/config files and source URDF remain unchanged.

The agent stopped its original headless sensor launch and body-IMU adapter, then
restarted both with the exact approved test-only CycloneDDS profile using a process
environment override AFTER normal setup. The profile requests min/max 8 MiB receive
buffers. ss verified rb=16,777,216 on the adapter's UDP sockets; Linux reports a
doubled accounting size. This is not a second, unapproved buffer-setting change.

## Same passive comparison repeated

Five independent observer processes, the same QoS variants, the same 68-second run
lengths and one-minute shared source-time analysis windows, with/without the same
ten-topic recording workload. No new movement, mapper or replay. The observers and
load recorder are stopped; the two passive sensor/IMU launches remain active.

| Measurement | Baseline before -> after | Recording load before -> after |
|---|---:|---:|
| Host UDP receive-buffer overflows | 1182 -> 0 | 6708 -> 0 |
| Existing adapter UDP socket dropped datagrams | 107 -> 0 | 456 -> 0 |
| Shallow BE5 source observer missing stamps | 30 -> 11 | 440 -> 103 |
| Actual calibrated-output capture missing source-union stamps | 74 -> 79 | 713 -> 302 |

With larger socket buffers, BE2000 and BOTH reliable2000 source observers each
received every timestamp in the observed source union: 19,441 baseline and 19,520
under recording load. This is not proof of every sample generated inside the robot.
The calibrated-output row includes both adapter and observer delivery; it cannot
attribute each missing stamp to one stage. The increase 74 -> 79 at baseline also
shows that fixing kernel buffers does not alone fix shallow application queues.

Reliable source queueing still has delay tails under load: relative callback-delay
p99 78.83 ms, maximum 193.67 ms for reliable2000. BE2000 p99 was 100.32 ms, maximum
239.14 ms. These are relative callback times, not physical sensor acquisition lag.
The live calibrated-output capture had p99 27.49 ms, maximum 119.05 ms, but missed
302 observed source stamps (about 1.55% of 19,520). Completeness and latency must be
considered together. Never retimestamp delayed samples to hide their age.

All matched adapter/source six-channel bias corrections remain exact. All observed
selected stamps progress without backwards steps. The unchanged developer model,
43 runtime links, 12 measured joints and source-pose decomposition passed after
restart. Each raw/body/deskewed cloud resolved at five original source timestamps.
Publisher endpoints confirm one body-IMU output, one private clock, one static-TF
owner and the expected two non-competing dynamic-TF owners. No extra TF publisher.
The load bag closed normally and passed SQLite integrity/metadata count checks.

## Next proposed fix (not applied)

`next_adapter_qos_review/adapter_qos.patch` in the after-test directory changes ONLY
the adapter source subscription from BEST_EFFORT depth 5 to RELIABLE depth 2000,
matching a tested observer profile. The candidate compiles. Calibration math,
unknown covariance, source stamp/duplicate/backward/invalid-data guards, output
publisher QoS, TF and Point-LIO settings are unchanged. Restart only the body-IMU
adapter and repeat the same whole-output comparison after approval. This is a
hypothesis to test for remaining delivery misses, not a claim that every missing
output is caused solely by the source queue.

User confirmation is required before changing the active Python file. Do not
advance to temporal calibration until the dropout substep is reviewed.

## Temporary runtime and restart limits

Current headless launch session 79938; body-IMU adapter session 27719. Both use
`buffer_test_runtime/cyclonedds.buffer_test.xml`. The kernel cap resets at laptop
reboot; the ordinary setup/launch still uses the unchanged active DDS config.
The test wrappers check the kernel prerequisite and must not be used after the
cap is reverted. A persistent solution has NOT been installed. The user may later
approve one after the full delivery path passes. No persistence is silently added.

Rollback: stop the two test-profile processes, restore the old kernel cap with
`sudo sysctl -w net.core.rmem_max=212992`, then start the original sensor launcher
and adapter with the unchanged active DDS config. Restoring the cap requires the
user's sudo password unless their sudo session is already authorized. Do not
restore the cap while retaining the strict 8 MiB test profile for new processes.

Evidence: `before_after_result.json`, `socket_buffer_verification.json`,
`tf_after_restart.json`, `ownership_after_restart.json`, and both timestamped
comparison directories. Administrative changes were laptop-only.
