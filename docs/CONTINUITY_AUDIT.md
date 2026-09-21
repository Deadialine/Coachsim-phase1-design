# Continuity audit

Audit date 2026-09-21. Sources: the user-provided Coachsim plan.docx, deadline image, the two GitHub repositories, and the adjacent Arduino_imu_icu reference folder. Source files were read or copied, never modified. Embedded document statements are evidence of prior plans/reports, not authorization to recruit people, submit a paper or claim experimental success.

## Baselines

| Repository | Audited main commit | Actual contents |
| --- | --- | --- |
| Deadialine/coachsim | 4d22924e8815057d3edc5cc6da6d7191c47d51c8 | Vite/React concept simulator, random sensor generation, CSV demo log, sensor-layout and 3D views, tracked build artifacts. README incorrectly described Create React App. |
| Deadialine/Coachsim-phase1-design | 4d67b82d2ef86ac4d0dfb34a78827a6f3d0cb4c5 | Arduino/ESP32 acquisition drafts, schema v1 headers, reported bus/timing results, sensor decision document, scheduler policy, schema workbook. Python log/plot tools were placeholders. ESP32 acq_task.cpp includes a dummy IMU read. |

The full tracked-file inventory and hashes are stored at `evidence/repository_inventory.json`. Original file identity is retained by baseline commit and archive hashes. New experiment code belongs in coachsim; system/protocol/evidence documents belong in the phase-one design repository.

## Prior evidence and gaps

| Claim | Evidence available | Audit conclusion |
| --- | --- | --- |
| Shared Arduino I2C bus works | Existing SYSTEM_SPEC reports 300 scans over 5 min with DS1307 0x68 and MPU-6050 0x69 present | Prior reported success, not independently reproduced here. Raw scan log and wiring photograph were not located. |
| Monotonic ESP32 timebase | Existing spec reports 5,929 frames, zero non-increasing timestamps, intervals 10,104–11,228 microseconds | Preserve as reported evidence. Current source has a stub sensor path, so do not infer integrated acquisition. |
| 100 Hz scheduler | Existing spec reports 27,500 rows, 2,499 dropped frames, 91.665 Hz and -8.33% error | Prior failure remains open for physical implementation. Low jitter alone does not satisfy rate acceptance. |
| Wiring voltage | Existing spec reports pull-ups to 5 V on Mega bus | ESP32 migration needs verified voltage compatibility; do not reuse the Mega wiring as a certified ESP32 pinout. |
| Actual raw EMG | No verified raw acquisition or calibration in sources | New synthetic recordings exercise software only. |
| Physical full sleeve | Plan explicitly states none exists | Deferred modalities remain out of scope. |
| Hosted CoachSim | Plan reports a hosted site; Vite config references coachsim.loca.lt | No verified persistent hosted deployment or historical screen recording was supplied. Local demo and build are reproducible; deployment evidence is separately identified. |

## Archived material

`evidence/legacy/` contains a byte-for-byte copy of the prior SYSTEM_SPEC plus supplied Arduino wiring text and IMU/packet-loss/I2C scanner sketches. The source `loss measurements.txt` is zero bytes, so it is not a bench log. These archives preserve source/report evidence and do not manufacture the missing serial captures.

## D1–D4 implementation decisions

Adopt multirate v2; preserve v1 IMU replay; freeze only the simulation configuration; retain seven classes and LDA/RBF-SVM protocol. Introduce deterministic synthetic bench evidence, strict import validation, cue/events, raw traces, replay, quality-gated overlay and ZIP export. The synthetic overlay is not a trained classifier. Real device transport, physical qualification and real-time latency measurements remain explicitly unverified.
