# CoachSim system specification v2

Version 2.0, 2026-09-21. This specification defines the Fall 2026 sparse forearm sensing-to-feedback system and its implemented simulation profile. The existing concept simulator is retained; this work adds an experiment workflow and reproducible synthetic evidence. Physical hardware qualification remains outstanding.

## Purpose and scope

Recognize seven maintained wrist/forearm states across separate sessions and stream a posture estimate and confidence into CoachSim. The intended research contribution is sparse acquisition, reproducible cross-session evaluation and the integrated feedback path. No clinical efficacy claim is made.

Included: raw EMG design, MPU-6050 IMU, DS1307 session reference, ESP32 migration requirements, synchronized multirate storage, cue/events, replay, confidence gating and LDA/RBF-SVM evaluation protocol. Deferred: strain, FSR, respiration, textile sleeve, haptics/actuation, clinical patients and deep-learning comparisons.

## Architecture

Physical target: MyoWare RAW outputs -> qualified ADC -> buffered ESP32 acquisition; MPU-6050 at 100 Hz -> same monotonic timeline; DS1307 -> session wall-clock metadata. A transport/storage layer preserves sample and packet indices. Host processing produces causal features, calibrated posture predictions and aligned timing events. CoachSim displays traces, target cues, predictions, uncertainty and recording status, and exports a session archive.

Implemented profile: deterministic virtual EMG/IMU sources -> full-rate arrays/CSV -> strict schema validation -> quality monitor and synthetic prediction overlay -> time-indexed replay -> five-file ZIP. The Python bench generator independently writes/re-reads 60 s one-channel, 600 s four-channel and fault-injected files. The overlay uses generated labels; no trained LDA or SVM model is represented as implemented.

## Timing and acquisition

EMG: target four channels at 2000 samples/s/channel (8000 channel values/s). IMU: 100 samples/s. Capture monotonic timestamps at acquisition boundaries, never at UI refresh. Preserve actual channel skew for multiplexed ADCs. Use scheduled absolute sample slots and advance counters for missed slots; do not accumulate read/print duration into the sample period. RTC: obtain session t0_unix and health, not sample timing. All CSV t_us values are relative to session start.

Acquisition must not block on CSV formatting, wireless transmission or chart rendering. Use a bounded producer/consumer buffer, count overruns and mark invalid samples. A physical implementation must establish sustained throughput under the final power, ADC and transport configuration. No firmware is flashed or electrically qualified in this delivery.

The inherited hardware record reports stable shared-bus addressing but a failed 100 Hz rate target (91.665 Hz, -8.33%). Preserve that failure in the audit; synthetic zero-jitter timing does not resolve it. Original Mega bus pull-ups were reported at 5 V; ESP32 migration requires measured voltage compatibility rather than a copied pinout.

## Interfaces and configuration

See [channel map](CHANNEL_MAP.md), [schema v2](SCHEMA_V2.md) and [v1 migration](MIGRATION_V1.md). The machine-readable simulation freeze is `config/study_config_v2.json`. The browser accepts four-channel v2 ZIPs and exact-header v1 CSVs. One-channel qualification archives are intentionally offline-only. The source original v1 system spec remains in `evidence/legacy/`.

## CoachSim behavior

Cue mode stores a deterministic randomized eight-block sequence, seven trials/block, 3 s cue and 3 s rest. It displays countdown, participant/session identifiers and actual synthetic timeline events. Raw EMG and IMU traces use independent rates; plotting does not downsample stored evidence. Replay follows a shared cursor with target and prediction labels. Export contains session.json, emg.csv, imu.csv, events.csv and predictions.csv.

Prediction confidence below 0.70, quality failure, missing/stale input or age above 250 ms produces uncertain. A same-label accepted prediction must remain consistent for 250 ms before positive feedback. Raw ADC/sensor flags can veto even a confident imported prediction. No real device or model connection is claimed. The simulator can inject a clipping fault to exercise this path.

## Acceptance and evidence policy

Targets: >=90% valid trials; <1% EMG clipping per channel; <0.5% missing samples and separately measured dropped packets; <=1% per-stream rate error; median measured sample-to-display latency <=300 ms. Model macro-F1 >=0.80 within session is a planning target, not an exclusion criterion.

Each result must carry provenance (synthetic, recorded, or legacy_unverified), commit/config IDs, raw artifact hashes and a reproducible calculation. Historical claims without original logs remain reported, not independently verified. Never reclassify synthetic evidence as physical acceptance.

## Study and analysis contracts

[Protocol](EXPERIMENT_PROTOCOL.md): four planned participants, two separate-day sessions, repeatable measured placement, central two-second held-state interval, explicit QC/exclusions. [Analysis plan](ANALYSIS_PLAN.md): 200 ms windows, 50 ms hop, training-only preprocessing, whole-trial grouped validation, session holdout, LDA primary and RBF-SVM comparison, EMG/IMU/fusion ablation. Future human collection requires the applicable institutional determination; the attached plan's lab-only wording is not proof of one.

## Current verification

The simulator core tests cover deterministic cue order, multirate chunk invariance, archive roundtrip, malformed inputs, confidence/quality/staleness/stability gates and v1 migration. The Python QC tests cover nominal/fault files, duplicates/trailing loss and refusal to overwrite evidence. See the D3 report and the app browser validation record for executed results. Original physical D1/D3 criteria remain itemized in the delivery index.
