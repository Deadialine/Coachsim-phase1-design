# CoachSim multirate session schema v2

Version 2.0, simulation freeze 2026-09-21. A bundle is a ZIP containing exactly the five required root files below, UTF-8 CSV/JSON. Additional offline manifests and QC reports live beside the archive. CSV files always include a header, even if empty. No implicit resampling or raw/derived substitution is permitted.

## Session metadata

`session.json`: required schema_version=2, participant_id, session_id, t0_unix (Unix seconds), time_origin='session_start', provenance ('synthetic', 'recorded', or 'legacy_unverified'), device_version, firmware_version, config, channel_map, placement_metadata, duration_us. The configuration records ID, channels, EMG/IMU rates, ADC range, window/hop and gate thresholds. Channel map entries give ID, unit, placement and calibrated status. Real recordings must additionally record ADC model, gain/reference/offset calibration, sample alignment/skew, firmware commit, clock alignment method, protocol version and placement measurements.

Synthetic CLI sessions use t0_unix=0 deliberately; this is not a DS1307 read or a claimed historical measurement date. Browser synthetic sessions use host start time only. The browser currently supports the frozen four-channel configuration. The one-channel bench variant is valid for offline QC and explicitly rejected by the four-channel UI.

`duration_us` is the exclusive acquisition boundary: samples must satisfy 0 <= t_us < duration_us; a terminal event may equal duration_us. Indices count intended sampling slots from zero, including slots lost before export. Missing slots never cause renumbering. Use 64-bit safe integer microseconds and indices (maximum JavaScript safe integer for browser imports). Device boot time must be rebased once to session start. RTC jumps never change monotonic sample timestamps.

## Stream files

| File | Ordered header | Units and rules |
| --- | --- | --- |
| emg.csv | sample_index,t_us,emg_ch1,emg_ch2,emg_ch3,emg_ch4,adc_flags | 2 kHz rows; four simultaneous or documented-skew channel values in ADC counts 0–4095. Flags bit 0 = clipping, bit 1 = acquisition invalid. Retain row but exclude invalid samples from inference. One-channel bench header contains only emg_ch1. |
| imu.csv | sample_index,t_us,ax,ay,az,gx,gy,gz,sensor_flags | 100 Hz; acceleration in g, angular velocity in degrees/s; nonzero flag means sample cannot be assumed valid. Preserve actual acquisition time. |
| events.csv | t_us,event_type,target_posture,repetition,block | cue, rest, complete, stop, fault_on, fault_off; fixed seven posture codes; repetition/block 1–8 for task cues and 0 for bench/operator markers. Equal-time events retain file order. |
| predictions.csv | t_us,predicted_posture,confidence,model_version,latency_ms,quality_ok | t_us is prediction availability on session clock; class code or uncertain; confidence [0,1], nonnegative latency, quality_ok 0/1. Simulated latency is explicitly injected. Real latency semantics and clock alignment require additional metadata. |

EMG, IMU and prediction times are strictly increasing within each stream; events are nondecreasing. Sequence numbers are strictly increasing and gaps are preserved. Numeric fields cannot be blank, NaN or infinite. Never encode a missing sensor as an apparently valid zero reading. Unsupported configurations and malformed headers fail import with a visible message.

## Transport and feedback contract

Future live transport carries stream_id, packet_sequence, first_sample_index, sample_count, first_t_us, sample interval/skew, flags and payload integrity check. Keep separate counters for sent/received/duplicate/reordered packets and missing sample slots. No transport is connected in the present simulation workflow. A 2 kHz, four-channel stream requires 8,000 channel values/s; typical CSV cannot fit a 115,200-baud link. Buffer acquisition into bounded blocks and use a measured higher-throughput binary link or on-device storage. UI chart refresh must not determine acquisition cadence.

Live coaching must fail to uncertain on invalid data, confidence <0.70 or prediction age >250 ms. Require 250 ms consecutive accepted same-label predictions before a positive coaching message. Do not treat the target cue as inferred ground truth. The simulator's overlay intentionally follows generated labels to test interface plumbing and is named `synthetic-overlay-1`, never LDA.

## Replay and migration

Replay uses time-indexed lookups and shows data up to the cursor. Charts show short raw windows while the archive retains full-resolution samples. A browser ZIP is limited to 80 MB compressed and 240 MB uncompressed. Large research datasets should use offline tooling. See [V1 migration](MIGRATION_V1.md).
