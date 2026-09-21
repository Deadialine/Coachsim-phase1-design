# Simulated acquisition qualification report

Date: 2026-09-21. D3 was executed using simulated runs at the user's request. These results qualify the synthetic files and QC checks, not an sEMG–IMU device. There are no human recordings, measured electrodes, physical packets or actual sensor-to-screen latency measurements.

## Runs and results

| Run | Duration | EMG channels / rows | IMU rows | Result |
| --- | --- | --- | --- | --- |
| one-channel | 60 s virtual time | 1 / 120,000 | 6,000 | Synthetic one-channel software proof passed |
| four-channel-600s | 600 s virtual time | 4 / 1,200,000 | 60,000 | Frozen simulated configuration passed |
| faults | 60 s virtual time | 4 / 118,800 | 5,940 | Expected failure detected |

Each run uses its first half as a still-state signal and its second half as a motion/activity signal. The four-channel run therefore contains 300 s still and 300 s motion. Generation is accelerated file creation; it is not a claim that a physical device operated for ten wall-clock minutes. A deterministic multisine with different channel frequencies is stored as raw 12-bit ADC counts centered near 2048. The signal is a test waveform, not a physiological EMG model.

The nominal four-channel run has achieved/clock EMG rate 2000 Hz and IMU rate 100 Hz; EMG intervals are exactly 500 microseconds and IMU intervals exactly 10,000 microseconds. There are zero missing slots, nonmonotonic timestamps, invalid values and clipped values. Ideal zero jitter is expected from the generator and is not a hardware result.

| EMG channel | Still AC RMS counts | Motion AC RMS counts | Motion/still amplitude ratio dB |
| --- | --- | --- | --- |
| 1 | 9.717 | 146.248 | 23.551 |
| 2 | 10.560 | 158.394 | 23.521 |
| 3 | 11.374 | 170.686 | 23.526 |
| 4 | 12.202 | 183.079 | 23.524 |

These ratios describe the designed amplitude change. They are not physiological signal-to-noise ratios. The simulated files cannot demonstrate line-noise rejection, electrode contact, cross-talk, anti-aliasing, analog calibration, RTC accuracy or muscle selectivity.

## Fault sensitivity

The fault run removes every hundredth EMG and IMU sample (1% loss), drives EMG channel 1 to the upper ADC rail on two slots per hundred, and slows EMG timestamps by 2%. Checks detect missing samples, >1% channel clipping, clock-rate error and timestamps beyond the declared acquisition interval. The failure archive is intentionally invalid and must be rejected by strict replay; it is a negative test, not an accepted session.

Physical packet loss is **not measured**. A separate reproducible packet-accounting simulation (`python tools/transport_sim.py`) models 60,000 packets per stream over 600 s: nominal loss is 0%; dropping every 100th packet produces 600 missing packets/stream (1%) and fails the threshold. It records duplicates/reordering and corresponding missing sample slots in `evidence/reports/transport-simulation.json`. This separate virtual transport test is not a measurement of the CSV generator or a device link. A missing sample count does not uniquely identify missing packets. Predictions contain a fixed synthetic 25 ms latency to exercise the field; median end-to-end latency has **not** been verified against the 300 ms target.

## Reproduction and evidence

From repository root, run:

```sh
python -m unittest discover -s tools -p test_bench_v2.py
python tools/bench_v2.py --out evidence/generated/one-channel --seconds 60 --channels 1
python tools/bench_v2.py --out evidence/generated/four-channel-600s --seconds 600 --channels 4
python tools/bench_v2.py --out evidence/generated/faults --seconds 60 --channels 4 --faults
```

Use fresh output paths; the generator refuses to overwrite evidence. Machine-readable reports and SHA-256 file/archive hashes are in [evidence/reports](../evidence/reports). The generator reads the written CSVs independently before issuing a report. Full compressed archives are supplied in the evidence package. No physical firmware was built, flashed or certified in this milestone.

## Physical acceptance still required by the original deadline

Run one actual channel with known input and electrode acquisition; freeze the real ADC/channel configuration only after success. Repeat the 600 s still/motion acquisition on that configuration, retain raw serial/transport logs and wiring evidence, and measure every acceptance metric. Correct the historical 91.665 Hz rate failure. Then update this report with measured results without replacing the synthetic evidence or earlier failure record.

Archived generated ZIPs: [raw synthetic sessions](../evidence/archives). These are synthetic, with content hashes in the reports.
