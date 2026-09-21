# D1 through D4 delivery index

Prepared 2026-09-21 against the attached Fall 2026 plan and deadline table. The user's instruction to run simulated experiments supersedes physical execution for this delivery. The table distinguishes the delivered simulation work from the original physical acceptance evidence that remains unavailable.

| ID | Deadline | Delivered | Original evidence not established |
| --- | --- | --- | --- |
| D1 | Aug 31 | Repository inventory; continuity audit; source archives; system spec v2; channel map; v2 schema; v1 migration/replay; reproducible application build and local demo | Historical hosted demo and independently archived five-minute physical serial log/wiring photograph |
| D2 | Sep 7 | Primary-study evidence matrix; fixed RQs and seven classes; placement/acquisition protocol; prespecified QC/analysis; manuscript outline and methods draft | No participant recruitment or collection; final physical setup awaits qualification |
| D3 | Sep 21 | One-channel 60 s synthetic proof, four-channel 600 s still/motion run, frozen simulation config, machine-readable QC, raw ZIPs/hashes, negative fault test | Verified physical node, physical configuration freeze, true packet loss, analogue quality and measured end-to-end latency |
| D4 | Sep 28 | Seeded cues/countdown; events; live simulated raw EMG/IMU traces; confidence/stability/uncertain overlay; synchronized replay; five-file ZIP export/import; v1 replay; automated tests | Real device transport and trained-model connection are future integration work; current overlay is synthetic |

## Review order

1. Read [continuity audit](CONTINUITY_AUDIT.md) and [system specification](SYSTEM_SPEC.md).
2. Review [literature](LITERATURE_MATRIX.md), [protocol](EXPERIMENT_PROTOCOL.md), [analysis](ANALYSIS_PLAN.md) and [manuscript structure](MANUSCRIPT_OUTLINE.md).
3. Inspect [bench report](D3_BENCH_REPORT.md) and the [JSON evidence](../evidence/reports).
4. Run CoachSim, record a simulation, inject a clipping fault, stop, export, import and replay. See the application repository README and browser validation evidence.

## Handoff to physical qualification

Confirm ADC/ESP32 variant and actual MyoWare module count; qualify the one-channel analogue chain and isolated power arrangement; fix the historical sampling-rate failure; freeze the actual four-channel setup only after measurements. Repeat the archived QC protocol on physical streams, add raw transport/RTC evidence, and measure clock-aligned sample-to-display latency. Obtain the applicable institutional determination before future human research. D5–D10, participant accuracy, manuscript submission and clinical validation are not claimed complete.
