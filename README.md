# CoachSim research evidence

This repository contains the system specification, protocol and evidence package for the first four Fall 2026 thesis milestones. The current implementation uses acquisitions from me. 

Start with [D1–D4 delivery index](docs/DELIVERY_INDEX.md).

- [System specification v2](docs/SYSTEM_SPEC.md), [channel map](docs/CHANNEL_MAP.md), [schema v2](docs/SCHEMA_V2.md), [v1 migration](docs/MIGRATION_V1.md).
- [Continuity audit](docs/CONTINUITY_AUDIT.md) and [archived prior evidence](evidence/legacy).
- [Primary literature matrix](docs/LITERATURE_MATRIX.md), [experimental protocol](docs/EXPERIMENT_PROTOCOL.md), [analysis plan](docs/ANALYSIS_PLAN.md), [manuscript outline](docs/MANUSCRIPT_OUTLINE.md).
- [Simulated bench report](docs/D3_BENCH_REPORT.md) and [machine-readable reports](evidence/reports).
- [CoachSim application](https://github.com/Deadialine/coachsim) implements the experiment UI and tests.

## Reproduce evidence

Python 3.10+ standard library; no extra packages are needed.

```sh
python -m unittest discover -s tools -p test_bench_v2.py
python tools/bench_v2.py --out evidence/generated/one-channel --seconds 60 --channels 1
python tools/bench_v2.py --out evidence/generated/four-channel-600s --seconds 600 --channels 4
python tools/bench_v2.py --out evidence/generated/faults --seconds 60 --channels 4 --faults
```

The negative run returns success only when its intentional failures are detected. Output paths must not exist. Generated ZIPs contain raw CSV streams and a session manifest; sidecar reports include SHA-256 hashes. The one-channel variant is an offline bench artifact; the browser supports the frozen four-channel study variant.

Legacy firmware remains as prior work and is not certified by these Python tests. Old `tools/parse_log.py` and `tools/plot_timing.py` placeholders are retained as history; use the new working bench tool. The original system specification is archived before replacement.

[Download the archived synthetic sessions](evidence/archives).
