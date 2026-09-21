# Manuscript outline and methods draft

Working title: A sparse sEMG and IMU node for cross-session forearm posture recognition with CoachSim feedback

## Abstract structure

Motivation: low-cost wearable interfaces require reproducible sensing and robustness to session changes. Objective: test seven maintained wrist/forearm states with sparse EMG and optional IMU fusion. Methods: state actual participants/sessions, channels, acquisition rates, grouped LDA/SVM validation and interface timing. Results: insert only measured participant and hardware results when available. Conclusion: describe demonstrated capability and limitations without a rehabilitation-efficacy claim.

No participant accuracy, clinical benefit or physical acquisition success has been established in D1–D4. Current software evidence may support an implementation subsection only, explicitly labeled synthetic.

## Introduction

Paragraph 1: scope of wearable myoelectric interfaces and the need for accessible implementations. Paragraph 2: session/electrode variability (Kim; Botros; Eby). Paragraph 3: fusion tradeoffs (Colli Alfaro and Trejos) and the distinction from high-density acquisition (Tacca). Paragraph 4: contributions and fixed RQs: sparse architecture, separate-session validation and a reproducible CoachSim integration.

## Methods draft

CoachSim separates raw EMG, IMU measurements, cue events and predictions into versioned files on a shared monotonic session clock. The proposed study configuration contains four bipolar forearm EMG channels at 2 kHz per channel and an MPU-6050 at 100 Hz. A DS1307 provides a session wall-clock reference only; it is not the sampling clock. The physical ADC and analogue front end must be qualified before final hardware details are reported.

The protocol comprises seven maintained states: neutral/rest, wrist flexion, wrist extension, radial deviation, ulnar deviation, forearm pronation and forearm supination. Each session contains eight randomized blocks, each including all seven states. A state is cued for three seconds followed by three seconds of rest. A stored seed determines the cue order. Analysis uses the central two seconds of each cue to reduce transition-label ambiguity. Cue labels describe intended performance and are not direct angle measurements.

EMG features are extracted from causal 200 ms windows with 50 ms hop. The feature set consists of MAV, RMS, waveform length, zero crossings and slope-sign changes per channel. LDA is the primary model and RBF-SVM is the secondary model. Hyperparameters and preprocessing are fitted on training data only. The primary evaluation trains on the first session and tests on the second session within each participant. Whole trials remain together in all nested within-session folds. EMG, IMU and fused modalities use the same splits.

Report actual hardware, participant authorization, achieved dataset, deviations and software versions in the final Methods; this proposed design is not a statement that collection has occurred.

## Results to populate after D6 and D7

1. Dataset accounting table: planned/recorded/valid trials and reasoned exclusions by class and session.
2. Acquisition table: physical rates, gaps, per-channel clipping/noise, clock alignment uncertainty and timing percentiles.
3. Model table: participant-specific within- and cross-session macro-F1 for six model/modality combinations.
4. Confusion figure: cross-session LDA EMG and fusion, with raw counts and normalized rows.
5. System figure: synchronized traces, target/predicted posture, uncertainty intervals and measured latency.
6. Ablation plot: paired participant-level fusion minus EMG differences.

## Discussion

Discuss sparse channel coverage, pronation/supination separability, placement/reapplication drift, cue-to-execution mismatch, dataset size, IMU orientation, calibration dependence, and uncertainty coverage. Separate missing evidence from observed failure. Explain why synthetic timing does not characterize hardware or browser-to-device latency. Avoid population generalization from four participants.

## Submission and reproducibility

Select a venue after recorded results establish scope. Export an IEEE-format manuscript only once venue requirements and actual results are available. Include code/data availability, ethics determination, author contributions, funding and conflicts with verified details. Do not invent these statements. Link the versioned configuration, split/exclusion ledgers, raw archives where authorized, and release hashes. D9 manuscript submission remains a later milestone.
