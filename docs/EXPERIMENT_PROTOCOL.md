# CoachSim maintained posture protocol

Protocol CS-P02, frozen for simulation on 2026-09-21. This protocol defines the planned study; the accompanying D1–D4 evidence contains no human participants and no physical acquisition measurements. Hardware qualification must precede a physical configuration freeze.

## Research questions and endpoints

RQ1: How well does a sparse forearm sEMG node distinguish seven maintained wrist and forearm states within a session and across separate sessions? Primary endpoint: participant-specific cross-session macro-F1 across the seven classes, with each class included even when never predicted. Report within-session macro-F1 as a secondary reference.

RQ2: Does adding an IMU improve cross-session macro-F1 or reduce specific confusions? Compare EMG alone, IMU alone, and feature-level EMG plus IMU fusion using identical trial splits. The primary contrast is fusion minus EMG for LDA; RBF-SVM is the prespecified comparison model.

RQ3: Can the sensing-to-display path meet acquisition and feedback targets? Report per-stream rate error, sample loss, separately measured packet loss, clipping per channel, median/p95/p99 latency, uncertainty coverage, and prediction switching per minute. Targets are >=90% valid trials, <1% clipped samples per EMG channel, <0.5% missing samples and packets, <=1% per-stream rate error, and median end-to-end update latency <=300 ms. These are engineering targets, not findings. Within-session macro-F1 >=0.80 is an aspirational planning target, not an inclusion criterion.

## Fixed classes

| Code | Maintained state | Operational instruction |
| --- | --- | --- |
| neutral_rest | Neutral wrist and relaxed forearm | Relax fingers, wrist aligned with forearm |
| wrist_flexion | Palmar flexion | Bend palm toward the inner forearm without moving elbow |
| wrist_extension | Dorsal extension | Bend back of hand toward outer forearm |
| radial_deviation | Thumb-side deviation | Move wrist toward thumb side without forearm rotation |
| ulnar_deviation | Little-finger-side deviation | Move wrist toward little finger without forearm rotation |
| forearm_pronation | Forearm rotation toward palm down | Rotate forearm, maintain a neutral wrist |
| forearm_supination | Forearm rotation toward palm up | Rotate forearm, maintain a neutral wrist |

These are cue labels, not independently measured joint-angle ground truth. The IMU is a predictor and cannot independently certify its own labels. Record protocol deviations when the intended posture was not maintained. Do not claim clinical efficacy or diagnostic performance.

## Planned participants and sessions

Target four complete healthy-adult participants with two sessions on separate days; a third session is exploratory and separately identified. Record actual elapsed days and time of day. Remove and replace sensors between sessions. Never reuse an identical placement record without measuring it again. A 2–3 participant pilot is a later readiness activity, not part of these simulated results.

The attached plan says “No IRB, only within our lab.” That phrase is not an institutional determination. Before any human research collection, obtain the applicable Purdue HRPP determination and advisor authorization, and retain its identifier with the protocol. Purdue states that research using human participant data must be reviewed by its HRPP/IRB: [Getting started](https://research.purdue.edu/resources-for-researchers/compliance-and-security/hrpp/hrpp-getting-started/). No recruitment, consent, or participant collection is performed by this package.

## Repeatable placement

Use the dominant forearm supported on a table, elbow approximately 90 degrees, shoulder relaxed, neutral wrist during setup. Use four evenly spaced bipolar channels around a transverse ring at one third of the measured distance from the elbow crease to the wrist crease, measured from the elbow. Define channel 1 at the volar midline; channels 2–4 follow around the circumference toward the radial side in quarter-circumference increments. Record arm side and orientation explicitly so left and right arms are not silently mirrored. This is a reproducible geometric proposal, not a claim that each channel isolates a particular muscle.

Orient each bipolar pair along the forearm long axis. Record actual electrode center-to-center spacing imposed by the MyoWare module, module orientation, electrode type/lot, ring distance, forearm length and circumference, distances from landmarks, attachment method, skin preparation, and cable routing. Do not invent 20 mm spacing if the board geometry differs. Use a non-identifying placement diagram and measurements, not identifiable photographs. Attach the IMU rigidly on the dorsal hand/wrist with X toward fingers, Y toward thumb, and Z outward; document deviations and axis transforms. Record reference electrode locations according to the actual module design and manufacturer guide.

## Acquisition and trial procedure

1. Verify battery isolation, signal range, ADC settings, shared clock behavior, address selection, and channel continuity. Record firmware commit, device IDs and calibration status. Preserve ADC counts until a calibrated conversion is available.
2. Acquire 30 s of relaxed baseline and a gentle familiarization sequence. Inspect each channel for rails, a flat line, large low-frequency artifacts, and line interference. Do not normalize against test-session data in the primary analysis.
3. Explain and practice all seven states at comfortable effort without forcing end range. Stop for discomfort. Record an operator deviation if a posture cannot be performed as defined.
4. Record eight blocks. Each block contains one randomized trial for each of the seven classes, using Fisher–Yates shuffle with the stored seed. Each trial is 3 s cue followed by 3 s neutral rest: 56 trials, 336 s nominal task duration. The neutral cue is a scored trial; intertrial rest is not automatically added as extra neutral training data.
5. Log every cue and rest boundary with session-relative microseconds, repetition and block. Additional rest or interruptions in a future physical run require explicit events and a protocol version supporting them. The current simulator implements the fixed 336 s sequence and a stop event; it does not collect human sessions.
6. Export the five-file session ZIP and verify its contents before starting another session. Preserve original raw files and a SHA-256 manifest. Assign pseudonymous participant/session IDs; keep any identity key outside the public repositories.

## Trial quality and exclusions

For held-state analysis use only 0.5–2.5 s after cue onset, removing onset and offset transitions without shifting labels to improve results. With 200 ms windows and 50 ms hop this produces 37 windows per trial. Reject windows that cross a cue boundary or contain a timestamp discontinuity, missing sample, sensor error or invalid ADC code. Reject a trial if fewer than 90% of its expected windows survive, any channel has >=1% clipped samples in its 2 s analysis interval, or the operator records incorrect execution. Record exclusion reason before examining model performance.

Preserve all rejected rows and trials in the archive; store exclusions in a separate ledger. Report valid/planned trials per class, session and participant. Do not replace low-performing participants or omit a difficult posture to meet a performance target. A session with missing classes is reported as incomplete, with missingness shown explicitly.

## Changes and freeze

Any change to class definitions, channel count, placement, preprocessing, split policy, exclusion thresholds, or primary metric increments the protocol/configuration version. Record date, rationale, author, and whether outcomes had been inspected. Four channels are frozen only in the simulated configuration. The number of functioning physical MyoWare channels and the ADC are not established by the attached plan.
