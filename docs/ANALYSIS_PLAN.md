# CoachSim prespecified analysis plan

Analysis CS-A02, 2026-09-21. This specifies D2 methods for future recorded data. D3 simulation exercises storage and quality control only; it cannot answer RQ1 or RQ2. D6/D7 dataset collection and trained-model evaluation are outside the first four milestones.

## Inputs and processing

Read raw multirate streams on the same monotonic session clock. Preserve raw ADC counts. Convert to voltage only with recorded reference, gain, offset and ADC calibration; otherwise report count-based features. A 100 Hz legacy EMG placeholder is never a 2 kHz raw recording.

Use causal fourth-order Butterworth EMG bandpass at 20–450 Hz, sampled at 2 kHz. Record coefficients and software versions with the trained model. Continuous causal filtering precedes segmentation; restart state after gaps and exclude the first 500 ms after each restart. Use a 60 Hz notch only if a prespecified baseline check demonstrates line contamination, and freeze that decision on training-session data. No zero-phase offline filter may be substituted silently into a real-time comparison. No high-pass operation is applied to IMU acceleration because its gravity component may be informative.

On each 200 ms window (400 EMG samples/channel and nominally 20 IMU samples), use a 50 ms hop. Do not fill EMG gaps with interpolation. Align modalities by interval boundaries, not by duplicating IMU rows into EMG. For each EMG channel extract MAV, RMS, waveform length, zero crossings and slope-sign changes. Use per-channel ZC/SSC difference threshold equal to 3 times training-session resting baseline standard deviation, stored with the model. Define ZC as a sign reversal with absolute consecutive difference >= threshold; define SSC as a local extremum whose two adjacent absolute differences both exceed threshold. Define waveform length as sum of absolute adjacent differences, not a sample-rate-invariant physical quantity.

Use mean and standard deviation of each of six IMU axes, plus mean acceleration magnitude and gyro magnitude (14 IMU features). Four EMG channels yield 20 EMG features and 34 fused features. Record sensor units (g and degrees/s). Fit feature standardization using training windows only, separately within every validation fold. Drop zero-variance features based on training data only and preserve feature order.

## Splits and leakage control

Primary: for each participant train on session 1 and test on session 2, without test-session calibration or relabeling. Choose hyperparameters only using grouped validation inside session 1. Report session 2 -> session 1 as a secondary directional sensitivity analysis, not as independent additional participants.

Within-session reference: group whole trials by repetition block. Use four outer folds, each holding out two entire repetition blocks; fit preprocessing and tune the model using grouped inner folds in the remaining blocks. All overlapping windows from a trial remain in one fold. Save a split ledger listing participant, session, block, trial, outer fold and inner fold. Do not randomly split windows. Use the same outer splits for all modalities and both models.

Required baseline: LDA with shrinkage='auto', solver='lsqr'. Comparison: RBF-SVM with C in {0.1,1,10,100} and gamma in {'scale',0.01,0.1}; select mean inner-fold macro-F1, breaking ties toward lower C then the listed gamma order. Training weights may be balanced by class, but this choice is fixed for all comparisons and recorded. No deep-learning model sweep or test-set model selection.

Confidence calibration uses held-out training blocks only. For an SVM, fit probability calibration on training-only grouped folds; do not rely on an internal random window split. Store the calibration method and data split. The current UI's 0.87 confidence is a synthetic overlay value; no calibrated model is bundled.

## Metrics and uncertainty

Primary macro-F1 is the unweighted mean of seven per-class F1 values, with undefined class F1 set to zero and missing classes explicitly marked. Report accuracy, balanced accuracy, per-class precision/recall/F1, raw confusion counts, row-normalized confusion matrices and valid trial counts. Provide window-level metrics and a secondary trial-level majority vote with a prespecified class-order tie break. The unit for cross-participant summaries is the participant, not windows.

Show all four participant scores, mean, median, range, and paired fusion-minus-EMG differences. With four participants, emphasize descriptive estimation; a participant bootstrap interval, if shown, is exploratory and unstable. Do not use thousands of overlapping windows to claim a precise population confidence interval. Avoid a confirmatory significance claim from this small feasibility study. Report SVM and IMU-only contrasts as secondary.

Retain pre-gate predictions. Report raw classification metrics across all valid windows; separately report fraction rejected by the confidence/quality gate and accuracy among accepted predictions. This prevents an abstaining system from inflating headline accuracy. Count switches per minute on cue-stable intervals, excluding transition margins. Require 250 ms consistent accepted predictions before a positive coaching message. Stale (>250 ms), low-confidence (<0.70), missing or flagged input returns uncertain.

## Timing and resource measurements

For each prediction record window start/end, host receive time, inference completion and browser paint acknowledgement on a documented aligned clock. Report sample-to-display latency from the oldest sample and from the window end separately. A 200 ms window plus 25 ms computation is not a measured 25 ms sensor-to-display result. Characterize clock offset/drift and uncertainty before subtracting timestamps from different devices.

Estimate clock sample rate as (last index - first index)/(last time - first time); estimate received rate as received rows divided by declared acquisition duration. Loss uses intended sample slots including missing leading/trailing rows; index gaps isolate internal loss. Record packets sent, received, duplicated and reordered independently of sample counts. No packet-loss number can be inferred uniquely from CSV sample gaps without packet metadata.

Report median/p95/p99/max intersample interval, normalized interval per index increment, dropped slots, clipping per channel, flags, signal RMS, rest/activity power spectra, 60 Hz contamination, drift, and rest-to-activity amplitude contrast. A multisine simulation does not establish electrode contact, physiological SNR, anti-alias performance, RTC accuracy or electrical suitability.

## Reproducibility outputs

Archive raw five-file ZIPs, SHA-256 manifests, protocol/config IDs, software and firmware commits, exclusion and split ledgers, model hyperparameters, preprocessing/calibration coefficients, feature order, random seeds, metric tables and plotting scripts. Maintain separate directories for synthetic, recorded and derived data. Include negative results and unsuccessful qualification runs. Draft manuscript results remain empty of participant/model claims until recorded evidence exists.
