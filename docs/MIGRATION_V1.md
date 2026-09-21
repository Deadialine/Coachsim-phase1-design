# Preserving schema v1

The original v1 source and reported results are archived at [legacy evidence](../evidence/legacy/SYSTEM_SPEC_v1_as_found.md). The exact CSV header remains:

```text
schema_version,seq,t_us,rtc_health,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,emg_uV,fsr_N,strain_uE,resp_raw,reserved0,reserved1
```

The CoachSim experiment screen accepts a legacy CSV using that exact header. `seq` maps to IMU sample_index; t_us is rebased to the first observed timestamp. Preserve the source time origin in metadata. Acceleration and gyro columns map by their names with unchanged units (g, degrees/s). RTC health rows are retained in session metadata. Because v1 did not carry a separate IMU validity flag, mark imported IMU validity as unverified instead of treating zero values as healthy measurements.

Legacy EMG/FSR/strain/respiration values are reserved placeholders. Do not produce an emg.csv raw recording from them; v2 EMG and predictions files contain only their headers on import. Do not invent participant identity, calibrated channel values, a model or historical wall-clock time. The migration function can accept the original sidecar t0_unix; the browser's single-CSV import cannot select that sidecar, so it records unknown wall time (0, explicitly marked unknown). Keep the original CSV and sidecar unchanged as the provenance record.

The browser version is four-channel only; absence of EMG in a migrated file is displayed as missing data rather than four measured zero channels. Unit tests cover valid migration, nonmatching headers and preservation of relative timestamps. Future physical v2 acquisition uses an independent EMG stream and cannot be backported to the 100 Hz v1 frame without information loss.
