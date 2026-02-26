Over a continuous 5-minute I²C stability test on the shared bus, the system consistently detected exactly two devices—  DS1307 at 0x68   (fixed address) and   MPU-6050 at 0x69  —across   300 scans  , with   zero intermittent dropouts   (Ever missing 0x68:   NO  , Ever missing 0x69:   NO  ), therefore the bus configuration and address conflict mitigation are verified as stable; the MPU-6050 address selection was achieved by tying   AD0 HIGH to 3.3V   while keeping both modules on the same SDA/SCL lines with a common ground.

| Item | Device / Module   | I²C Address | How address is achieved                                 | I²C Bus Connections              | Power / Ground                            | Verification evidence                 | Status |
| ---- | ----------------- | ----------: | ------------------------------------------------------- | -------------------------------- | ----------------------------------------- | ------------------------------------- | ------ |
| 1    | DS1307 RTC v03    |        0x68 | Fixed (non-configurable)                                | SDA + SCL shared on the same bus | VCC powered, GND common with MCU + MPU    | 300 scans / 5 min; never missing 0x68 | PASS   |
| 2    | GY-521 (MPU-6050) |        0x69 |   AD0 tied HIGH to 3.3V   (forces 0x69 instead of 0x68) | SDA + SCL shared on the same bus | VCC powered, GND common with MCU + DS1307 | 300 scans / 5 min; never missing 0x69 | PASS   |




  I²C electrical targets (measured, power OFF, Mega2560 bus on D20/D21): Effective pull-ups are to   5V   (not 3.3V). Measured resistance   SDA→5V = 10.08 kΩ  ,   SCL→5V = 10.1 kΩ  ; leakage/parallel paths to 3.3V are effectively open (  SDA→3.3V = 1.56 MΩ  ,   SCL→3.3V = 1.4 MΩ  ), indicating no meaningful 3.3V pull-ups present.   Bus speed validation:   at   100 kHz  , the shared bus with   DS1307 @ 0x68   and   MPU-6050 @ 0x69 (AD0 tied HIGH to 3.3V)   remained stable for   5 minutes / 300 scans   with   zero dropouts   (Ever missing 0x68: NO; Ever missing 0x69: NO) →   PASS  .

| Parameter                            |   SDA (D20) |   SCL (D21) | Notes                                           |
| ------------------------------------ | ----------: | ----------: | ----------------------------------------------- |
| Pull-up resistance to 5V (measured)  |    10.08 kΩ |     10.1 kΩ | Dominant pull-ups are to 5V rail                |
| Pull-up / leakage to 3.3V (measured) |     1.56 MΩ |      1.4 MΩ | No meaningful 3.3V pull-ups present             |
| I²C bus speed verified               |     100 kHz |     100 kHz | Wire.setClock(100000)                           |
| Stability test duration / scans      | 5 min / 300 | 5 min / 300 | 1 scan/sec                                      |
| Expected devices present every scan  |  0x68, 0x69 |  0x68, 0x69 | DS1307 fixed @0x68; MPU-6050 @0x69 via AD0=3.3V |
| Result                               |        PASS |        PASS | Ever missing 0x68: NO; Ever missing 0x69: NO    |



Timebase verification was executed on the ESP32 at   I²C = 100 kHz   using a frame loop that captures   monotonic `t_us` exactly once at frame start   via `esp_timer_get_time()` and forbids time calls inside drivers; across   5929 frames  , the monotonicity acceptance criteria were met with   0 non-increasing violations  , and the observed inter-frame delta remained strictly positive with   min_dt_us = 10104 µs   and   max_dt_us = 11228 µs  , confirming no backward jumps and no negative `dt` during the test (OVERALL:   PASS  ).

| Test                            | Configuration                                                                                                                 |     Samples | Acceptance checks                                             | Result evidence                                                                        | Status |
| ------------------------------- | ----------------------------------------------------------------------------------------------------------------------------- | ----------: | ------------------------------------------------------------- | -------------------------------------------------------------------------------------- | ------ |
| 3.1 Monotonic timebase (`t_us`) | ESP32 `esp_timer_get_time()`, captured   once per frame at frame start  ; drivers do   not   call time; bus speed   100 kHz   | 5929 frames | `t_us` strictly increasing; no negative `dt` / backward jumps | Violations:   0  ; `min_dt_us`   10104 µs  ; `max_dt_us`   11228 µs  ; final:   PASS   | PASS   |



Frame schema v1 (CSV, SCHEMA_VERSION=1)
Phase 1 logging format is CSV. Firmware MUST write the header line exactly once at file/session start, then append one line per frame. Column order is frozen and tools MUST parse by header match (exact string). t_us is captured once per frame at frame start using the monotonic timer (esp_timer_get_time() on ESP32) and MUST be strictly increasing. t0_unix is stored in <log>.meta.json (session start wall-clock) and is not repeated per frame.
| Column (ordered) | Type     | Units | Meaning                                                        | Source            | Reserved policy                        |
| ---------------- | -------- | ----: | -------------------------------------------------------------- | ----------------- | -------------------------------------- |
| `schema_version` | `uint8`  |     — | Schema version tag (always `1`)                                | firmware constant | always populated                       |
| `seq`            | `uint32` |     — | Frame counter starting at 0                                    | firmware          | always populated                       |
| `t_us`           | `int64`  |    µs | Monotonic timestamp at frame start                             | esp_timer         | always populated                       |
| `rtc_health`     | `uint8`  |     — | RTC health code (`0=OK,1=MISSING,2=READ_ERROR,3=INVALID_TIME`) | firmware          | always populated (even if RTC missing) |
| `ax_g`           | `float`  |     g | MPU accel X                                                    | IMU               | if sensor absent/unread: `0`           |
| `ay_g`           | `float`  |     g | MPU accel Y                                                    | IMU               | if sensor absent/unread: `0`           |
| `az_g`           | `float`  |     g | MPU accel Z                                                    | IMU               | if sensor absent/unread: `0`           |
| `gx_dps`         | `float`  | deg/s | MPU gyro X                                                     | IMU               | if sensor absent/unread: `0`           |
| `gy_dps`         | `float`  | deg/s | MPU gyro Y                                                     | IMU               | if sensor absent/unread: `0`           |
| `gz_dps`         | `float`  | deg/s | MPU gyro Z                                                     | IMU               | if sensor absent/unread: `0`           |
| `emg_uV`         | `float`  |    µV | EMG channel (future)                                           | reserved          | **reserved; zero until implemented**   |
| `fsr_N`          | `float`  |     N | Force/pressure (future)                                        | reserved          | **reserved; zero until implemented**   |
| `strain_uE`      | `float`  |    µε | Strain (future)                                                | reserved          | **reserved; zero until implemented**   |
| `resp_raw`       | `float`  |     — | Respiration raw (future)                                       | reserved          | **reserved; zero until implemented**   |
| `reserved0`      | `float`  |     — | Extra reserved channel                                         | reserved          | **reserved; zero until implemented**   |
| `reserved1`      | `float`  |     — | Extra reserved channel                                         | reserved          | **reserved; zero until implemented**   |
Reserved fields policy (must not break tooling):
All reserved channels are always present in the CSV header and always emitted for every frame, but may be zero-filled (or blank if explicitly allowed by tooling; default is zero) until the sensor/channel is implemented. Adding future sensors MUST NOT add/remove/reorder CSV columns; the firmware only starts populating existing reserved columns.
# Non-increasing t_us violations: 0
# ===== FINAL RESULT =====
# OVERALL: PASS (header emitted + columns fixed + reserved present + monotonic t_us)
# ========================










