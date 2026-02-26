#include "acq_task.h"

#include <Arduino.h>
#include <Wire.h>
#include <limits.h>
#include "schema.h"   // must define SCHEMA_VERSION and CSV_HEADER_V1

// =======================
// Phase 1: fixed policy
// =======================
static constexpr uint32_t IMU_RATE_HZ          = 100;
static constexpr uint32_t IMU_PERIOD_US        = 1000000UL / IMU_RATE_HZ;   // 10,000 us
static constexpr uint32_t RTC_HEALTH_PERIOD_US = 1000000UL;                 // 1 Hz
static constexpr uint32_t RUN_WINDOW_MS        = 5UL * 60UL * 1000UL;       // 5 minutes

static constexpr float    RATE_TOL             = 0.02f;    // ±2%
static constexpr uint32_t JITTER_BOUND_US      = 5000UL;   // 5 ms

// Mega I2C pins are fixed: SDA=20, SCL=21
static constexpr uint32_t I2C_FREQ_HZ          = 100000UL; // Phase 1 baseline
static constexpr uint8_t  DS1307_I2C_ADDR      = 0x68;

// RTC health codes (match spec)
enum class RtcHealth : uint8_t { OK=0, MISSING=1, READ_ERROR=2, INVALID_TIME=3 };

// =======================
// Helpers
// =======================
static const char* rtc_health_str(RtcHealth h) {
  switch (h) {
    case RtcHealth::OK:           return "OK";
    case RtcHealth::MISSING:      return "MISSING";
    case RtcHealth::READ_ERROR:   return "READ_ERROR";
    case RtcHealth::INVALID_TIME: return "INVALID_TIME";
    default:                      return "UNKNOWN";
  }
}

static void print_i64(Print& p, int64_t v) {
  char buf[24];
  snprintf(buf, sizeof(buf), "%lld", (long long)v);
  p.print(buf);
}

// =======================
// I2C init + ACK probe
// =======================
static void i2c_init_if_needed() {
  static bool inited = false;
  if (inited) return;

  Wire.begin();               // Mega fixed pins
  Wire.setClock(I2C_FREQ_HZ);
  inited = true;
}

static bool i2c_ack_probe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission(true) == 0);
}

// =======================
// "IMU read" stub
// Replace later with real MPU6050 read.
// IMPORTANT: no micros() inside.
// =======================
struct ImuSample { float ax, ay, az, gx, gy, gz; };

static bool imu_read(ImuSample* out) {
  if (!out) return false;
  out->ax = 0.01f; out->ay = 0.02f; out->az = 1.00f;
  out->gx = 0.10f; out->gy = 0.20f; out->gz = 0.30f;
  return true;
}

// =======================
// Scheduler wait-until (wrap-safe)
// =======================
static void wait_until_us(uint32_t target_us) {
  while ((int32_t)(micros() - target_us) < 0) {
    // spin; short 100 Hz schedule on AVR
  }
}

// =======================
// CSV helpers (Serial output)
// =======================
static void write_header_once() {
  static bool written = false;
  if (written) return;
  Serial.println(CSV_HEADER_V1);
  written = true;
}

static void write_frame_csv(uint32_t seq,
                            uint32_t t_us,
                            RtcHealth rtc_health,
                            const ImuSample& imu) {
  // Reserved fields: always present, zero-filled
  float emg_uV, fsr_N, strain_uE, resp_raw, reserved0, reserved1;
  schema_fill_reserved_v1(emg_uV, fsr_N, strain_uE, resp_raw, reserved0, reserved1);

  // IMPORTANT: order must match CSV_HEADER_V1 exactly
  Serial.print((unsigned)SCHEMA_VERSION);       Serial.print(',');
  Serial.print((unsigned long)seq);             Serial.print(',');
  Serial.print((unsigned long)t_us);            Serial.print(',');
  Serial.print((unsigned)(uint8_t)rtc_health);  Serial.print(',');

  Serial.print(imu.ax, 6); Serial.print(',');
  Serial.print(imu.ay, 6); Serial.print(',');
  Serial.print(imu.az, 6); Serial.print(',');
  Serial.print(imu.gx, 6); Serial.print(',');
  Serial.print(imu.gy, 6); Serial.print(',');
  Serial.print(imu.gz, 6); Serial.print(',');

  Serial.print(emg_uV, 6);    Serial.print(',');
  Serial.print(fsr_N, 6);     Serial.print(',');
  Serial.print(strain_uE, 6); Serial.print(',');
  Serial.print(resp_raw, 6);  Serial.print(',');
  Serial.print(reserved0, 6); Serial.print(',');
  Serial.println(reserved1, 6);
}

// =======================
// State
// =======================
static bool     g_started            = false;
static uint32_t g_seq                = 0;
static uint32_t g_dropped_frames     = 0;
static uint32_t g_i2c_err            = 0;

static uint32_t g_min_dt_us          = UINT32_MAX;
static uint32_t g_max_dt_us          = 0;

static uint32_t g_start_ms           = 0;
static uint32_t g_next_tick_us       = 0;
static uint32_t g_next_rtc_probe_us  = 0;

static RtcHealth g_rtc_health        = RtcHealth::MISSING;

static uint32_t g_last_t_us          = 0;
static bool     g_have_last_t        = false;

// =======================
// Public entry points
// =======================
void acq_setup() {
  Serial.begin(115200);
  delay(200);

  i2c_init_if_needed();

  Serial.println();
  Serial.println(F("=================================================="));
  Serial.println(F("PHASE 1 acquisition run (Arduino Mega 2560)"));
  Serial.println(F("Output mode: CSV streamed over Serial"));
  Serial.println(F("Capture Serial output on PC to save the log."));
  Serial.print(F("I2C SDA=20 SCL=21 Freq=")); Serial.print(I2C_FREQ_HZ); Serial.println(F(" Hz"));
  Serial.print(F("Target rate (Hz): ")); Serial.println(IMU_RATE_HZ);
  Serial.print(F("Run window (ms): ")); Serial.println(RUN_WINDOW_MS);
  Serial.println(F("PASS criteria: |rate_err|<=2% AND (max_dt-min_dt)<5000us"));
  Serial.println(F("=================================================="));
  Serial.println();

  write_header_once();

  g_started           = true;
  g_seq               = 0;
  g_dropped_frames    = 0;
  g_i2c_err           = 0;
  g_min_dt_us         = UINT32_MAX;
  g_max_dt_us         = 0;
  g_start_ms          = millis();

  uint32_t now_us     = micros();
  g_next_tick_us      = now_us + IMU_PERIOD_US;
  g_next_rtc_probe_us = now_us;
  g_rtc_health        = RtcHealth::MISSING;
  g_have_last_t       = false;
}

void acq_loop() {
  if (!g_started) return;

  // stop after 5 minutes
  uint32_t elapsed_ms = millis() - g_start_ms;
  if (elapsed_ms >= RUN_WINDOW_MS) {
    float elapsed_s = elapsed_ms / 1000.0f;
    float achieved_rate = (elapsed_s > 0.0f) ? ((float)g_seq / elapsed_s) : 0.0f;
    float rate_err = achieved_rate / (float)IMU_RATE_HZ - 1.0f;

    uint32_t jitter_span_us = (g_seq > 1) ? (g_max_dt_us - g_min_dt_us) : 0;

    bool pass_rate   = (rate_err >= -RATE_TOL) && (rate_err <= RATE_TOL);
    bool pass_jitter = (jitter_span_us < JITTER_BOUND_US);

    Serial.println();
    Serial.println(F("==================== FINAL RESULT ===================="));
    Serial.print(F("Bus speed (Hz): ")); Serial.println(I2C_FREQ_HZ);
    Serial.print(F("Target rate (Hz): ")); Serial.println(IMU_RATE_HZ);
    Serial.print(F("Frames written: ")); Serial.println(g_seq);
    Serial.print(F("Dropped frames: ")); Serial.println(g_dropped_frames);
    Serial.print(F("I2C errors (cumulative): ")); Serial.println(g_i2c_err);
    Serial.print(F("Final rtc_health: ")); Serial.println(rtc_health_str(g_rtc_health));
    Serial.print(F("Achieved rate (Hz): ")); Serial.println(achieved_rate, 3);
    Serial.print(F("Rate error: ")); Serial.print(rate_err * 100.0f, 2); Serial.println(F("%"));
    Serial.print(F("min_dt_us: ")); Serial.println(g_min_dt_us);
    Serial.print(F("max_dt_us: ")); Serial.println(g_max_dt_us);
    Serial.print(F("jitter span (max-min): ")); Serial.print(jitter_span_us); Serial.println(F(" us"));

    if (pass_rate && pass_jitter) {
      Serial.println(F("OVERALL: PASS (rate within ±2% and jitter span < 5 ms)"));
    } else {
      Serial.println(F("OVERALL: FAIL"));
    }
    Serial.println(F("======================================================"));

    while (true) delay(1000);
  }

  wait_until_us(g_next_tick_us);

  uint32_t now_us = micros();

  // dropped frames if late by more than one period
  if ((uint32_t)(now_us - g_next_tick_us) > IMU_PERIOD_US) {
    uint32_t late_by = (uint32_t)(now_us - g_next_tick_us);
    uint32_t missed  = late_by / IMU_PERIOD_US;
    g_dropped_frames += missed;
    g_next_tick_us   += missed * IMU_PERIOD_US;
  }

  // RTC health probe @ 1 Hz (ACK only)
  if ((int32_t)(now_us - g_next_rtc_probe_us) >= 0) {
    g_rtc_health = i2c_ack_probe(DS1307_I2C_ADDR) ? RtcHealth::OK : RtcHealth::MISSING;
    g_next_rtc_probe_us += RTC_HEALTH_PERIOD_US;
  }

  // ---- FRAME START (freeze) ----
  uint32_t t_us = micros();

  if (g_have_last_t) {
    uint32_t dt = (uint32_t)(t_us - g_last_t_us);
    if (dt < g_min_dt_us) g_min_dt_us = dt;
    if (dt > g_max_dt_us) g_max_dt_us = dt;
  }
  g_last_t_us = t_us;
  g_have_last_t = true;

  ImuSample imu{};
  bool ok = imu_read(&imu);
  if (!ok) {
    g_i2c_err++;
    imu = ImuSample{};
  }

  write_frame_csv(g_seq, t_us, g_rtc_health, imu);
  g_seq++;
  g_next_tick_us += IMU_PERIOD_US;
}