#include "acq_task.h"
#include "schema.h"
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// =======================
// CONFIG (freeze these)
// =======================
static constexpr uint8_t  DS1307_I2C_ADDR = 0x68;
static constexpr uint32_t FRAME_PERIOD_US = 10000;   // 100 Hz example
static constexpr uint8_t  SD_CS_PIN       = 4;       // change to your shield/module CS

static const char* LOG_PATH = "run_0001.bin";

// =======================
// RTC health
// =======================
enum class RtcHealth : uint8_t {
  OK = 0,
  MISSING = 1,
  READ_ERROR = 2,
  INVALID_TIME = 3
};
static void print_i64(Print& p, int64_t v) {
  char buf[24];
  // %lld is supported by avr-libc snprintf for long long
  snprintf(buf, sizeof(buf), "%lld", (long long)v);
  p.print(buf);
}
static const char* rtc_health_str(RtcHealth h) {
  switch (h) {
    case RtcHealth::OK:           return "OK";
    case RtcHealth::MISSING:      return "MISSING";
    case RtcHealth::READ_ERROR:   return "READ_ERROR";
    case RtcHealth::INVALID_TIME: return "INVALID_TIME";
    default:                      return "UNKNOWN";
  }
}

// =======================
// Helpers: BCD decode
// =======================
static inline uint8_t bcd2bin(uint8_t v) { return (uint8_t)((v >> 4) * 10 + (v & 0x0F)); }

// =======================
// Unix conversion (UTC)
// =======================
static int64_t unix_from_utc(int year, int mon, int day, int hour, int min, int sec) {
  auto is_leap = [](int y) {
    return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
  };

  static const int days_before_month[] = {0,31,59,90,120,151,181,212,243,273,304,334};

  if (mon < 1 || mon > 12 || day < 1 || day > 31 ||
      hour < 0 || hour > 23 || min < 0 || min > 59 || sec < 0 || sec > 59)
    return -1;

  int64_t days = 0;
  for (int yy = 1970; yy < year; yy++) days += is_leap(yy) ? 366 : 365;

  days += days_before_month[mon - 1];
  if (mon > 2 && is_leap(year)) days += 1;
  days += (day - 1);

  return days * 86400LL + (int64_t)hour * 3600LL + (int64_t)min * 60LL + sec;
}

// =======================
// DS1307 I2C helpers
// =======================
static bool ds1307_present() {
  Wire.beginTransmission(DS1307_I2C_ADDR);
  return (Wire.endTransmission() == 0);
}

static bool ds1307_read_regs(uint8_t start_reg, uint8_t* out, size_t n) {
  Wire.beginTransmission(DS1307_I2C_ADDR);
  Wire.write(start_reg);
  if (Wire.endTransmission(false) != 0) return false; // repeated start

  size_t got = Wire.requestFrom((int)DS1307_I2C_ADDR, (int)n, (int)true);
  if (got != n) return false;

  for (size_t i = 0; i < n; i++) out[i] = Wire.read();
  return true;
}

static bool ds1307_read_unix(int64_t* t0_unix_out, RtcHealth* health_out) {
  if (!t0_unix_out || !health_out) return false;

  if (!ds1307_present()) {
    *health_out = RtcHealth::MISSING;
    return false;
  }

  uint8_t r[7] = {0};
  if (!ds1307_read_regs(0x00, r, sizeof(r))) {
    *health_out = RtcHealth::READ_ERROR;
    return false;
  }

  const bool ch = (r[0] & 0x80) != 0;
  const bool is_12h = (r[2] & 0x40) != 0;
  if (ch || is_12h) {
    *health_out = RtcHealth::INVALID_TIME;
    return false;
  }

  int sec  = bcd2bin(r[0] & 0x7F);
  int min  = bcd2bin(r[1] & 0x7F);
  int hour = bcd2bin(r[2] & 0x3F);
  int day  = bcd2bin(r[4] & 0x3F);
  int mon  = bcd2bin(r[5] & 0x1F);
  int year = 2000 + bcd2bin(r[6]);

  int64_t unix_s = unix_from_utc(year, mon, day, hour, min, sec);
  if (unix_s < 0) {
    *health_out = RtcHealth::INVALID_TIME;
    return false;
  }

  *t0_unix_out = unix_s;
  *health_out = RtcHealth::OK;
  return true;
}

// =======================
// .meta.json writer (SD)
// =======================
static bool write_meta_json_sd(const char* log_path, int64_t t0_unix, RtcHealth rtc_health) {
  if (!log_path || log_path[0] == '\0') return false;

  char meta_path[48];
  snprintf(meta_path, sizeof(meta_path), "%s.meta.json", log_path);

  File f = SD.open(meta_path, FILE_WRITE);
  if (!f) return false;

  f.print("{\n  \"t0_unix\": ");
  print_i64(f, t0_unix);
  f.print(",\n  \"rtc_health\": \"");
  f.print(rtc_health_str(rtc_health));
  f.print("\"\n}\n");
  f.close();
  return true;
}

// =======================
// Frame schema (Phase 1 minimal)
// =======================
struct FrameHeader {
  uint32_t t_us;  // micros() captured ONCE at frame start (wraps ~70 min)
  uint32_t seq;
};

// RULE: no time calls inside drivers
static void read_mpu6050(void* /*buf*/) { /* TODO */ }
static void read_other_sensors(void* /*buf*/) { /* TODO */ }

static void append_frame(File& logf, const FrameHeader& h) {
  logf.write((const uint8_t*)&h, sizeof(h));
}

// =======================
// State
// =======================
static File g_log;
static uint32_t g_seq = 0;
static uint32_t g_next_deadline_us = 0;
static bool     g_started = false;

// =======================
// Public entry points
// =======================
void acq_setup() {
  Serial.begin(115200);

  Wire.begin();              // Mega: SDA=20, SCL=21
  Wire.setClock(100000UL);   // Phase 1

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("SD init failed. Logging disabled."));
  } else {
    g_log = SD.open(LOG_PATH, FILE_WRITE);
    if (!g_log) Serial.println(F("Failed to open log file."));
  }

  int64_t t0_unix = -1;
  RtcHealth rtc_health = RtcHealth::MISSING;

  if (ds1307_read_unix(&t0_unix, &rtc_health)) {
    Serial.print(F("RTC OK: t0_unix="));
    print_i64(Serial, t0_unix);
    Serial.println();
  } else {
    Serial.print(F("RTC not OK ("));
    Serial.print(rtc_health_str(rtc_health));
    Serial.println(F("). Logging continues."));
  }

  if (SD.begin(SD_CS_PIN)) {
    if (!write_meta_json_sd(LOG_PATH, t0_unix, rtc_health)) {
      Serial.println(F("Failed to write meta json."));
    }
  }

  if (g_log) g_log.flush();

  g_started = false; // reset scheduler
  g_seq = 0;
}

void acq_loop() {
  if (!g_started) {
    g_started = true;
    g_next_deadline_us = micros();
  }

  // Wait until deadline (wrap-safe)
  while ((int32_t)(micros() - g_next_deadline_us) < 0) {
    // optional: delayMicroseconds(50);
  }
  g_next_deadline_us += FRAME_PERIOD_US;

  // ---- Frame start: freeze t_us here ----
  FrameHeader h;
  h.t_us = micros();
  h.seq  = g_seq++;

  // Drivers: no timing calls inside
  read_mpu6050(nullptr);
  read_other_sensors(nullptr);

  if (g_log) {
    append_frame(g_log, h);
    if ((h.seq % 100) == 0) g_log.flush();
  }

  if ((h.seq % 200) == 0) {
    Serial.print(F("seq=")); Serial.print(h.seq);
    Serial.print(F(" t_us=")); Serial.println(h.t_us);
  }
}