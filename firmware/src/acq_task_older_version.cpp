// firmware/src/acq_task.cpp
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

extern "C" {
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c.h"
}

static const char* TAG = "acq_task";

// =======================
// CONFIG (freeze these)
// =======================
static constexpr uint8_t DS1307_I2C_ADDR = 0x68;
static constexpr uint32_t FRAME_PERIOD_US = 10000; // example 100 Hz; set your real value

// I2C config used for DS1307 reads (set these to match your board)
static constexpr i2c_port_t I2C_PORT = I2C_NUM_0;
static constexpr gpio_num_t I2C_SDA = GPIO_NUM_21; // set to your ESP32 SDA
static constexpr gpio_num_t I2C_SCL = GPIO_NUM_22; // set to your ESP32 SCL
static constexpr uint32_t I2C_FREQ_HZ = 100000;    // Phase 1: 100 kHz

// =======================
// RTC health
// =======================
enum class RtcHealth : uint8_t {
  OK = 0,
  MISSING = 1,
  READ_ERROR = 2,
  INVALID_TIME = 3
};

static const char* rtc_health_str(RtcHealth h) {
  switch (h) {
    case RtcHealth::OK: return "OK";
    case RtcHealth::MISSING: return "MISSING";
    case RtcHealth::READ_ERROR: return "READ_ERROR";
    case RtcHealth::INVALID_TIME: return "INVALID_TIME";
    default: return "UNKNOWN";
  }
}

// =======================
// Helpers: BCD decode
// =======================
static inline uint8_t bcd2bin(uint8_t v) { return (uint8_t)((v >> 4) * 10 + (v & 0x0F)); }

// =======================
// Unix conversion (UTC)
// =======================
// Convert civil date/time to Unix seconds (UTC) without relying on timegm().
static int64_t unix_from_utc(int year, int mon, int day, int hour, int min, int sec) {
  // year: full (e.g., 2026), mon: 1-12
  // Algorithm: days since 1970-01-01 (Howard Hinnant style)
  auto is_leap = [](int y) {
    return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
  };

  static const int days_before_month[] =
      {0,31,59,90,120,151,181,212,243,273,304,334};

  int y = year;
  int m = mon;
  int d = day;

  if (m < 1 || m > 12 || d < 1 || d > 31 || hour < 0 || hour > 23 || min < 0 || min > 59 || sec < 0 || sec > 59)
    return -1;

  // days from 1970-01-01 to year-01-01
  int64_t days = 0;
  for (int yy = 1970; yy < y; yy++) days += is_leap(yy) ? 366 : 365;

  // add days within year
  days += days_before_month[m - 1];
  if (m > 2 && is_leap(y)) days += 1;
  days += (d - 1);

  return days * 86400LL + hour * 3600LL + min * 60LL + sec;
}

// =======================
// DS1307 read -> Unix (session start)
// =======================
static esp_err_t i2c_init_if_needed() {
  static bool inited = false;
  if (inited) return ESP_OK;

  i2c_config_t cfg{};
  cfg.mode = I2C_MODE_MASTER;
  cfg.sda_io_num = I2C_SDA;
  cfg.scl_io_num = I2C_SCL;
  cfg.sda_pullup_en = GPIO_PULLUP_ENABLE; // OK to enable; physical pullups dominate anyway
  cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
  cfg.master.clk_speed = I2C_FREQ_HZ;

  ESP_RETURN_ON_ERROR(i2c_param_config(I2C_PORT, &cfg), TAG, "i2c_param_config failed");
  ESP_RETURN_ON_ERROR(i2c_driver_install(I2C_PORT, cfg.mode, 0, 0, 0), TAG, "i2c_driver_install failed");

  inited = true;
  return ESP_OK;
}

static esp_err_t ds1307_read_regs(uint8_t start_reg, uint8_t* out, size_t n) {
  // write register pointer, then read n bytes
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (DS1307_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, start_reg, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (DS1307_I2C_ADDR << 1) | I2C_MASTER_READ, true);
  if (n > 1) i2c_master_read(cmd, out, n - 1, I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, out + (n - 1), I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(200));
  i2c_cmd_link_delete(cmd);
  return err;
}

static bool ds1307_present() {
  // quick ACK probe
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (DS1307_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_stop(cmd);
  esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
  i2c_cmd_link_delete(cmd);
  return (err == ESP_OK);
}

// Returns Unix time (UTC) in *t0_unix_out if OK; otherwise sets health and returns error.
static esp_err_t ds1307_read_unix(int64_t* t0_unix_out, RtcHealth* health_out) {
  if (!t0_unix_out || !health_out) return ESP_ERR_INVALID_ARG;

  ESP_RETURN_ON_ERROR(i2c_init_if_needed(), TAG, "i2c init failed");

  if (!ds1307_present()) {
    *health_out = RtcHealth::MISSING;
    return ESP_ERR_NOT_FOUND;
  }

  uint8_t r[7] = {0}; // sec, min, hour, day-of-week, date, month, year
  esp_err_t err = ds1307_read_regs(0x00, r, sizeof(r));
  if (err != ESP_OK) {
    *health_out = RtcHealth::READ_ERROR;
    return err;
  }

  // DS1307 seconds register bit7 is CH (clock halt). If set, time is not running.
  const bool ch = (r[0] & 0x80) != 0;
  uint8_t sec_bcd = r[0] & 0x7F;

  // Hours: handle 24h mode; if 12h mode bit6 set, you’d need extra parsing.
  const bool is_12h = (r[2] & 0x40) != 0;
  if (is_12h) {
    // Phase 1: treat as invalid rather than silently wrong
    *health_out = RtcHealth::INVALID_TIME;
    return ESP_ERR_INVALID_STATE;
  }

  int sec  = bcd2bin(sec_bcd);
  int min  = bcd2bin(r[1] & 0x7F);
  int hour = bcd2bin(r[2] & 0x3F);
  int day  = bcd2bin(r[4] & 0x3F);
  int mon  = bcd2bin(r[5] & 0x1F);
  int year = 2000 + bcd2bin(r[6]); // DS1307 gives 00-99

  if (ch) {
    *health_out = RtcHealth::INVALID_TIME;
    return ESP_ERR_INVALID_STATE;
  }

  int64_t unix_s = unix_from_utc(year, mon, day, hour, min, sec);
  if (unix_s < 0) {
    *health_out = RtcHealth::INVALID_TIME;
    return ESP_ERR_INVALID_RESPONSE;
  }

  *t0_unix_out = unix_s;
  *health_out = RtcHealth::OK;
  return ESP_OK;
}

// =======================
// .meta.json writer
// =======================
static esp_err_t write_meta_json(const char* log_path, int64_t t0_unix, RtcHealth rtc_health) {
  // Writes "<log_path>.meta.json" next to the log file.
  // Assumes log_path is something like "/sdcard/run_0001.bin" or "/spiffs/run_0001.bin".
  if (!log_path || std::strlen(log_path) == 0) return ESP_ERR_INVALID_ARG;

  std::string meta_path = std::string(log_path) + ".meta.json";
  FILE* f = std::fopen(meta_path.c_str(), "w");
  if (!f) return ESP_FAIL;

  // Minimal JSON; keep it stable.
  // rtc_health always written; t0_unix written even if invalid/missing (use -1).
  std::fprintf(f,
    "{\n"
    "  \"t0_unix\": %lld,\n"
    "  \"rtc_health\": \"%s\"\n"
    "}\n",
    (long long)t0_unix,
    rtc_health_str(rtc_health)
  );

  std::fclose(f);
  return ESP_OK;
}

// =======================
// Frame schema (Phase 1 minimal)
// =======================
struct FrameHeader {
  int64_t t_us;     // monotonic microseconds since boot, captured ONCE at frame start
  uint32_t seq;     // frame counter
  // Add more fields later (sensor validity flags, etc.)
};

// Example driver read stubs: IMPORTANT RULE: no time calls inside drivers.
// They accept no timing; caller provides header.t_us if needed externally.
static void read_mpu6050(/*out*/void* /*buf*/) {
  // do I2C reads, no esp_timer_get_time() here
}

static void read_other_sensors(/*out*/void* /*buf*/) {
  // no time calls
}

// =======================
// Acquisition task
// =======================
extern "C" void acq_task_main(void* arg) {
  const char* log_path = (const char*)arg; // pass your log path in task create

  // ---- Session wall-clock at start (t0_unix) ----
  int64_t t0_unix = -1;
  RtcHealth rtc_health = RtcHealth::MISSING;
  esp_err_t rtc_err = ds1307_read_unix(&t0_unix, &rtc_health);

  if (rtc_err == ESP_OK) {
    ESP_LOGI(TAG, "RTC OK: t0_unix=%lld", (long long)t0_unix);
  } else {
    ESP_LOGW(TAG, "RTC not OK (%s). Logging continues.", rtc_health_str(rtc_health));
  }

  // Write meta immediately alongside the log
  if (log_path) {
    esp_err_t m = write_meta_json(log_path, t0_unix, rtc_health);
    if (m != ESP_OK) ESP_LOGW(TAG, "Failed to write meta json for %s", log_path);
  }

  // ---- Monotonic timebase (t_us) per frame ----
  int64_t last_t_us = -1;
  uint32_t seq = 0;

  while (true) {
    FrameHeader h{};

    // Freeze the capture point: FRAME START
    h.t_us = (int64_t)esp_timer_get_time();
    h.seq = seq++;

    // Acceptance checks: strictly increasing, no backward jumps
    if (last_t_us >= 0 && h.t_us <= last_t_us) {
      ESP_LOGE(TAG, "Timebase violation: t_us not strictly increasing. last=%lld now=%lld",
               (long long)last_t_us, (long long)h.t_us);
      // Phase 1: treat as fatal; or set a flag and continue, your choice.
      // vTaskDelay(pdMS_TO_TICKS(1000));
    }
    last_t_us = h.t_us;

    // Read sensors (no timing calls inside these)
    read_mpu6050(nullptr);
    read_other_sensors(nullptr);

    // TODO: append frame header + payload to your log file here

    // pacing (simple; you can replace with a more precise scheduler)
    // Keep dt positive; esp_timer is monotonic so this is safe.
    ets_delay_us(FRAME_PERIOD_US);
  }
}