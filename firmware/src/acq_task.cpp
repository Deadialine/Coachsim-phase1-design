// firmware/src/acq_task.cpp
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <limits>

extern "C" {
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
}

#include "schema.h"  // SCHEMA_VERSION + CSV_HEADER_V1

static const char* TAG = "acq_task";

// =======================
// PHASE 1 POLICY (freeze)
// =======================
static constexpr int64_t IMU_RATE_HZ = 100;
static constexpr int64_t IMU_PERIOD_US = 1000000LL / IMU_RATE_HZ; // 10,000 us
static constexpr int64_t RTC_HEALTH_HZ = 1;
static constexpr int64_t RTC_HEALTH_PERIOD_US = 1000000LL / RTC_HEALTH_HZ; // 1,000,000 us

static constexpr int64_t RUN_WINDOW_US = 5LL * 60LL * 1000000LL; // 5 minutes
static constexpr int64_t JITTER_BOUND_US = 5000;                 // 5 ms
static constexpr float   RATE_TOL = 0.02f;                       // ±2%

// I2C (for RTC health probe + IMU reads)
static constexpr uint8_t DS1307_I2C_ADDR = 0x68;
static constexpr i2c_port_t I2C_PORT = I2C_NUM_0;
static constexpr gpio_num_t I2C_SDA = GPIO_NUM_21;  // set to your board
static constexpr gpio_num_t I2C_SCL = GPIO_NUM_22;  // set to your board
static constexpr uint32_t I2C_FREQ_HZ = 100000;     // Phase 1 baseline

// =======================
// RTC health enum (matches spec)
// =======================
enum class RtcHealth : uint8_t { OK=0, MISSING=1, READ_ERROR=2, INVALID_TIME=3 };

static const char* rtc_health_str(RtcHealth h){
  switch(h){
    case RtcHealth::OK: return "OK";
    case RtcHealth::MISSING: return "MISSING";
    case RtcHealth::READ_ERROR: return "READ_ERROR";
    case RtcHealth::INVALID_TIME: return "INVALID_TIME";
    default: return "UNKNOWN";
  }
}

// =======================
// I2C init + probes
// =======================
static esp_err_t i2c_init_if_needed() {
  static bool inited = false;
  if (inited) return ESP_OK;

  i2c_config_t cfg{};
  cfg.mode = I2C_MODE_MASTER;
  cfg.sda_io_num = I2C_SDA;
  cfg.scl_io_num = I2C_SCL;
  cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
  cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
  cfg.master.clk_speed = I2C_FREQ_HZ;

  ESP_RETURN_ON_ERROR(i2c_param_config(I2C_PORT, &cfg), TAG, "i2c_param_config failed");
  ESP_RETURN_ON_ERROR(i2c_driver_install(I2C_PORT, cfg.mode, 0, 0, 0), TAG, "i2c_driver_install failed");

  inited = true;
  return ESP_OK;
}

static bool i2c_ack_probe(uint8_t addr) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_stop(cmd);
  esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
  i2c_cmd_link_delete(cmd);
  return (err == ESP_OK);
}

// =======================
// Driver rule: NO time calls inside drivers.
// Return errors so scheduler can count i2c_err.
// =======================
struct ImuSample {
  float ax_g, ay_g, az_g;
  float gx_dps, gy_dps, gz_dps;
};

// Stub: replace with your real MPU6050 read that uses I2C but does NOT call esp_timer_get_time().
static esp_err_t imu_read(ImuSample* out) {
  if (!out) return ESP_ERR_INVALID_ARG;

  // TODO: real IMU read over I2C. For now deterministic dummy.
  out->ax_g = 0.01f; out->ay_g = 0.02f; out->az_g = 1.00f;
  out->gx_dps = 0.10f; out->gy_dps = 0.20f; out->gz_dps = 0.30f;
  return ESP_OK;
}

// =======================
// CSV logging helpers
// =======================
static esp_err_t write_header_once(FILE* f) {
  if (!f) return ESP_ERR_INVALID_ARG;
  int n = std::fprintf(f, "%s\n", CSV_HEADER_V1);
  return (n > 0) ? ESP_OK : ESP_FAIL;
}

static esp_err_t write_frame_csv(FILE* f,
                                uint32_t seq,
                                int64_t t_us,
                                RtcHealth rtc_health,
                                const ImuSample& imu,
                                uint32_t& i2c_err_accum) {
  if (!f) return ESP_ERR_INVALID_ARG;

  // Reserved fields: always present, zero-filled until implemented
  float emg_uV=0, fsr_N=0, strain_uE=0, resp_raw=0, reserved0=0, reserved1=0;

  // Schema v1: order MUST match CSV_HEADER_V1 exactly
  int n = std::fprintf(
    f,
    "%u,%lu,%lld,%u,"
    "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
    "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
    (unsigned)SCHEMA_VERSION,
    (unsigned long)seq,
    (long long)t_us,
    (unsigned)(uint8_t)rtc_health,
    imu.ax_g, imu.ay_g, imu.az_g,
    imu.gx_dps, imu.gy_dps, imu.gz_dps,
    emg_uV, fsr_N, strain_uE, resp_raw, reserved0, reserved1
  );

  if (n <= 0) return ESP_FAIL;
  (void)i2c_err_accum; // i2c_err tracked by caller; kept to match your phase wording
  return ESP_OK;
}

// =======================
// Precise-ish wait until next tick (microseconds)
// - Uses vTaskDelay for coarse sleep, busy-waits only the last ~1ms
// =======================
static void wait_until_us(int64_t target_us) {
  while (true) {
    int64_t now = (int64_t)esp_timer_get_time();
    int64_t remaining = target_us - now;
    if (remaining <= 0) return;

    if (remaining > 2000) {
      // sleep most of it (ms resolution)
      vTaskDelay(pdMS_TO_TICKS((remaining - 1000) / 1000));
    } else if (remaining > 50) {
      // short busy-wait
      ets_delay_us((uint32_t)remaining);
      return;
    } else {
      // tiny remainder: spin
    }
  }
}

// =======================
// Main acquisition task
// =======================
extern "C" void acq_task_main(void* arg) {
  const char* log_path = (const char*)arg; // e.g., "/sdcard/run_0001.csv" or "/spiffs/run_0001.csv"
  ESP_ERROR_CHECK(i2c_init_if_needed());

  // Open log file
  if (!log_path) {
    ESP_LOGE(TAG, "log_path is null");
    vTaskDelete(nullptr);
  }
  FILE* f = std::fopen(log_path, "w");
  if (!f) {
    ESP_LOGE(TAG, "Failed to open log file: %s", log_path);
    vTaskDelete(nullptr);
  }

  // Write header once
  ESP_ERROR_CHECK(write_header_once(f));
  std::fflush(f);

  // Scheduler + measurement state
  uint32_t seq = 0;
  uint32_t dropped_frames = 0;
  uint32_t i2c_err = 0;

  int64_t min_dt_us = std::numeric_limits<int64_t>::max();
  int64_t max_dt_us = 0;

  int64_t t_start_us = (int64_t)esp_timer_get_time();
  int64_t t_end_us = t_start_us + RUN_WINDOW_US;

  // schedule ticks on an absolute timeline
  int64_t next_tick_us = t_start_us + IMU_PERIOD_US;

  // RTC health probe schedule
  int64_t next_rtc_probe_us = t_start_us; // probe immediately
  RtcHealth rtc_health = RtcHealth::MISSING;

  // last frame timestamp to compute dt stats
  int64_t last_t_us = -1;

  while ((int64_t)esp_timer_get_time() < t_end_us) {
    // wait for schedule
    wait_until_us(next_tick_us);

    // If we are late, account for missed ticks
    int64_t now_us = (int64_t)esp_timer_get_time();
    if (now_us > next_tick_us + IMU_PERIOD_US) {
      int64_t late_by = now_us - next_tick_us;
      uint32_t missed = (uint32_t)(late_by / IMU_PERIOD_US);
      dropped_frames += missed;
      next_tick_us += (int64_t)missed * IMU_PERIOD_US;
    }

    // RTC health check at 1 Hz (ACK probe only)
    if (now_us >= next_rtc_probe_us) {
      bool ok = i2c_ack_probe(DS1307_I2C_ADDR);
      rtc_health = ok ? RtcHealth::OK : RtcHealth::MISSING;
      next_rtc_probe_us += RTC_HEALTH_PERIOD_US;
    }

    // ---- Frame start (freeze) ----
    int64_t t_us = (int64_t)esp_timer_get_time();

    // dt stats (jitter measurement)
    if (last_t_us >= 0) {
      int64_t dt = t_us - last_t_us;
      if (dt < min_dt_us) min_dt_us = dt;
      if (dt > max_dt_us) max_dt_us = dt;
    }
    last_t_us = t_us;

    // Read IMU (no time calls inside driver)
    ImuSample imu{};
    esp_err_t imu_err = imu_read(&imu);
    if (imu_err != ESP_OK) {
      i2c_err++;
      // zero-fill on failure (schema stable)
      imu = ImuSample{};
    }

    // Assemble + write frame
    esp_err_t w = write_frame_csv(f, seq, t_us, rtc_health, imu, i2c_err);
    if (w != ESP_OK) {
      ESP_LOGE(TAG, "Write failed at seq=%lu", (unsigned long)seq);
      break;
    }

    seq++;
    next_tick_us += IMU_PERIOD_US;

    // flush occasionally to avoid losing everything on crash (tune later)
    if ((seq % 200) == 0) std::fflush(f);
  }

  std::fflush(f);
  std::fclose(f);

  // =======================
  // Acceptance evaluation
  // =======================
  const float elapsed_s = (float)((RUN_WINDOW_US) / 1000000.0);
  const float achieved_rate_hz = (elapsed_s > 0) ? ((float)seq / elapsed_s) : 0.0f;
  const float rate_err = (IMU_RATE_HZ > 0) ? (achieved_rate_hz / (float)IMU_RATE_HZ - 1.0f) : 0.0f;

  int64_t jitter_span_us = (seq > 1) ? (max_dt_us - min_dt_us) : 0;

  bool pass_rate = (rate_err >= -RATE_TOL) && (rate_err <= RATE_TOL);
  bool pass_jitter = (jitter_span_us < JITTER_BOUND_US);

  ESP_LOGI(TAG, "==================== FINAL RESULT ====================");
  ESP_LOGI(TAG, "Bus speed (Hz): %lu", (unsigned long)I2C_FREQ_HZ);
  ESP_LOGI(TAG, "Target rate (Hz): %lld", (long long)IMU_RATE_HZ);
  ESP_LOGI(TAG, "Frames written: %lu", (unsigned long)seq);
  ESP_LOGI(TAG, "Dropped frames: %lu", (unsigned long)dropped_frames);
  ESP_LOGI(TAG, "I2C errors (cumulative): %lu", (unsigned long)i2c_err);
  ESP_LOGI(TAG, "Achieved rate (Hz): %.3f", achieved_rate_hz);
  ESP_LOGI(TAG, "Rate error: %.2f%%", rate_err * 100.0f);
  ESP_LOGI(TAG, "min_dt_us: %lld", (long long)min_dt_us);
  ESP_LOGI(TAG, "max_dt_us: %lld", (long long)max_dt_us);
  ESP_LOGI(TAG, "jitter span (max-min): %lld us", (long long)jitter_span_us);

  if (pass_rate && pass_jitter) {
    ESP_LOGI(TAG, "OVERALL: PASS (rate within ±2%% and jitter span < 5 ms)");
  } else {
    ESP_LOGW(TAG, "OVERALL: FAIL");
    ESP_LOGW(TAG, "PASS criteria: |rate_err|<=2%% AND (max_dt-min_dt)<5000us");
  }
  ESP_LOGI(TAG, "======================================================");

  vTaskDelete(nullptr);
}