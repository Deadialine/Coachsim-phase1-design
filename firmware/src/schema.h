#pragma once

// Schema freeze v1: CSV columns are fixed and tools must match this header exactly.
#define SCHEMA_VERSION 1

// Ordered CSV header string (exact match required)
static const char* const CSV_HEADER_V1 =
  "schema_version,seq,t_us,rtc_health,"
  "ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,"
  "emg_uV,fsr_N,strain_uE,resp_raw,reserved0,reserved1";

// Reserved fields policy:
// - Reserved columns are ALWAYS present in header and frame rows.
// - Until implemented, firmware MUST emit zero-filled values for them.
// - Future sensors must NOT change column count/order; only start populating existing fields.
static inline void schema_fill_reserved_v1(
  float &emg_uV,
  float &fsr_N,
  float &strain_uE,
  float &resp_raw,
  float &reserved0,
  float &reserved1
) {
  // Default: zero until implemented
  emg_uV = 0.0f;
  fsr_N = 0.0f;
  strain_uE = 0.0f;
  resp_raw = 0.0f;
  reserved0 = 0.0f;
  reserved1 = 0.0f;
}