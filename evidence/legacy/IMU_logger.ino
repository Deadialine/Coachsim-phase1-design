#include <Wire.h>

// ---------- I2C addresses ----------
static const uint8_t DS1307_ADDR = 0x68;
static const uint8_t MPU_ADDR    = 0x69; // AD0 HIGH

// ---------- MPU6050 registers ----------
static const uint8_t REG_WHOAMI      = 0x75;
static const uint8_t REG_PWR_MGMT_1  = 0x6B;
static const uint8_t REG_SMPLRT_DIV  = 0x19;
static const uint8_t REG_CONFIG      = 0x1A;
static const uint8_t REG_GYRO_CONFIG = 0x1B;
static const uint8_t REG_ACCEL_CONFIG= 0x1C;
static const uint8_t REG_ACCEL_XOUT_H= 0x3B;

// ---------- DS1307 registers ----------
static const uint8_t DS1307_REG_TIME = 0x00;

// ---------- Sampling ----------
static const uint32_t SAMPLE_HZ = 100;
static const uint32_t PERIOD_US = 1000000UL / SAMPLE_HZ;

uint32_t nextTickUs = 0;
uint32_t lastSampleUs = 0;
uint32_t seq = 0;

// ---------- Error counters ----------
uint32_t i2cErr_mpu_tx = 0;
uint32_t i2cErr_mpu_rxShort = 0;
uint32_t i2cErr_rtc_tx = 0;
uint32_t whoamiMismatch = 0;
uint32_t missedSamples = 0;
uint32_t dtMaxUs = 0;

// ---------- Helpers ----------
static uint8_t bcd2bin(uint8_t v) { return (v & 0x0F) + 10 * (v >> 4); }

bool i2cWriteByte(uint8_t addr, uint8_t reg, uint8_t val, uint32_t &errCounter) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  uint8_t err = Wire.endTransmission(true);
  if (err != 0) { errCounter++; return false; }
  return true;
}

bool i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len, uint32_t &txErrCounter, uint32_t &rxShortCounter) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission(false); // repeated start
  if (err != 0) { txErrCounter++; return false; }

  uint8_t got = Wire.requestFrom((int)addr, (int)len, (int)true);
  if (got != len) { rxShortCounter++; return false; }
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// returns unix time (seconds) using a simple conversion (no DST/timezone handling)
// If you only need relative timing + "wall clock-ish", this is fine.
uint32_t rtcReadUnix(uint32_t &txErrCounter) {
  // Read 7 bytes: sec, min, hour, dayOfWeek, day, month, year(00-99)
  Wire.beginTransmission(DS1307_ADDR);
  Wire.write(DS1307_REG_TIME);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) { txErrCounter++; return 0; }

  uint8_t got = Wire.requestFrom((int)DS1307_ADDR, 7, (int)true);
  if (got != 7) { txErrCounter++; return 0; }

  uint8_t sec  = bcd2bin(Wire.read() & 0x7F);
  uint8_t min  = bcd2bin(Wire.read());
  uint8_t hour = bcd2bin(Wire.read() & 0x3F);
  Wire.read(); // day of week (unused)
  uint8_t day  = bcd2bin(Wire.read());
  uint8_t mon  = bcd2bin(Wire.read());
  uint8_t yr   = bcd2bin(Wire.read()); // 00-99 -> 2000-2099

  // Minimal date->unix conversion (valid for 2000-2099)
  // If you don't care about absolute unix correctness, you can just print Y-M-D H:M:S instead.
  auto isLeap = [](int y){ return (y % 4) == 0; };
  static const uint16_t daysBeforeMonth[12] = {0,31,59,90,120,151,181,212,243,273,304,334};

  int Y = 2000 + yr;
  int M = mon;
  int D = day;

  // days since 1970-01-01 (rough but correct for 2000-2099 using Gregorian rules above)
  // compute days from 1970 to Y-01-01
  int days = 0;
  for (int y = 1970; y < Y; y++) {
    days += 365 + ((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0));
  }
  // add days in current year before month
  int doy = daysBeforeMonth[M-1] + (D-1);
  if (M > 2 && isLeap(Y)) doy += 1;
  days += doy;

  uint32_t unixTime = (uint32_t)days * 86400UL + (uint32_t)hour * 3600UL + (uint32_t)min * 60UL + sec;
  return unixTime;
}

bool mpuInit() {
  // Wake up
  if (!i2cWriteByte(MPU_ADDR, REG_PWR_MGMT_1, 0x00, i2cErr_mpu_tx)) return false;

  // Set sample rate divider, DLPF, ranges (safe defaults)
  // Internal gyro output rate is 8kHz (DLPF disabled) or 1kHz (DLPF enabled).
  // We'll enable DLPF and target 1kHz base, then divide down.
  if (!i2cWriteByte(MPU_ADDR, REG_CONFIG, 0x03, i2cErr_mpu_tx)) return false;      // DLPF ~44Hz accel, ~42Hz gyro
  if (!i2cWriteByte(MPU_ADDR, REG_SMPLRT_DIV, 9, i2cErr_mpu_tx)) return false;     // 1kHz/(1+9)=100Hz
  if (!i2cWriteByte(MPU_ADDR, REG_GYRO_CONFIG, 0x00, i2cErr_mpu_tx)) return false; // ±250 dps
  if (!i2cWriteByte(MPU_ADDR, REG_ACCEL_CONFIG,0x00, i2cErr_mpu_tx)) return false; // ±2g

  // WHO_AM_I check
  uint8_t who = 0;
  if (!i2cReadBytes(MPU_ADDR, REG_WHOAMI, &who, 1, i2cErr_mpu_tx, i2cErr_mpu_rxShort)) return false;
  if (who != 0x68) {
    whoamiMismatch++;
    return false;
  }
  return true;
}

void setup() {
  Serial.begin(31250);
  while (!Serial) {}

  Wire.begin();
  Wire.setClock(400000);

  delay(200);

  if (!mpuInit()) {
    Serial.println("MPU init failed. Check wiring, AD0=3.3V, addresses, pullups.");
  } else {
    Serial.println("MPU init OK.");
  }

  // CSV header
  Serial.println("seq,rtc_unix,us,dt_us,ax,ay,az,gx,gy,gz,i2c_mpu_tx_err,i2c_mpu_short,i2c_rtc_err,whoami_mismatch,missed,dtMax");
  nextTickUs = micros();
  lastSampleUs = nextTickUs;
}

int16_t read16(const uint8_t *b, uint8_t i) {
  return (int16_t)((uint16_t)b[i] << 8 | b[i+1]);
}

void loop() {
  uint32_t now = micros();

  // fixed-rate scheduler (no delay())
  if ((int32_t)(now - nextTickUs) < 0) return;
  nextTickUs += PERIOD_US;

  // timing stats
  uint32_t dt = now - lastSampleUs;
  lastSampleUs = now;
  if (dt > dtMaxUs) dtMaxUs = dt;
  if (dt > (PERIOD_US + PERIOD_US/2)) missedSamples++; // >1.5x period => likely missed

  // read IMU burst: accel(6) temp(2) gyro(6) = 14 bytes
  uint8_t buf[14];
  bool ok = i2cReadBytes(MPU_ADDR, REG_ACCEL_XOUT_H, buf, 14, i2cErr_mpu_tx, i2cErr_mpu_rxShort);

  // periodic WHO_AM_I re-check (catches bus garbage)
  if ((seq % 500) == 0) {
    uint8_t who = 0;
    if (i2cReadBytes(MPU_ADDR, REG_WHOAMI, &who, 1, i2cErr_mpu_tx, i2cErr_mpu_rxShort)) {
      if (who != 0x68) whoamiMismatch++;
    }
  }

  uint32_t rtcUnix = rtcReadUnix(i2cErr_rtc_tx);

  int16_t ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
  if (ok) {
    ax = read16(buf, 0);
    ay = read16(buf, 2);
    az = read16(buf, 4);
    gx = read16(buf, 8);
    gy = read16(buf,10);
    gz = read16(buf,12);
  } else {
    // If IMU read failed, you still output a line with zeros; host can see errors spike.
  }

  // Output CSV
  Serial.print(seq); Serial.print(',');
  Serial.print(rtcUnix); Serial.print(',');
  Serial.print(now); Serial.print(',');
  Serial.print(dt); Serial.print(',');
  Serial.print(ax); Serial.print(',');
  Serial.print(ay); Serial.print(',');
  Serial.print(az); Serial.print(',');
  Serial.print(gx); Serial.print(',');
  Serial.print(gy); Serial.print(',');
  Serial.print(gz); Serial.print(',');
  Serial.print(i2cErr_mpu_tx); Serial.print(',');
  Serial.print(i2cErr_mpu_rxShort); Serial.print(',');
  Serial.print(i2cErr_rtc_tx); Serial.print(',');
  Serial.print(whoamiMismatch); Serial.print(',');
  Serial.print(missedSamples); Serial.print(',');
  Serial.println(dtMaxUs);

  seq++;
}
