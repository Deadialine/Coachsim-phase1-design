/*
  Robust binary logger (fixed frames) for:
    - DS1307 RTC @ 0x68  (I2C)
    - MPU-6050/MPU-9250 class IMU @ 0x69 (AD0 high)  (I2C)

  Frame format (fixed length):
    [0]  0xAA
    [1]  0x55
    [2]  ver = 1
    [3]  len = sizeof(Packet)  (helps resync; still fixed)
    [4..] payload fields (packed, little-endian)
    [end-2..end-1] CRC16-CCITT (poly 0x1021, init 0xFFFF), over bytes [0..end-3]

  Host can detect:
    - dropped packets: seq gap
    - corruption: CRC fail
    - timing/jitter: us + dt_us

  Notes:
    - DS1307 is typically 100kHz I2C, so we use Wire.setClock(100000).
    - Use a higher Serial baud for 100 Hz binary streaming (e.g., 230400 or 460800).
*/

#include <Arduino.h>
#include <Wire.h>

// I2C addresses
static const uint8_t DS1307_ADDR = 0x68;
static const uint8_t MPU_ADDR    = 0x69;

// MPU registers
static const uint8_t REG_WHOAMI       = 0x75;
static const uint8_t REG_PWR_MGMT_1   = 0x6B;
static const uint8_t REG_SMPLRT_DIV   = 0x19;
static const uint8_t REG_CONFIG       = 0x1A;
static const uint8_t REG_GYRO_CONFIG  = 0x1B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

// Sample rate
static const uint32_t SAMPLE_HZ  = 100;
static const uint32_t PERIOD_US  = 1000000UL / SAMPLE_HZ;

// Serial
static const uint32_t SERIAL_BAUD = 230400; // use 115200+; 230400 is a good baseline

// Timing and counters
static uint32_t nextTickUs = 0;
static uint32_t lastUs     = 0;
static uint32_t seq        = 0;

static uint32_t i2cErr_mpu_tx    = 0;
static uint32_t i2cErr_mpu_short = 0;
static uint32_t i2cErr_rtc       = 0;

// ---------- utils ----------
static inline uint8_t bcd2bin(uint8_t v) { return (v & 0x0F) + 10 * (v >> 4); }

static uint16_t crc16_ccitt_false(const uint8_t *data, size_t len) {
  // CRC-16/CCITT-FALSE: init 0xFFFF, poly 0x1021, no xorout
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static bool i2cWriteByte(uint8_t addr, uint8_t reg, uint8_t val, uint32_t &errCounter) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  uint8_t err = Wire.endTransmission(true);
  if (err != 0) { errCounter++; return false; }
  return true;
}

static bool i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len,
                         uint32_t &txErrCounter, uint32_t &rxShortCounter) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission(false); // repeated start
  if (err != 0) { txErrCounter++; return false; }

  uint8_t got = Wire.requestFrom(addr, len, (uint8_t)true);
  if (got != len) { rxShortCounter++; return false; }

  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

static inline int16_t read16_be(const uint8_t *b, uint8_t i) {
  // MPU registers are big-endian pairs
  return (int16_t)((uint16_t)b[i] << 8 | (uint16_t)b[i + 1]);
}

// ---------- RTC to unix ----------
static uint32_t rtcReadUnix(uint32_t &errCounter) {
  Wire.beginTransmission(DS1307_ADDR);
  Wire.write((uint8_t)0x00);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) { errCounter++; return 0; }

  uint8_t got = Wire.requestFrom(DS1307_ADDR, (uint8_t)7, (uint8_t)true);
  if (got != 7) { errCounter++; return 0; }

  uint8_t sec  = bcd2bin(Wire.read() & 0x7F);
  uint8_t min  = bcd2bin(Wire.read());
  uint8_t hour = bcd2bin(Wire.read() & 0x3F);
  Wire.read(); // day-of-week (ignored)
  uint8_t day  = bcd2bin(Wire.read());
  uint8_t mon  = bcd2bin(Wire.read());
  uint8_t yr   = bcd2bin(Wire.read()); // 00-99 => 2000-2099

  int Y = 2000 + (int)yr;
  int M = (int)mon;
  int D = (int)day;

  if (M < 1 || M > 12 || D < 1 || D > 31) { errCounter++; return 0; }

  auto isLeap = [](int y) {
    return ((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0));
  };
  static const uint16_t daysBeforeMonth[12] = {0,31,59,90,120,151,181,212,243,273,304,334};

  int days = 0;
  for (int y = 1970; y < Y; y++) days += 365 + (isLeap(y) ? 1 : 0);

  int doy = (int)daysBeforeMonth[M - 1] + (D - 1);
  if (M > 2 && isLeap(Y)) doy += 1;
  days += doy;

  return (uint32_t)days * 86400UL + (uint32_t)hour * 3600UL + (uint32_t)min * 60UL + (uint32_t)sec;
}

// ---------- MPU init ----------
static bool mpuInit() {
  // wake up
  if (!i2cWriteByte(MPU_ADDR, REG_PWR_MGMT_1, 0x00, i2cErr_mpu_tx)) return false;

  // low-pass filter config (0x03 ~ 44Hz accel / 42Hz gyro on MPU6050 family)
  if (!i2cWriteByte(MPU_ADDR, REG_CONFIG, 0x03, i2cErr_mpu_tx)) return false;

  // sample rate = gyro output rate (1kHz) / (1 + div) => div=9 => 100 Hz
  if (!i2cWriteByte(MPU_ADDR, REG_SMPLRT_DIV, 9, i2cErr_mpu_tx)) return false;

  // full-scale ranges: gyro ±250 dps, accel ±2g
  if (!i2cWriteByte(MPU_ADDR, REG_GYRO_CONFIG,  0x00, i2cErr_mpu_tx)) return false;
  if (!i2cWriteByte(MPU_ADDR, REG_ACCEL_CONFIG, 0x00, i2cErr_mpu_tx)) return false;

  uint8_t who = 0;
  if (!i2cReadBytes(MPU_ADDR, REG_WHOAMI, &who, 1, i2cErr_mpu_tx, i2cErr_mpu_short)) return false;

  // Many MPU6050 report 0x68 regardless of I2C addr pin; 0x71 is MPU9250; etc.
  // We'll accept 0x68 as your original check, but you can broaden if needed.
  return (who == 0x68);
}

// ---------- packet ----------
#pragma pack(push, 1)
struct Packet {
  uint8_t  pre1;       // 0xAA
  uint8_t  pre2;       // 0x55
  uint8_t  ver;        // 1
  uint8_t  len;        // sizeof(Packet)

  uint32_t seq;
  uint32_t rtc_unix;
  uint32_t us;
  uint32_t dt_us;

  int16_t  ax, ay, az;
  int16_t  gx, gy, gz;

  uint16_t i2c_mpu_tx_err;
  uint16_t i2c_mpu_short;
  uint16_t i2c_rtc_err;

  uint16_t crc;        // CRC16 over bytes [0..offsetof(crc)-1]
};
#pragma pack(pop)


// ---------- setup/loop ----------
void setup() {
  Serial.begin(SERIAL_BAUD);

  Wire.begin();
  Wire.setClock(100000); // DS1307-friendly

  // Optional: give I2C devices time to power up
  delay(50);

  if (!mpuInit()) {
    // Send a readable line ONCE (safe), then continue emitting packets anyway.
    // If you don't want any text at all, comment this out.
    Serial.println("MPU init failed (continuing)");
  }

  nextTickUs = micros();
  lastUs     = nextTickUs;
  seq        = 0;
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - nextTickUs) < 0) return;
  nextTickUs += PERIOD_US;

  uint32_t dt = now - lastUs;
  lastUs = now;

  // Read IMU (14 bytes: accel(6) + temp(2) + gyro(6))
  uint8_t buf[14];
  bool ok = i2cReadBytes(MPU_ADDR, REG_ACCEL_XOUT_H, buf, 14, i2cErr_mpu_tx, i2cErr_mpu_short);

  int16_t ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
  if (ok) {
    ax = read16_be(buf, 0);
    ay = read16_be(buf, 2);
    az = read16_be(buf, 4);
    // temp is buf[6..7] (ignored here)
    gx = read16_be(buf, 8);
    gy = read16_be(buf,10);
    gz = read16_be(buf,12);
  }

  uint32_t rtcUnix = rtcReadUnix(i2cErr_rtc);

  Packet p;
  p.pre1 = 0xAA;
  p.pre2 = 0x55;
  p.ver  = 1;
  p.len  = (uint8_t)sizeof(Packet);

  p.seq      = seq++;
  p.rtc_unix = rtcUnix;
  p.us       = now;
  p.dt_us    = dt;

  p.ax = ax; p.ay = ay; p.az = az;
  p.gx = gx; p.gy = gy; p.gz = gz;

  // saturate 32-bit error counters into 16-bit fields
  p.i2c_mpu_tx_err = (uint16_t)((i2cErr_mpu_tx    > 65535UL) ? 65535UL : i2cErr_mpu_tx);
  p.i2c_mpu_short  = (uint16_t)((i2cErr_mpu_short > 65535UL) ? 65535UL : i2cErr_mpu_short);
  p.i2c_rtc_err    = (uint16_t)((i2cErr_rtc       > 65535UL) ? 65535UL : i2cErr_rtc);

  // CRC over everything except crc field
  p.crc = 0;
  const size_t crc_len = sizeof(Packet) - sizeof(uint16_t);
  p.crc = crc16_ccitt_false((const uint8_t*)&p, crc_len);

  // Emit binary frame
  Serial.write((const uint8_t*)&p, sizeof(Packet));
}
