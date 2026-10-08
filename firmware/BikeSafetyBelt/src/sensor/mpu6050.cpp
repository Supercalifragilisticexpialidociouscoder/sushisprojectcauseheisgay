#include "mpu6050.h"
#include "../../config.h"

FStr imuStatusName(ImuStatus s) {
  switch (s) {
    case ImuStatus::OK: return FS("OK");
    case ImuStatus::BUS_ERROR: return FS("I2C bus error");
    case ImuStatus::INVALID_DATA: return FS("invalid data");
    case ImuStatus::STUCK: return FS("frozen data");
    default: return FS("not initialised");
  }
}

#ifdef ARDUINO
#include <Wire.h>

namespace {
const uint8_t REG_SMPLRT_DIV = 0x19;
const uint8_t REG_CONFIG = 0x1A;
const uint8_t REG_GYRO_CONFIG = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_WHO_AM_I = 0x75;

constexpr uint8_t accelFsSel(int g) { return g == 2 ? 0 : g == 4 ? 1 : g == 8 ? 2 : g == 16 ? 3 : 0xFF; }
constexpr uint8_t gyroFsSel(int dps) { return dps == 250 ? 0 : dps == 500 ? 1 : dps == 1000 ? 2 : dps == 2000 ? 3 : 0xFF; }

const uint8_t kAccelSel = accelFsSel(MPU_ACCEL_RANGE_G);
const uint8_t kGyroSel = gyroFsSel(MPU_GYRO_RANGE_DPS);
static_assert(accelFsSel(MPU_ACCEL_RANGE_G) != 0xFF, "MPU_ACCEL_RANGE_G must be 2, 4, 8 or 16");
static_assert(gyroFsSel(MPU_GYRO_RANGE_DPS) != 0xFF, "MPU_GYRO_RANGE_DPS must be 250, 500, 1000 or 2000");
// Gyro output rate is 1 kHz when the DLPF is enabled (DLPF_CFG 1..6).
static_assert(MPU_DLPF_CFG >= 1, "Use MPU_DLPF_CFG 1..6 (DLPF enabled) so the sample divider math holds");
const uint8_t kSmplrtDiv = (uint8_t)(1000 / SAMPLE_RATE_HZ - 1);
}  // namespace

bool Mpu6050::writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr_);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool Mpu6050::readReg(uint8_t reg, uint8_t& val) {
  Wire.beginTransmission(addr_);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)addr_, (uint8_t)1) != 1) return false;
  val = (uint8_t)Wire.read();
  return true;
}

bool Mpu6050::begin() {
  ready_ = false;
  sameCount_ = 0;
  who_ = 0;
  if (!readReg(REG_WHO_AM_I, who_)) return false;
  if (who_ == 0x00 || who_ == 0xFF) return false;   // nothing sensible on the bus
  // Genuine MPU-6050 returns 0x68; many compatible clones return other IDs
  // (0x70, 0x72, 0x98 ...). They are accepted; the ID is reported in the log.

  if (!writeReg(REG_PWR_MGMT_1, 0x01)) return false;          // wake, PLL with X-gyro clock
  if (!writeReg(REG_CONFIG, MPU_DLPF_CFG)) return false;
  if (!writeReg(REG_SMPLRT_DIV, kSmplrtDiv)) return false;
  if (!writeReg(REG_GYRO_CONFIG, (uint8_t)(kGyroSel << 3))) return false;
  if (!writeReg(REG_ACCEL_CONFIG, (uint8_t)(kAccelSel << 3))) return false;

  // Read back: catches wrong device, bus corruption, or a sensor that did not wake.
  uint8_t v = 0;
  if (!readReg(REG_PWR_MGMT_1, v) || (v & 0x40)) return false;          // still asleep
  if (!readReg(REG_GYRO_CONFIG, v) || ((v >> 3) & 0x03) != kGyroSel) return false;
  if (!readReg(REG_ACCEL_CONFIG, v) || ((v >> 3) & 0x03) != kAccelSel) return false;
  ready_ = true;
  return true;
}

ImuStatus Mpu6050::read(RawImu& out) {
  if (!ready_) return ImuStatus::NOT_READY;
  Wire.beginTransmission(addr_);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return ImuStatus::BUS_ERROR;
  const uint8_t n = Wire.requestFrom((uint8_t)addr_, (uint8_t)14);
  if (n != 14) {
    while (Wire.available()) Wire.read();
    return ImuStatus::BUS_ERROR;
  }
  uint8_t b[14];
  bool allZero = true, allFF = true;
  for (uint8_t i = 0; i < 14; ++i) {
    b[i] = (uint8_t)Wire.read();
    if (b[i] != 0x00) allZero = false;
    if (b[i] != 0xFF) allFF = false;
  }
  if (allZero || allFF) return ImuStatus::INVALID_DATA;

  out.ax = (int16_t)((b[0] << 8) | b[1]);
  out.ay = (int16_t)((b[2] << 8) | b[3]);
  out.az = (int16_t)((b[4] << 8) | b[5]);
  // b[6..7] = temperature (unused)
  out.gx = (int16_t)((b[8] << 8) | b[9]);
  out.gy = (int16_t)((b[10] << 8) | b[11]);
  out.gz = (int16_t)((b[12] << 8) | b[13]);

  // A live sensor always has LSB noise; identical frames mean the data is frozen.
  const bool same = out.ax == last_.ax && out.ay == last_.ay && out.az == last_.az &&
                    out.gx == last_.gx && out.gy == last_.gy && out.gz == last_.gz;
  last_ = out;
  if (same) {
    if (sameCount_ < 255) ++sameCount_;
    if (sameCount_ >= IMU_STUCK_SAMPLES) return ImuStatus::STUCK;
  } else {
    sameCount_ = 0;
  }
  return ImuStatus::OK;
}

float Mpu6050::accelLsbPerG() const { return 16384.0f / (float)(1 << kAccelSel); }

float Mpu6050::gyroLsbPerDps() const {
  static const float kGyroLsb[4] = {131.0f, 65.5f, 32.8f, 16.4f};
  return kGyroLsb[kGyroSel];
}
#endif  // ARDUINO
