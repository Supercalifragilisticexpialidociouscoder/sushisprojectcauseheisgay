// MPU-6050 register-level driver (no external library needed).
#pragma once

#include "../core/types.h"

struct RawImu {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};

enum class ImuStatus : uint8_t {
  OK = 0,
  BUS_ERROR,      // I2C NACK / timeout / short read  -> wiring or power problem
  INVALID_DATA,   // all-zero or all-0xFF frame         -> sensor reset / asleep / bus stuck
  STUCK,          // identical frames for too long       -> frozen sensor
  NOT_READY       // begin() has not succeeded
};

// Raw 6-axis sensor interface (lets the native test build plug in a fake sensor).
class RawImuSensor {
 public:
  virtual bool begin() = 0;
  virtual ImuStatus read(RawImu& out) = 0;
  virtual float accelLsbPerG() const = 0;
  virtual float gyroLsbPerDps() const = 0;
  virtual uint8_t whoAmI() const = 0;
};

FStr imuStatusName(ImuStatus s);

#ifdef ARDUINO
class Mpu6050 : public RawImuSensor {
 public:
  explicit Mpu6050(uint8_t address) : addr_(address) {}
  bool begin() override;
  ImuStatus read(RawImu& out) override;
  float accelLsbPerG() const override;
  float gyroLsbPerDps() const override;
  uint8_t whoAmI() const override { return who_; }

 private:
  bool writeReg(uint8_t reg, uint8_t val);
  bool readReg(uint8_t reg, uint8_t& val);

  uint8_t addr_;
  uint8_t who_ = 0;
  bool ready_ = false;
  RawImu last_ = {0, 0, 0, 0, 0, 0};
  uint8_t sameCount_ = 0;
};
#endif
