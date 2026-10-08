// IMU pipeline and calibration.
//
//   raw counts ──scale──► sensor frame (g, deg/s) ──mount──► vehicle frame
//              ──bias──► (gyro bias, accel offset) ──level──► calibrated ImuSample
//
// * Mount transform: which sensor axes point forward / up (config.h). Coarse.
// * Startup calibration (every boot, bike stationary): gyro bias per axis and the
//   accelerometer offset along gravity (the part of the accel offset observable in
//   one pose). Rejected and retried if the bike moves.
// * Level calibration ("CAL LEVEL", bike upright & still, stored in EEPROM):
//   measures residual mounting tilt and rotates it away, so 0 deg roll/pitch = the
//   bike's normal riding orientation. The sensor never needs to be mounted level.
#pragma once

#include "../core/types.h"
#include "mpu6050.h"

struct Mat3 {
  float m[3][3];
};

class MountTransform {
 public:
  bool configure(uint8_t forwardAxis, uint8_t upAxis);   // false if axes are not perpendicular
  Vec3 apply(const Vec3& s) const;
 private:
  int8_t m_[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
};

class ImuPipeline {
 public:
  void begin(RawImuSensor* sensor, uint8_t forwardAxis, uint8_t upAxis);
  bool mountValid() const { return mountOk_; }
  bool initSensor() { return sensor_ && sensor_->begin(); }
  RawImuSensor* sensor() { return sensor_; }

  // Scaled + mounted (vehicle frame) but NOT bias corrected — used by calibration.
  ImuStatus readUncorrected(Vec3& acc, Vec3& gyro, bool& saturated);
  // Applies gyro bias, accel offset and level rotation.
  void correct(Vec3& acc, Vec3& gyro) const;
  // Applies only bias/offset (used by level calibration).
  void correctBias(Vec3& acc, Vec3& gyro) const;

  void setGyroBias(const Vec3& b) { gyroBias_ = b; }
  void setAccOffset(const Vec3& o) { accOffset_ = o; }
  void setLevel(const Vec3& upUnit);     // computes rotation that maps upUnit -> +Z
  void clearLevel();
  const Vec3& gyroBias() const { return gyroBias_; }
  bool levelSet() const { return levelSet_; }

 private:
  RawImuSensor* sensor_ = nullptr;
  MountTransform mount_;
  bool mountOk_ = false;
  Vec3 gyroBias_ = {0, 0, 0};
  Vec3 accOffset_ = {0, 0, 0};
  Mat3 level_;
  bool levelSet_ = false;
};

enum class CalStatus : uint8_t { RUNNING = 0, DONE, MOTION, BAD_ORIENTATION };

// Startup gyro-bias / accel-offset calibration (bike must be stationary).
class StartupCalibrator {
 public:
  void start(uint16_t samples);
  CalStatus add(const Vec3& acc, const Vec3& gyro);
  Vec3 gyroBias() const { return gMean_; }
  Vec3 accOffset() const;
  float gravityMagnitude() const;
  float gyroStd() const;
 private:
  uint16_t target_ = 0, n_ = 0;
  Vec3 gMean_ = {0, 0, 0}, gM2_ = {0, 0, 0}, aSum_ = {0, 0, 0};
};

// Riding-orientation ("level") calibration.
class LevelCalibrator {
 public:
  void start(uint16_t samples);
  CalStatus add(const Vec3& accBiasCorrected, const Vec3& gyroBiasCorrected);
  Vec3 upVector() const;          // unit gravity direction in vehicle frame
  float mountTiltDeg() const;     // residual mount tilt that will be removed
 private:
  uint16_t target_ = 0, n_ = 0;
  Vec3 aSum_ = {0, 0, 0};
};
