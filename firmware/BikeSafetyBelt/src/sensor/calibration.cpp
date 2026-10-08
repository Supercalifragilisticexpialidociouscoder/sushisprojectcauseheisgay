#include "calibration.h"
#include "../../config.h"

// ---------------------------------------------------------------------------
// Mount transform
// ---------------------------------------------------------------------------
static void axisVec(uint8_t sel, int8_t out[3]) {
  out[0] = out[1] = out[2] = 0;
  const uint8_t axis = (uint8_t)(sel / 2);
  out[axis < 3 ? axis : 0] = (sel % 2) ? -1 : 1;
}

bool MountTransform::configure(uint8_t forwardAxis, uint8_t upAxis) {
  int8_t f[3], u[3];
  axisVec(forwardAxis, f);
  axisVec(upAxis, u);
  if (forwardAxis > AXIS_NEG_Z || upAxis > AXIS_NEG_Z || forwardAxis / 2 == upAxis / 2) {
    // Invalid: fall back to identity so the system still runs; caller reports the error.
    for (uint8_t r = 0; r < 3; ++r)
      for (uint8_t c = 0; c < 3; ++c) m_[r][c] = (int8_t)(r == c);
    return false;
  }
  // Vehicle left (Y) = up x forward
  const int8_t l[3] = {(int8_t)(u[1] * f[2] - u[2] * f[1]), (int8_t)(u[2] * f[0] - u[0] * f[2]),
                       (int8_t)(u[0] * f[1] - u[1] * f[0])};
  for (uint8_t c = 0; c < 3; ++c) {
    m_[0][c] = f[c];
    m_[1][c] = l[c];
    m_[2][c] = u[c];
  }
  return true;
}

Vec3 MountTransform::apply(const Vec3& s) const {
  return v3(m_[0][0] * s.x + m_[0][1] * s.y + m_[0][2] * s.z,
            m_[1][0] * s.x + m_[1][1] * s.y + m_[1][2] * s.z,
            m_[2][0] * s.x + m_[2][1] * s.y + m_[2][2] * s.z);
}

// ---------------------------------------------------------------------------
// Pipeline
// ---------------------------------------------------------------------------
void ImuPipeline::begin(RawImuSensor* sensor, uint8_t forwardAxis, uint8_t upAxis) {
  sensor_ = sensor;
  mountOk_ = mount_.configure(forwardAxis, upAxis);
  clearLevel();
}

ImuStatus ImuPipeline::readUncorrected(Vec3& acc, Vec3& gyro, bool& saturated) {
  RawImu r;
  const ImuStatus st = sensor_->read(r);
  if (st != ImuStatus::OK) return st;
  const float ka = 1.0f / sensor_->accelLsbPerG();
  const float kg = 1.0f / sensor_->gyroLsbPerDps();
  saturated = r.ax == 32767 || r.ax == -32768 || r.ay == 32767 || r.ay == -32768 ||
              r.az == 32767 || r.az == -32768;
  acc = mount_.apply(v3(r.ax * ka, r.ay * ka, r.az * ka));
  gyro = mount_.apply(v3(r.gx * kg, r.gy * kg, r.gz * kg));
  return ImuStatus::OK;
}

void ImuPipeline::correctBias(Vec3& acc, Vec3& gyro) const {
  acc = vsub(acc, accOffset_);
  gyro = vsub(gyro, gyroBias_);
}

static Vec3 mul(const Mat3& R, const Vec3& v) {
  return v3(R.m[0][0] * v.x + R.m[0][1] * v.y + R.m[0][2] * v.z,
            R.m[1][0] * v.x + R.m[1][1] * v.y + R.m[1][2] * v.z,
            R.m[2][0] * v.x + R.m[2][1] * v.y + R.m[2][2] * v.z);
}

void ImuPipeline::correct(Vec3& acc, Vec3& gyro) const {
  correctBias(acc, gyro);
  if (levelSet_) {
    acc = mul(level_, acc);
    gyro = mul(level_, gyro);
  }
}

void ImuPipeline::clearLevel() {
  for (uint8_t r = 0; r < 3; ++r)
    for (uint8_t c = 0; c < 3; ++c) level_.m[r][c] = (r == c) ? 1.0f : 0.0f;
  levelSet_ = false;
}

void ImuPipeline::setLevel(const Vec3& upIn) {
  clearLevel();
  const float n = vlen(upIn);
  if (n < 0.5f) return;
  const Vec3 u = vscale(upIn, 1.0f / n);
  // Rodrigues: rotation taking u onto z = (0,0,1).  v = u x z, s = |v|, c = u.z
  const Vec3 v = v3(u.y, -u.x, 0.0f);
  const float s2 = v.x * v.x + v.y * v.y;
  const float c = u.z;
  if (s2 < 1e-10f || c <= 0.0f) return;   // already level, or upside-down (rejected earlier)
  const float k = (1.0f - c) / s2;
  // R = I + [v]x + k [v]x^2   with v.z = 0
  level_.m[0][0] = 1.0f - k * v.y * v.y;
  level_.m[0][1] = k * v.x * v.y;
  level_.m[0][2] = v.y;
  level_.m[1][0] = k * v.x * v.y;
  level_.m[1][1] = 1.0f - k * v.x * v.x;
  level_.m[1][2] = -v.x;
  level_.m[2][0] = -v.y;
  level_.m[2][1] = v.x;
  level_.m[2][2] = 1.0f - k * s2;
  levelSet_ = true;
}

// ---------------------------------------------------------------------------
// Startup calibration (Welford running mean/variance on the gyro)
// ---------------------------------------------------------------------------
void StartupCalibrator::start(uint16_t samples) {
  target_ = samples < 20 ? 20 : samples;
  n_ = 0;
  gMean_ = gM2_ = aSum_ = v3(0, 0, 0);
}

CalStatus StartupCalibrator::add(const Vec3& acc, const Vec3& gyro) {
  ++n_;
  const float inv = 1.0f / (float)n_;
  const Vec3 d = vsub(gyro, gMean_);
  gMean_ = vadd(gMean_, vscale(d, inv));
  const Vec3 d2 = vsub(gyro, gMean_);
  gM2_ = vadd(gM2_, v3(d.x * d2.x, d.y * d2.y, d.z * d2.z));
  aSum_ = vadd(aSum_, acc);

  // Early abort on an obvious movement.
  if (n_ > 10 && vlen(vsub(gyro, gMean_)) > 10.0f * CAL_MAX_GYRO_STD_DPS) return CalStatus::MOTION;
  if (n_ < target_) return CalStatus::RUNNING;

  const bool calm = gyroStd() <= CAL_MAX_GYRO_STD_DPS && vlen(gMean_) <= CAL_MAX_GYRO_BIAS_DPS;
  const bool gravityOk = fabsf(gravityMagnitude() - 1.0f) <= CAL_GRAVITY_TOLERANCE_G;
  return (calm && gravityOk) ? CalStatus::DONE : CalStatus::MOTION;
}

float StartupCalibrator::gyroStd() const {
  if (n_ < 2) return 0;
  const float k = 1.0f / (float)(n_ - 1);
  float m = gM2_.x;
  if (gM2_.y > m) m = gM2_.y;
  if (gM2_.z > m) m = gM2_.z;
  return sqrtf(m * k);
}

float StartupCalibrator::gravityMagnitude() const {
  return n_ ? vlen(aSum_) / (float)n_ : 0.0f;
}

Vec3 StartupCalibrator::accOffset() const {
  // Only the offset component along gravity is observable in a single pose:
  // remove it so a stationary sensor reads exactly 1 g.
  if (!n_) return v3(0, 0, 0);
  const Vec3 mean = vscale(aSum_, 1.0f / (float)n_);
  const float m = vlen(mean);
  if (m < 0.5f) return v3(0, 0, 0);
  return vscale(mean, 1.0f - 1.0f / m);
}

// ---------------------------------------------------------------------------
// Level (riding orientation) calibration
// ---------------------------------------------------------------------------
void LevelCalibrator::start(uint16_t samples) {
  target_ = samples < 20 ? 20 : samples;
  n_ = 0;
  aSum_ = v3(0, 0, 0);
}

CalStatus LevelCalibrator::add(const Vec3& acc, const Vec3& gyro) {
  ++n_;
  aSum_ = vadd(aSum_, acc);
  if (vlen(gyro) > 5.0f * CAL_MAX_GYRO_STD_DPS) return CalStatus::MOTION;
  if (fabsf(vlen(acc) - 1.0f) > 3.0f * CAL_GRAVITY_TOLERANCE_G) return CalStatus::MOTION;
  if (n_ < target_) return CalStatus::RUNNING;
  if (mountTiltDeg() > CAL_LEVEL_MAX_TILT_DEG) return CalStatus::BAD_ORIENTATION;
  return CalStatus::DONE;
}

Vec3 LevelCalibrator::upVector() const {
  const float m = vlen(aSum_);
  return m > 1e-6f ? vscale(aSum_, 1.0f / m) : v3(0, 0, 1);
}

float LevelCalibrator::mountTiltDeg() const {
  return acosf(clampf(upVector().z, -1.0f, 1.0f)) * RAD2DEG;
}
