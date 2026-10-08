#include "sensor_fusion.h"

static const float kMaxTheta = 85.0f * DEG2RAD;   // keep tan()/cos() finite
static const float kHalfPi = 1.5707963f;

void SensorFusion::begin(const FusionConfig* cfg) {
  cfg_ = cfg;
  reset();
}

void SensorFusion::reset() {
  init_ = false;
  att_.valid = false;
  att_.ref = AttitudeRef::NONE;
}

Vec3 SensorFusion::gravityBody() const {
  // Specific force of gravity alone, expressed in the vehicle frame.
  const float st = sinf(theta_), ct = cosf(theta_);
  const float sp = sinf(phi_), cp = cosf(phi_);
  return v3(-st, sp * ct, cp * ct);
}

void SensorFusion::update(const ImuSample& s) {
  const Vec3& f = s.acc;
  if (!init_) {
    accLp_ = f;
    phi_ = atan2f(f.y, f.z);
    theta_ = atan2f(-f.x, sqrtf(f.y * f.y + f.z * f.z));
    init_ = true;
    publish(0, 0, 0, AttitudeRef::ACCEL);
    return;
  }

  const float dt = clampf(s.dt, 0.0f, 0.05f);
  const float a = cfg_->accLpfAlpha;
  accLp_ = vadd(accLp_, vscale(vsub(f, accLp_), a));

  // Body rates (rad/s)
  const float p = s.gyro.x * DEG2RAD, q = s.gyro.y * DEG2RAD, r = s.gyro.z * DEG2RAD;
  const float sphi = sinf(phi_), cphi = cosf(phi_);
  const float th = clampf(theta_, -kMaxTheta, kMaxTheta);
  const float tth = tanf(th), cth = cosf(th);

  // ZYX Euler kinematics
  const float phiDot = p + (q * sphi + r * cphi) * tth;
  const float thetaDot = q * cphi - r * sphi;
  const float psiDot = (q * sphi + r * cphi) / cth;

  float phiG = phi_ + phiDot * dt;
  float thetaG = theta_ + thetaDot * dt;

  const float accMag = vlen(accLp_);
  const float wMag = vlen(s.gyro);
  const bool turning = fabsf(psiDot * RAD2DEG) >= cfg_->turnRateMinDps;
  const bool accelOk = !turning && fabsf(accMag - 1.0f) <= cfg_->accTrustBandG &&
                       wMag <= cfg_->gyroTrustMaxDps;
  const bool turnOk = turning && fabsf(phiDot * RAD2DEG) <= cfg_->steadyRollDps &&
                      fabsf(phiG) <= cfg_->turnMaxLeanDeg * DEG2RAD &&
                      fabsf(thetaG) <= 30.0f * DEG2RAD && fabsf(r) > 1e-4f;

  AttitudeRef ref = AttitudeRef::GYRO;
  if (accelOk) {
    const float phiA = atan2f(accLp_.y, accLp_.z);
    const float thetaA = atan2f(-accLp_.x, sqrtf(accLp_.y * accLp_.y + accLp_.z * accLp_.z));
    const float k = 1.0f - cfg_->alpha;
    phiG += k * wrapPi(phiA - phiG);
    thetaG += k * (thetaA - thetaG);
    ref = AttitudeRef::ACCEL;
  } else if (turnOk) {
    // Coordinated turn: q = yawRate*sin(lean), r = yawRate*cos(lean)  ->  lean = atan(q/r)
    const float phiK = atanf(q / r);
    phiG += (1.0f - cfg_->alphaTurn) * wrapPi(phiK - phiG);
    ref = AttitudeRef::TURN;
  }

  // Keep Euler angles in their canonical range (theta in [-90, 90]).
  if (thetaG > kHalfPi) { thetaG = 3.14159265f - thetaG; phiG += 3.14159265f; }
  else if (thetaG < -kHalfPi) { thetaG = -3.14159265f - thetaG; phiG += 3.14159265f; }
  phi_ = wrapPi(phiG);
  theta_ = thetaG;

  publish(phiDot, thetaDot, psiDot, ref);
}

void SensorFusion::publish(float phiDot, float thetaDot, float psiDot, AttitudeRef ref) {
  att_.rollDeg = phi_ * RAD2DEG;
  att_.pitchDeg = -theta_ * RAD2DEG;
  const float c = clampf(cosf(phi_) * cosf(theta_), -1.0f, 1.0f);
  att_.tiltDeg = acosf(c) * RAD2DEG;
  att_.rollRateDps = phiDot * RAD2DEG;
  att_.pitchRateDps = -thetaDot * RAD2DEG;
  att_.yawRateDps = psiDot * RAD2DEG;
  att_.ref = ref;
  att_.valid = true;
}
