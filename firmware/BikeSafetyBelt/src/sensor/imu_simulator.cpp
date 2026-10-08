#include "imu_simulator.h"
#include "../../config.h"

static const float kPi = 3.14159265f;
static const float kG = 9.80665f;

// Noise levels (1 sigma): accelerometer g, gyroscope deg/s
static const float kNoiseParkedAcc = 0.003f, kNoiseParkedGyro = 0.05f;   // engine off
static const float kNoiseIdleAcc = 0.03f, kNoiseIdleGyro = 0.4f;         // engine idling
static const float kNoiseRideAcc = 0.08f, kNoiseRideGyro = 1.2f;         // road + engine
static const float kNoiseCrashAcc = 0.35f, kNoiseCrashGyro = 8.0f;       // sliding on tarmac

// Smooth 0 -> 1 transition between t0 and t1.
static float ss(float t, float t0, float t1) {
  if (t <= t0) return 0;
  if (t >= t1) return 1;
  const float u = (t - t0) / (t1 - t0);
  return u * u * (3.0f - 2.0f * u);
}
// Half-sine pulse of width w starting at t0 (peak 1).
static float hs(float t, float t0, float w) {
  if (t < t0 || t > t0 + w) return 0;
  return sinf(kPi * (t - t0) / w);
}
// Hold between two transitions: rises over [a,b], falls over [c,d].
static float hold(float t, float a, float b, float c, float d) { return ss(t, a, b) - ss(t, c, d); }

bool ImuSimulator::valid(uint8_t id) {
  switch (id) {
    case SIM_STATIONARY: case SIM_NORMAL_LEAN: case SIM_CORNERING: case SIM_BRAKING:
    case SIM_SPEED_BREAKER: case SIM_SUDDEN_ROTATION: case SIM_CRASH: case SIM_DISCONNECT:
    case SIM_POTHOLE: case SIM_TIPOVER_PARKED: case SIM_HIGHSIDE:
      return true;
    default:
      return false;
  }
}

FStr ImuSimulator::name(uint8_t id) {
  switch (id) {
    case SIM_STATIONARY: return FS("Normal stationary state");
    case SIM_NORMAL_LEAN: return FS("Normal lean (side stand / walking the bike)");
    case SIM_CORNERING: return FS("Simulated cornering up to 45 deg");
    case SIM_BRAKING: return FS("Sudden braking + quick acceleration");
    case SIM_SPEED_BREAKER: return FS("Speed breaker");
    case SIM_SUDDEN_ROTATION: return FS("Sudden rotation (emergency swerve)");
    case SIM_CRASH: return FS("Crash-like combined motion (low-side)");
    case SIM_DISCONNECT: return FS("Sensor disconnect (6 s)");
    case SIM_POTHOLE: return FS("Pothole impacts");
    case SIM_TIPOVER_PARKED: return FS("Parked bike knocked over");
    case SIM_HIGHSIDE: return FS("High-side crash with tumble");
    default: return FS("none");
  }
}

uint32_t ImuSimulator::durationMs(uint8_t id) {
  switch (id) {
    case SIM_NORMAL_LEAN: return 16000;
    case SIM_CORNERING: return 26000;
    case SIM_CRASH: return 16000;
    case SIM_POTHOLE: return 10000;
    case SIM_TIPOVER_PARKED: return 48000;
    case SIM_HIGHSIDE: return 16000;
    default: return 12000;
  }
}

bool ImuSimulator::start(uint8_t id, uint32_t nowMs, uint32_t seed) {
  if (!valid(id)) return false;
  id_ = id;
  startMs_ = nowMs;
  rng_ = seed ? seed : 1;
  return true;
}

bool ImuSimulator::finished(uint32_t nowMs) const {
  return !active() || elapsed(nowMs, startMs_, durationMs(id_));
}

float ImuSimulator::gauss() {
  // Sum of 4 uniforms -> approx. normal, unit variance.
  float s = 0;
  for (uint8_t i = 0; i < 4; ++i) {
    rng_ = rng_ * 1664525UL + 1013904223UL;
    s += (float)(rng_ >> 8) * (1.0f / 16777216.0f);
  }
  return (s - 2.0f) * 1.7320508f;
}

void ImuSimulator::evaluate(float t, Truth& tr) const {
  tr.rollDeg = tr.pitchUpDeg = tr.yawExtraDps = tr.aLongG = tr.aVertG = 0;
  tr.impulse = v3(0, 0, 0);
  tr.noiseAcc = kNoiseRideAcc;
  tr.noiseGyro = kNoiseRideGyro;
  tr.speed = 0;
  tr.coordinated = false;
  tr.fail = false;

  switch (id_) {
    case SIM_STATIONARY:
      tr.noiseAcc = kNoiseIdleAcc; tr.noiseGyro = kNoiseIdleGyro;
      break;

    case SIM_NORMAL_LEAN:
      tr.noiseAcc = kNoiseIdleAcc; tr.noiseGyro = kNoiseIdleGyro;
      tr.rollDeg = -12.0f * hold(t, 2.0f, 3.5f, 6.0f, 7.5f) + 25.0f * hold(t, 9.0f, 11.0f, 13.0f, 15.0f);
      break;

    case SIM_CORNERING:
      tr.speed = 15.0f; tr.coordinated = true;
      tr.rollDeg = -40.0f * hold(t, 2.0f, 3.5f, 7.0f, 8.5f) + 40.0f * hold(t, 8.5f, 10.0f, 13.0f, 14.5f)
                   - 30.0f * ss(t, 14.5f, 16.0f) + 60.0f * ss(t, 16.0f, 16.8f) - 30.0f * ss(t, 16.8f, 17.6f)
                   - 45.0f * hold(t, 18.0f, 19.5f, 22.5f, 24.0f);
      break;

    case SIM_BRAKING: {
      tr.speed = 20.0f; tr.coordinated = true;
      const float brake = hold(t, 3.0f, 3.15f, 5.0f, 5.3f);
      const float accel = hold(t, 7.0f, 7.3f, 9.0f, 9.3f);
      tr.aLongG = -1.0f * brake + 0.15f * sinf(2.0f * kPi * 15.0f * t) * brake + 0.6f * accel;
      tr.pitchUpDeg = -4.0f * hold(t, 3.0f, 3.2f, 5.0f, 5.3f) + 2.0f * accel;
      break;
    }

    case SIM_SPEED_BREAKER:
      tr.speed = 6.0f; tr.coordinated = true;
      tr.aLongG = -0.3f * hs(t, 3.5f, 0.5f);
      tr.aVertG = 2.0f * hs(t, 4.0f, 0.08f) - 0.6f * hs(t, 4.1f, 0.12f)
                  + 1.8f * hs(t, 4.45f, 0.08f) - 0.5f * hs(t, 4.55f, 0.12f);
      tr.pitchUpDeg = 5.0f * hs(t, 3.95f, 0.4f) - 4.0f * hs(t, 4.4f, 0.4f);
      break;

    case SIM_POTHOLE:
      tr.speed = 12.0f; tr.coordinated = true;
      tr.aVertG = 3.2f * hs(t, 4.0f, 0.04f) - 0.6f * hs(t, 4.05f, 0.08f)
                  + 2.8f * hs(t, 4.2f, 0.04f) - 0.5f * hs(t, 4.25f, 0.08f);
      tr.aLongG = -1.2f * hs(t, 4.0f, 0.04f) - 1.0f * hs(t, 4.2f, 0.04f);
      tr.pitchUpDeg = -3.0f * hs(t, 4.0f, 0.2f) + 3.0f * hs(t, 4.2f, 0.2f);
      break;

    case SIM_SUDDEN_ROTATION:
      tr.speed = 12.0f; tr.coordinated = true;
      tr.rollDeg = 30.0f * ss(t, 4.0f, 4.3f) - 55.0f * ss(t, 4.3f, 4.8f) + 25.0f * ss(t, 4.8f, 5.3f);
      break;

    case SIM_CRASH: {
      // Riding a left-hander at 35 deg, front tucks at t = 3.0 s, bike falls on its
      // left side, hits the ground, slides and spins, comes to rest at ~84 deg.
      tr.speed = 12.0f;
      const bool riding = t < 3.0f;
      tr.coordinated = riding;
      const float settle = 1.0f - ss(t, 5.0f, 5.5f);
      tr.rollDeg = -35.0f * ss(t, 1.0f, 2.0f) - 53.0f * ss(t, 3.0f, 3.35f)
                   + 4.0f * ss(t, 3.35f, 3.6f)
                   + 3.0f * sinf(2.0f * kPi * 1.3f * t) * ss(t, 3.4f, 3.6f) * settle;
      tr.pitchUpDeg = 3.0f * ss(t, 3.3f, 4.0f);
      if (t > 3.1f) tr.yawExtraDps = 80.0f * expf(-(t - 3.1f) / 1.5f) * settle;
      tr.impulse = vadd(vscale(v3(-1.5f, -6.0f, 1.0f), hs(t, 3.35f, 0.04f)),
                        vscale(v3(0.5f, -2.5f, 0.5f), hs(t, 3.5f, 0.04f)));
      tr.aLongG = -0.7f * hold(t, 3.4f, 3.5f, 5.0f, 5.5f);
      if (riding) { tr.noiseAcc = kNoiseRideAcc; tr.noiseGyro = kNoiseRideGyro; }
      else if (t < 5.5f) { tr.noiseAcc = kNoiseCrashAcc; tr.noiseGyro = kNoiseCrashGyro; }
      else { tr.noiseAcc = kNoiseParkedAcc * 2.0f; tr.noiseGyro = kNoiseParkedGyro * 2.0f; }
      break;
    }

    case SIM_HIGHSIDE: {
      // Right-hander at 20 deg, rear regains grip at t = 3.0 s and throws the bike
      // over to the left, airborne, 9 g impact, tumbles and slides, rests at ~95 deg.
      tr.speed = 18.0f;
      const bool riding = t < 3.0f;
      tr.coordinated = riding;
      tr.rollDeg = 20.0f * ss(t, 1.0f, 2.0f) - 130.0f * ss(t, 3.0f, 3.35f) - 155.0f * ss(t, 3.5f, 4.5f);
      tr.pitchUpDeg = 15.0f * hs(t, 3.0f, 0.5f) + 10.0f * hs(t, 3.5f, 1.0f) + 4.0f * ss(t, 4.5f, 5.0f);
      tr.aVertG = -0.8f * hold(t, 3.12f, 3.18f, 3.38f, 3.42f);
      tr.impulse = vadd(vscale(v3(-2.0f, -7.5f, 3.0f), hs(t, 3.42f, 0.03f)),
                        vscale(v3(1.0f, 3.0f, -2.0f), hs(t, 3.9f, 0.04f)));
      tr.yawExtraDps = 120.0f * ss(t, 3.4f, 3.6f) * expf(-(t > 3.6f ? t - 3.6f : 0.0f) / 1.2f) *
                       (1.0f - ss(t, 5.8f, 6.3f));
      tr.aLongG = -0.8f * hold(t, 4.4f, 4.6f, 5.8f, 6.2f);
      if (riding) { tr.noiseAcc = kNoiseRideAcc; tr.noiseGyro = kNoiseRideGyro; }
      else if (t < 6.2f) { tr.noiseAcc = kNoiseCrashAcc; tr.noiseGyro = kNoiseCrashGyro; }
      else { tr.noiseAcc = kNoiseParkedAcc * 2.0f; tr.noiseGyro = kNoiseParkedGyro * 2.0f; }
      break;
    }

    case SIM_TIPOVER_PARKED: {
      // Engine off on the side stand (-12 deg) for 35 s, then knocked over.
      tr.noiseAcc = kNoiseParkedAcc; tr.noiseGyro = kNoiseParkedGyro;
      const float u = clampf((t - 35.0f) / 0.6f, 0.0f, 1.0f);
      tr.rollDeg = -12.0f - 74.0f * u * u + 2.0f * hs(t, 35.6f, 0.15f);
      tr.impulse = vscale(v3(0.3f, -3.0f, 0.8f), hs(t, 35.6f, 0.04f));
      if (t > 34.9f && t < 36.0f) { tr.noiseAcc = 0.05f; tr.noiseGyro = 1.0f; }
      break;
    }

    case SIM_DISCONNECT:
      tr.noiseAcc = kNoiseIdleAcc; tr.noiseGyro = kNoiseIdleGyro;
      tr.fail = t >= 2.0f && t < 8.0f;
      break;

    default:
      break;
  }
}

float ImuSimulator::truthRollDeg(uint32_t nowMs) {
  Truth tr;
  evaluate((nowMs - startMs_) * 0.001f, tr);
  return wrapPi(tr.rollDeg * DEG2RAD) * RAD2DEG;
}

bool ImuSimulator::next(uint32_t nowMs, ImuSample& out) {
  const float t = (nowMs - startMs_) * 0.001f;
  Truth tr, a, b;
  evaluate(t, tr);
  if (tr.fail) return false;

  // Euler angle rates by central difference of the ground-truth trajectory.
  const float h = 0.005f;
  evaluate(t - h, a);
  evaluate(t + h, b);
  const float phi = tr.rollDeg * DEG2RAD;
  const float theta = -tr.pitchUpDeg * DEG2RAD;             // internal: + = nose down
  const float phiDot = (b.rollDeg - a.rollDeg) * DEG2RAD / (2.0f * h);
  const float thetaDot = -(b.pitchUpDeg - a.pitchUpDeg) * DEG2RAD / (2.0f * h);

  float psiDot = tr.yawExtraDps * DEG2RAD;
  float aLat = 0;
  if (tr.coordinated && tr.speed > 0.5f) {
    // Coordinated turn: lean balances centripetal acceleration (tan(lean) = v*yawRate/g).
    const float tp = tanf(phi);
    aLat = -tp;
    psiDot += -kG * tp / tr.speed;
  }

  // Specific force in the heading frame -> vehicle frame: f_b = Rx(phi)^T * Ry(theta)^T * f_h
  const float fx = tr.aLongG, fy = aLat, fz = 1.0f + tr.aVertG;
  const float st = sinf(theta), ct = cosf(theta), sp = sinf(phi), cp = cosf(phi);
  const float ux = ct * fx - st * fz, uy = fy, uz = st * fx + ct * fz;
  Vec3 f = v3(ux, cp * uy + sp * uz, -sp * uy + cp * uz);
  f = vadd(f, tr.impulse);

  // Body angular rates from Euler rates (ZYX).
  const float p = phiDot - psiDot * st;
  const float q = thetaDot * cp + psiDot * ct * sp;
  const float r = -thetaDot * sp + psiDot * ct * cp;

  const float range = (float)MPU_ACCEL_RANGE_G;
  out.tMs = nowMs;
  out.acc = v3(clampf(f.x + tr.noiseAcc * gauss(), -range, range),
               clampf(f.y + tr.noiseAcc * gauss(), -range, range),
               clampf(f.z + tr.noiseAcc * gauss(), -range, range));
  out.gyro = v3(p * RAD2DEG + tr.noiseGyro * gauss(), q * RAD2DEG + tr.noiseGyro * gauss(),
                r * RAD2DEG + tr.noiseGyro * gauss());
  out.saturated = fabsf(out.acc.x) >= range || fabsf(out.acc.y) >= range || fabsf(out.acc.z) >= range;
  return true;
}
