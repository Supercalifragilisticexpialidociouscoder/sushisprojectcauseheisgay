// Attitude estimation: adaptive complementary filter.
//
//   angle = alpha * (angle + euler_rate * dt) + (1 - alpha) * reference_angle
//
// The reference angle is chosen per sample, because on a motorcycle the
// accelerometer does NOT measure lean while cornering (in a coordinated turn
// gravity + centripetal acceleration point straight down the bike's own axis,
// so an accelerometer-only "lean" reads ~0 deg at 45 deg of real lean):
//   * ACCEL : bike not cornering, |a| ~ 1 g, low rotation  -> accelerometer angles
//   * TURN  : steady cornering -> lean from turn kinematics, atan(gyroY / gyroZ)
//             (yaw rate projects onto the leaned bike's Y and Z axes)
//   * GYRO  : anything else (impacts, braking, crash dynamics) -> gyro only
// Gyro integration uses proper ZYX Euler-rate kinematics so yaw rotation during
// a turn does not leak into roll/pitch.
#pragma once

#include "../core/types.h"
#include "../config/vehicle_profiles.h"

enum class AttitudeRef : uint8_t { NONE = 0, ACCEL = 1, TURN = 2, GYRO = 3 };

struct Attitude {
  float rollDeg;       // + leaning right
  float pitchDeg;      // + nose up
  float tiltDeg;       // angle between bike's up axis and vertical (0 = upright)
  float rollRateDps;   // Euler angle rates (exclude yaw)
  float pitchRateDps;
  float yawRateDps;
  AttitudeRef ref;
  bool valid;
};

class SensorFusion {
 public:
  void begin(const FusionConfig* cfg);
  void reset();                     // next sample re-initialises from accelerometer
  void update(const ImuSample& s);
  const Attitude& attitude() const { return att_; }
  // Expected gravity direction in the vehicle frame for the current attitude.
  Vec3 gravityBody() const;

 private:
  void publish(float phiDot, float thetaDot, float psiDot, AttitudeRef ref);

  const FusionConfig* cfg_ = nullptr;
  float phi_ = 0, theta_ = 0;       // rad; theta_ positive = nose DOWN (ZYX, FLU)
  Vec3 accLp_ = {0, 0, 1};
  bool init_ = false;
  Attitude att_ = {0, 0, 0, 0, 0, 0, AttitudeRef::NONE, false};
};
