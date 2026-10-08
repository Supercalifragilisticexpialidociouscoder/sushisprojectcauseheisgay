// Deterministic IMU scenario generator for the software TEST MODE.
//
// Each scenario defines the "true" motion of the vehicle (roll, pitch, yaw rate,
// linear accelerations, impacts). The generator converts it into the specific
// force and body angular rates an ideal MPU-6050 would measure — using the same
// frame conventions and rigid-body kinematics as the real pipeline — and adds
// sensor/road noise. The samples are fed through the REAL fusion + detector +
// state machine, so a test exercises exactly the code that runs on the bike.
//
// Anything produced while a simulation runs is flagged "sim" and an SOS raised
// by a simulation is never forwarded to emergency contacts by the app.
#pragma once

#include "../core/types.h"

enum SimScenario : uint8_t {
  SIM_NONE = 0,
  SIM_STATIONARY = 1,       // test 1
  SIM_NORMAL_LEAN = 2,      // test 2  (side stand / walking the bike)
  SIM_CORNERING = 3,        // test 3
  SIM_BRAKING = 4,          // test 4
  SIM_SPEED_BREAKER = 5,    // test 5
  SIM_SUDDEN_ROTATION = 6,  // test 6  (emergency swerve)
  SIM_CRASH = 7,            // test 7  (low-side crash, combined motion)
  SIM_DISCONNECT = 9,       // test 9  (IMU stops responding for 6 s)
  SIM_POTHOLE = 14,         // extra   (sharp vertical impacts)
  SIM_TIPOVER_PARKED = 15,  // extra   (parked bike knocked over)
  SIM_HIGHSIDE = 16         // extra   (violent high-side crash with tumble)
};

class ImuSimulator {
 public:
  static bool valid(uint8_t id);
  static FStr name(uint8_t id);
  static uint32_t durationMs(uint8_t id);

  bool start(uint8_t id, uint32_t nowMs, uint32_t seed);
  void stop() { id_ = SIM_NONE; }
  bool active() const { return id_ != SIM_NONE; }
  uint8_t scenario() const { return id_; }
  bool finished(uint32_t nowMs) const;
  // Fills `out` (vehicle frame, calibrated units). Returns false when the
  // scenario simulates a failed sensor read.
  bool next(uint32_t nowMs, ImuSample& out);

  // Ground-truth lean for the current time (used by the native tests).
  float truthRollDeg(uint32_t nowMs);

 private:
  struct Truth {
    float rollDeg, pitchUpDeg;   // attitude
    float yawExtraDps;           // yaw rate on top of coordinated-turn yaw
    float aLongG, aVertG;        // heading-frame dynamic acceleration
    Vec3 impulse;                // body-frame impact specific force (g)
    float noiseAcc, noiseGyro;   // 1-sigma noise
    float speed;                 // m/s (coordinated turn kinematics)
    bool coordinated;            // lateral accel/yaw follow lean (normal riding)
    bool fail;                   // simulated I2C failure
  };
  void evaluate(float t, Truth& tr) const;
  float gauss();

  uint8_t id_ = SIM_NONE;
  uint32_t startMs_ = 0;
  uint32_t rng_ = 1;
};
