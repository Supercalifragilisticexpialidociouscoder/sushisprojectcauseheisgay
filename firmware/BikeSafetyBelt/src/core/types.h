// Shared data types. Vehicle frame convention used everywhere after the
// mounting transform:  X = forward,  Y = left,  Z = up  (right-handed).
//   roll  > 0  : bike leaning to the RIGHT
//   pitch > 0  : nose UP
//   acceleration is specific force in g (reads +1 g on Z when upright & still)
//   angular rate in deg/s
#pragma once

#include "platform.h"

static const float DEG2RAD = 0.017453292519943295f;
static const float RAD2DEG = 57.29577951308232f;

struct Vec3 {
  float x, y, z;
};

inline Vec3 v3(float x, float y, float z) { Vec3 v; v.x = x; v.y = y; v.z = z; return v; }
inline float vlen(const Vec3& v) { return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z); }
inline Vec3 vsub(const Vec3& a, const Vec3& b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
inline Vec3 vadd(const Vec3& a, const Vec3& b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
inline Vec3 vscale(const Vec3& a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
inline float vdot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 vcross(const Vec3& a, const Vec3& b) {
  return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
// Linear 0..1 score: 0 at or below `lo`, 1 at or above `hi`.
inline float ramp(float v, float lo, float hi) {
  if (hi <= lo) return v >= hi ? 1.0f : 0.0f;
  return clampf((v - lo) / (hi - lo), 0.0f, 1.0f);
}
inline float wrapPi(float a) {
  while (a > 3.14159265f) a -= 6.2831853f;
  while (a < -3.14159265f) a += 6.2831853f;
  return a;
}

// One calibrated IMU sample in the vehicle frame.
struct ImuSample {
  uint32_t tMs;
  float dt;          // seconds since the previous sample
  Vec3 acc;          // g
  Vec3 gyro;         // deg/s
  bool saturated;    // a raw accelerometer axis hit full scale
};

// Sensor axis selectors for MOUNT_FORWARD_AXIS / MOUNT_UP_AXIS
enum AxisSel : uint8_t { AXIS_POS_X = 0, AXIS_NEG_X, AXIS_POS_Y, AXIS_NEG_Y, AXIS_POS_Z, AXIS_NEG_Z };

// Vehicle profiles
enum ProfileId : uint8_t { PROFILE_SCOOTER = 0, PROFILE_STANDARD = 1, PROFILE_SPORT = 2, PROFILE_COUNT = 3 };

// System state machine states
enum class SystemState : uint8_t {
  BOOT = 0,
  CALIBRATING,
  SAFE,
  MONITORING,
  POSSIBLE_CRASH,
  CRASH_DETECTED,
  USER_CANCELLED,
  SOS_SENT,
  RECOVERY,
  MANUAL_TEST,
  IMU_FAULT
};

enum class IncidentType : uint8_t { AUTO = 0, MANUAL_TEST = 1, SIMULATION = 2 };

// IncidentRecord.flags
enum IncidentFlag : uint8_t {
  INC_SUSPECTED = 0x01,   // reached POSSIBLE_CRASH
  INC_CONFIRMED = 0x02,   // crash confidence crossed threshold (CRASH_DETECTED)
  INC_CANCELLED = 0x04,   // rider cancelled
  INC_SOS_SENT = 0x08,    // countdown expired, SOS issued
  INC_SOS_ACKED = 0x10,   // app acknowledged receipt of the SOS
  INC_GPS_VALID = 0x20,   // lat/lon are from a valid GPS fix
  INC_RESUMED = 0x40,     // resumed after a controller restart
  INC_IMU_LOST = 0x80     // IMU stopped responding during the incident
};

// Compact, persistable incident record (stored in EEPROM ring + sent to app).
struct IncidentRecord {
  uint32_t id;            // bootCount * 100 + sequence
  uint32_t uptimeMs;      // device uptime when the incident started
  uint16_t bootNo;
  uint8_t type;           // IncidentType
  uint8_t flags;          // IncidentFlag
  uint8_t confidence;     // %
  uint8_t outcome;        // IncidentOutcome
  int16_t maxAccCg;       // max resultant acceleration, centi-g
  int16_t maxGyroDps;     // max angular velocity
  int16_t maxLeanDd;      // max tilt from upright, deci-degrees
  int32_t latE7;          // degrees * 1e7 (only if INC_GPS_VALID)
  int32_t lonE7;
};

enum IncidentOutcome : uint8_t {
  OUTCOME_OPEN = 0,
  OUTCOME_DISMISSED = 1,     // detector cleared the suspicion
  OUTCOME_CANCELLED = 2,     // rider said "I'm OK"
  OUTCOME_SOS = 3,           // SOS issued
  OUTCOME_RESOLVED = 4,      // explicit reset after SOS
  OUTCOME_TEST_CLEARED = 5   // manual test cleared
};
