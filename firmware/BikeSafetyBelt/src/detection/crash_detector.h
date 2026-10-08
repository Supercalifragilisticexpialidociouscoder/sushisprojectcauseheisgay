// Multi-indicator crash detector with temporal logic.
//
//  IDLE ──(K of last M samples anomalous)──► ACTIVE ──(motion calms down)──► SETTLE
//    ▲                                         │  peaks tracked                 │ wait for rest,
//    └──────────── verdict (CRASH / DISMISSED) ◄────────────────────────────────┘ measure resting tilt
//
// Five indicators are scored 0..1 against the active vehicle profile:
//   1 LEAN    peak tilt from upright
//   2 RATE    peak angular velocity
//   3 IMPACT  peak resultant acceleration (down-weighted if purely vertical = road bump)
//   4 CHANGE  orientation change within 0.5 s, or sustained high rotation (tumbling)
//   5 POST    resting tilt after the event (only counted after a MAJOR motion event)
// confidence = weighted sum (x parked factor). A crash verdict needs
// confidence >= crash threshold AND at least DET_MIN_INDICATORS indicators >= 50 %.
// No single indicator can produce a crash verdict on its own.
#pragma once

#include "../core/types.h"
#include "../config/vehicle_profiles.h"
#include "../sensor/sensor_fusion.h"

enum class DetPhase : uint8_t { IDLE = 0, ACTIVE = 1, SETTLE = 2 };
enum class DetVerdict : uint8_t { NONE = 0, CRASH = 1, DISMISSED = 2 };

enum DetIndicator : uint8_t { IND_LEAN = 0, IND_RATE, IND_IMPACT, IND_CHANGE, IND_POST, IND_COUNT };

enum DetTrigger : uint8_t {
  TRIG_IMPACT = 0x01,
  TRIG_GYRO = 0x02,
  TRIG_TILT_RATE = 0x04,
  TRIG_SATURATION = 0x08
};

enum DetEvent : uint8_t {
  DEV_NONE = 0,
  DEV_CANDIDATE = 0x01,   // abnormal motion candidate started (silent)
  DEV_POSSIBLE = 0x02,    // live confidence crossed the possible-crash threshold
  DEV_VERDICT = 0x04,     // evaluation finished; see report().verdict
  DEV_SECONDARY = 0x08    // new violent motion while settling (secondary impact)
};

struct DetectionReport {
  uint32_t startMs;
  uint32_t endMs;
  uint8_t triggers;           // DetTrigger bits seen during the event
  float peakAccG;
  float peakGyroDps;
  float peakTiltDeg;
  float peakRollDeg;          // signed roll at peak tilt
  float peakTiltChangeDeg;    // max orientation change within 0.5 s
  float sustainedRateMs;      // time spent above the profile's normal gyro rate
  float baselineTiltDeg;      // tilt ~1 s before the event
  float restTiltDeg;          // tilt after coming to rest (or at timeout)
  bool settled;
  bool verticalImpact;        // peak impact mostly along bike's vertical axis
  bool parked;                // vehicle was completely still before the event
  bool postEligible;          // major motion happened -> post-impact indicator counts
  float scores[IND_COUNT];    // 0..1
  float confidence;           // 0..1
  uint8_t activeIndicators;
  DetVerdict verdict;
};

class CrashDetector {
 public:
  void begin(const DetectorConfig* cfg, const VehicleProfile* profile);
  void setProfile(const VehicleProfile* profile) { prof_ = profile; }
  void reset();                       // drop any event in progress, keep arming state
  void disarmUntilUpright();          // after an incident: ignore until bike is upright again

  uint8_t update(const ImuSample& s, const Attitude& att, const Vec3& gravityBody);

  DetPhase phase() const { return phase_; }
  bool armed() const { return armed_; }
  float liveConfidence() const { return liveConf_; }   // 0..1, decaying peak for display
  bool possibleRaised() const { return possibleRaised_; }
  bool abnormalStaticTilt() const { return staticTiltAbnormal_; }
  const DetectionReport& report() const { return rep_; }
  float tiltChange500ms() const { return tiltChange_; }

 private:
  void startEvent(uint32_t t, uint8_t trig);
  void trackPeaks(const ImuSample& s, const Attitude& att, const Vec3& gBody, float aMag, float wMag);
  void computeScores(bool final, float currentTilt);
  void finish(uint32_t t);
  uint8_t anomalyBits(const ImuSample& s, float aMag, float wMag, float tiltRate) const;

  const DetectorConfig* cfg_ = nullptr;
  const VehicleProfile* prof_ = nullptr;

  DetPhase phase_ = DetPhase::IDLE;
  DetectionReport rep_;

  // 10 Hz tilt history (1 s) for orientation-change and baseline measurement
  static const uint8_t kHist = 10;
  float tiltHist_[kHist];
  uint8_t histHead_ = 0, histCount_ = 0;
  uint32_t lastHistMs_ = 0;
  float tiltChange_ = 0;

  uint8_t anomalyShift_ = 0;     // bit0 = newest sample
  uint8_t pendingTrig_ = 0;
  float preAcc_ = 0, preGyro_ = 0;   // peaks of the anomalous samples before the trigger
  bool preVert_ = false;

  float lpGyro_ = 0, lpAccDev_ = 0;   // low-passed rotation / accel deviation
  bool stillNow_ = false;             // vehicle perfectly still (engine off, parked)
  uint32_t stillStartMs_ = 0;
  bool longStillSeen_ = false;        // has been still for >= parkedMinMs ...
  uint32_t longStillMs_ = 0;          // ... most recently at this time
  uint32_t phaseMs_ = 0;              // start of current phase
  uint32_t calmSinceMs_ = 0;
  uint32_t restSinceMs_ = 0;
  bool calm_ = false, rest_ = false;

  bool armed_ = true;
  uint32_t uprightSinceMs_ = 0;
  bool uprightTiming_ = false;

  float liveConf_ = 0;
  bool possibleRaised_ = false;
  bool staticTiltAbnormal_ = false;
  bool havePrev_ = false;
};
