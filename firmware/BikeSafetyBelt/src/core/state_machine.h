// System state machine (pure logic, unit-tested natively).
//
//  BOOT ─► CALIBRATING ─► SAFE ◄──────────────────────────────┐
//                          │ ▲ candidate / cleared             │ (after 3 s)
//                          ▼ │                                 │
//                       MONITORING ──possible──► POSSIBLE_CRASH ──cancel──► USER_CANCELLED
//                          │                         │  │cleared ─► SAFE       ▲
//                          └──crash verdict──► CRASH_DETECTED (countdown) ──cancel┘
//                                                    │ expired
//                                                    ▼
//                                                SOS_SENT ──(alarm period)──► RECOVERY
//                                                    └──── explicit RESET (long press / app) ──► SAFE
//  MANUAL_TEST  : button press in SAFE/MONITORING/IMU_FAULT; second press clears. Never sends SOS.
//  IMU_FAULT    : sensor lost. If it is lost DURING a possible crash, the system
//                 escalates to CRASH_DETECTED (an impact can rip the sensor wiring),
//                 the rider can still cancel.
#pragma once

#include "types.h"
#include "../detection/crash_detector.h"
#include "../emergency/countdown.h"

enum class StateReason : uint8_t {
  BOOT = 0,
  CALIBRATION,
  DETECTOR,
  DETECTOR_CLEARED,
  BUTTON,
  APP,
  COUNTDOWN_EXPIRED,
  TIMEOUT,
  IMU_FAULT,
  IMU_RECOVERED,
  IMU_LOST_DURING_EVENT,
  RESTORED,
  RESET
};

struct ControllerConfig {
  uint8_t countdownS;
  uint32_t cancelDisplayMs;
  uint32_t sosAlarmMs;
  uint32_t buttonLockoutMs;
};

class ControllerListener {
 public:
  virtual void onStateChange(SystemState from, SystemState to, StateReason why) = 0;
  virtual void onIncidentUpdate(const IncidentRecord& rec) = 0;
  virtual void onSosIssued(const IncidentRecord& rec) = 0;
  // Fills a valid GPS location; returns false if none (never invent coordinates).
  virtual bool currentLocation(int32_t& latE7, int32_t& lonE7) = 0;
};

class SafetyController {
 public:
  void begin(ControllerListener* l, const ControllerConfig* cfg, uint16_t bootNo, uint32_t now);

  void startCalibration(uint32_t now);
  void calibrationFinished(uint32_t now);
  void setImuHealthy(bool ok, uint32_t now);
  void onDetector(uint8_t ev, const DetectionReport& rep, bool simulated, uint32_t now);
  void onButton(uint8_t btnEvents, uint32_t now);
  // The detector was reset (simulation start/end, profile change): drop a pending evaluation.
  void detectorReset(uint32_t now);
  bool cancel(uint32_t now, StateReason src);
  bool reset(uint32_t now);
  bool sosAcknowledged(uint32_t id);
  void restore(const IncidentRecord& rec, bool sosAlreadyIssued, uint32_t now);
  void tick(uint32_t now);

  SystemState state() const { return state_; }
  uint8_t countdownRemaining(uint32_t now) const { return countdown_.remaining(now); }
  bool incidentOpen() const { return incidentOpen_; }
  const IncidentRecord& incident() const { return inc_; }
  bool incidentSimulated() const { return inc_.type == (uint8_t)IncidentType::SIMULATION; }
  bool imuHealthy() const { return imuHealthy_; }
  bool inIncidentState() const;

  static FStr stateName(SystemState s);
  static FStr reasonName(StateReason r);

 private:
  void transition(SystemState to, StateReason why);
  SystemState idleState() const;
  void openIncident(IncidentType type, uint32_t now);
  void closeIncident(uint8_t outcome);
  void fillFromReport(const DetectionReport& rep);
  void snapshotLocation();
  void enterCrashDetected(uint32_t now, StateReason why);
  void issueSos(uint32_t now);
  void publishIncident();

  ControllerListener* l_ = nullptr;
  const ControllerConfig* cfg_ = nullptr;
  SystemState state_ = SystemState::BOOT;
  uint32_t stateSinceMs_ = 0;
  uint32_t lockoutUntilMs_ = 0;
  bool lockout_ = false;
  bool calibrating_ = false;
  bool imuHealthy_ = true;
  uint16_t bootNo_ = 0;
  uint16_t seq_ = 0;
  bool incidentOpen_ = false;
  IncidentRecord inc_;
  Countdown countdown_;
};
