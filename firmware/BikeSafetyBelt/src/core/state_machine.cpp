#include "state_machine.h"
#include "../hardware/button.h"

#include <string.h>

void SafetyController::begin(ControllerListener* l, const ControllerConfig* cfg, uint16_t bootNo, uint32_t now) {
  l_ = l;
  cfg_ = cfg;
  bootNo_ = bootNo;
  state_ = SystemState::BOOT;
  stateSinceMs_ = now;
  memset(&inc_, 0, sizeof(inc_));
  incidentOpen_ = false;
  calibrating_ = false;
  imuHealthy_ = true;
  lockout_ = false;
  seq_ = 0;
}

FStr SafetyController::stateName(SystemState s) {
  switch (s) {
    case SystemState::BOOT: return FS("BOOT");
    case SystemState::CALIBRATING: return FS("CALIBRATING");
    case SystemState::SAFE: return FS("SAFE");
    case SystemState::MONITORING: return FS("MONITORING");
    case SystemState::POSSIBLE_CRASH: return FS("POSSIBLE_CRASH");
    case SystemState::CRASH_DETECTED: return FS("CRASH_DETECTED");
    case SystemState::USER_CANCELLED: return FS("USER_CANCELLED");
    case SystemState::SOS_SENT: return FS("SOS_SENT");
    case SystemState::RECOVERY: return FS("RECOVERY");
    case SystemState::MANUAL_TEST: return FS("MANUAL_TEST");
    case SystemState::IMU_FAULT: return FS("IMU_FAULT");
  }
  return FS("UNKNOWN");
}

FStr SafetyController::reasonName(StateReason r) {
  switch (r) {
    case StateReason::BOOT: return FS("boot");
    case StateReason::CALIBRATION: return FS("calibration complete");
    case StateReason::DETECTOR: return FS("crash detector");
    case StateReason::DETECTOR_CLEARED: return FS("motion evaluated as normal");
    case StateReason::BUTTON: return FS("button");
    case StateReason::APP: return FS("app");
    case StateReason::COUNTDOWN_EXPIRED: return FS("countdown expired");
    case StateReason::TIMEOUT: return FS("timeout");
    case StateReason::IMU_FAULT: return FS("IMU not responding");
    case StateReason::IMU_RECOVERED: return FS("IMU reconnected");
    case StateReason::IMU_LOST_DURING_EVENT: return FS("IMU lost during possible crash");
    case StateReason::RESTORED: return FS("restored after restart");
    case StateReason::RESET: return FS("explicit reset");
  }
  return FS("?");
}

bool SafetyController::inIncidentState() const {
  switch (state_) {
    case SystemState::POSSIBLE_CRASH:
    case SystemState::CRASH_DETECTED:
    case SystemState::USER_CANCELLED:
    case SystemState::SOS_SENT:
    case SystemState::RECOVERY:
    case SystemState::MANUAL_TEST:
      return true;
    default:
      return false;
  }
}

SystemState SafetyController::idleState() const {
  if (!imuHealthy_) return SystemState::IMU_FAULT;
  if (calibrating_) return SystemState::CALIBRATING;
  return SystemState::SAFE;
}

void SafetyController::transition(SystemState to, StateReason why) {
  if (to == state_) return;
  const SystemState from = state_;
  state_ = to;
  if (l_) l_->onStateChange(from, to, why);
}

void SafetyController::openIncident(IncidentType type, uint32_t now) {
  memset(&inc_, 0, sizeof(inc_));
  seq_ = (uint16_t)(seq_ >= 999 ? 1 : seq_ + 1);
  inc_.id = (uint32_t)bootNo_ * 1000UL + seq_;
  inc_.uptimeMs = now;
  inc_.bootNo = bootNo_;
  inc_.type = (uint8_t)type;
  inc_.outcome = OUTCOME_OPEN;
  incidentOpen_ = true;
}

void SafetyController::publishIncident() {
  if (l_) l_->onIncidentUpdate(inc_);
}

void SafetyController::closeIncident(uint8_t outcome) {
  inc_.outcome = outcome;
  publishIncident();
  incidentOpen_ = false;
}

static int16_t sat16(float v) { return (int16_t)(v > 32767.0f ? 32767 : (v < -32767.0f ? -32767 : v)); }

void SafetyController::fillFromReport(const DetectionReport& rep) {
  const uint8_t conf = (uint8_t)(clampf(rep.confidence, 0, 1) * 100.0f + 0.5f);
  if (conf > inc_.confidence || rep.verdict != DetVerdict::NONE) inc_.confidence = conf;
  const int16_t acc = sat16(rep.peakAccG * 100.0f);
  const int16_t gyr = sat16(rep.peakGyroDps);
  const int16_t lean = sat16(rep.peakTiltDeg * 10.0f);
  if (acc > inc_.maxAccCg) inc_.maxAccCg = acc;
  if (gyr > inc_.maxGyroDps) inc_.maxGyroDps = gyr;
  if (lean > inc_.maxLeanDd) inc_.maxLeanDd = lean;
}

void SafetyController::snapshotLocation() {
  int32_t lat = 0, lon = 0;
  if (l_ && l_->currentLocation(lat, lon)) {
    inc_.latE7 = lat;
    inc_.lonE7 = lon;
    inc_.flags |= INC_GPS_VALID;
  }
}

void SafetyController::enterCrashDetected(uint32_t now, StateReason why) {
  inc_.flags |= INC_SUSPECTED | INC_CONFIRMED;
  snapshotLocation();
  countdown_.start(now, cfg_->countdownS);
  stateSinceMs_ = now;
  publishIncident();
  transition(SystemState::CRASH_DETECTED, why);
}

void SafetyController::issueSos(uint32_t now) {
  countdown_.stop();
  inc_.flags |= INC_SOS_SENT;
  inc_.outcome = OUTCOME_SOS;
  snapshotLocation();
  publishIncident();
  stateSinceMs_ = now;
  if (l_) l_->onSosIssued(inc_);
  transition(SystemState::SOS_SENT, StateReason::COUNTDOWN_EXPIRED);
}

void SafetyController::startCalibration(uint32_t now) {
  calibrating_ = true;
  if (state_ == SystemState::BOOT || state_ == SystemState::SAFE || state_ == SystemState::MONITORING) {
    stateSinceMs_ = now;
    transition(idleState(), StateReason::BOOT);
  }
}

void SafetyController::calibrationFinished(uint32_t now) {
  calibrating_ = false;
  if (state_ == SystemState::CALIBRATING) {
    stateSinceMs_ = now;
    transition(idleState(), StateReason::CALIBRATION);
  }
}

void SafetyController::setImuHealthy(bool ok, uint32_t now) {
  if (ok == imuHealthy_) return;
  imuHealthy_ = ok;
  if (!ok) {
    switch (state_) {
      case SystemState::BOOT:
      case SystemState::CALIBRATING:
      case SystemState::SAFE:
      case SystemState::MONITORING:
        stateSinceMs_ = now;
        transition(SystemState::IMU_FAULT, StateReason::IMU_FAULT);
        break;
      case SystemState::POSSIBLE_CRASH:
        // Sensor lost in the middle of a suspected crash: fail towards alerting.
        inc_.flags |= INC_IMU_LOST;
        enterCrashDetected(now, StateReason::IMU_LOST_DURING_EVENT);
        break;
      default:
        if (incidentOpen_) {
          inc_.flags |= INC_IMU_LOST;
          publishIncident();
        }
        break;
    }
  } else if (state_ == SystemState::IMU_FAULT) {
    stateSinceMs_ = now;
    transition(idleState(), StateReason::IMU_RECOVERED);
  }
}

void SafetyController::onDetector(uint8_t ev, const DetectionReport& rep, bool simulated, uint32_t now) {
  const IncidentType type = simulated ? IncidentType::SIMULATION : IncidentType::AUTO;
  const bool normal = state_ == SystemState::SAFE || state_ == SystemState::MONITORING;

  if ((ev & DEV_CANDIDATE) && state_ == SystemState::SAFE) {
    stateSinceMs_ = now;
    transition(SystemState::MONITORING, StateReason::DETECTOR);
  }

  if ((ev & DEV_POSSIBLE) && (normal || state_ == SystemState::MANUAL_TEST)) {
    if (state_ == SystemState::MANUAL_TEST) closeIncident(OUTCOME_TEST_CLEARED);   // real event wins
    openIncident(type, now);
    inc_.flags |= INC_SUSPECTED;
    fillFromReport(rep);
    publishIncident();
    stateSinceMs_ = now;
    transition(SystemState::POSSIBLE_CRASH, StateReason::DETECTOR);
  }

  if (ev & DEV_VERDICT) {
    if (rep.verdict == DetVerdict::CRASH &&
        (normal || state_ == SystemState::POSSIBLE_CRASH || state_ == SystemState::MANUAL_TEST)) {
      if (state_ == SystemState::MANUAL_TEST) closeIncident(OUTCOME_TEST_CLEARED);
      if (!incidentOpen_ || state_ == SystemState::MANUAL_TEST) openIncident(type, now);
      fillFromReport(rep);
      enterCrashDetected(now, StateReason::DETECTOR);
    } else if (rep.verdict == DetVerdict::DISMISSED) {
      if (state_ == SystemState::POSSIBLE_CRASH) {
        fillFromReport(rep);
        closeIncident(OUTCOME_DISMISSED);
        stateSinceMs_ = now;
        transition(idleState(), StateReason::DETECTOR_CLEARED);
      } else if (state_ == SystemState::MONITORING) {
        stateSinceMs_ = now;
        transition(idleState(), StateReason::DETECTOR_CLEARED);
      }
    }
  }
}

void SafetyController::detectorReset(uint32_t now) {
  if (state_ == SystemState::MONITORING) {
    stateSinceMs_ = now;
    transition(idleState(), StateReason::DETECTOR_CLEARED);
  } else if (state_ == SystemState::POSSIBLE_CRASH) {
    closeIncident(OUTCOME_DISMISSED);
    stateSinceMs_ = now;
    transition(idleState(), StateReason::DETECTOR_CLEARED);
  }
}

void SafetyController::onButton(uint8_t ev, uint32_t now) {
  if (ev & BTN_PRESS) {
    if (lockout_ && (int32_t)(now - lockoutUntilMs_) < 0) return;   // just cancelled: ignore bounce/double press
    switch (state_) {
      case SystemState::SAFE:
      case SystemState::MONITORING:
      case SystemState::CALIBRATING:
      case SystemState::IMU_FAULT:
        openIncident(IncidentType::MANUAL_TEST, now);
        publishIncident();
        stateSinceMs_ = now;
        transition(SystemState::MANUAL_TEST, StateReason::BUTTON);
        break;
      case SystemState::MANUAL_TEST:
      case SystemState::POSSIBLE_CRASH:
      case SystemState::CRASH_DETECTED:
        cancel(now, StateReason::BUTTON);
        break;
      default:
        break;   // SOS_SENT / RECOVERY need a long press
    }
  }
  if ((ev & BTN_LONG) && (state_ == SystemState::SOS_SENT || state_ == SystemState::RECOVERY)) reset(now);
}

bool SafetyController::cancel(uint32_t now, StateReason src) {
  if (state_ == SystemState::POSSIBLE_CRASH || state_ == SystemState::CRASH_DETECTED) {
    countdown_.stop();
    inc_.flags |= INC_CANCELLED;
    closeIncident(OUTCOME_CANCELLED);
    lockout_ = true;
    lockoutUntilMs_ = now + cfg_->buttonLockoutMs;
    stateSinceMs_ = now;
    transition(SystemState::USER_CANCELLED, src);
    return true;
  }
  if (state_ == SystemState::MANUAL_TEST) {
    closeIncident(OUTCOME_TEST_CLEARED);
    lockout_ = true;
    lockoutUntilMs_ = now + cfg_->buttonLockoutMs / 4;
    stateSinceMs_ = now;
    transition(idleState(), src);
    return true;
  }
  return false;
}

bool SafetyController::reset(uint32_t now) {
  if (state_ != SystemState::SOS_SENT && state_ != SystemState::RECOVERY) return false;
  if (inc_.outcome == OUTCOME_SOS) inc_.outcome = OUTCOME_RESOLVED;
  publishIncident();
  incidentOpen_ = false;
  lockout_ = true;
  lockoutUntilMs_ = now + cfg_->buttonLockoutMs;
  stateSinceMs_ = now;
  transition(idleState(), StateReason::RESET);
  return true;
}

bool SafetyController::sosAcknowledged(uint32_t id) {
  if (inc_.id != id || !(inc_.flags & INC_SOS_SENT)) return false;
  if (!(inc_.flags & INC_SOS_ACKED)) {
    inc_.flags |= INC_SOS_ACKED;
    publishIncident();
  }
  return true;
}

void SafetyController::restore(const IncidentRecord& rec, bool sosAlreadyIssued, uint32_t now) {
  inc_ = rec;
  inc_.flags |= INC_RESUMED;
  incidentOpen_ = true;
  if (sosAlreadyIssued) {
    inc_.outcome = OUTCOME_SOS;
    publishIncident();
    stateSinceMs_ = now;
    transition(SystemState::RECOVERY, StateReason::RESTORED);
  } else {
    inc_.outcome = OUTCOME_OPEN;
    enterCrashDetected(now, StateReason::RESTORED);
  }
}

void SafetyController::tick(uint32_t now) {
  if (lockout_ && (int32_t)(now - lockoutUntilMs_) >= 0) lockout_ = false;
  switch (state_) {
    case SystemState::CRASH_DETECTED:
      if (countdown_.expired(now)) issueSos(now);
      break;
    case SystemState::USER_CANCELLED:
      if (elapsed(now, stateSinceMs_, cfg_->cancelDisplayMs)) {
        stateSinceMs_ = now;
        transition(idleState(), StateReason::TIMEOUT);
      }
      break;
    case SystemState::SOS_SENT:
      if (elapsed(now, stateSinceMs_, cfg_->sosAlarmMs)) {
        stateSinceMs_ = now;
        transition(SystemState::RECOVERY, StateReason::TIMEOUT);
      }
      break;
    default:
      break;
  }
}
