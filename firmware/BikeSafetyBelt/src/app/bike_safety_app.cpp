#include "bike_safety_app.h"

#include <stdlib.h>
#include <string.h>

static const uint32_t kSamplePeriodUs = 1000000UL / SAMPLE_RATE_HZ;
static const float kNominalDt = 1.0f / (float)SAMPLE_RATE_HZ;

static FStr axisName(uint8_t a) {
  switch (a) {
    case AXIS_POS_X: return FS("+X");
    case AXIS_NEG_X: return FS("-X");
    case AXIS_POS_Y: return FS("+Y");
    case AXIS_NEG_Y: return FS("-Y");
    case AXIS_POS_Z: return FS("+Z");
    case AXIS_NEG_Z: return FS("-Z");
    default: return FS("?");
  }
}

static FStr typeName(uint8_t t) {
  switch (t) {
    case (uint8_t)IncidentType::MANUAL_TEST: return FS("TEST");
    case (uint8_t)IncidentType::SIMULATION: return FS("SIM");
    default: return FS("AUTO");
  }
}

static FStr outcomeName(uint8_t o) {
  switch (o) {
    case OUTCOME_DISMISSED: return FS("DISMISSED");
    case OUTCOME_CANCELLED: return FS("CANCELLED");
    case OUTCOME_SOS: return FS("SOS");
    case OUTCOME_RESOLVED: return FS("RESOLVED");
    case OUTCOME_TEST_CLEARED: return FS("TEST_CLEARED");
    default: return FS("OPEN");
  }
}

static FStr levelWord(float score) {
  if (score < 0.25f) return FS("normal");
  if (score < 0.6f) return FS("elevated");
  return FS("HIGH");
}

static uint8_t pct(float v) { return (uint8_t)(clampf(v, 0, 1) * 100.0f + 0.5f); }

// Case-insensitive match of the first word; on success `p` advances to the arguments.
static bool cmdIs(const char*& p, FStr word) {
  for (size_t i = 0;; ++i) {
    const char w = fsRead(word, i);
    char c = p[i];
    if (!w) {
      if (c == '\0' || c == ' ') {
        p += i;
        while (*p == ' ') ++p;
        return true;
      }
      return false;
    }
    if (c >= 'a' && c <= 'z') c = (char)(c - 32);
    if (c != w) return false;
  }
}

// =============================================================================
// Setup
// =============================================================================
void BikeSafetyApp::begin(RawImuSensor* imu, NvBackend* nv, Transport* t0, Transport* t1, CharSource* gps,
                          uint16_t seed) {
  const uint32_t now = millis();
  gps_ = gps;

  store_.begin(nv, seed);
  StoredSettings& st = store_.settings();
  st.bootCount++;
  store_.saveSettings();
  bootNo_ = (uint16_t)st.bootCount;
  static const char kHex[] = "0123456789ABCDEF";
  for (uint8_t i = 0; i < 4; ++i) deviceName_[4 + i] = kHex[(st.deviceId >> (12 - 4 * i)) & 0x0F];
  deviceName_[8] = '\0';

  loadVehicleProfile(st.profile, profile_);
  link_.begin(t0, t1);

  if (w_.log(FS("BOOT"))) { w_.s(FS("Bike Safety Belt firmware ")).s(FS(FIRMWARE_VERSION)).s(FS(" (" BSB_BOARD_NAME ")")).end(); }
  if (w_.log(FS("BOOT"))) { w_.s(FS("Boot #")).u(bootNo_).s(FS(", device ")).r(deviceName_).end(); }
  if (w_.log(FS("CONFIG"))) {
    w_.s(FS("Profile: ")).s(profileName(st.profile)).s(FS(" | Mount fwd ")).s(axisName(MOUNT_FORWARD_AXIS))
        .s(FS(" up ")).s(axisName(MOUNT_UP_AXIS)).s(FS(" | Accel +/-")).u(MPU_ACCEL_RANGE_G).s(FS(" g | Gyro +/-"))
        .u(MPU_GYRO_RANGE_DPS).s(FS(" deg/s | ")).u(SAMPLE_RATE_HZ).s(FS(" Hz")).end();
  }
  if (w_.log(FS("CONFIG"))) {
    w_.s(FS("GPS module: ")).s(GPS_ENABLED ? FS("enabled") : FS("not installed (location UNAVAILABLE)"))
        .s(FS(" | BLE: ")).s(BLE_ENABLED ? FS("on") : FS("none")).s(FS(" | GSM/SMS: none (alerts go via the app)")).end();
  }

  pipeline_.begin(imu, MOUNT_FORWARD_AXIS, MOUNT_UP_AXIS);
  if (!pipeline_.mountValid() && w_.log(FS("CONFIG")))
    w_.s(FS("ERROR: MOUNT_FORWARD_AXIS and MOUNT_UP_AXIS must be perpendicular - using defaults")).end();
  if (st.flags & PersistentStore::FLAG_LEVEL) pipeline_.setLevel(st.levelUp);

  fusion_.begin(&fusionConfig());
  detector_.begin(&detectorConfig(), &profile_);
  ctrlCfg_.countdownS = COUNTDOWN_SECONDS;
  ctrlCfg_.cancelDisplayMs = CANCEL_ACK_DISPLAY_MS;
  ctrlCfg_.sosAlarmMs = SOS_ALARM_MS;
  ctrlCfg_.buttonLockoutMs = BUTTON_LOCKOUT_MS;
  controller_.begin(this, &ctrlCfg_, bootNo_, now);

  leds_.begin(PIN_RED_LED, PIN_GREEN_LED, LED_ACTIVE_HIGH != 0);
  buzzer_.begin(PIN_BUZZER);
  button_.begin(PIN_BUTTON, BUTTON_ACTIVE_LOW != 0, now);
  if (BUZZER_STARTUP_CHIRP) buzzer_.chirp(now);

  imuReady_ = pipeline_.initSensor();
  lastImuRetryMs_ = now;
  if (w_.log(FS("IMU"))) {
    if (imuReady_) w_.s(FS("Connected (WHO_AM_I 0x")).hex2(imu->whoAmI()).s(imu->whoAmI() == 0x68 ? FS(")") : FS(", MPU-6050 compatible clone)")).end();
    else w_.s(FS("NOT FOUND - check wiring (SDA/SCL/VCC/GND). Crash detection inactive.")).end();
  }

  // Power was lost during an incident? Do not silently return to SAFE.
  if (st.activeState != ACTIVE_NONE) {
    IncidentRecord rec;
    if (store_.findIncident(st.activeId, rec)) {
      const bool sosPhase = st.activeState == ACTIVE_SOS;
      if (w_.log(FS("BOOT"))) {
        w_.s(FS("Restarted during incident #")).u(rec.id)
            .s(sosPhase ? FS(" after SOS - entering RECOVERY") : FS(" countdown - restarting countdown")).end();
      }
      controller_.restore(rec, sosPhase, now);
      if (sosPhase && !(rec.flags & INC_SOS_ACKED)) sos_.issue(controller_.incident(), false, now);
    } else {
      store_.setActiveIncident(ACTIVE_NONE, 0);
    }
  }

  beginStartupCalibration(now);
  if (!imuReady_) markImuFault(now, false);
  nextSampleUs_ = lastSampleUs_ = micros();
}

// =============================================================================
// Main loop (cooperative, never blocks)
// =============================================================================
void BikeSafetyApp::loop() {
  const uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextSampleUs_) >= 0) {
    float dt = (float)(nowUs - lastSampleUs_) * 1e-6f;
    lastSampleUs_ = nowUs;
    nextSampleUs_ += kSamplePeriodUs;
    if ((int32_t)(nowUs - nextSampleUs_) > (int32_t)(5 * kSamplePeriodUs)) nextSampleUs_ = nowUs + kSamplePeriodUs;
    if (dt * 1000.0f > (float)IMU_MAX_GAP_MS) {   // long stall: do not integrate across it
      fusion_.reset();
      dt = kNominalDt;
    }
    processSample(millis(), dt);
  }
  const uint32_t now = millis();
  serviceLink(now);
  serviceButton(now);
  serviceImu(now);
  serviceSim(now);
  serviceGps(now);
  controller_.tick(now);
  serviceOutputs(now);
  serviceSos(now);
  serviceCountdownLog(now);
  serviceTelemetry(now);
  serviceStatusLog(now);
  store_.service();
}

// =============================================================================
// Sensor processing
// =============================================================================
void BikeSafetyApp::processSample(uint32_t now, float dt) {
  Vec3 acc = {0, 0, 1}, gyro = {0, 0, 0};
  bool sat = false, ok = false;
  const bool simActive = sim_.active();

  if (simActive) {
    ImuSample s;
    ok = sim_.next(now, s);
    if (ok) { acc = s.acc; gyro = s.gyro; sat = s.saturated; }
    else lastImuStatus_ = ImuStatus::BUS_ERROR;
  } else if (imuReady_) {
    const ImuStatus st = pipeline_.readUncorrected(acc, gyro, sat);
    ok = st == ImuStatus::OK;
    if (!ok) lastImuStatus_ = st;
  } else {
    return;   // waiting for re-initialisation in serviceImu()
  }

  if (!ok) {
    if (failCount_ < 255) ++failCount_;
    if (failCount_ >= IMU_FAIL_LIMIT && imuHealthy_) markImuFault(now, simActive);
    return;
  }
  failCount_ = 0;
  if (!imuHealthy_) markImuRecovered(now);

  if (!simActive) {
    if (calibrating_) { handleStartupCalibration(acc, gyro, now); return; }
    if (levelCalActive_) {
      pipeline_.correctBias(acc, gyro);
      handleLevelCalibration(acc, gyro, now);
      return;
    }
    pipeline_.correct(acc, gyro);
  }

  ImuSample s;
  s.tMs = now;
  s.dt = dt;
  s.acc = acc;
  s.gyro = gyro;
  s.saturated = sat;
  fusion_.update(s);
  const uint8_t ev = detector_.update(s, fusion_.attitude(), fusion_.gravityBody());

  lastAcc_ = vlen(acc);
  lastGyro_ = vlen(gyro);
  for (uint8_t i = 0; i < Link::kMax; ++i) {
    if (lastAcc_ > peakAcc_[i]) peakAcc_[i] = lastAcc_;
    if (lastGyro_ > peakGyro_[i]) peakGyro_[i] = lastGyro_;
  }
  if (ev) handleDetectorEvents(ev, now);
}

void BikeSafetyApp::markImuFault(uint32_t now, bool simulated) {
  imuHealthy_ = false;
  if (!simulated) imuReady_ = false;
  fusion_.reset();
  detector_.reset();
  if (w_.log(FS("IMU"))) {
    w_.s(FS("DISCONNECTED (")).s(imuStatusName(lastImuStatus_)).s(simulated ? FS(", simulated") : FS(""))
        .s(FS(") - crash detection is NOT active")).end();
  }
  controller_.setImuHealthy(false, now);
}

void BikeSafetyApp::markImuRecovered(uint32_t now) {
  imuHealthy_ = true;
  fusion_.reset();
  detector_.reset();
  if (w_.log(FS("IMU"))) w_.s(FS("Connected - crash detection active")).end();
  if (calibrating_) startupCal_.start(CAL_SAMPLES);
  controller_.setImuHealthy(true, now);
}

void BikeSafetyApp::resetDetection(uint32_t now) {
  fusion_.reset();
  detector_.begin(&detectorConfig(), &profile_);
  controller_.detectorReset(now);
}

void BikeSafetyApp::beginStartupCalibration(uint32_t now) {
  calibrating_ = true;
  calAttempts_ = 0;
  calQuality_ = CAL_RUNNING;
  startupCal_.start(CAL_SAMPLES);
  if (w_.log(FS("CALIBRATION"))) w_.s(FS("Keep the bike still - measuring sensor offsets...")).end();
  controller_.startCalibration(now);
}

void BikeSafetyApp::handleStartupCalibration(const Vec3& acc, const Vec3& gyro, uint32_t now) {
  const CalStatus cs = startupCal_.add(acc, gyro);
  if (cs == CalStatus::RUNNING) return;
  StoredSettings& st = store_.settings();
  if (cs == CalStatus::DONE) {
    const Vec3 bias = startupCal_.gyroBias();
    pipeline_.setGyroBias(bias);
    pipeline_.setAccOffset(startupCal_.accOffset());
    calQuality_ = CAL_OK;
    if (!(st.flags & PersistentStore::FLAG_GYRO) || vlen(vsub(bias, st.gyroBias)) > CAL_GYRO_SAVE_DELTA_DPS) {
      st.gyroBias = bias;
      st.flags |= PersistentStore::FLAG_GYRO;
      store_.saveSettings();
    }
    if (w_.log(FS("CALIBRATION"))) {
      w_.s(FS("Complete - gyro bias (")).f(bias.x, 2).s(FS(", ")).f(bias.y, 2).s(FS(", ")).f(bias.z, 2)
          .s(FS(") deg/s, gravity ")).f(startupCal_.gravityMagnitude(), 3).s(FS(" g")).end();
    }
    finishCalibration(now);
    return;
  }
  if (++calAttempts_ < CAL_MAX_ATTEMPTS) {
    if (w_.log(FS("CALIBRATION"))) w_.s(FS("Movement detected - restarting (attempt ")).u(calAttempts_ + 1).c('/').u(CAL_MAX_ATTEMPTS).c(')').end();
    startupCal_.start(CAL_SAMPLES);
    return;
  }
  if (st.flags & PersistentStore::FLAG_GYRO) {
    pipeline_.setGyroBias(st.gyroBias);
    calQuality_ = CAL_DEGRADED;
    if (w_.log(FS("CALIBRATION"))) w_.s(FS("WARNING: bike kept moving - using last saved gyro calibration")).end();
  } else {
    calQuality_ = CAL_UNCALIBRATED;
    if (w_.log(FS("CALIBRATION"))) w_.s(FS("WARNING: could not calibrate (moving). Running UNCALIBRATED - send CAL GYRO when still")).end();
  }
  finishCalibration(now);
}

void BikeSafetyApp::finishCalibration(uint32_t now) {
  calibrating_ = false;
  fusion_.reset();
  detector_.reset();
  controller_.calibrationFinished(now);
  sendConfig();
}

void BikeSafetyApp::handleLevelCalibration(const Vec3& acc, const Vec3& gyro, uint32_t now) {
  (void)now;
  const CalStatus cs = levelCal_.add(acc, gyro);
  if (cs == CalStatus::RUNNING) return;
  levelCalActive_ = false;
  if (cs == CalStatus::DONE) {
    StoredSettings& st = store_.settings();
    st.levelUp = levelCal_.upVector();
    st.flags |= PersistentStore::FLAG_LEVEL;
    store_.saveSettings();
    pipeline_.setLevel(st.levelUp);
    fusion_.reset();
    detector_.reset();
    if (w_.log(FS("CALIBRATION"))) w_.s(FS("Riding orientation saved - removed ")).f(levelCal_.mountTiltDeg(), 1).s(FS("° mounting tilt")).end();
    sendConfig();
  } else if (cs == CalStatus::BAD_ORIENTATION) {
    if (w_.log(FS("CALIBRATION"))) {
      w_.s(FS("Rejected - sensor is ")).f(levelCal_.mountTiltDeg(), 0)
          .s(FS("° off the configured axes. Check MOUNT_FORWARD_AXIS / MOUNT_UP_AXIS in config.h")).end();
    }
  } else if (w_.log(FS("CALIBRATION"))) {
    w_.s(FS("Level calibration aborted - bike moved. Hold it upright and still, then retry")).end();
  }
}

// =============================================================================
// Detector reporting
// =============================================================================
void BikeSafetyApp::handleDetectorEvents(uint8_t ev, uint32_t now) {
  const DetectionReport& rep = detector_.report();
  if (ev & DEV_CANDIDATE) {
    candLogged_ = elapsed(now, lastCandLogMs_, 3000) || lastCandLogMs_ == 0;
    if (candLogged_ && w_.log(FS("DETECTOR"))) {
      lastCandLogMs_ = now;
      w_.s(FS("Abnormal motion detected - evaluating (trigger:"));
      if (rep.triggers & TRIG_IMPACT) w_.s(FS(" acceleration"));
      if (rep.triggers & TRIG_GYRO) w_.s(FS(" angular-velocity"));
      if (rep.triggers & TRIG_TILT_RATE) w_.s(FS(" lean-rate"));
      if (rep.triggers & TRIG_SATURATION) w_.s(FS(" sensor-saturated"));
      w_.c(')').end();
    }
  }
  if ((ev & DEV_SECONDARY) && w_.log(FS("DETECTOR"))) w_.s(FS("Secondary impact / renewed violent motion")).end();
  if ((ev & DEV_POSSIBLE) && w_.log(FS("DETECTOR")))
    w_.s(FS("Possible crash - confidence ")).u(pct(rep.confidence)).s(FS("% (still evaluating)")).end();
  if (ev & DEV_VERDICT) logVerdict(rep);

  controller_.onDetector(ev, rep, sim_.active(), now);

  if (ev & DEV_VERDICT) sendDetection(FS("verdict"), rep);
  else if (ev & DEV_POSSIBLE) sendDetection(FS("possible"), rep);
  else if (ev & DEV_CANDIDATE) sendDetection(FS("candidate"), rep);
}

void BikeSafetyApp::logIndicator(FStr label, float value, uint8_t dec, FStr unit, float score) {
  if (!w_.log(FS("DETECTOR"))) return;
  w_.s(label).f(value, dec).s(unit).s(FS(" (")).s(levelWord(score)).s(FS(", ")).u(pct(score)).s(FS("%)")).end();
}

void BikeSafetyApp::logVerdict(const DetectionReport& rep) {
  const bool crash = rep.verdict == DetVerdict::CRASH;
  if (!crash && !detector_.possibleRaised()) {
    if (candLogged_ && w_.log(FS("DETECTOR"))) {
      w_.s(FS("Motion cleared - normal pattern (confidence ")).u(pct(rep.confidence)).c('%');
      if (rep.verticalImpact) w_.s(FS(", vertical impact = road bump"));
      w_.c(')').end();
    }
    return;
  }
  if (w_.log(FS("DETECTOR"))) {
    w_.s(crash ? FS("Evaluation: CRASH LIKELY") : FS("Evaluation: not a crash")).s(FS(" | ")).u(rep.activeIndicators)
        .s(FS(" of 5 indicators active")).end();
  }
  const float* sc = rep.scores;
  if (w_.log(FS("DETECTOR"))) {
    w_.s(FS("Lean: ")).f(rep.peakTiltDeg, 0).s(FS("° (")).s(levelWord(sc[IND_LEAN])).s(FS(") - normal max "))
        .f(profile_.leanNormalDeg, 0).s(FS("°")).end();
  }
  logIndicator(FS("Angular velocity: "), rep.peakGyroDps, 0, FS("°/s"), sc[IND_RATE]);
  logIndicator(FS("Acceleration spike: "), rep.peakAccG, 1, rep.verticalImpact ? FS("g vertical") : FS("g"), sc[IND_IMPACT]);
  logIndicator(FS("Orientation change: "), rep.peakTiltChangeDeg, 0, FS("° in 0.5 s"), sc[IND_CHANGE]);
  if (w_.log(FS("DETECTOR"))) {
    w_.s(FS("Post-impact: "));
    if (!rep.postEligible) w_.s(FS("not counted (no major motion)"));
    else w_.s(rep.settled ? FS("resting at ") : FS("not at rest, tilt ")).f(rep.restTiltDeg, 0).s(FS("° (")).s(levelWord(sc[IND_POST])).c(')');
    w_.end();
  }
  if (rep.parked && w_.log(FS("DETECTOR"))) w_.s(FS("Vehicle was parked & still before the event - confidence reduced")).end();
  if (w_.log(FS("DETECTOR"))) w_.s(FS("Crash confidence: ")).u(pct(rep.confidence)).c('%').end();
}

// =============================================================================
// ControllerListener
// =============================================================================
void BikeSafetyApp::onStateChange(SystemState from, SystemState to, StateReason why) {
  const uint32_t now = millis();
  const IncidentRecord& inc = controller_.incident();
  const bool realIncident = inc.type == (uint8_t)IncidentType::AUTO;

  if (w_.log(FS("STATE"))) w_.s(SafetyController::stateName(to)).s(FS(" (")).s(SafetyController::reasonName(why)).c(')').end();
  leds_.showState(to, now);
  buzzer_.showState(to, controller_.countdownRemaining(now), now);

  switch (to) {
    case SystemState::MANUAL_TEST:
      if (w_.log(FS("TEST"))) w_.s(FS("TEST / MANUAL TRIGGER - simulated alarm, NOT a real crash. Press again to clear.")).end();
      break;
    case SystemState::CRASH_DETECTED:
      lastCountdownLogged_ = 0xFF;
      if (realIncident) store_.setActiveIncident(ACTIVE_COUNTDOWN, inc.id);
      if (w_.log(FS("EMERGENCY"))) {
        w_.s(FS("POSSIBLE ACCIDENT - SOS in ")).u(COUNTDOWN_SECONDS)
            .s(FS(" s. Press the button or tap I'M OKAY in the app to cancel.")).end();
      }
      break;
    case SystemState::USER_CANCELLED:
      store_.setActiveIncident(ACTIVE_NONE, 0);
      detector_.disarmUntilUpright();
      if (w_.log(FS("EMERGENCY"))) w_.s(FS("Cancelled by rider (")).s(SafetyController::reasonName(why)).s(FS(") - no SOS")).end();
      break;
    case SystemState::SOS_SENT:
      if (realIncident) store_.setActiveIncident(ACTIVE_SOS, inc.id);
      break;
    case SystemState::RECOVERY:
      if (w_.log(FS("RECOVERY"))) w_.s(FS("Check rider and bike, then hold the button 3 s (or RESET in the app)")).end();
      break;
    default:
      break;
  }
  if (why == StateReason::RESET) {
    store_.setActiveIncident(ACTIVE_NONE, 0);
    detector_.disarmUntilUpright();
  }
  if (from == SystemState::MANUAL_TEST && to != SystemState::POSSIBLE_CRASH && to != SystemState::CRASH_DETECTED &&
      w_.log(FS("TEST"))) {
    w_.s(FS("Manual test cleared")).end();
  }
  sendState(from, to, why);
}

void BikeSafetyApp::onIncidentUpdate(const IncidentRecord& rec) {
  store_.saveIncident(rec);
  sos_.updateRecord(rec);
  sendIncident(rec);
}

void BikeSafetyApp::onSosIssued(const IncidentRecord& rec) {
  const uint32_t now = millis();
  const bool sim = rec.type == (uint8_t)IncidentType::SIMULATION;
  if (!sos_.issue(rec, sim, now)) {
    if (w_.log(FS("SOS"))) w_.s(FS("Duplicate SOS for incident #")).u(rec.id).s(FS(" suppressed")).end();
    return;
  }
  if (w_.log(FS("SOS"))) {
    w_.s(FS("Countdown expired - EMERGENCY ALERT for incident #")).u(rec.id).s(FS(" (confidence ")).u(rec.confidence).s(FS("%)")).end();
  }
  logLocation(now);
  if (w_.log(FS("SOS"))) w_.s(FS("No GSM/SMS hardware on this device: alert is delivered through the app link")).end();
  if (sim && w_.log(FS("SOS"))) w_.s(FS("SIMULATION - the app will NOT notify emergency contacts")).end();
}

bool BikeSafetyApp::gpsValid(uint32_t now) const {
#if GPS_ENABLED
  return gpsParser_.hasValidFix(now);
#else
  (void)now;
  return false;
#endif
}

bool BikeSafetyApp::currentLocation(int32_t& latE7, int32_t& lonE7) {
#if GPS_ENABLED
  if (gpsParser_.hasValidFix(millis())) {
    latE7 = gpsParser_.fix().latE7;
    lonE7 = gpsParser_.fix().lonE7;
    return true;
  }
#else
  (void)latE7;
  (void)lonE7;
#endif
  return false;
}

void BikeSafetyApp::logLocation(uint32_t now) {
  if (!w_.log(FS("SOS"))) return;
  w_.s(FS("Location: "));
#if GPS_ENABLED
  if (gpsParser_.hasValidFix(now)) {
    w_.e7(gpsParser_.fix().latE7).s(FS(", ")).e7(gpsParser_.fix().lonE7);
    const float acc = gpsParser_.accuracyM();
    if (acc > 0) w_.s(FS(" (GPS, estimated +/-")).f(acc, 0).s(FS(" m)"));
    w_.end();
    return;
  }
  w_.s(FS("LOCATION UNAVAILABLE (no valid GPS fix)")).end();
#else
  (void)now;
  w_.s(FS("LOCATION UNAVAILABLE (no GPS module installed)")).end();
#endif
}

// =============================================================================
// Services
// =============================================================================
void BikeSafetyApp::serviceLink(uint32_t now) {
  link_.service(now);
  const uint8_t lost = link_.takeLostSessions();
  for (uint8_t i = 0; i < Link::kMax; ++i) {
    if ((lost & (1u << i)) && w_.log(FS("LINK")))
      w_.s(FS("App heartbeat lost on ")).s(link_.transport(i)->name()).s(FS(" - app link OFFLINE")).end();
  }
  uint8_t src = 0;
  for (uint8_t n = 0; n < 4; ++n) {
    const char* line = link_.poll(now, src);
    if (!line) break;
    handleCommand(line, src, now);
  }
}

void BikeSafetyApp::serviceButton(uint32_t now) {
  const uint8_t ev = button_.update(now);
  if (!ev) return;
  if ((ev & BTN_PRESS) && w_.log(FS("BUTTON"))) w_.s(FS("Pressed")).end();
  if ((ev & BTN_STUCK) && w_.log(FS("BUTTON"))) w_.s(FS("WARNING: held > 15 s - ignored until released (stuck or shorted?)")).end();
  const SystemState s = controller_.state();
  if ((ev & BTN_PRESS) && (s == SystemState::SOS_SENT || s == SystemState::RECOVERY) && w_.log(FS("BUTTON")))
    w_.s(FS("Hold 3 s to reset after an SOS")).end();
  controller_.onButton(ev, now);
}

void BikeSafetyApp::serviceOutputs(uint32_t now) {
  const SystemState s = controller_.state();
  leds_.showState(s, now);
  leds_.update(now);
  buzzer_.showState(s, controller_.countdownRemaining(now), now);
  buzzer_.update(now);
}

void BikeSafetyApp::serviceImu(uint32_t now) {
  if (imuReady_ || sim_.active() || !elapsed(now, lastImuRetryMs_, IMU_RETRY_MS)) return;
  lastImuRetryMs_ = now;
  if (pipeline_.initSensor()) {
    imuReady_ = true;
    failCount_ = 0;
    if (w_.log(FS("IMU"))) w_.s(FS("Re-initialised (WHO_AM_I 0x")).hex2(pipeline_.sensor()->whoAmI()).c(')').end();
  }
}

void BikeSafetyApp::serviceSim(uint32_t now) {
  if (!sim_.active() || !sim_.finished(now)) return;
  sim_.stop();
  failCount_ = 0;
  if (w_.log(FS("TEST"))) w_.s(FS("Simulation finished - real sensor resumed")).end();
  resetDetection(now);
  if (!imuReady_ && imuHealthy_) {
    lastImuStatus_ = ImuStatus::NOT_READY;
    markImuFault(now, false);
  }
}

void BikeSafetyApp::serviceGps(uint32_t now) {
#if GPS_ENABLED
  if (!gps_) return;
  for (uint8_t n = 0; n < 64; ++n) {
    const int c = gps_->read();
    if (c < 0) break;
    gpsParser_.feed((char)c, now);
  }
  const bool v = gpsParser_.hasValidFix(now);
  if (v != gpsWasValid_) {
    gpsWasValid_ = v;
    if (w_.log(FS("GPS"))) {
      if (v) w_.s(FS("Fix acquired (")).u(gpsParser_.fix().satellites).s(FS(" satellites, HDOP ")).f(gpsParser_.fix().hdop, 1).c(')').end();
      else w_.s(FS("Fix lost - LOCATION UNAVAILABLE")).end();
    }
  }
#else
  (void)now;
#endif
#if BATTERY_ADC_PIN >= 0
  if (elapsed(now, lastBatteryMs_, 5000) || lastBatteryMs_ == 0) {
    lastBatteryMs_ = now;
    batteryMv_ = (int16_t)((float)analogRead(BATTERY_ADC_PIN) / BATTERY_ADC_MAX * BATTERY_ADC_REF_MV * BATTERY_DIVIDER);
  }
#endif
}

void BikeSafetyApp::serviceSos(uint32_t now) {
  if (!sos_.pending()) return;
  const bool link = link_.anySession();
  if (sos_.sendDue(now, link)) {
    sendSos(now);
    if (sos_.transmissions() == 1 && w_.log(FS("SOS"))) w_.s(FS("Alert sent to app - waiting for confirmation")).end();
  }
  if (sos_.reminderDue(now, link) && w_.log(FS("SOS"))) {
    w_.s(FS("NOT DELIVERED - no app connected. Buzzer/LED alarm only until the app reconnects.")).end();
  }
}

void BikeSafetyApp::serviceCountdownLog(uint32_t now) {
  if (controller_.state() != SystemState::CRASH_DETECTED) return;
  const uint8_t r = controller_.countdownRemaining(now);
  if (r == lastCountdownLogged_ || r == 0) return;
  lastCountdownLogged_ = r;
  if (w_.log(FS("EMERGENCY"))) w_.s(FS("SOS in ")).u(r).s(FS(" s")).end();
}

void BikeSafetyApp::serviceTelemetry(uint32_t now) {
  if (link_.dataSuspended()) return;
  for (uint8_t i = 0; i < Link::kMax; ++i) {
    if (!link_.sessionActive(i)) continue;
    const uint32_t period = i == 0 ? TELEMETRY_PERIOD_MS : BLE_TELEMETRY_PERIOD_MS;
    if (!elapsed(now, lastTelMs_[i], period) || !link_.telemetryWritable(i)) continue;
    lastTelMs_[i] = now;
    sendTelemetry(i, now);
  }
}

void BikeSafetyApp::serviceStatusLog(uint32_t now) {
  if (STATUS_LOG_PERIOD_MS == 0 || !elapsed(now, lastStatusMs_, STATUS_LOG_PERIOD_MS)) return;
  lastStatusMs_ = now;
  if (calibrating_ && imuHealthy_) return;
  logStatus(now);
}

void BikeSafetyApp::logStatus(uint32_t now) {
  (void)now;
  const Attitude& a = fusion_.attitude();
  if (w_.log(FS("IMU"))) {
    if (!imuHealthy_) w_.s(FS("DISCONNECTED - crash detection inactive (retrying)")).end();
    else {
      w_.s(FS("Roll: ")).f(a.rollDeg, 1).s(FS("° | Pitch: ")).f(a.pitchDeg, 1).s(FS("° | Acceleration: ")).f(lastAcc_, 2)
          .s(FS("g | Angular velocity: ")).f(lastGyro_, 0).s(FS("°/s")).end();
    }
  }
  if (w_.log(FS("DETECTOR"))) {
    w_.s(FS("Crash confidence: ")).u(pct(detector_.liveConfidence())).s(FS("% | State: "))
        .s(SafetyController::stateName(controller_.state()));
    if (detector_.abnormalStaticTilt()) w_.s(FS(" | bike not upright"));
    if (!detector_.armed()) w_.s(FS(" | detection re-arms when upright"));
    if (calQuality_ >= CAL_DEGRADED) w_.s(FS(" | calibration degraded"));
    w_.end();
  }
}

// =============================================================================
// Commands from the app / serial monitor
// =============================================================================
bool BikeSafetyApp::commandsAllowed() const {
  const SystemState s = controller_.state();
  return s == SystemState::SAFE || s == SystemState::MONITORING || s == SystemState::IMU_FAULT;
}

void BikeSafetyApp::ack(const char* cmd, bool ok) {
  if (!w_.obj(FS("ack"))) return;
  char name[12];
  uint8_t n = 0;
  while (cmd[n] && cmd[n] != ' ' && n < sizeof(name) - 1) { name[n] = cmd[n]; ++n; }
  name[n] = '\0';
  w_.kr(FS("cmd"), name).kb(FS("ok"), ok).endObj();
}

void BikeSafetyApp::printHelp() {
  if (w_.log(FS("HELP"))) w_.s(FS("CANCEL | RESET | STATUS | LOG | CLEARLOG | BTN | PROFILE scooter|standard|sport")).end();
  if (w_.log(FS("HELP"))) w_.s(FS("CAL LEVEL | CAL GYRO | CAL CLEAR | COMMS <s> | GPSLOSS <s> | SIM <n> | SIM STOP")).end();
  if (w_.log(FS("HELP"))) w_.s(FS("SIM: 1 stationary 2 lean 3 cornering 4 braking 5 speed-breaker 6 swerve 7 crash")).end();
  if (w_.log(FS("HELP"))) w_.s(FS("     9 sensor-disconnect 14 pothole 15 parked-tip-over 16 high-side")).end();
}

void BikeSafetyApp::handleCommand(const char* line, uint8_t src, uint32_t now) {
  const char* p = line;
  bool ok = true;

  if (cmdIs(p, FS("PING"))) {
    if (!link_.sessionActive(src)) {   // heartbeat after a link loss: resume the session
      link_.startSession(src, now);
      sendHello(now);
    }
    return;
  }
  if (cmdIs(p, FS("HELLO"))) {
    link_.startSession(src, now);
    if (w_.log(FS("LINK"))) w_.s(FS("App connected via ")).s(link_.transport(src)->name()).end();
    sendHello(now);
    return;
  }
  if (cmdIs(p, FS("CANCEL"))) {
    ok = controller_.cancel(now, StateReason::APP);
  } else if (cmdIs(p, FS("RESET"))) {
    ok = controller_.reset(now);
  } else if (cmdIs(p, FS("SOSACK"))) {
    const uint32_t id = (uint32_t)strtoul(p, nullptr, 10);
    ok = sos_.acknowledge(id);
    controller_.sosAcknowledged(id);
    if (ok && w_.log(FS("SOS"))) w_.s(FS("App confirmed receipt of SOS #")).u(id).end();
  } else if (cmdIs(p, FS("BTN"))) {
    if (w_.log(FS("BUTTON"))) w_.s(FS("Emulated press from app")).end();
    controller_.onButton(BTN_PRESS, now);
  } else if (cmdIs(p, FS("PROFILE"))) {
    const uint8_t id = parseProfileName(p);
    ok = id < PROFILE_COUNT && commandsAllowed() && !sim_.active();
    if (ok) {
      store_.settings().profile = id;
      store_.saveSettings();
      loadVehicleProfile(id, profile_);
      resetDetection(now);
      if (w_.log(FS("CONFIG"))) w_.s(FS("Vehicle profile: ")).s(profileName(id)).end();
      sendConfig();
    }
  } else if (cmdIs(p, FS("CAL"))) {
    const bool idle = controller_.state() == SystemState::SAFE && !sim_.active() && !calibrating_ && !levelCalActive_;
    if (cmdIs(p, FS("LEVEL"))) {
      ok = idle;
      if (ok) {
        levelCal_.start(CAL_LEVEL_SAMPLES);
        levelCalActive_ = true;
        if (w_.log(FS("CALIBRATION"))) w_.s(FS("Capturing riding orientation - hold the bike upright and still (2 s)")).end();
      }
    } else if (cmdIs(p, FS("GYRO"))) {
      ok = idle;
      if (ok) beginStartupCalibration(now);
    } else if (cmdIs(p, FS("CLEAR"))) {
      ok = idle;
      if (ok) {
        pipeline_.clearLevel();
        store_.settings().flags &= (uint8_t)~PersistentStore::FLAG_LEVEL;
        store_.saveSettings();
        resetDetection(now);
        if (w_.log(FS("CALIBRATION"))) w_.s(FS("Riding orientation calibration cleared")).end();
        sendConfig();
      }
    } else {
      ok = false;
    }
    if (!ok && w_.log(FS("CALIBRATION"))) w_.s(FS("Not now - calibration needs state SAFE, no simulation running")).end();
  } else if (cmdIs(p, FS("SIM"))) {
    if (cmdIs(p, FS("STOP"))) {
      ok = sim_.active();
      if (ok) sim_.stop();
      if (ok) {
        if (w_.log(FS("TEST"))) w_.s(FS("Simulation stopped - real sensor resumed")).end();
        resetDetection(now);
      }
    } else {
      const uint8_t id = (uint8_t)atoi(p);
      ok = ImuSimulator::valid(id) && commandsAllowed() && !calibrating_ && !levelCalActive_;
      if (ok) {
        sim_.start(id, now, 12345UL + now);
        failCount_ = 0;
        resetDetection(now);
        if (w_.log(FS("TEST"))) {
          w_.s(FS("SIMULATION ")).u(id).s(FS(": ")).s(ImuSimulator::name(id)).s(FS(" (")).u(ImuSimulator::durationMs(id) / 1000)
              .s(FS(" s) - real sensor paused, real SOS disabled")).end();
        }
      } else if (w_.log(FS("TEST"))) {
        w_.s(FS("Simulation refused (unknown scenario, or system busy)")).end();
      }
    }
  } else if (cmdIs(p, FS("COMMS"))) {
    long s = atol(p);
    if (s < 1) s = 1;
    if (s > 60) s = 60;
    if (w_.log(FS("TEST"))) w_.s(FS("Communication-failure test: app data paused for ")).u((uint32_t)s).s(FS(" s")).end();
    ack(line, true);
    link_.suspendData(now, (uint32_t)s * 1000UL);
    return;
  } else if (cmdIs(p, FS("GPSLOSS"))) {
#if GPS_ENABLED
    long s = atol(p);
    if (s < 1) s = 1;
    if (s > 600) s = 600;
    gpsParser_.forceLoss(now, (uint32_t)s * 1000UL);
    if (w_.log(FS("TEST"))) w_.s(FS("GPS-unavailable test: fix ignored for ")).u((uint32_t)s).s(FS(" s")).end();
#else
    if (w_.log(FS("GPS"))) w_.s(FS("No GPS module installed - location is always LOCATION UNAVAILABLE")).end();
#endif
  } else if (cmdIs(p, FS("LOG"))) {
    const uint8_t n = store_.incidentCount();
    if (w_.log(FS("LOG"))) w_.u(n).s(FS(" stored incident(s)")).end();
    for (uint8_t i = 0; i < n; ++i) {
      IncidentRecord rec;
      if (!store_.incidentAt(i, rec)) continue;
      sendIncident(rec);
      if (w_.log(FS("LOG"))) {
        w_.c('#').u(rec.id).c(' ').s(typeName(rec.type)).c(' ').s(outcomeName(rec.outcome)).s(FS(" conf ")).u(rec.confidence)
            .s(FS("% acc ")).f(rec.maxAccCg / 100.0f, 1).s(FS("g gyro ")).i(rec.maxGyroDps).s(FS("°/s lean "))
            .f(rec.maxLeanDd / 10.0f, 0).s(FS("°")).end();
      }
    }
  } else if (cmdIs(p, FS("CLEARLOG"))) {
    store_.clearIncidents();
    if (w_.log(FS("LOG"))) w_.s(FS("Incident log cleared")).end();
  } else if (cmdIs(p, FS("STATUS"))) {
    logStatus(now);
    if (w_.log(FS("STATUS"))) {
      w_.s(FS("Profile ")).s(profileName(store_.settings().profile)).s(FS(" | level cal ")).s(pipeline_.levelSet() ? FS("yes") : FS("no"))
          .s(FS(" | calibration ")).u(calQuality_).s(FS(" | app link ")).s(link_.anySession() ? FS("online") : FS("offline"))
          .s(FS(" | stored incidents ")).u(store_.incidentCount()).end();
    }
  } else if (cmdIs(p, FS("HELP"))) {
    printHelp();
  } else {
    ok = false;
    if (w_.log(FS("LINK"))) w_.s(FS("Unknown command: ")).r(line).s(FS(" (type HELP)")).end();
  }
  ack(line, ok);
}

// =============================================================================
// JSON messages
// =============================================================================
void BikeSafetyApp::sendHello(uint32_t now) {
  if (w_.obj(FS("hello"))) {
    w_.ks(FS("fw"), FS(FIRMWARE_VERSION)).ki(FS("proto"), PROTOCOL_VERSION).kr(FS("dev"), deviceName_)
        .ks(FS("board"), FS(BSB_BOARD_NAME)).ku(FS("boot"), bootNo_).kb(FS("gpsHw"), GPS_ENABLED != 0)
        .kb(FS("bleHw"), BLE_ENABLED != 0).kb(FS("batHw"), BATTERY_ADC_PIN >= 0).ki(FS("cd"), COUNTDOWN_SECONDS)
        .ku(FS("ms"), now).endObj();
  }
  sendConfig();
  sendState(controller_.state(), controller_.state(), StateReason::BOOT);
  if (sos_.pending()) sendSos(now);
}

void BikeSafetyApp::sendConfig() {
  if (!w_.obj(FS("cfg"))) return;
  const VehicleProfile& p = profile_;
  const DetectorConfig& d = detectorConfig();
  w_.ks(FS("profile"), profileName(store_.settings().profile)).kf(FS("leanN"), p.leanNormalDeg, 0)
      .kf(FS("leanC"), p.leanCrashDeg, 0).kf(FS("gyrN"), p.gyroNormalDps, 0).kf(FS("gyrC"), p.gyroCrashDps, 0)
      .kf(FS("accN"), p.impactNormalG, 1).kf(FS("accC"), p.impactCrashG, 1).kf(FS("tcN"), p.tiltChangeNormalDeg, 0)
      .kf(FS("tcC"), p.tiltChangeCrashDeg, 0).kf(FS("restN"), p.restTiltNormalDeg, 0).kf(FS("restC"), p.restTiltCrashDeg, 0)
      .ki(FS("possible"), pct(d.possibleThreshold)).ki(FS("crash"), pct(d.crashThreshold)).ki(FS("minInd"), d.minIndicators)
      .ki(FS("accRange"), MPU_ACCEL_RANGE_G).ki(FS("gyrRange"), MPU_GYRO_RANGE_DPS).ki(FS("rate"), SAMPLE_RATE_HZ)
      .kf(FS("alpha"), FUSION_ALPHA, 3).ks(FS("mount"), axisName(MOUNT_FORWARD_AXIS)).ks(FS("up"), axisName(MOUNT_UP_AXIS))
      .kb(FS("level"), pipeline_.levelSet()).ki(FS("cal"), calQuality_).endObj();
}

void BikeSafetyApp::sendState(SystemState from, SystemState to, StateReason why) {
  if (!w_.obj(FS("state"))) return;
  const IncidentRecord& inc = controller_.incident();
  const uint32_t now = millis();
  w_.ks(FS("st"), SafetyController::stateName(to)).ks(FS("prev"), SafetyController::stateName(from))
      .ks(FS("why"), SafetyController::reasonName(why)).ku(FS("inc"), controller_.incidentOpen() ? inc.id : 0)
      .ks(FS("type"), typeName(inc.type)).ki(FS("cd"), controller_.countdownRemaining(now)).ku(FS("ms"), now).endObj();
}

void BikeSafetyApp::writeLocationFields(uint32_t now) {
#if GPS_ENABLED
  if (gpsParser_.hasValidFix(now)) {
    w_.ki(FS("gps"), 2).k(FS("lat")).e7(gpsParser_.fix().latE7).k(FS("lon")).e7(gpsParser_.fix().lonE7)
        .kf(FS("hacc"), gpsParser_.accuracyM(), 0).ki(FS("sat"), gpsParser_.fix().satellites);
    return;
  }
  w_.ki(FS("gps"), 1);
#else
  (void)now;
  w_.ki(FS("gps"), 0);
#endif
}

void BikeSafetyApp::sendTelemetry(uint8_t idx, uint32_t now) {
  if (!w_.obj(FS("tel"), (uint8_t)(1u << idx))) return;
  const Attitude& a = fusion_.attitude();
  const bool v = imuHealthy_ && a.valid && !calibrating_;
  const float nan = NAN;
  w_.ku(FS("ms"), now).ks(FS("st"), SafetyController::stateName(controller_.state()))
      .kf(FS("r"), v ? a.rollDeg : nan, 1).kf(FS("p"), v ? a.pitchDeg : nan, 1).kf(FS("tl"), v ? a.tiltDeg : nan, 1)
      .kf(FS("a"), v ? lastAcc_ : nan, 2).kf(FS("ap"), v ? peakAcc_[idx] : nan, 2).kf(FS("g"), v ? lastGyro_ : nan, 0)
      .kf(FS("gp"), v ? peakGyro_[idx] : nan, 0).ki(FS("c"), pct(detector_.liveConfidence()))
      .ki(FS("ph"), (int)detector_.phase()).kb(FS("imu"), imuHealthy_).ki(FS("cal"), calQuality_)
      .kb(FS("lvl"), pipeline_.levelSet()).ki(FS("sim"), sim_.scenario()).ki(FS("cd"), controller_.countdownRemaining(now))
      .kb(FS("ab"), detector_.abnormalStaticTilt()).kb(FS("arm"), detector_.armed()).ki(FS("ref"), (int)a.ref)
      .ki(FS("bat"), batteryMv_);
  writeLocationFields(now);
  w_.endObj();
  peakAcc_[idx] = 0;
  peakGyro_[idx] = 0;
}

void BikeSafetyApp::sendDetection(FStr kind, const DetectionReport& rep) {
  if (!w_.obj(FS("det"))) return;
  const IncidentRecord& inc = controller_.incident();
  w_.ks(FS("ev"), kind).ku(FS("inc"), controller_.incidentOpen() || (inc.flags & INC_SUSPECTED) ? inc.id : 0)
      .kb(FS("sim"), sim_.active()).ki(FS("conf"), pct(rep.confidence)).k(FS("s")).c('[');
  for (uint8_t i = 0; i < IND_COUNT; ++i) {
    if (i) w_.c(',');
    w_.u(pct(rep.scores[i]));
  }
  w_.c(']').ki(FS("act"), rep.activeIndicators).kf(FS("pkA"), rep.peakAccG, 2).kf(FS("pkG"), rep.peakGyroDps, 0)
      .kf(FS("pkT"), rep.peakTiltDeg, 1).kf(FS("pkR"), rep.peakRollDeg, 1).kf(FS("dT"), rep.peakTiltChangeDeg, 1)
      .kf(FS("sus"), rep.sustainedRateMs, 0).kf(FS("base"), rep.baselineTiltDeg, 1).kf(FS("rest"), rep.restTiltDeg, 1)
      .kb(FS("set"), rep.settled).kb(FS("vert"), rep.verticalImpact).kb(FS("park"), rep.parked)
      .kb(FS("post"), rep.postEligible).ki(FS("trig"), rep.triggers)
      .ks(FS("verdict"), rep.verdict == DetVerdict::CRASH ? FS("CRASH") : rep.verdict == DetVerdict::DISMISSED ? FS("DISMISSED") : FS(""))
      .endObj();
}

void BikeSafetyApp::sendIncident(const IncidentRecord& rec) {
  if (!w_.obj(FS("inc"))) return;
  w_.ku(FS("id"), rec.id).ku(FS("boot"), rec.bootNo).ku(FS("up"), rec.uptimeMs).kb(FS("cur"), rec.bootNo == bootNo_)
      .ks(FS("type"), typeName(rec.type)).ks(FS("out"), outcomeName(rec.outcome)).ki(FS("flags"), rec.flags)
      .ki(FS("conf"), rec.confidence).kf(FS("acc"), rec.maxAccCg / 100.0f, 2).ki(FS("gyr"), rec.maxGyroDps)
      .kf(FS("lean"), rec.maxLeanDd / 10.0f, 1).kb(FS("gpsOk"), (rec.flags & INC_GPS_VALID) != 0);
  if (rec.flags & INC_GPS_VALID) w_.k(FS("lat")).e7(rec.latE7).k(FS("lon")).e7(rec.lonE7);
  w_.endObj();
}

void BikeSafetyApp::sendSos(uint32_t now) {
  if (!w_.obj(FS("sos"))) return;
  const IncidentRecord& r = sos_.record();
  w_.ku(FS("id"), r.id).kr(FS("dev"), deviceName_).kb(FS("sim"), sos_.simulated()).ki(FS("conf"), r.confidence)
      .ku(FS("boot"), r.bootNo).ku(FS("up"), r.uptimeMs).kf(FS("acc"), r.maxAccCg / 100.0f, 2).ki(FS("gyr"), r.maxGyroDps)
      .kf(FS("lean"), r.maxLeanDd / 10.0f, 1).ku(FS("tx"), sos_.transmissions()).ku(FS("ms"), now)
      .kb(FS("resumed"), (r.flags & INC_RESUMED) != 0);
  if (r.flags & INC_GPS_VALID) {
    w_.ki(FS("gps"), 2).k(FS("lat")).e7(r.latE7).k(FS("lon")).e7(r.lonE7);
#if GPS_ENABLED
    w_.kf(FS("hacc"), gpsParser_.accuracyM(), 0);
    const GpsFix& f = gpsParser_.fix();
    if (f.timeValid) {
      w_.k(FS("utc")).c('"').s(FS("20")).u(f.year / 10).u(f.year % 10).c('-').u(f.month / 10).u(f.month % 10).c('-')
          .u(f.day / 10).u(f.day % 10).c('T').u(f.hour / 10).u(f.hour % 10).c(':').u(f.minute / 10).u(f.minute % 10)
          .c(':').u(f.second / 10).u(f.second % 10).s(FS("Z\""));
    }
#endif
  } else {
    w_.ki(FS("gps"), GPS_ENABLED ? 1 : 0);
  }
  w_.endObj();
}
