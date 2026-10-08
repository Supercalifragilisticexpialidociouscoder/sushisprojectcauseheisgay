#include "crash_detector.h"

#include <string.h>

static const float kLpAlpha = 0.05f;          // ~0.2 s time constant at 100 Hz
static const uint32_t kHistPeriodMs = 100;    // tilt history resolution
static const uint8_t kChangeWindow = 5;       // 5 x 100 ms = 0.5 s
static const uint32_t kParkedGraceMs = 2000;  // motion onset lag allowance
static const float kMinDynamicG = 0.3f;       // below this the impact direction is meaningless

static inline float maxf(float a, float b) { return a > b ? a : b; }

static uint8_t popcount8(uint8_t v) {
  uint8_t c = 0;
  while (v) { c += v & 1u; v >>= 1; }
  return c;
}

void CrashDetector::begin(const DetectorConfig* cfg, const VehicleProfile* profile) {
  cfg_ = cfg;
  prof_ = profile;
  armed_ = true;
  histCount_ = 0;
  histHead_ = 0;
  havePrev_ = false;
  liveConf_ = 0;
  reset();
}

void CrashDetector::reset() {
  phase_ = DetPhase::IDLE;
  memset(&rep_, 0, sizeof(rep_));
  anomalyShift_ = 0;
  pendingTrig_ = 0;
  preAcc_ = preGyro_ = 0;
  preVert_ = false;
  possibleRaised_ = false;
  calm_ = rest_ = false;
}

void CrashDetector::disarmUntilUpright() {
  reset();
  armed_ = false;
  uprightTiming_ = false;
}

uint8_t CrashDetector::anomalyBits(const ImuSample& s, float aMag, float wMag, float tiltRate) const {
  uint8_t b = 0;
  if (aMag >= cfg_->triggerImpactG) b |= TRIG_IMPACT;
  if (wMag >= cfg_->triggerGyroDps) b |= TRIG_GYRO;
  if (tiltRate >= cfg_->triggerTiltRateDps) b |= TRIG_TILT_RATE;
  if (s.saturated) b |= TRIG_SATURATION;
  return b;
}

static bool isVerticalImpact(const Vec3& acc, const Vec3& gBody, float fraction) {
  const Vec3 dyn = vsub(acc, gBody);
  const float d = vlen(dyn);
  if (d < kMinDynamicG) return false;
  return fabsf(dyn.z) / d >= fraction;
}

uint8_t CrashDetector::update(const ImuSample& s, const Attitude& att, const Vec3& gBody) {
  uint8_t ev = DEV_NONE;
  const uint32_t t = s.tMs;
  const float aMag = vlen(s.acc);
  const float wMag = vlen(s.gyro);
  const float tilt = att.tiltDeg;
  const float tiltRate = sqrtf(att.rollRateDps * att.rollRateDps + att.pitchRateDps * att.pitchRateDps);

  // ---- Stillness tracking (parked-vehicle guard) -----------------------------
  lpGyro_ += kLpAlpha * (wMag - lpGyro_);
  lpAccDev_ += kLpAlpha * (fabsf(aMag - 1.0f) - lpAccDev_);
  const bool still = lpGyro_ <= cfg_->stillGyroDps && lpAccDev_ <= cfg_->stillAccG;
  if (!havePrev_) {
    havePrev_ = true;
    stillNow_ = false;
    longStillSeen_ = false;
  }
  if (still && !stillNow_) { stillNow_ = true; stillStartMs_ = t; }
  if (!still) stillNow_ = false;
  if (stillNow_ && elapsed(t, stillStartMs_, cfg_->parkedMinMs)) {
    longStillSeen_ = true;       // remember the last moment the bike had been parked long enough
    longStillMs_ = t;
  }

  // ---- Tilt history (10 Hz, 1 s) -------------------------------------------------
  if (histCount_ == 0 || elapsed(t, lastHistMs_, kHistPeriodMs)) {
    tiltHist_[histHead_] = tilt;
    histHead_ = (uint8_t)((histHead_ + 1) % kHist);
    if (histCount_ < kHist) ++histCount_;
    lastHistMs_ = t;
  }
  tiltChange_ = 0;
  const uint8_t n = histCount_ < kChangeWindow ? histCount_ : kChangeWindow;
  for (uint8_t k = 0; k < n; ++k) {
    const uint8_t idx = (uint8_t)((histHead_ + kHist - 1 - k) % kHist);
    tiltChange_ = maxf(tiltChange_, fabsf(tilt - tiltHist_[idx]));
  }

  // ---- Static orientation (information only, never an alarm by itself) ------
  if (tilt >= prof_->leanNormalDeg) staticTiltAbnormal_ = true;
  else if (tilt < prof_->leanNormalDeg - cfg_->leanHysteresisDeg) staticTiltAbnormal_ = false;

  // ---- Re-arm after an incident once the bike is upright again ---------------
  if (!armed_) {
    const bool upright = tilt < prof_->leanNormalDeg - cfg_->leanHysteresisDeg && wMag < cfg_->quietGyroDps;
    if (!upright) {
      uprightTiming_ = false;
    } else if (!uprightTiming_) {
      uprightTiming_ = true;
      uprightSinceMs_ = t;
    } else if (elapsed(t, uprightSinceMs_, cfg_->rearmUprightMs)) {
      armed_ = true;
    }
  }

  // ---- Candidate trigger: K of the last M samples anomalous ------------------
  const uint8_t bits = anomalyBits(s, aMag, wMag, tiltRate);
  const uint8_t mask = (uint8_t)((1u << cfg_->triggerWindow) - 1u);
  anomalyShift_ = (uint8_t)((anomalyShift_ << 1) | (bits ? 1u : 0u));
  if ((anomalyShift_ & mask) == 0) {
    pendingTrig_ = 0;
    preAcc_ = preGyro_ = 0;
    preVert_ = false;
  } else {
    pendingTrig_ |= bits;
    if (aMag > preAcc_) { preAcc_ = aMag; preVert_ = isVerticalImpact(s.acc, gBody, cfg_->verticalImpactFraction); }
    preGyro_ = maxf(preGyro_, wMag);
  }
  const bool triggered = popcount8((uint8_t)(anomalyShift_ & mask)) >= cfg_->triggerMinSamples;

  switch (phase_) {
    case DetPhase::IDLE: {
      if (armed_ && triggered) {
        startEvent(t, pendingTrig_);
        ev |= DEV_CANDIDATE;
        trackPeaks(s, att, gBody, aMag, wMag);
      } else {
        // Instantaneous indicator level, shown as a decaying "confidence" while riding.
        const VehicleProfile& p = *prof_;
        const float inst = cfg_->wLean * ramp(tilt, p.leanNormalDeg, p.leanCrashDeg) +
                           cfg_->wRate * ramp(wMag, p.gyroNormalDps, p.gyroCrashDps) +
                           cfg_->wImpact * ramp(aMag, p.impactNormalG, p.impactCrashG) +
                           cfg_->wChange * ramp(tiltChange_, p.tiltChangeNormalDeg, p.tiltChangeCrashDeg);
        liveConf_ = maxf(inst, liveConf_ * 0.98f);
      }
      break;
    }

    case DetPhase::ACTIVE: {
      trackPeaks(s, att, gBody, aMag, wMag);
      const bool calm = wMag < cfg_->quietGyroDps && fabsf(aMag - 1.0f) < cfg_->quietAccG;
      if (!calm) calm_ = false;
      else if (!calm_) { calm_ = true; calmSinceMs_ = t; }
      if ((calm_ && elapsed(t, calmSinceMs_, cfg_->quietMs)) || elapsed(t, phaseMs_, cfg_->activeMaxMs)) {
        computeScores(false, tilt);
        if (!rep_.postEligible) {
          // No major motion (bump, pothole, quick flick): nothing to wait for.
          rep_.restTiltDeg = tilt;
          finish(t);
          ev |= DEV_VERDICT;
          return ev;
        }
        phase_ = DetPhase::SETTLE;
        phaseMs_ = t;
        rest_ = false;
      }
      break;
    }

    case DetPhase::SETTLE: {
      if (triggered && bits) {
        // Secondary impact / renewed tumbling: back to active tracking, keep peaks.
        phase_ = DetPhase::ACTIVE;
        phaseMs_ = t;
        calm_ = false;
        ev |= DEV_SECONDARY;
        trackPeaks(s, att, gBody, aMag, wMag);
        break;
      }
      const bool atRest = lpGyro_ < cfg_->restGyroDps && lpAccDev_ < cfg_->restAccTolG;
      if (!atRest) rest_ = false;
      else if (!rest_) { rest_ = true; restSinceMs_ = t; }
      if (rest_ && elapsed(t, restSinceMs_, cfg_->restConfirmMs)) {
        rep_.settled = true;
        rep_.restTiltDeg = tilt;
        finish(t);
        ev |= DEV_VERDICT;
        return ev;
      }
      if (elapsed(t, phaseMs_, cfg_->postMaxMs)) {
        rep_.settled = false;
        rep_.restTiltDeg = tilt;
        finish(t);
        ev |= DEV_VERDICT;
        return ev;
      }
      break;
    }
  }

  if (phase_ != DetPhase::IDLE) {
    computeScores(false, tilt);
    liveConf_ = rep_.confidence;
    if (!possibleRaised_ && rep_.confidence >= cfg_->possibleThreshold) {
      possibleRaised_ = true;
      ev |= DEV_POSSIBLE;
    }
  }
  return ev;
}

void CrashDetector::startEvent(uint32_t t, uint8_t trig) {
  memset(&rep_, 0, sizeof(rep_));
  rep_.startMs = t;
  rep_.triggers = trig;
  // Oldest history entry is ~1 s before the event.
  const uint8_t oldest = histCount_ < kHist ? 0 : histHead_;
  rep_.baselineTiltDeg = histCount_ ? tiltHist_[oldest] : 0;
  rep_.parked = longStillSeen_ && !elapsed(t, longStillMs_, kParkedGraceMs);
  rep_.peakAccG = preAcc_;
  rep_.verticalImpact = preVert_;
  rep_.peakGyroDps = preGyro_;
  phase_ = DetPhase::ACTIVE;
  phaseMs_ = t;
  calm_ = rest_ = false;
  possibleRaised_ = false;
}

void CrashDetector::trackPeaks(const ImuSample& s, const Attitude& att, const Vec3& gBody, float aMag, float wMag) {
  if (aMag > rep_.peakAccG) {
    rep_.peakAccG = aMag;
    rep_.verticalImpact = isVerticalImpact(s.acc, gBody, cfg_->verticalImpactFraction);
  }
  rep_.peakGyroDps = maxf(rep_.peakGyroDps, wMag);
  if (att.tiltDeg > rep_.peakTiltDeg) {
    rep_.peakTiltDeg = att.tiltDeg;
    rep_.peakRollDeg = att.rollDeg;
  }
  rep_.peakTiltChangeDeg = maxf(rep_.peakTiltChangeDeg, tiltChange_);
  if (wMag >= prof_->gyroNormalDps) rep_.sustainedRateMs += s.dt * 1000.0f;
  if (s.saturated) rep_.triggers |= TRIG_SATURATION;
}

void CrashDetector::computeScores(bool final, float currentTilt) {
  const VehicleProfile& p = *prof_;
  float* sc = rep_.scores;
  sc[IND_LEAN] = ramp(rep_.peakTiltDeg, p.leanNormalDeg, p.leanCrashDeg);
  sc[IND_RATE] = ramp(rep_.peakGyroDps, p.gyroNormalDps, p.gyroCrashDps);
  sc[IND_IMPACT] = ramp(rep_.peakAccG, p.impactNormalG, p.impactCrashG);
  if (rep_.verticalImpact && rep_.peakTiltDeg < p.leanNormalDeg) sc[IND_IMPACT] *= cfg_->verticalImpactWeight;
  sc[IND_CHANGE] = maxf(ramp(rep_.peakTiltChangeDeg, p.tiltChangeNormalDeg, p.tiltChangeCrashDeg),
                        ramp(rep_.sustainedRateMs, cfg_->sustainedRateNormalMs, cfg_->sustainedRateCrashMs));

  rep_.postEligible = maxf(sc[IND_RATE], maxf(sc[IND_IMPACT], sc[IND_CHANGE])) >= cfg_->majorMotionLevel;
  sc[IND_POST] = 0;
  // Post-impact orientation is only meaningful once the violent motion is over:
  // final verdict (resting tilt) or the settle phase (current tilt, half weight).
  if (rep_.postEligible && (final || phase_ == DetPhase::SETTLE)) {
    const float restTilt = final ? rep_.restTiltDeg : currentTilt;
    sc[IND_POST] = ramp(restTilt, p.restTiltNormalDeg, p.restTiltCrashDeg);
    if (!(final && rep_.settled)) sc[IND_POST] *= cfg_->unsettledPostFactor;
  }

  float conf = cfg_->wLean * sc[IND_LEAN] + cfg_->wRate * sc[IND_RATE] + cfg_->wImpact * sc[IND_IMPACT] +
               cfg_->wChange * sc[IND_CHANGE] + cfg_->wPost * sc[IND_POST];
  if (rep_.parked) conf *= cfg_->parkedFactor;
  rep_.confidence = clampf(conf, 0.0f, 1.0f);

  uint8_t active = 0;
  for (uint8_t i = 0; i < IND_COUNT; ++i)
    if (sc[i] >= cfg_->indicatorActiveLevel) ++active;
  rep_.activeIndicators = active;
}

void CrashDetector::finish(uint32_t t) {
  computeScores(true, rep_.restTiltDeg);
  const bool crash = rep_.confidence >= cfg_->crashThreshold && rep_.activeIndicators >= cfg_->minIndicators;
  rep_.verdict = crash ? DetVerdict::CRASH : DetVerdict::DISMISSED;
  rep_.endMs = t;
  phase_ = DetPhase::IDLE;
  anomalyShift_ = 0;
  pendingTrig_ = 0;
  liveConf_ = rep_.confidence;
  if (crash) {
    armed_ = false;            // no new detection until the bike is upright again
    uprightTiming_ = false;
  }
}
