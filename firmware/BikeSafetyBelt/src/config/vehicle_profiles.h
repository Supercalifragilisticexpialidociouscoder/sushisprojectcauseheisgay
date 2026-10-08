// Vehicle profiles and typed configuration structures.
// All numeric values come from config.h — this file only gives them structure.
#pragma once

#include "../core/types.h"

struct VehicleProfile {
  float leanNormalDeg, leanCrashDeg;
  float gyroNormalDps, gyroCrashDps;
  float impactNormalG, impactCrashG;
  float tiltChangeNormalDeg, tiltChangeCrashDeg;
  float restTiltNormalDeg, restTiltCrashDeg;
};

struct FusionConfig {
  float alpha;
  float alphaTurn;
  float accLpfAlpha;
  float accTrustBandG;
  float gyroTrustMaxDps;
  float turnRateMinDps;
  float steadyRollDps;
  float turnMaxLeanDeg;
};

struct DetectorConfig {
  float triggerImpactG, triggerGyroDps, triggerTiltRateDps;
  uint8_t triggerMinSamples, triggerWindow;
  float quietGyroDps, quietAccG;
  uint16_t quietMs, activeMaxMs;
  float restGyroDps, restAccTolG;
  uint16_t restConfirmMs, postMaxMs;
  float unsettledPostFactor;
  float wLean, wRate, wImpact, wChange, wPost;
  float sustainedRateNormalMs, sustainedRateCrashMs;
  float verticalImpactFraction, verticalImpactWeight;
  float majorMotionLevel, indicatorActiveLevel;
  uint8_t minIndicators;
  float possibleThreshold, crashThreshold;
  uint32_t parkedMinMs;
  float stillGyroDps, stillAccG, parkedFactor;
  uint16_t rearmUprightMs;
  float leanHysteresisDeg;
};

// Loads profile `id` (clamped to a valid id). Returns false if id was invalid.
bool loadVehicleProfile(uint8_t id, VehicleProfile& out);
FStr profileName(uint8_t id);
// Parses "scooter" / "standard" / "sport" (or "0".."2"). Returns PROFILE_COUNT on failure.
uint8_t parseProfileName(const char* s);

const FusionConfig& fusionConfig();
const DetectorConfig& detectorConfig();
