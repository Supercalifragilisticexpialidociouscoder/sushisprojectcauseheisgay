#include "vehicle_profiles.h"
#include "../../config.h"

#include <string.h>

static const VehicleProfile kProfiles[PROFILE_COUNT] BSB_PROGMEM = {
  { PROFILE_SCOOTER_VALUES },
  { PROFILE_STANDARD_VALUES },
  { PROFILE_SPORT_VALUES },
};

bool loadVehicleProfile(uint8_t id, VehicleProfile& out) {
  bool ok = id < PROFILE_COUNT;
  if (!ok) id = PROFILE_STANDARD;
#if defined(ARDUINO_ARCH_AVR)
  memcpy_P(&out, &kProfiles[id], sizeof(VehicleProfile));
#else
  memcpy(&out, &kProfiles[id], sizeof(VehicleProfile));
#endif
  return ok;
}

FStr profileName(uint8_t id) {
  switch (id) {
    case PROFILE_SCOOTER: return FS("scooter");
    case PROFILE_SPORT: return FS("sport");
    case PROFILE_STANDARD: return FS("standard");
    default: return FS("unknown");
  }
}

static bool ieq(const char* a, const char* b) {
  while (*a && *b) {
    char ca = (*a >= 'A' && *a <= 'Z') ? (char)(*a + 32) : *a;
    char cb = (*b >= 'A' && *b <= 'Z') ? (char)(*b + 32) : *b;
    if (ca != cb) return false;
    ++a; ++b;
  }
  return *a == *b;
}

uint8_t parseProfileName(const char* s) {
  if (ieq(s, "scooter") || ieq(s, "0")) return PROFILE_SCOOTER;
  if (ieq(s, "standard") || ieq(s, "1")) return PROFILE_STANDARD;
  if (ieq(s, "sport") || ieq(s, "2")) return PROFILE_SPORT;
  return PROFILE_COUNT;
}

static const FusionConfig kFusion = {
  FUSION_ALPHA, FUSION_ALPHA_TURN, FUSION_ACC_LPF_ALPHA, FUSION_ACC_TRUST_BAND_G,
  FUSION_GYRO_TRUST_MAX_DPS, FUSION_TURN_RATE_MIN_DPS, FUSION_STEADY_ROLL_DPS,
  FUSION_TURN_MAX_LEAN_DEG,
};

static const DetectorConfig kDetector = {
  DET_TRIGGER_IMPACT_G, DET_TRIGGER_GYRO_DPS, DET_TRIGGER_TILT_RATE_DPS,
  DET_TRIGGER_MIN_SAMPLES, DET_TRIGGER_WINDOW,
  DET_QUIET_GYRO_DPS, DET_QUIET_ACC_G,
  DET_QUIET_MS, DET_ACTIVE_MAX_MS,
  DET_REST_GYRO_DPS, DET_REST_ACC_TOL_G,
  DET_REST_CONFIRM_MS, DET_POST_MAX_MS,
  DET_UNSETTLED_POST_FACTOR,
  DET_W_LEAN, DET_W_RATE, DET_W_IMPACT, DET_W_CHANGE, DET_W_POST,
  DET_SUSTAINED_RATE_NORMAL_MS, DET_SUSTAINED_RATE_CRASH_MS,
  DET_VERTICAL_IMPACT_FRACTION, DET_VERTICAL_IMPACT_WEIGHT,
  DET_MAJOR_MOTION_LEVEL, DET_INDICATOR_ACTIVE_LEVEL,
  DET_MIN_INDICATORS,
  DET_POSSIBLE_THRESHOLD, DET_CRASH_THRESHOLD,
  DET_PARKED_MIN_MS,
  DET_STILL_GYRO_DPS, DET_STILL_ACC_G, DET_PARKED_FACTOR,
  DET_REARM_UPRIGHT_MS,
  DET_LEAN_HYSTERESIS_DEG,
};

const FusionConfig& fusionConfig() { return kFusion; }
const DetectorConfig& detectorConfig() { return kDetector; }

// ---- Compile-time sanity checks on config.h --------------------------------
static_assert(DET_TRIGGER_WINDOW >= 1 && DET_TRIGGER_WINDOW <= 8, "DET_TRIGGER_WINDOW must be 1..8");
static_assert(DET_TRIGGER_MIN_SAMPLES >= 1 && DET_TRIGGER_MIN_SAMPLES <= DET_TRIGGER_WINDOW,
              "DET_TRIGGER_MIN_SAMPLES must be 1..DET_TRIGGER_WINDOW");
static_assert(DET_POSSIBLE_THRESHOLD < DET_CRASH_THRESHOLD, "possible threshold must be below crash threshold");
static_assert(DET_MIN_INDICATORS >= 1 && DET_MIN_INDICATORS <= 5, "DET_MIN_INDICATORS must be 1..5");
static_assert(FUSION_ALPHA > 0.0f && FUSION_ALPHA < 1.0f, "FUSION_ALPHA must be in (0,1)");
static_assert(COUNTDOWN_SECONDS >= 3 && COUNTDOWN_SECONDS <= 60, "COUNTDOWN_SECONDS must be 3..60");
static_assert(MPU_DLPF_CFG >= 0 && MPU_DLPF_CFG <= 6, "MPU_DLPF_CFG must be 0..6");
