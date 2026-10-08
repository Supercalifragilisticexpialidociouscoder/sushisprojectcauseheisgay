#include "buzzer.h"
#include "../../config.h"

static const uint16_t kPossible[] BSB_PROGMEM = {80, 120, 80, 1220};
static const uint16_t kCountdown[] BSB_PROGMEM = {250, 750};
static const uint16_t kCountdownFinal[] BSB_PROGMEM = {120, 130};
static const uint16_t kRecovery[] BSB_PROGMEM = {40, 19960};
static const uint16_t kCancelled[] BSB_PROGMEM = {80, 100, 80};
static const uint16_t kFault[] BSB_PROGMEM = {600, 29400};
static const uint16_t kTestBeep[] BSB_PROGMEM = {200, 200};
static const uint16_t kChirp[] BSB_PROGMEM = {60};

void BuzzerController::begin(uint8_t pin) {
  pin_ = pin;
  pinMode(pin_, OUTPUT);
  on_ = true;          // force a write
  drive(false);
}

void BuzzerController::drive(bool on) {
  if (!BUZZER_ENABLED) on = false;
  if (on == on_) return;
  on_ = on;
#if BUZZER_PASSIVE
  if (on) tone(pin_, BUZZER_FREQ_HZ);
  else noTone(pin_);
#else
  digitalWrite(pin_, (on == (BUZZER_ACTIVE_HIGH != 0)) ? HIGH : LOW);
#endif
}

void BuzzerController::setMode(uint8_t m, uint32_t now) {
  if (m == mode_) return;
  mode_ = m;
  switch (m) {
    case M_POSSIBLE: p_.play(kPossible, 4, true, now); break;
    case M_COUNTDOWN: p_.play(kCountdown, 2, true, now); break;
    case M_COUNTDOWN_FINAL: p_.play(kCountdownFinal, 2, true, now); break;
    case M_SOS: p_.play(kPatSos, kPatSosLen, true, now); break;
    case M_RECOVERY: p_.play(kRecovery, 2, true, now); break;
    case M_CANCELLED: p_.play(kCancelled, 3, false, now); break;
    case M_FAULT: p_.play(kFault, 2, true, now); break;
    case M_CHIRP: p_.play(kChirp, 1, false, now); break;
    case M_TEST:
      if (BUZZER_TEST_CONTINUOUS) p_.steady();
      else p_.play(kTestBeep, 2, true, now);
      break;
    default: p_.off(); break;
  }
}

void BuzzerController::showState(SystemState s, uint8_t countdownRemainingS, uint32_t now) {
  switch (s) {
    case SystemState::POSSIBLE_CRASH: setMode(M_POSSIBLE, now); break;
    case SystemState::CRASH_DETECTED:
      setMode(countdownRemainingS <= 3 ? M_COUNTDOWN_FINAL : M_COUNTDOWN, now);
      break;
    case SystemState::SOS_SENT: setMode(M_SOS, now); break;
    case SystemState::RECOVERY: setMode(M_RECOVERY, now); break;
    case SystemState::USER_CANCELLED: setMode(M_CANCELLED, now); break;
    case SystemState::MANUAL_TEST: setMode(M_TEST, now); break;
    case SystemState::IMU_FAULT: setMode(M_FAULT, now); break;
    default:
      if (mode_ != M_CHIRP) setMode(M_OFF, now);   // let a chirp finish
      break;
  }
}

void BuzzerController::chirp(uint32_t now) {
  mode_ = 0xFF;
  setMode(M_CHIRP, now);
}

void BuzzerController::update(uint32_t now) { drive(p_.level(now)); }
