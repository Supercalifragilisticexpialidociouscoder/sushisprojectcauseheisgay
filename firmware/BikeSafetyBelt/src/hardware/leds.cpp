#include "leds.h"

static const uint16_t kBlink2Hz[] BSB_PROGMEM = {250, 250};
static const uint16_t kBlink5Hz[] BSB_PROGMEM = {100, 100};
static const uint16_t kFlash2s[] BSB_PROGMEM = {100, 1900};
static const uint16_t kAlt1Hz[] BSB_PROGMEM = {500, 500};

void LedController::begin(uint8_t redPin, uint8_t greenPin, bool activeHigh) {
  redPin_ = redPin;
  greenPin_ = greenPin;
  activeHigh_ = activeHigh;
  pinMode(redPin_, OUTPUT);
  pinMode(greenPin_, OUTPUT);
  write(redPin_, false);
  write(greenPin_, false);
}

void LedController::write(uint8_t pin, bool on) { digitalWrite(pin, (on == activeHigh_) ? HIGH : LOW); }

void LedController::showState(SystemState s, uint32_t now) {
  if (haveState_ && s == shown_) return;
  haveState_ = true;
  shown_ = s;
  alternate_ = false;
  redP_.off();
  greenP_.off();
  switch (s) {
    case SystemState::BOOT:
    case SystemState::CALIBRATING: greenP_.play(kBlink2Hz, 2, true, now); break;
    case SystemState::SAFE:
    case SystemState::MONITORING:
    case SystemState::USER_CANCELLED: greenP_.steady(); break;
    case SystemState::POSSIBLE_CRASH: redP_.play(kBlink2Hz, 2, true, now); break;
    case SystemState::CRASH_DETECTED: redP_.play(kBlink5Hz, 2, true, now); break;
    case SystemState::SOS_SENT: redP_.play(kPatSos, kPatSosLen, true, now); break;
    case SystemState::RECOVERY: redP_.play(kFlash2s, 2, true, now); break;
    case SystemState::MANUAL_TEST: redP_.steady(); break;
    case SystemState::IMU_FAULT:
      redP_.play(kAlt1Hz, 2, true, now);
      alternate_ = true;
      break;
  }
  update(now);
}

void LedController::update(uint32_t now) {
  const bool r = redP_.level(now);
  const bool g = alternate_ ? !r : greenP_.level(now);
  if (r != red_) { red_ = r; write(redPin_, r); }
  if (g != green_) { green_ = g; write(greenPin_, g); }
}
