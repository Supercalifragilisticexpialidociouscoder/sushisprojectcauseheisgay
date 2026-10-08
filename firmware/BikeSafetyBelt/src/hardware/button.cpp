#include "button.h"
#include "../../config.h"

void ButtonDebouncer::begin(uint16_t debounceMs, uint16_t longMs, uint16_t stuckMs, bool initiallyPressed,
                            uint32_t now) {
  debounceMs_ = debounceMs;
  longMs_ = longMs;
  stuckMs_ = stuckMs;
  stable_ = lastRaw_ = initiallyPressed;
  ignore_ = initiallyPressed;   // must be released before it counts
  longFired_ = false;
  rawChangeMs_ = pressMs_ = now;
}

uint8_t ButtonDebouncer::update(bool raw, uint32_t now) {
  uint8_t ev = BTN_NONE;
  if (raw != lastRaw_) {
    lastRaw_ = raw;
    rawChangeMs_ = now;
  }
  if (raw != stable_ && elapsed(now, rawChangeMs_, debounceMs_)) {
    stable_ = raw;
    if (stable_) {
      pressMs_ = now;
      longFired_ = false;
      if (!ignore_) ev |= BTN_PRESS;
    } else {
      if (!ignore_) ev |= BTN_RELEASE;
      ignore_ = false;
    }
  }
  if (stable_ && !ignore_) {
    if (!longFired_ && elapsed(now, pressMs_, longMs_)) {
      longFired_ = true;
      ev |= BTN_LONG;
    }
    if (elapsed(now, pressMs_, stuckMs_)) {
      ignore_ = true;
      ev |= BTN_STUCK;
    }
  }
  return ev;
}

void Button::begin(uint8_t pin, bool activeLow, uint32_t now) {
  pin_ = pin;
  activeLow_ = activeLow;
  pinMode(pin_, activeLow ? INPUT_PULLUP : INPUT);
  deb_.begin(BUTTON_DEBOUNCE_MS, BUTTON_LONG_PRESS_MS, BUTTON_STUCK_MS, readRaw(), now);
}

bool Button::readRaw() const {
  const int v = digitalRead(pin_);
  return activeLow_ ? (v == LOW) : (v == HIGH);
}

uint8_t Button::update(uint32_t now) { return deb_.update(readRaw(), now); }
