// Push button with debouncing and edge detection.
//   * PRESS fires once on the debounced press edge (holding never repeats it)
//   * LONG fires once when held for BUTTON_LONG_PRESS_MS
//   * A button already held at power-up is ignored until released
//   * A button held for BUTTON_STUCK_MS is reported stuck and ignored until released
#pragma once

#include "../core/platform.h"

enum ButtonEvent : uint8_t {
  BTN_NONE = 0,
  BTN_PRESS = 0x01,
  BTN_LONG = 0x02,
  BTN_RELEASE = 0x04,
  BTN_STUCK = 0x08
};

// Pure logic (unit-tested natively).
class ButtonDebouncer {
 public:
  void begin(uint16_t debounceMs, uint16_t longMs, uint16_t stuckMs, bool initiallyPressed, uint32_t now);
  uint8_t update(bool rawPressed, uint32_t now);
  bool pressed() const { return stable_; }

 private:
  uint16_t debounceMs_ = 30, longMs_ = 3000, stuckMs_ = 15000;
  bool stable_ = false, lastRaw_ = false;
  bool longFired_ = false, ignore_ = false;
  uint32_t rawChangeMs_ = 0, pressMs_ = 0;
};

class Button {
 public:
  void begin(uint8_t pin, bool activeLow, uint32_t now);
  uint8_t update(uint32_t now);
  bool pressed() const { return deb_.pressed(); }
 private:
  bool readRaw() const;
  uint8_t pin_ = 0;
  bool activeLow_ = true;
  ButtonDebouncer deb_;
};
