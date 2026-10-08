// Non-blocking on/off pattern player shared by the LEDs and the buzzer.
// A pattern is a list of durations in ms, alternating ON, OFF, ON, OFF ...
#pragma once

#include "../core/platform.h"

class PatternPlayer {
 public:
  void off() { steps_ = nullptr; count_ = 0; steady_ = false; }
  void steady() { steps_ = nullptr; count_ = 0; steady_ = true; }
  void play(const uint16_t* steps, uint8_t count, bool repeat, uint32_t now);
  bool level(uint32_t now) const;

 private:
  const uint16_t* steps_ = nullptr;   // PROGMEM on AVR
  uint8_t count_ = 0;
  bool repeat_ = false;
  bool steady_ = false;
  uint32_t startMs_ = 0;
  uint32_t total_ = 0;
};

// Shared timing tables
extern const uint16_t kPatSos[] BSB_PROGMEM;
static const uint8_t kPatSosLen = 18;
