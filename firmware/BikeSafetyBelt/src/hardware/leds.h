// Deterministic two-LED status display.
//
//   State              Green            Red
//   SAFE / MONITORING  ON               OFF
//   CALIBRATING        blink 2 Hz       OFF
//   POSSIBLE_CRASH     OFF              blink 2 Hz
//   CRASH_DETECTED     OFF              rapid blink 5 Hz (countdown)
//   SOS_SENT           OFF              Morse "SOS"
//   RECOVERY           OFF              short flash every 2 s
//   USER_CANCELLED     ON               OFF
//   MANUAL_TEST        OFF              ON (steady)
//   IMU_FAULT (ERROR)  alternating with red, 1 Hz
#pragma once

#include "../core/types.h"
#include "pattern.h"

class LedController {
 public:
  void begin(uint8_t redPin, uint8_t greenPin, bool activeHigh);
  void showState(SystemState s, uint32_t now);
  void update(uint32_t now);
  bool redOn() const { return red_; }
  bool greenOn() const { return green_; }

 private:
  void write(uint8_t pin, bool on);
  uint8_t redPin_ = 0, greenPin_ = 0;
  bool activeHigh_ = true;
  SystemState shown_ = SystemState::BOOT;
  bool haveState_ = false;
  bool alternate_ = false;   // green = !red (error pattern)
  PatternPlayer redP_, greenP_;
  bool red_ = false, green_ = false;
};
