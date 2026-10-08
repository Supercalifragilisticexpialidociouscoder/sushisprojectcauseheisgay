// Buzzer status patterns (non-blocking).
//
//   SAFE / MONITORING   off
//   POSSIBLE_CRASH      two short beeps every 1.5 s
//   CRASH_DETECTED      250 ms beep every second; last 3 s rapid beeping
//   SOS_SENT            Morse "SOS" (distinct emergency pattern)
//   RECOVERY            short reminder chirp every 20 s
//   USER_CANCELLED      two short acknowledgement beeps (once)
//   MANUAL_TEST         continuous (prototype behaviour; configurable)
//   IMU_FAULT           one long beep, then a chirp every 30 s
#pragma once

#include "../core/types.h"
#include "pattern.h"

class BuzzerController {
 public:
  void begin(uint8_t pin);
  void showState(SystemState s, uint8_t countdownRemainingS, uint32_t now);
  void chirp(uint32_t now);              // short self-test / acknowledgement beep
  void update(uint32_t now);
  bool on() const { return on_; }

 private:
  enum Mode : uint8_t { M_OFF, M_POSSIBLE, M_COUNTDOWN, M_COUNTDOWN_FINAL, M_SOS, M_RECOVERY,
                        M_CANCELLED, M_TEST, M_FAULT, M_CHIRP };
  void setMode(uint8_t m, uint32_t now);
  void drive(bool on);
  uint8_t pin_ = 0;
  uint8_t mode_ = 0xFF;
  PatternPlayer p_;
  bool on_ = false;
};
