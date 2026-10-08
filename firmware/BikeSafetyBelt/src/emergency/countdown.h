// Emergency countdown (non-blocking).
#pragma once

#include "../core/platform.h"

class Countdown {
 public:
  void start(uint32_t now, uint8_t seconds) { startMs_ = now; durationMs_ = (uint32_t)seconds * 1000UL; running_ = true; }
  void stop() { running_ = false; }
  bool running() const { return running_; }
  bool expired(uint32_t now) const { return running_ && elapsed(now, startMs_, durationMs_); }
  // Whole seconds remaining, rounded up (10, 9, ... 1, then 0 when expired).
  uint8_t remaining(uint32_t now) const {
    if (!running_) return 0;
    const uint32_t e = now - startMs_;
    if (e >= durationMs_) return 0;
    return (uint8_t)((durationMs_ - e + 999UL) / 1000UL);
  }

 private:
  uint32_t startMs_ = 0, durationMs_ = 0;
  bool running_ = false;
};
