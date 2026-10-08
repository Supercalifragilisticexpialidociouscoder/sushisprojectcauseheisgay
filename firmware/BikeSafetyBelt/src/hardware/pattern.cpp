#include "pattern.h"

// Morse "SOS": dot 150, dash 450, element gap 150, letter gap 450, word gap 1050 ms
const uint16_t kPatSos[] BSB_PROGMEM = {
  150, 150, 150, 150, 150, 450,
  450, 150, 450, 150, 450, 450,
  150, 150, 150, 150, 150, 1050,
};

void PatternPlayer::play(const uint16_t* steps, uint8_t count, bool repeat, uint32_t now) {
  steps_ = steps;
  count_ = count;
  repeat_ = repeat;
  steady_ = false;
  startMs_ = now;
  total_ = 0;
  for (uint8_t i = 0; i < count; ++i) total_ += bsbReadU16(&steps[i]);
}

bool PatternPlayer::level(uint32_t now) const {
  if (steady_) return true;
  if (!steps_ || count_ == 0 || total_ == 0) return false;
  uint32_t t = now - startMs_;
  if (t >= total_) {
    if (!repeat_) return false;
    t %= total_;
  }
  for (uint8_t i = 0; i < count_; ++i) {
    const uint16_t d = bsbReadU16(&steps_[i]);
    if (t < d) return (i % 2) == 0;
    t -= d;
  }
  return false;
}
