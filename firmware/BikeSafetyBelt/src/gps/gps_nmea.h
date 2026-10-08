// Minimal, allocation-free NMEA-0183 parser (RMC + GGA) for an optional GPS
// module. Coordinates are kept as integer degrees * 1e7 (float has too little
// precision on AVR). A fix is reported valid ONLY when:
//   * RMC status is 'A' (active) and coordinates parsed, checksum correct
//   * the fix is newer than GPS_MAX_FIX_AGE_MS
//   * if GGA is available: fix quality >= 1, satellites >= GPS_MIN_SATELLITES,
//     HDOP <= GPS_MAX_HDOP
// Accuracy is an ESTIMATE (HDOP x UERE), never a guarantee.
#pragma once

#include "../core/platform.h"

struct GpsFix {
  int32_t latE7, lonE7;
  float hdop;
  uint8_t satellites;
  uint8_t quality;
  bool rmcActive;
  bool ggaSeen;
  uint32_t updatedMs;    // millis() of the last RMC fix
  // UTC from RMC
  bool timeValid;
  uint8_t year, month, day, hour, minute, second;   // year = 2000 + year
};

class NmeaParser {
 public:
  void reset();
  // Feed one received character. Returns true when a sentence was accepted.
  bool feed(char c, uint32_t nowMs);
  bool hasValidFix(uint32_t nowMs) const;
  float accuracyM() const;    // estimated horizontal accuracy (m), -1 if unknown
  const GpsFix& fix() const { return fix_; }
  uint32_t sentences() const { return sentences_; }
  uint32_t checksumErrors() const { return csErrors_; }
  // Test hook: force "no fix" until nowMs + ms (GPS-unavailable test).
  void forceLoss(uint32_t nowMs, uint32_t ms) { lossUntil_ = nowMs + ms; lossActive_ = true; }

 private:
  void process(uint32_t nowMs);
  char buf_[83];
  uint8_t len_ = 0;
  bool inSentence_ = false;
  GpsFix fix_ = {0, 0, 99.0f, 0, 0, false, false, 0, false, 0, 0, 0, 0, 0, 0};
  uint32_t sentences_ = 0, csErrors_ = 0;
  bool lossActive_ = false;
  uint32_t lossUntil_ = 0;
};

// Parses "ddmm.mmmmm" / "dddmm.mmmmm" + hemisphere into degrees * 1e7.
bool nmeaParseCoord(const char* s, char hemi, int32_t& outE7);
