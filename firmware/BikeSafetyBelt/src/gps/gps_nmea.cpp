#include "gps_nmea.h"
#include "../../config.h"

#include <string.h>

static int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

static uint32_t parseUint(const char* s, uint8_t maxDigits, uint8_t* used) {
  uint32_t v = 0;
  uint8_t n = 0;
  while (n < maxDigits && s[n] >= '0' && s[n] <= '9') {
    v = v * 10 + (uint32_t)(s[n] - '0');
    ++n;
  }
  if (used) *used = n;
  return v;
}

static float parseFloat(const char* s, bool* ok) {
  float v = 0, scale = 0.1f;
  bool any = false, frac = false;
  for (; *s; ++s) {
    if (*s >= '0' && *s <= '9') {
      any = true;
      if (frac) { v += (float)(*s - '0') * scale; scale *= 0.1f; }
      else v = v * 10.0f + (float)(*s - '0');
    } else if (*s == '.' && !frac) {
      frac = true;
    } else {
      break;
    }
  }
  if (ok) *ok = any;
  return v;
}

bool nmeaParseCoord(const char* s, char hemi, int32_t& outE7) {
  const char* dot = strchr(s, '.');
  const size_t intLen = dot ? (size_t)(dot - s) : strlen(s);
  if (intLen < 4 || intLen > 5) return false;          // ddmm or dddmm
  const uint8_t degDigits = (uint8_t)(intLen - 2);
  uint8_t used = 0;
  const uint32_t deg = parseUint(s, degDigits, &used);
  if (used != degDigits) return false;
  const uint32_t minInt = parseUint(s + degDigits, 2, &used);
  if (used != 2 || minInt >= 60) return false;
  // Fractional minutes normalised to 5 digits
  uint32_t minFrac = 0;
  uint8_t fracDigits = 0;
  if (dot) {
    const char* f = dot + 1;
    while (*f >= '0' && *f <= '9' && fracDigits < 5) {
      minFrac = minFrac * 10 + (uint32_t)(*f - '0');
      ++fracDigits;
      ++f;
    }
  }
  while (fracDigits < 5) { minFrac *= 10; ++fracDigits; }
  const uint32_t minutesE5 = minInt * 100000UL + minFrac;     // <= 5,999,999
  int32_t v = (int32_t)(deg * 10000000UL + (minutesE5 * 5UL) / 3UL);   // minutes/60 in 1e7 units
  if (hemi == 'S' || hemi == 'W') v = -v;
  else if (hemi != 'N' && hemi != 'E') return false;
  const int32_t limit = (degDigits == 2) ? 900000000L : 1800000000L;
  if (v > limit || v < -limit) return false;
  outE7 = v;
  return true;
}

void NmeaParser::reset() {
  len_ = 0;
  inSentence_ = false;
  memset(&fix_, 0, sizeof(fix_));
  fix_.hdop = 99.0f;
}

bool NmeaParser::feed(char c, uint32_t nowMs) {
  if (c == '$') {
    inSentence_ = true;
    len_ = 0;
    return false;
  }
  if (!inSentence_) return false;
  if (c == '\r' || c == '\n') {
    inSentence_ = false;
    buf_[len_] = '\0';
    // Verify "*HH" checksum
    char* star = strchr(buf_, '*');
    if (!star || hexVal(star[1]) < 0 || hexVal(star[2]) < 0) { ++csErrors_; return false; }
    uint8_t cs = 0;
    for (const char* p = buf_; p < star; ++p) cs ^= (uint8_t)*p;
    if (cs != (uint8_t)(hexVal(star[1]) * 16 + hexVal(star[2]))) { ++csErrors_; return false; }
    *star = '\0';
    ++sentences_;
    process(nowMs);
    return true;
  }
  if (len_ < sizeof(buf_) - 1) buf_[len_++] = c;
  else inSentence_ = false;   // too long: not valid NMEA
  return false;
}

void NmeaParser::process(uint32_t nowMs) {
  // Split into fields in place.
  const uint8_t kMaxFields = 16;
  char* field[kMaxFields];
  uint8_t n = 0;
  field[n++] = buf_;
  for (char* p = buf_; *p && n < kMaxFields; ++p) {
    if (*p == ',') { *p = '\0'; field[n++] = p + 1; }
  }
  if (strlen(field[0]) != 5) return;
  const char* type = field[0] + 2;   // skip talker (GP, GN, GL, ...)

  if (strcmp(type, "RMC") == 0 && n >= 10) {
    const bool active = field[2][0] == 'A';
    int32_t lat = 0, lon = 0;
    const bool coordOk = nmeaParseCoord(field[3], field[4][0], lat) && nmeaParseCoord(field[5], field[6][0], lon);
    fix_.rmcActive = active && coordOk;
    if (fix_.rmcActive) {
      fix_.latE7 = lat;
      fix_.lonE7 = lon;
      fix_.updatedMs = nowMs;
    }
    uint8_t u1 = 0, u2 = 0;
    const uint32_t hms = parseUint(field[1], 6, &u1);
    const uint32_t dmy = parseUint(field[9], 6, &u2);
    fix_.timeValid = u1 == 6 && u2 == 6;
    if (fix_.timeValid) {
      fix_.hour = (uint8_t)(hms / 10000); fix_.minute = (uint8_t)((hms / 100) % 100); fix_.second = (uint8_t)(hms % 100);
      fix_.day = (uint8_t)(dmy / 10000); fix_.month = (uint8_t)((dmy / 100) % 100); fix_.year = (uint8_t)(dmy % 100);
    }
  } else if (strcmp(type, "GGA") == 0 && n >= 9) {
    fix_.ggaSeen = true;
    fix_.quality = (uint8_t)parseUint(field[6], 1, nullptr);
    fix_.satellites = (uint8_t)parseUint(field[7], 2, nullptr);
    bool ok = false;
    const float h = parseFloat(field[8], &ok);
    fix_.hdop = ok ? h : 99.0f;
  }
}

bool NmeaParser::hasValidFix(uint32_t nowMs) const {
  if (lossActive_ && (int32_t)(nowMs - lossUntil_) < 0) return false;
  if (!fix_.rmcActive || elapsed(nowMs, fix_.updatedMs, GPS_MAX_FIX_AGE_MS)) return false;
  if (fix_.ggaSeen) {
    if (fix_.quality < 1 || fix_.satellites < GPS_MIN_SATELLITES || fix_.hdop > GPS_MAX_HDOP) return false;
  }
  return true;
}

float NmeaParser::accuracyM() const {
  if (!fix_.ggaSeen) return -1.0f;
  return fix_.hdop * GPS_UERE_M;
}
