// Small streaming formatter for log lines and JSON messages. No heap, no
// printf (AVR printf has no float support), fixed 48-byte staging buffer.
#pragma once

#include "transport.h"

class LineWriter {
 public:
  explicit LineWriter(LineSink& sink) : sink_(sink) {}

  bool begin(Channel ch, uint8_t transportMask = 0xFF);
  bool active() const { return active_; }

  // Plain text
  LineWriter& s(FStr str);
  LineWriter& r(const char* ramStr);
  LineWriter& c(char ch);
  LineWriter& i(int32_t v);
  LineWriter& u(uint32_t v);
  LineWriter& f(float v, uint8_t decimals);
  LineWriter& hex2(uint8_t v);
  LineWriter& e7(int32_t v);          // degrees*1e7 -> "12.3456789"
  void end();

  // Log line helper: "[TAG] "
  bool log(FStr tag);

  // JSON helpers
  bool obj(FStr type, uint8_t transportMask = 0xFF);   // {"t":"type"
  LineWriter& k(FStr key);                                // ,"key":
  LineWriter& ks(FStr key, FStr val);
  LineWriter& kr(FStr key, const char* val);
  LineWriter& ki(FStr key, int32_t v) { k(key); return i(v); }
  LineWriter& ku(FStr key, uint32_t v) { k(key); return u(v); }
  LineWriter& kf(FStr key, float v, uint8_t dec) { k(key); return jf(v, dec); }
  LineWriter& kb(FStr key, bool v) { k(key); return c(v ? '1' : '0'); }
  LineWriter& jf(float v, uint8_t dec);    // JSON number (NaN/Inf -> null)
  void endObj() { c('}'); end(); }

 private:
  void put(char ch);
  void flush();
  LineSink& sink_;
  char buf_[48];
  uint8_t len_ = 0;
  bool active_ = false;
  bool json_ = false;
};
