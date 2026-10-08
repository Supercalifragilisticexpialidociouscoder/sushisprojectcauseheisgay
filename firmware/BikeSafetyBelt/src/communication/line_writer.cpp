#include "line_writer.h"

bool LineWriter::begin(Channel ch, uint8_t transportMask) {
  len_ = 0;
  json_ = ch == Channel::DATA;
  active_ = sink_.beginLine(ch, transportMask);
  return active_;
}

void LineWriter::put(char ch) {
  if (!active_) return;
  buf_[len_++] = ch;
  if (len_ >= sizeof(buf_)) flush();
}

void LineWriter::flush() {
  if (len_ && active_) sink_.write(buf_, len_);
  len_ = 0;
}

void LineWriter::end() {
  if (!active_) return;
  flush();
  sink_.endLine();
  active_ = false;
}

LineWriter& LineWriter::s(FStr str) {
  if (!active_ || !str) return *this;
  for (size_t n = 0;; ++n) {
    const char ch = fsRead(str, n);
    if (!ch) break;
    put(ch);
  }
  return *this;
}

LineWriter& LineWriter::r(const char* str) {
  if (!active_ || !str) return *this;
  while (*str) {
    char ch = *str++;
    if (json_ && (ch == '"' || ch == '\\')) put('\\');
    if ((uint8_t)ch < 0x20) ch = ' ';
    put(ch);
  }
  return *this;
}

LineWriter& LineWriter::c(char ch) {
  put(ch);
  return *this;
}

LineWriter& LineWriter::u(uint32_t v) {
  if (!active_) return *this;
  char tmp[11];
  uint8_t n = 0;
  do {
    tmp[n++] = (char)('0' + v % 10);
    v /= 10;
  } while (v);
  while (n) put(tmp[--n]);
  return *this;
}

LineWriter& LineWriter::i(int32_t v) {
  if (v < 0) {
    put('-');
    return u((uint32_t)(-(v + 1)) + 1u);
  }
  return u((uint32_t)v);
}

LineWriter& LineWriter::f(float v, uint8_t dec) {
  if (!active_) return *this;
  if (isnan(v) || isinf(v)) return s(FS("nan"));
  if (dec > 4) dec = 4;
  uint32_t scale = 1;
  for (uint8_t d = 0; d < dec; ++d) scale *= 10;
  const bool neg = v < 0;
  float a = neg ? -v : v;
  if (a > 4.0e6f) a = 4.0e6f;
  const uint32_t scaled = (uint32_t)(a * (float)scale + 0.5f);
  if (neg && scaled) put('-');
  u(scaled / scale);
  if (dec) {
    put('.');
    uint32_t frac = scaled % scale;
    for (uint32_t p = scale / 10; p >= 1; p /= 10) {
      put((char)('0' + (frac / p) % 10));
      if (p == 1) break;
    }
  }
  return *this;
}

LineWriter& LineWriter::jf(float v, uint8_t dec) {
  if (isnan(v) || isinf(v)) return s(FS("null"));
  return f(v, dec);
}

LineWriter& LineWriter::hex2(uint8_t v) {
  static const char kHex[] = "0123456789ABCDEF";
  put(kHex[v >> 4]);
  put(kHex[v & 0x0F]);
  return *this;
}

LineWriter& LineWriter::e7(int32_t v) {
  if (v < 0) put('-');
  const uint32_t a = v < 0 ? (uint32_t)(-(v + 1)) + 1u : (uint32_t)v;
  u(a / 10000000UL);
  put('.');
  uint32_t frac = a % 10000000UL;
  for (uint32_t p = 1000000UL; p >= 1; p /= 10) {
    put((char)('0' + (frac / p) % 10));
    if (p == 1) break;
  }
  return *this;
}

bool LineWriter::log(FStr tag) {
  if (!begin(Channel::LOG)) return false;
  c('[').s(tag).c(']').c(' ');
  return true;
}

bool LineWriter::obj(FStr type, uint8_t transportMask) {
  if (!begin(Channel::DATA, transportMask)) return false;
  s(FS("{\"t\":\"")).s(type).c('"');
  return true;
}

LineWriter& LineWriter::k(FStr key) {
  c(',').c('"').s(key).c('"').c(':');
  return *this;
}

LineWriter& LineWriter::ks(FStr key, FStr val) {
  k(key);
  c('"').s(val).c('"');
  return *this;
}

LineWriter& LineWriter::kr(FStr key, const char* val) {
  k(key);
  c('"').r(val).c('"');
  return *this;
}
