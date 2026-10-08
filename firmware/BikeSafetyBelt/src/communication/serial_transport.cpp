#include "serial_transport.h"

#ifdef ARDUINO
#include "../../config.h"

void SerialTransport::begin() { Serial.begin(SERIAL_BAUD); }

const char* SerialTransport::pollLine() {
  while (Serial.available() > 0) {
    const char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (overflow_) {          // drop over-long garbage lines entirely
        overflow_ = false;
        len_ = 0;
        continue;
      }
      if (len_ == 0) continue;
      rx_[len_] = '\0';
      len_ = 0;
      return rx_;
    }
    if (len_ < sizeof(rx_) - 1) rx_[len_++] = ch;
    else overflow_ = true;
  }
  return nullptr;
}

void SerialTransport::write(const char* data, uint8_t len) { Serial.write(reinterpret_cast<const uint8_t*>(data), len); }

void SerialTransport::endLine() { Serial.write('\n'); }

bool SerialTransport::canWriteTelemetry() const {
  // Only start a telemetry line when the TX buffer is (nearly) empty, so the
  // write blocks for at most a few milliseconds and never delays sampling much.
  return Serial.availableForWrite() >= 48;
}
#endif
