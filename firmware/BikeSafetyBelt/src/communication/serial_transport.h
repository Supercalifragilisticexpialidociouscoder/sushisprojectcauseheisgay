// USB serial transport (all boards). Carries human-readable logs always and
// JSON data while the app session is active (Web Serial in Chrome / Edge).
#pragma once

#include "transport.h"

#ifdef ARDUINO
class SerialTransport : public Transport {
 public:
  void begin() override;
  const char* pollLine() override;
  void write(const char* data, uint8_t len) override;
  void endLine() override;
  bool carriesLogs() const override { return true; }
  bool linkUp() const override { return true; }
  bool canWriteTelemetry() const override;
  FStr name() const override { return FS("USB"); }

 private:
  char rx_[64];
  uint8_t len_ = 0;
  bool overflow_ = false;
};
#endif
