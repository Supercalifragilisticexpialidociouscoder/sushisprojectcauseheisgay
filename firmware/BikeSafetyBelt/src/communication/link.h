// Link manager: fans output out to up to two transports and tracks the app
// session on each one.
//
// Session protocol: the app sends "HELLO" when it connects and "PING" at least
// every second. JSON data is streamed only while a session is alive, so a plain
// serial monitor sees clean human-readable logs. If no line arrives for
// APP_HEARTBEAT_TIMEOUT_MS the session is considered lost (communication failure).
#pragma once

#include "transport.h"

class Link : public LineSink {
 public:
  static const uint8_t kMax = 2;

  void begin(Transport* a, Transport* b);
  void service(uint32_t now);
  // Returns the next received command line (and its transport index) or nullptr.
  const char* poll(uint32_t now, uint8_t& src);

  void startSession(uint8_t i, uint32_t now);
  bool sessionActive(uint8_t i) const { return i < kMax && session_[i]; }
  bool anySession() const { return session_[0] || session_[1]; }
  // Returns a bitmask of transports whose session just ended (heartbeat lost).
  uint8_t takeLostSessions() { uint8_t m = lost_; lost_ = 0; return m; }

  void suspendData(uint32_t now, uint32_t ms) { suspendUntil_ = now + ms; suspended_ = true; }
  bool dataSuspended() const { return suspended_; }

  Transport* transport(uint8_t i) { return i < kMax ? t_[i] : nullptr; }
  bool telemetryWritable(uint8_t i) const { return t_[i] && t_[i]->canWriteTelemetry(); }

  // LineSink
  bool beginLine(Channel ch, uint8_t transportMask) override;
  void write(const char* data, uint8_t len) override;
  void endLine() override;

 private:
  Transport* t_[kMax] = {nullptr, nullptr};
  bool session_[kMax] = {false, false};
  uint32_t lastHeard_[kMax] = {0, 0};
  uint8_t lost_ = 0;
  uint8_t cur_ = 0;   // transports receiving the current line
  bool suspended_ = false;
  uint32_t suspendUntil_ = 0;
};
