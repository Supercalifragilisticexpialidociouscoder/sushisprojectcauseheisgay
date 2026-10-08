// Transport abstraction. The crash-detection code never talks to a transport
// directly; it only produces messages. Adding Wi-Fi, LoRa, GSM, ... means
// implementing this interface — nothing else changes.
#pragma once

#include "../core/platform.h"

enum class Channel : uint8_t {
  LOG = 0,    // human-readable "[TAG] message" lines
  DATA = 1    // JSON lines for the app (only sent while an app session is active)
};

class Transport {
 public:
  virtual void begin() = 0;
  virtual void service(uint32_t now) { (void)now; }
  // Returns a complete received line (no line terminator) or nullptr.
  virtual const char* pollLine() = 0;
  virtual void write(const char* data, uint8_t len) = 0;
  virtual void endLine() = 0;
  virtual bool carriesLogs() const = 0;        // human-readable log lines go here too
  virtual bool linkUp() const = 0;             // physical link present (BLE connected)
  virtual bool canWriteTelemetry() const { return true; }  // TX has room (non-blocking)
  virtual FStr name() const = 0;
};

// Line-oriented output sink used by LineWriter.
class LineSink {
 public:
  // Starts a line; returns false when nobody would receive it.
  virtual bool beginLine(Channel ch, uint8_t transportMask) = 0;
  virtual void write(const char* data, uint8_t len) = 0;
  virtual void endLine() = 0;
};
