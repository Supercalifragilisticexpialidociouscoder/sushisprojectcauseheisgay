// Bluetooth Low Energy transport (ESP32 only) using the Nordic UART Service
// (NUS), which Web Bluetooth (Chrome on Android / desktop) and most BLE
// terminal apps understand. Carries the same JSON line protocol as USB serial.
//
//   Service 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
//   RX      6E400002-...  (app -> device, write)
//   TX      6E400003-...  (device -> app, notify)
//
// BLE callbacks run in the Bluetooth task, not in loop(): received bytes go
// through a single-producer / single-consumer ring buffer so there is no data
// race with the main loop.
#pragma once

#include "transport.h"
#include "../../config.h"

#if BLE_ENABLED && defined(ARDUINO_ARCH_ESP32)
#define BSB_HAS_BLE 1
#include <atomic>

class BleTransport : public Transport {
 public:
  explicit BleTransport(const char* deviceName) : name_(deviceName) {}
  void begin() override;
  void service(uint32_t now) override;
  const char* pollLine() override;
  void write(const char* data, uint8_t len) override;
  void endLine() override;
  bool carriesLogs() const override { return false; }
  bool linkUp() const override { return connected_; }
  FStr name() const override { return FS("BLE"); }

  // Called from BLE callbacks (other task)
  void onRxBytes(const uint8_t* data, size_t len);
  void onConnect() { connected_ = true; }
  void onDisconnect() { connected_ = false; restartAdvertising_ = true; }

 private:
  void flushChunk();
  const char* name_;
  std::atomic<bool> connected_{false};
  std::atomic<bool> restartAdvertising_{false};
  // RX ring buffer (producer: BLE task, consumer: loop)
  static const uint16_t kRing = 128;
  uint8_t ring_[kRing];
  std::atomic<uint16_t> head_{0}, tail_{0};
  char line_[64];
  uint8_t lineLen_ = 0;
  bool overflow_ = false;
  // TX: notifications of at most 20 bytes (default ATT MTU) for compatibility
  uint8_t chunk_[20];
  uint8_t chunkLen_ = 0;
};
#else
#define BSB_HAS_BLE 0
#endif
