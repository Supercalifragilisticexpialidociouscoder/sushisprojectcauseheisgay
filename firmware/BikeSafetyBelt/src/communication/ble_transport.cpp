#include "ble_transport.h"

#if BSB_HAS_BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

namespace {
const char* kServiceUuid = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
const char* kRxUuid = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
const char* kTxUuid = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

BleTransport* gBle = nullptr;
BLEServer* gServer = nullptr;
BLECharacteristic* gTx = nullptr;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override { if (gBle) gBle->onConnect(); }
  void onDisconnect(BLEServer*) override { if (gBle) gBle->onDisconnect(); }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    // getValue() returns std::string (core 2.x) or String (core 3.x); both work here.
    auto v = c->getValue();
    if (gBle && v.length()) gBle->onRxBytes(reinterpret_cast<const uint8_t*>(v.c_str()), v.length());
  }
};
}  // namespace

void BleTransport::begin() {
  gBle = this;
  BLEDevice::init(name_);
  gServer = BLEDevice::createServer();
  gServer->setCallbacks(new ServerCallbacks());
  BLEService* svc = gServer->createService(kServiceUuid);
  gTx = svc->createCharacteristic(kTxUuid, BLECharacteristic::PROPERTY_NOTIFY);
  gTx->addDescriptor(new BLE2902());
  BLECharacteristic* rx =
      svc->createCharacteristic(kRxUuid, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCallbacks());
  svc->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(kServiceUuid);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
}

void BleTransport::service(uint32_t now) {
  (void)now;
  if (restartAdvertising_) {
    restartAdvertising_ = false;
    BLEDevice::startAdvertising();
  }
}

void BleTransport::onRxBytes(const uint8_t* data, size_t len) {
  uint16_t head = head_.load(std::memory_order_relaxed);
  for (size_t i = 0; i < len; ++i) {
    const uint16_t next = (uint16_t)((head + 1) % kRing);
    if (next == tail_.load(std::memory_order_acquire)) break;   // full: drop the rest
    ring_[head] = data[i];
    head = next;
  }
  head_.store(head, std::memory_order_release);
}

const char* BleTransport::pollLine() {
  uint16_t tail = tail_.load(std::memory_order_relaxed);
  const uint16_t head = head_.load(std::memory_order_acquire);
  while (tail != head) {
    const char ch = (char)ring_[tail];
    tail = (uint16_t)((tail + 1) % kRing);
    tail_.store(tail, std::memory_order_release);
    if (ch == '\n' || ch == '\r') {
      if (overflow_) { overflow_ = false; lineLen_ = 0; continue; }
      if (!lineLen_) continue;
      line_[lineLen_] = '\0';
      lineLen_ = 0;
      return line_;
    }
    if (lineLen_ < sizeof(line_) - 1) line_[lineLen_++] = ch;
    else overflow_ = true;
  }
  return nullptr;
}

void BleTransport::flushChunk() {
  if (!chunkLen_ || !gTx) return;
  if (connected_) {
    gTx->setValue(chunk_, chunkLen_);
    gTx->notify();
  }
  chunkLen_ = 0;
}

void BleTransport::write(const char* data, uint8_t len) {
  if (!connected_) return;
  for (uint8_t i = 0; i < len; ++i) {
    chunk_[chunkLen_++] = (uint8_t)data[i];
    if (chunkLen_ == sizeof(chunk_)) flushChunk();
  }
}

void BleTransport::endLine() {
  if (!connected_) { chunkLen_ = 0; return; }
  if (chunkLen_ == sizeof(chunk_)) flushChunk();
  chunk_[chunkLen_++] = '\n';
  flushChunk();
}
#endif  // BSB_HAS_BLE
