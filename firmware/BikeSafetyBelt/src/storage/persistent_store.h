// Persistent storage (EEPROM on AVR, emulated EEPROM on ESP32).
//
// Holds: boot counter, device id, vehicle profile, level calibration, last good
// gyro bias, the incident that was active when power was lost (so a restart
// during a crash countdown / after an SOS is not silently forgotten), and a ring
// of the last incidents. Every block is CRC-protected.
//
// Writes are NON-BLOCKING: one byte is written per service() call when the
// EEPROM is idle (an AVR EEPROM byte write takes ~3.3 ms), so saving never
// stalls sensor sampling.
#pragma once

#include "../core/types.h"

class NvBackend {
 public:
  virtual uint16_t size() const = 0;
  virtual uint8_t read(uint16_t addr) = 0;
  virtual bool ready() = 0;                       // a byte write may start now
  virtual void write(uint16_t addr, uint8_t v) = 0;
  virtual void commit() {}                        // ESP32: flush RAM copy to flash
};

#ifdef ARDUINO
class EepromBackend : public NvBackend {
 public:
  void begin();
  uint16_t size() const override;
  uint8_t read(uint16_t addr) override;
  bool ready() override;
  void write(uint16_t addr, uint8_t v) override;
  void commit() override;
};
#endif

enum ActiveIncidentState : uint8_t { ACTIVE_NONE = 0, ACTIVE_COUNTDOWN = 1, ACTIVE_SOS = 2 };

struct StoredSettings {
  uint16_t magic;
  uint8_t version;
  uint8_t profile;
  uint32_t bootCount;
  uint16_t deviceId;
  uint8_t flags;            // bit0 level valid, bit1 gyro bias valid
  uint8_t activeState;      // ActiveIncidentState
  uint32_t activeId;
  Vec3 levelUp;
  Vec3 gyroBias;
  uint8_t ringHead;         // next slot to write
  uint8_t ringCount;
  uint8_t reserved;
  uint8_t crc;
};

class PersistentStore {
 public:
  static const uint8_t kRingSize = 6;
  static const uint8_t FLAG_LEVEL = 0x01, FLAG_GYRO = 0x02;

  // Loads settings; returns false if storage was blank/corrupt (defaults applied).
  bool begin(NvBackend* nv, uint16_t randomSeed);
  void service();                       // call every loop
  bool busy() const { return settingsDirty_ || recDirty_; }
  void flushBlocking();                 // only used when a write must not be lost

  StoredSettings& settings() { return s_; }
  void saveSettings();                  // queue settings write

  void setActiveIncident(uint8_t state, uint32_t id);
  void saveIncident(const IncidentRecord& rec);
  bool findIncident(uint32_t id, IncidentRecord& out);
  uint8_t incidentCount() const { return s_.ringCount; }
  bool incidentAt(uint8_t newestIndex, IncidentRecord& out);   // 0 = newest
  void clearIncidents();

 private:
  uint16_t slotAddr(uint8_t slot) const;
  bool readRecord(uint8_t slot, IncidentRecord& out);
  bool writeStep();

  NvBackend* nv_ = nullptr;
  StoredSettings s_;
  bool settingsDirty_ = false;
  // One pending record write (record + crc)
  uint8_t rec_[sizeof(IncidentRecord) + 1];
  uint16_t recAddr_ = 0;
  bool recDirty_ = false;
  uint32_t lastSlotId_ = 0;
  uint8_t lastSlot_ = 0xFF;
};

uint8_t crc8(const uint8_t* data, uint16_t len);
