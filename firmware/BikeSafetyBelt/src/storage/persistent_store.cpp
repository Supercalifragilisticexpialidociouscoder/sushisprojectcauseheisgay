#include "persistent_store.h"
#include "../../config.h"

#include <string.h>

static const uint16_t kMagic = 0xB5B1;
static const uint8_t kVersion = 2;
static const uint16_t kSettingsAddr = 0;
static const uint16_t kRingAddr = 64;
static const uint16_t kRecSize = sizeof(IncidentRecord) + 1;   // + crc
static_assert(sizeof(StoredSettings) <= kRingAddr, "settings block too large");
static_assert(kRingAddr + PersistentStore::kRingSize * (sizeof(IncidentRecord) + 1) <= 512, "storage layout too large");

uint8_t crc8(const uint8_t* d, uint16_t len) {
  uint8_t crc = 0x5A;
  while (len--) {
    crc ^= *d++;
    for (uint8_t i = 0; i < 8; ++i) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
  }
  return crc;
}

// --------------------------------------------------------------------------
#ifdef ARDUINO
#include <EEPROM.h>
static const uint16_t kEepromSize = 512;

void EepromBackend::begin() {
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
  EEPROM.begin(kEepromSize);
#endif
}
uint16_t EepromBackend::size() const {
#if defined(ARDUINO_ARCH_AVR)
  return (uint16_t)(E2END + 1);
#else
  return kEepromSize;
#endif
}
uint8_t EepromBackend::read(uint16_t a) { return EEPROM.read(a); }
bool EepromBackend::ready() {
#if defined(ARDUINO_ARCH_AVR)
  return eeprom_is_ready();
#else
  return true;
#endif
}
void EepromBackend::write(uint16_t a, uint8_t v) { EEPROM.write(a, v); }
void EepromBackend::commit() {
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
  EEPROM.commit();
#endif
}
#endif

// --------------------------------------------------------------------------
bool PersistentStore::begin(NvBackend* nv, uint16_t randomSeed) {
  nv_ = nv;
  uint8_t* p = reinterpret_cast<uint8_t*>(&s_);
  for (uint16_t i = 0; i < sizeof(s_); ++i) p[i] = nv_->read((uint16_t)(kSettingsAddr + i));
  const bool ok = s_.magic == kMagic && s_.version == kVersion &&
                  s_.crc == crc8(p, (uint16_t)(sizeof(s_) - 1)) && s_.ringHead < kRingSize &&
                  s_.ringCount <= kRingSize && s_.profile < PROFILE_COUNT;
  if (!ok) {
    memset(&s_, 0, sizeof(s_));
    s_.magic = kMagic;
    s_.version = kVersion;
    s_.profile = PROFILE_DEFAULT;
    s_.deviceId = randomSeed ? randomSeed : 0x5A5A;
    s_.levelUp = v3(0, 0, 1);
    saveSettings();
  }
  return ok;
}

void PersistentStore::saveSettings() {
  s_.crc = crc8(reinterpret_cast<const uint8_t*>(&s_), (uint16_t)(sizeof(s_) - 1));
  settingsDirty_ = true;
}

void PersistentStore::setActiveIncident(uint8_t state, uint32_t id) {
  if (s_.activeState == state && s_.activeId == id) return;
  s_.activeState = state;
  s_.activeId = id;
  saveSettings();
}

uint16_t PersistentStore::slotAddr(uint8_t slot) const { return (uint16_t)(kRingAddr + slot * kRecSize); }

void PersistentStore::saveIncident(const IncidentRecord& rec) {
  uint8_t slot;
  if (lastSlot_ != 0xFF && lastSlotId_ == rec.id) {
    slot = lastSlot_;   // update of the same incident: rewrite its slot
  } else {
    if (recDirty_) flushBlocking();   // a different record is still pending (rare)
    slot = s_.ringHead;
    s_.ringHead = (uint8_t)((s_.ringHead + 1) % kRingSize);
    if (s_.ringCount < kRingSize) ++s_.ringCount;
    saveSettings();
    lastSlot_ = slot;
    lastSlotId_ = rec.id;
  }
  memcpy(rec_, &rec, sizeof(IncidentRecord));
  rec_[sizeof(IncidentRecord)] = crc8(rec_, sizeof(IncidentRecord));
  recAddr_ = slotAddr(slot);
  recDirty_ = true;
}

bool PersistentStore::readRecord(uint8_t slot, IncidentRecord& out) {
  uint8_t b[kRecSize];
  const uint16_t a = slotAddr(slot);
  for (uint16_t i = 0; i < kRecSize; ++i) b[i] = nv_->read((uint16_t)(a + i));
  // A pending (not yet fully written) record is served from RAM.
  if (recDirty_ && recAddr_ == a) memcpy(b, rec_, kRecSize);
  if (b[sizeof(IncidentRecord)] != crc8(b, sizeof(IncidentRecord))) return false;
  memcpy(&out, b, sizeof(IncidentRecord));
  return true;
}

bool PersistentStore::incidentAt(uint8_t newestIndex, IncidentRecord& out) {
  if (newestIndex >= s_.ringCount) return false;
  const uint8_t slot = (uint8_t)((s_.ringHead + kRingSize - 1 - newestIndex) % kRingSize);
  return readRecord(slot, out);
}

bool PersistentStore::findIncident(uint32_t id, IncidentRecord& out) {
  for (uint8_t i = 0; i < s_.ringCount; ++i)
    if (incidentAt(i, out) && out.id == id) return true;
  return false;
}

void PersistentStore::clearIncidents() {
  s_.ringCount = 0;
  s_.ringHead = 0;
  lastSlot_ = 0xFF;
  saveSettings();
}

// Writes at most one changed byte. Returns true if work remains.
bool PersistentStore::writeStep() {
  if (!nv_->ready()) return true;
  if (settingsDirty_) {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&s_);
    for (uint16_t i = 0; i < sizeof(s_); ++i) {
      const uint16_t a = (uint16_t)(kSettingsAddr + i);
      if (nv_->read(a) != p[i]) { nv_->write(a, p[i]); return true; }
    }
    settingsDirty_ = false;
  }
  if (recDirty_) {
    for (uint16_t i = 0; i < kRecSize; ++i) {
      const uint16_t a = (uint16_t)(recAddr_ + i);
      if (nv_->read(a) != rec_[i]) { nv_->write(a, rec_[i]); return true; }
    }
    recDirty_ = false;
  }
  nv_->commit();
  return false;
}

void PersistentStore::service() {
  if (settingsDirty_ || recDirty_) writeStep();
}

void PersistentStore::flushBlocking() {
  while (writeStep()) {
  }
}
