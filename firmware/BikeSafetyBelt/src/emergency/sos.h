// SOS delivery bookkeeping.
//
// This hardware has NO GSM/LTE/SMS module. An SOS can only leave the device
// through the app link (USB serial or BLE); the app then notifies the
// configured emergency contacts. The SOS for an incident is therefore:
//   * issued exactly once per incident id (never re-issued for the same id)
//   * re-transmitted to the app every SOS_RETRY_MS until the app acknowledges
//     it ("SOSACK <id>") — the app de-duplicates by id
//   * kept pending while no app is connected, with a periodic log reminder
#pragma once

#include "../core/types.h"

class SosManager {
 public:
  // Returns false if this incident id already issued an SOS (duplicate prevented).
  bool issue(const IncidentRecord& rec, bool simulated, uint32_t now);
  bool pending() const { return pending_; }
  bool acknowledge(uint32_t id);
  bool sendDue(uint32_t now, bool linkAvailable);
  bool reminderDue(uint32_t now, bool linkAvailable);
  const IncidentRecord& record() const { return rec_; }
  bool simulated() const { return sim_; }
  uint16_t transmissions() const { return tx_; }
  void updateRecord(const IncidentRecord& rec) { if (rec.id == rec_.id) rec_ = rec; }

 private:
  IncidentRecord rec_;
  uint32_t lastIssuedId_ = 0;
  bool pending_ = false;
  bool sim_ = false;
  uint16_t tx_ = 0;
  uint32_t lastTxMs_ = 0, lastReminderMs_ = 0;
  bool everSent_ = false;
  bool reminded_ = false;
};
