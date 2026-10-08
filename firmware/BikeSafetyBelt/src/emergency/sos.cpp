#include "sos.h"
#include "../../config.h"

bool SosManager::issue(const IncidentRecord& rec, bool simulated, uint32_t now) {
  if (rec.id == lastIssuedId_ && rec.id != 0) return false;   // never twice for one incident
  lastIssuedId_ = rec.id;
  rec_ = rec;
  sim_ = simulated;
  pending_ = true;
  tx_ = 0;
  everSent_ = false;
  reminded_ = false;
  lastTxMs_ = now;
  lastReminderMs_ = now;
  return true;
}

bool SosManager::acknowledge(uint32_t id) {
  if (!pending_ || id != rec_.id) return false;
  pending_ = false;
  return true;
}

bool SosManager::sendDue(uint32_t now, bool linkAvailable) {
  if (!pending_ || !linkAvailable) return false;
  if (everSent_ && !elapsed(now, lastTxMs_, SOS_RETRY_MS)) return false;
  everSent_ = true;
  lastTxMs_ = now;
  if (tx_ < 0xFFFF) ++tx_;
  return true;
}

bool SosManager::reminderDue(uint32_t now, bool linkAvailable) {
  if (!pending_ || linkAvailable) return false;
  if (reminded_ && !elapsed(now, lastReminderMs_, SOS_UNDELIVERED_LOG_MS)) return false;
  reminded_ = true;
  lastReminderMs_ = now;
  return true;
}
