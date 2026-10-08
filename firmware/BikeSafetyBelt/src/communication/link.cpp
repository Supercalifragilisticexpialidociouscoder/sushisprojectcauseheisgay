#include "link.h"
#include "../../config.h"

void Link::begin(Transport* a, Transport* b) {
  t_[0] = a;
  t_[1] = b;
  for (uint8_t i = 0; i < kMax; ++i)
    if (t_[i]) t_[i]->begin();
}

void Link::service(uint32_t now) {
  for (uint8_t i = 0; i < kMax; ++i) {
    if (!t_[i]) continue;
    t_[i]->service(now);
    if (session_[i] && (!t_[i]->linkUp() || elapsed(now, lastHeard_[i], APP_HEARTBEAT_TIMEOUT_MS))) {
      session_[i] = false;
      lost_ |= (uint8_t)(1u << i);
    }
  }
  if (suspended_ && (int32_t)(now - suspendUntil_) >= 0) suspended_ = false;
}

const char* Link::poll(uint32_t now, uint8_t& src) {
  for (uint8_t i = 0; i < kMax; ++i) {
    if (!t_[i]) continue;
    const char* line = t_[i]->pollLine();
    if (line) {
      src = i;
      lastHeard_[i] = now;   // any traffic counts as a heartbeat
      return line;
    }
  }
  return nullptr;
}

void Link::startSession(uint8_t i, uint32_t now) {
  if (i >= kMax || !t_[i]) return;
  session_[i] = true;
  lastHeard_[i] = now;
}

bool Link::beginLine(Channel ch, uint8_t mask) {
  cur_ = 0;
  for (uint8_t i = 0; i < kMax; ++i) {
    if (!t_[i] || !(mask & (1u << i))) continue;
    const bool want = (ch == Channel::LOG) ? t_[i]->carriesLogs() : (session_[i] && !suspended_);
    if (want) cur_ |= (uint8_t)(1u << i);
  }
  return cur_ != 0;
}

void Link::write(const char* data, uint8_t len) {
  for (uint8_t i = 0; i < kMax; ++i)
    if (cur_ & (1u << i)) t_[i]->write(data, len);
}

void Link::endLine() {
  for (uint8_t i = 0; i < kMax; ++i)
    if (cur_ & (1u << i)) t_[i]->endLine();
  cur_ = 0;
}
