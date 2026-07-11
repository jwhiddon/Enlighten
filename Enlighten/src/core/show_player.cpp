#include "show_player.h"

void ShowPlayer::start(TimeMs now) {
  active_ = true;
  eof_ = false;
  head_ = 0;
  count_ = 0;
  len_ = 0;
  mask_ = 0;
  pos_ = 0;
  t0_ = now;
}

void ShowPlayer::stop() {
  active_ = false;
  mask_ = 0;
}

void ShowPlayer::feed(char c) {
  if (!active_) return;
  if (c == '\n' || c == '\r') {
    parseLine();
    len_ = 0;
    return;
  }
  if (len_ + 1 < (uint8_t)sizeof(line_)) line_[len_++] = c;
}

void ShowPlayer::feedEof() {
  if (!active_) return;
  parseLine();  // a final line without a newline still counts
  len_ = 0;
  eof_ = true;
}

void ShowPlayer::parseLine() {
  line_[len_] = 0;
  const char* p = line_;
  while (*p == ' ') ++p;
  if (*p == 0 || *p == '#') return;  // blank / comment

  // "<decimal ms> <hex mask>" — anything malformed is ignored.
  uint32_t t = 0;
  bool any = false;
  while (*p >= '0' && *p <= '9') {
    t = t * 10 + (uint32_t)(*p++ - '0');
    any = true;
  }
  if (!any || *p != ' ') return;
  while (*p == ' ') ++p;
  uint32_t m = 0;
  bool anyhex = false;
  for (;; ++p) {
    char c = *p;
    uint8_t d;
    if (c >= '0' && c <= '9') d = (uint8_t)(c - '0');
    else if (c >= 'a' && c <= 'f') d = (uint8_t)(c - 'a' + 10);
    else if (c >= 'A' && c <= 'F') d = (uint8_t)(c - 'A' + 10);
    else break;
    m = (m << 4) | d;
    anyhex = true;
  }
  if (!anyhex || count_ >= RING) return;

  uint8_t slot = (uint8_t)((head_ + count_) % RING);
  ring_[slot].t = t;
  ring_[slot].mask = (uint16_t)m;
  ++count_;
}

void ShowPlayer::update(TimeMs now) {
  if (!active_) {
    mask_ = 0;
    return;
  }
  pos_ = elapsedMs(t0_, now);
  while (count_ && ring_[head_].t <= pos_) {
    mask_ = ring_[head_].mask;
    head_ = (uint8_t)((head_ + 1) % RING);
    --count_;
  }
  // End of data = end of show: never leave a request hanging.
  if (eof_ && count_ == 0) {
    mask_ = 0;
    active_ = false;
  }
}
