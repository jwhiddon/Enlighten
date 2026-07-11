#include "duty_limiter.h"

void DutyLimiter::resetAt(TimeMs now) {
  open_ = false;
  opened_at_ = now;
  // Backdate the last close so the first open is allowed immediately.
  closed_at_ = now - cfg::MIN_CLOSE_MS;
  last_eval_ = now;
  bucket_start_ = now;
  bucket_i_ = 0;
  for (uint8_t i = 0; i < cfg::DUTY_BUCKETS; ++i) buckets_[i] = 0;
  inited_ = true;
}

uint32_t DutyLimiter::windowOpenMs() const {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < cfg::DUTY_BUCKETS; ++i) sum += buckets_[i];
  return sum;
}

void DutyLimiter::rotateBuckets(TimeMs now) {
  // If we haven't been evaluated for longer than the whole window, all
  // history has aged out — clear rather than looping bucket by bucket.
  if (elapsedMs(bucket_start_, now) >= (uint32_t)cfg::DUTY_WINDOW_MS + cfg::DUTY_BUCKET_MS) {
    for (uint8_t i = 0; i < cfg::DUTY_BUCKETS; ++i) buckets_[i] = 0;
    bucket_start_ = now;
    bucket_i_ = 0;
    return;
  }
  while (elapsedMs(bucket_start_, now) >= cfg::DUTY_BUCKET_MS) {
    bucket_start_ += cfg::DUTY_BUCKET_MS;
    bucket_i_ = (uint8_t)((bucket_i_ + 1) % cfg::DUTY_BUCKETS);
    buckets_[bucket_i_] = 0;
  }
}

bool DutyLimiter::evaluate(bool want, TimeMs now) {
  if (!inited_) resetAt(now);

  // Account open time since the last evaluation into the current bucket.
  // (Loop period is ~sub-millisecond, so cross-bucket attribution error is
  // negligible and always in the conservative direction of the extra bucket.)
  uint32_t delta = elapsedMs(last_eval_, now);
  if (open_ && delta) {
    uint32_t add = delta > 0xFFFFu ? 0xFFFFu : delta;
    uint32_t v = (uint32_t)buckets_[bucket_i_] + add;
    buckets_[bucket_i_] = v > 0xFFFFu ? 0xFFFFu : (uint16_t)v;
  }
  last_eval_ = now;
  rotateBuckets(now);

  uint32_t used = windowOpenMs();
  if (open_) {
    bool must_close = !want ||
                      elapsedMs(opened_at_, now) >= cfg::MAX_OPEN_MS ||
                      used >= cfg::MAX_OPEN_PER_WINDOW_MS;
    if (must_close) {
      open_ = false;
      closed_at_ = now;
    }
  } else {
    bool may_open = want &&
                    elapsedMs(closed_at_, now) >= cfg::MIN_CLOSE_MS &&
                    used < cfg::MAX_OPEN_PER_WINDOW_MS;
    if (may_open) {
      open_ = true;
      opened_at_ = now;
    }
  }
  return open_;
}
