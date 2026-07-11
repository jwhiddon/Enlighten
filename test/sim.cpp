#include "sim.h"

#include <cstdio>

void InvariantChecker::fail(const char* m) {
  if (!ok_) return;  // keep the first failure
  ok_ = false;
  std::snprintf(msg_, sizeof(msg_), "%s", m);
}

void InvariantChecker::observe(SafetyState state, bool estop_ok, uint16_t mask,
                               TimeMs now) {
  // Invariant 1: outputs only while ARMED.
  if (state != SafetyState::ARMED && mask != 0)
    fail("output nonzero while not ARMED");

  // Invariant 2: E-stop asserted => outputs closed.
  if (!estop_ok && mask != 0) fail("output nonzero with E-stop asserted");

  uint32_t dt = has_prev_ ? elapsedMs(prev_now_, now) : 0;

  for (uint8_t i = 0; i < cfg::NUM_POOFERS; ++i) {
    bool open = (mask >> i) & 1u;
    bool was_open = (prev_mask_ >> i) & 1u;

    // Invariant 3: continuous open time bounded (tick tolerance: dt).
    if (open) {
      open_run_ms_[i] += dt;
      if (open_run_ms_[i] > cfg::MAX_OPEN_MS)
        fail("channel open longer than MAX_OPEN_MS");
    } else {
      open_run_ms_[i] = 0;
    }

    // Invariant 4: minimum close time between opens.
    if (has_prev_) {
      if (was_open && !open) closed_at_[i] = now;
      if (!was_open && open && ever_open_[i]) {
        if (elapsedMs(closed_at_[i], now) < cfg::MIN_CLOSE_MS)
          fail("channel reopened before MIN_CLOSE_MS");
      }
    }
    if (open) ever_open_[i] = true;
  }

  // Invariant 5: sliding-window duty budget (small tick tolerance).
  // Ring position is an internal tick counter, NOT now % RING: the modulo
  // of a wrapping uint32 is discontinuous at the wrap (2^32 % RING != 0),
  // which would corrupt the rolling sums exactly when we test rollover.
  uint32_t slot = ring_pos_;
  ring_pos_ = (ring_pos_ + 1) % RING;
  if (ring_fill_ >= RING) {
    uint16_t old = ring_[slot];
    for (uint8_t i = 0; i < cfg::NUM_POOFERS; ++i)
      if ((old >> i) & 1u) --duty_ms_[i];
  } else {
    ++ring_fill_;
  }
  ring_[slot] = mask;
  for (uint8_t i = 0; i < cfg::NUM_POOFERS; ++i) {
    if ((mask >> i) & 1u) ++duty_ms_[i];
    if (duty_ms_[i] > cfg::MAX_OPEN_PER_WINDOW_MS + 10)
      fail("sliding 10s duty budget exceeded");
  }

  prev_mask_ = mask;
  prev_now_ = now;
  has_prev_ = true;
}
