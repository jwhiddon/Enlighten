#include "core/status_led.h"
#include "framework.h"

namespace {
// Count rising edges of the LED over one window.
uint32_t risingEdges(SafetyState st, FaultCode f, bool firing, TimeMs start,
                     uint32_t window_ms) {
  uint32_t edges = 0;
  bool prev = statusLedOn(st, f, firing, start);
  for (uint32_t t = 1; t <= window_ms; ++t) {
    bool on = statusLedOn(st, f, firing, start + t);
    if (on && !prev) ++edges;
    prev = on;
  }
  return edges;
}
}  // namespace

TEST(led_boot_solid) {
  for (TimeMs t = 0; t < 3000; t += 37)
    EXPECT_TRUE(statusLedOn(SafetyState::BOOT_SELFTEST, FaultCode::NONE, false, t));
}

TEST(led_safe_slow_blink) {
  // ~1 Hz: 3 rising edges over 3 s.
  EXPECT_EQ(risingEdges(SafetyState::SAFE, FaultCode::NONE, false, 0, 3000),
            (uint32_t)3);
}

TEST(led_armed_fast_blink_vs_firing_solid) {
  // 4 Hz idle: 12 rising edges over 3 s.
  EXPECT_EQ(risingEdges(SafetyState::ARMED, FaultCode::NONE, false, 0, 3000),
            (uint32_t)12);
  for (TimeMs t = 0; t < 2000; t += 13)
    EXPECT_TRUE(statusLedOn(SafetyState::ARMED, FaultCode::NONE, true, t));
}

TEST(led_fault_blink_count_matches_code) {
  // N blinks per cycle for each fault code.
  for (uint8_t code = 1; code <= 7; ++code) {
    uint32_t cycle = (uint32_t)code * 400 + 1000;
    uint32_t edges = risingEdges(SafetyState::FAULT_LOCKOUT, (FaultCode)code,
                                 false, 0, cycle * 3);
    EXPECT_EQ(edges, (uint32_t)code * 3);
  }
}

TEST(led_states_distinguishable) {
  // SAFE and ARMED patterns must differ somewhere within a second.
  bool differs = false;
  for (TimeMs t = 0; t < 1000 && !differs; ++t)
    differs = statusLedOn(SafetyState::SAFE, FaultCode::NONE, false, t) !=
              statusLedOn(SafetyState::ARMED, FaultCode::NONE, false, t);
  EXPECT_TRUE(differs);
}
