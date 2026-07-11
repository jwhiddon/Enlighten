#include "core/mode_select.h"
#include "framework.h"

namespace {
ModeId settle(ModeSelect& m, uint8_t raw, TimeMs& now, uint32_t ms) {
  ModeId r = ModeId::OFF;
  for (uint32_t i = 0; i < ms; ++i) r = m.update(raw, ++now);
  return r;
}
}  // namespace

TEST(mode_band_selection_after_debounce) {
  ModeSelect m;
  TimeMs now = 0;
  EXPECT_EQ(settle(m, 30, now, cfg::MODE_DEBOUNCE_MS + 10), ModeId::CHASE_UP);
  EXPECT_EQ(settle(m, 250, now, cfg::MODE_DEBOUNCE_MS + 10), ModeId::RAW);
  EXPECT_EQ(settle(m, 0, now, cfg::MODE_DEBOUNCE_MS + 10), ModeId::OFF);
  EXPECT_EQ(settle(m, 230, now, cfg::MODE_DEBOUNCE_MS + 10), ModeId::FIRE_ALL);
  EXPECT_EQ(settle(m, 190, now, cfg::MODE_DEBOUNCE_MS + 10), ModeId::ALTERNATE);
}

TEST(mode_debounce_rejects_blips) {
  ModeSelect m;
  TimeMs now = 0;
  settle(m, 30, now, cfg::MODE_DEBOUNCE_MS + 10);  // CHASE_UP
  // 100 ms excursion into another band: shorter than the debounce.
  settle(m, 90, now, 100);
  EXPECT_EQ(settle(m, 30, now, 5), ModeId::CHASE_UP);
}

TEST(mode_guard_zone_holds_current) {
  ModeSelect m;
  TimeMs now = 0;
  settle(m, 30, now, cfg::MODE_DEBOUNCE_MS + 10);  // CHASE_UP (band 20-39)
  // 38-41 are within the guard of the 39/40 boundary: hold forever.
  EXPECT_EQ(settle(m, 38, now, 1000), ModeId::CHASE_UP);
  EXPECT_EQ(settle(m, 41, now, 1000), ModeId::CHASE_UP);
  EXPECT_EQ(settle(m, 42, now, 1000), ModeId::CHASE_DOWN);  // past the guard
}

TEST(fader_sweep_does_not_glitch_modes) {
  ModeSelect m;
  TimeMs now = 0;
  settle(m, 0, now, cfg::MODE_DEBOUNCE_MS + 10);  // OFF
  // Sweep 0 -> 255 at 1 value/ms: no band persists long enough to latch.
  for (int v = 0; v <= 255; ++v) m.update((uint8_t)v, ++now);
  EXPECT_EQ(m.current(), ModeId::OFF);
  // Holding at the top then selects RAW.
  EXPECT_EQ(settle(m, 255, now, cfg::MODE_DEBOUNCE_MS + 10), ModeId::RAW);
}

TEST(mode_extremes_are_valid_not_guarded) {
  ModeSelect m;
  TimeMs now = 0;
  EXPECT_EQ(settle(m, 255, now, cfg::MODE_DEBOUNCE_MS + 10), ModeId::RAW);
  EXPECT_EQ(settle(m, 0, now, cfg::MODE_DEBOUNCE_MS + 10), ModeId::OFF);
}
