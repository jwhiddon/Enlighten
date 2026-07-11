// Exhaustive verification of the mode-band map against docs/DMX_MAP.md.
// Locks the table: any drift between code and documentation fails here.
#include "core/mode_select.h"
#include "framework.h"

namespace {
constexpr ModeId DOCUMENTED[13] = {
    ModeId::OFF,           ModeId::CHASE_UP,     ModeId::CHASE_DOWN,
    ModeId::CHASE_UP_DOWN, ModeId::CHASE_DOWN_UP, ModeId::CHASE_IN,
    ModeId::CHASE_OUT,     ModeId::CHASE_IN_OUT, ModeId::CHASE_OUT_IN,
    ModeId::ALTERNATE,     ModeId::OFF /*reserved*/, ModeId::FIRE_ALL,
    ModeId::RAW,
};
}  // namespace

TEST(mode_map_exhaustive_all_256_values) {
  for (int raw = 0; raw <= 255; ++raw) {
    ModeSelect m;  // fresh selector: current = OFF
    TimeMs now = 0;
    for (uint32_t i = 0; i < cfg::MODE_DEBOUNCE_MS + 20; ++i)
      m.update((uint8_t)raw, ++now);

    int band = raw / cfg::MODE_BAND_WIDTH;
    if (band > 12) band = 12;
    int lo = band * cfg::MODE_BAND_WIDTH;
    int hi = band == 12 ? 255 : lo + cfg::MODE_BAND_WIDTH - 1;
    bool guard = (band != 0 && raw - lo < cfg::MODE_GUARD) ||
                 (band != 12 && hi - raw < cfg::MODE_GUARD);
    ModeId expect = guard ? ModeId::OFF /* held initial */ : DOCUMENTED[band];

    if (m.current() != expect) {
      std::printf("raw=%d got=%d expect=%d\n", raw, (int)m.current(),
                  (int)expect);
      EXPECT_EQ((int)m.current(), (int)expect);
      return;
    }
  }
  EXPECT_TRUE(true);
}
