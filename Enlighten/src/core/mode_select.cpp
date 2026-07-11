#include "mode_select.h"

namespace {
// Band n covers raw values [n*20, n*20+19]; the last band extends to 255.
// Documented in docs/DMX_MAP.md — keep the two in sync.
constexpr ModeId BAND_MODES[] = {
    ModeId::OFF,            //   0- 19
    ModeId::CHASE_UP,       //  20- 39
    ModeId::CHASE_DOWN,     //  40- 59
    ModeId::CHASE_UP_DOWN,  //  60- 79
    ModeId::CHASE_DOWN_UP,  //  80- 99
    ModeId::CHASE_IN,       // 100-119
    ModeId::CHASE_OUT,      // 120-139
    ModeId::CHASE_IN_OUT,   // 140-159
    ModeId::CHASE_OUT_IN,   // 160-179
    ModeId::ALTERNATE,      // 180-199
    ModeId::OFF,            // 200-219 (reserved)
    ModeId::FIRE_ALL,       // 220-239
    ModeId::RAW,            // 240-255
};
constexpr uint8_t LAST_BAND = sizeof(BAND_MODES) / sizeof(BAND_MODES[0]) - 1;
}  // namespace

ModeId ModeSelect::update(uint8_t raw, TimeMs now) {
  uint8_t band = (uint8_t)(raw / cfg::MODE_BAND_WIDTH);
  if (band > LAST_BAND) band = LAST_BAND;
  uint8_t lo = (uint8_t)(band * cfg::MODE_BAND_WIDTH);
  uint8_t hi = (band == LAST_BAND) ? 255 : (uint8_t)(lo + cfg::MODE_BAND_WIDTH - 1);

  // Guard zones exist only at internal boundaries: raw 0 is always a valid
  // OFF and raw 255 always a valid RAW.
  bool guard = (band != 0 && (uint8_t)(raw - lo) < cfg::MODE_GUARD) ||
               (band != LAST_BAND && (uint8_t)(hi - raw) < cfg::MODE_GUARD);
  ModeId candidate = guard ? current_ : BAND_MODES[band];

  if (candidate == current_) {
    has_pending_ = false;
  } else if (!has_pending_ || candidate != pending_) {
    pending_ = candidate;
    pending_since_ = now;
    has_pending_ = true;
  } else if (elapsedMs(pending_since_, now) >= cfg::MODE_DEBOUNCE_MS) {
    current_ = candidate;
    has_pending_ = false;
  }
  return current_;
}
