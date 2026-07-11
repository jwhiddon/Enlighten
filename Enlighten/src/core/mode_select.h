// Banded mode decode with hysteresis and time debounce.
//
// The raw selector (CC22 scaled x2 onto 0-255) is split into 20-value bands.
// Values within MODE_GUARD of an internal band boundary "hold" the current
// mode, and a new band must persist MODE_DEBOUNCE_MS before it takes effect,
// so a fader sweeping across the range cannot glitch through modes.
#pragma once
#include "config.h"
#include "show_input.h"
#include "timebase.h"

class ModeSelect {
 public:
  ModeId update(uint8_t raw, TimeMs now);
  ModeId current() const { return current_; }

 private:
  ModeId current_ = ModeId::OFF;
  ModeId pending_ = ModeId::OFF;
  TimeMs pending_since_ = 0;
  bool has_pending_ = false;
};
