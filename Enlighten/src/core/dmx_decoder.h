// DMX front-end: a 24-channel snapshot -> ShowInput.
// Channel map (base-address relative, ch[0] = DMX channel 1) is documented
// in docs/DMX_MAP.md — keep the two in sync.
#pragma once
#include "config.h"
#include "mode_select.h"
#include "show_input.h"
#include "timebase.h"

class DmxDecoder {
 public:
  static constexpr uint8_t NUM_CHANNELS = 24;

  // `ch` points at NUM_CHANNELS bytes; `age_ms` is time since the last
  // valid DMX packet.
  ShowInput decode(const uint8_t* ch, uint32_t age_ms, TimeMs now);

 private:
  ModeSelect mode_;
  uint16_t trig_state_ = 0;  // dead-zone hysteresis memory
};
