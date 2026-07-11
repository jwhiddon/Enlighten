#include "channel_leds.h"

uint16_t channelLedMask(SafetyState state, uint16_t enabled_mask,
                        uint16_t firing_mask, TimeMs now) {
  // Everything dark unless the system is live: the panel doubles as an
  // unambiguous "rig is armed" indicator from across the room.
  if (state != SafetyState::ARMED) return 0;

  // 1-in-8 milliseconds on = flicker-free dim (the loop runs far faster
  // than 1 kHz, so every millisecond slot is actually driven).
  bool dim_tick = (now & 7u) == 0;
  return (uint16_t)(firing_mask | (dim_tick ? enabled_mask : 0));
}
