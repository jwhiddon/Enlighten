#include "channel_leds.h"

void channelLedColors(ChannelLedColor* out, SafetyState state,
                      uint16_t enabled_mask, uint16_t firing_mask,
                      uint16_t requested_mask, uint16_t playback_mask) {
  for (uint8_t i = 0; i < cfg::NUM_POOFERS; ++i) {
    uint16_t b = (uint16_t)(1u << i);
    if (state != SafetyState::ARMED) {
      out[i] = ChannelLedColor::OFF;
    } else if (firing_mask & b) {
      out[i] = (playback_mask & b) ? ChannelLedColor::BLUE
                                   : ChannelLedColor::RED;
    } else if (requested_mask & b) {
      // Requested but the safety filter is holding it closed: duty budget
      // spent or inside the forced MIN_CLOSE gap.
      out[i] = ChannelLedColor::AMBER;
    } else if (enabled_mask & b) {
      out[i] = ChannelLedColor::GREEN;
    } else {
      out[i] = ChannelLedColor::OFF;
    }
  }
}
