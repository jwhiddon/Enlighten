// Per-solenoid panel LEDs — tri-color, on an APA102/SK9822 addressable
// chain (2 wires for all 16 LEDs).  This module is the pure color model;
// the board layer renders it to the chain.
//
//   OFF    disarmed / lockout — the dark panel reads "rig is cold"
//   GREEN  system ARMED and this poofer enabled (in the pattern / held)
//   RED    the solenoid is OPEN right now, fired live (MIDI/panel)
//   BLUE   the solenoid is OPEN right now, fired by SD playback
//   AMBER  requested but held closed by the duty limiter — the previously
//          invisible "why is it quiet" state
//
// Colors derive from the FILTERED output mask (the truth), plus the
// request mask so throttling is visible.  Indicators only.
#pragma once
#include "config.h"
#include "safety.h"

enum class ChannelLedColor : uint8_t { OFF = 0, GREEN, RED, AMBER, BLUE };

// Fills out[cfg::NUM_POOFERS].
void channelLedColors(ChannelLedColor* out, SafetyState state,
                      uint16_t enabled_mask, uint16_t firing_mask,
                      uint16_t requested_mask, uint16_t playback_mask);
