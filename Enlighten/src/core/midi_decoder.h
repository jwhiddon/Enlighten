// MIDI front-end: parsed events -> gate/CC state -> ShowInput snapshots.
// Mapping is documented in docs/MIDI_MAP.md — keep the two in sync.
//
// Deadman over MIDI: once Active Sensing (0xFE) has been observed, its
// absence for MIDI_AS_TIMEOUT_MS drops the link; before any Active Sensing
// is seen, ANY byte within MIDI_KEEPALIVE_MS keeps the link alive.
#pragma once
#include "config.h"
#include "midi_parser.h"
#include "mode_select.h"
#include "show_input.h"
#include "timebase.h"

class MidiDecoder {
 public:
  // Call for every raw byte received (keepalive tracking), in addition to
  // onEvent() for completed events.
  void onByte(TimeMs now);
  void onEvent(const MidiEvent& ev, TimeMs now);
  ShowInput snapshot(TimeMs now);

 private:
  uint16_t gates_ = 0;
  // Last seen values for CC20..CC26; 0xFF = never received.
  static constexpr uint8_t CC_BASE = 20;
  static constexpr uint8_t CC_COUNT = 7;
  uint8_t cc_[CC_COUNT] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  ModeSelect mode_;
  bool any_byte_ = false;
  bool seen_active_sensing_ = false;
  TimeMs last_byte_ = 0;
  TimeMs last_active_sensing_ = 0;
};
