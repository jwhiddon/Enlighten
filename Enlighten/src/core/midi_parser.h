// Byte-level MIDI stream parser.  Pure function of the byte stream:
// handles running status, real-time bytes interleaved mid-message, and
// SysEx skipping.  Produces complete events; interpretation lives in
// MidiDecoder.
#pragma once
#include <stdint.h>

struct MidiEvent {
  uint8_t status = 0;  // full status byte (incl. channel) or realtime byte
  uint8_t data1 = 0;
  uint8_t data2 = 0;
};

class MidiParser {
 public:
  // Feed one byte; returns true when `out` holds a complete event.
  // Real-time bytes (0xF8-0xFF) are emitted immediately and do not disturb
  // an in-progress message.
  bool feed(uint8_t b, MidiEvent* out);

 private:
  uint8_t running_ = 0;
  uint8_t data_[2] = {};
  uint8_t have_ = 0;
  bool in_sysex_ = false;
};
