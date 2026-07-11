// The protocol-agnostic show-command model.  Decoders (DMX, MIDI) may ONLY
// produce this struct; they never touch the sequencer, safety logic, or
// outputs.  Both decoders are state-snapshot producers so the downstream
// pipeline is identical regardless of protocol.
#pragma once
#include <stdint.h>

// Effect modes.  The numeric order matters only to the DMX mode-band table
// in mode_select.cpp.
enum class ModeId : uint8_t {
  OFF = 0,
  RAW,            // per-poofer direct triggers
  FIRE_ALL,
  ALTERNATE,      // evens / odds
  CHASE_UP,
  CHASE_DOWN,
  CHASE_UP_DOWN,
  CHASE_DOWN_UP,
  CHASE_IN,       // mirrored pairs, outside -> in
  CHASE_OUT,
  CHASE_IN_OUT,
  CHASE_OUT_IN,
};

struct ShowInput {
  bool     link_ok = false;   // signal seen within the protocol's timeout
  bool     arm_a = false;     // arming handshake value A currently asserted
  bool     arm_b = false;     // arming handshake value B currently asserted
  ModeId   mode = ModeId::OFF;
  uint16_t trigger_mask = 0;  // bit i: poofer i requested (RAW) / enabled (patterns)
  uint16_t poof_ms = 0;       // decoder-mapped; sequencer and safety clamp again
  uint16_t rest_ms = 0;
  uint8_t  rate = 128;        // master tempo, 128 = 1.0x
  bool     repeat = false;
};
