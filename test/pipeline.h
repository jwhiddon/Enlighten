// End-to-end pipeline harness: raw MIDI bytes -> parser -> decoder ->
// sequencer -> safety filter, wired exactly like Enlighten.ino, stepped in
// 1 ms virtual-time ticks with the invariant checker watching every output.
#pragma once
#include <initializer_list>

#include "core/config.h"
#include "core/midi_decoder.h"
#include "core/midi_parser.h"
#include "core/safety.h"
#include "core/sequencer.h"
#include "sim.h"

struct MidiPipeline {
  MidiParser parser;
  MidiDecoder dec;
  Sequencer seq;
  SafetySupervisor safety;
  InvariantChecker checker;
  HwInputs hw{true, true};
  TimeMs now = 0;
  uint16_t mask = 0;

  explicit MidiPipeline(TimeMs start = 0, FaultCode boot = FaultCode::NONE)
      : now(start) {
    safety.begin(boot, now);
  }

  void feed(std::initializer_list<uint8_t> bytes) {
    MidiEvent ev;
    for (uint8_t b : bytes) {
      dec.onByte(now);
      if (parser.feed(b, &ev)) dec.onEvent(ev, now);
    }
  }

  void tick() {
    ++now;
    ShowInput in = dec.snapshot(now);
    seq.update(in, now);
    mask = safety.filter(seq.requestedMask(), in, hw, now);
    checker.observe(safety.state(), hw.estop_ok, mask, now);
  }

  void step(uint32_t ms) {
    while (ms--) tick();
  }

  // Step while sending Active Sensing each ms (device present and idle).
  void stepAlive(uint32_t ms) {
    while (ms--) {
      feed({0xFE});
      tick();
    }
  }

  void armViaCc() {
    stepAlive(5);  // link up, arm CCs unset: edge witnessed
    feed({0xB0, 20, cfg::MIDI_ARM_A});
    feed({0xB0, 21, cfg::MIDI_ARM_B});
    stepAlive(cfg::ARM_HOLD_MS + 10);
  }

  // Hold note-on for poofers first..last (1-based) — the per-poofer
  // enables in pattern modes.
  void holdNotes(uint8_t first = 1, uint8_t last = 16) {
    for (uint8_t p = first; p <= last; ++p)
      feed({0x90, (uint8_t)(cfg::MIDI_NOTE_FIRST + p - 1), 127});
  }

  // Select a mode via CC22 (value = band center / 2) and let it debounce.
  void selectMode(uint8_t cc22, uint32_t settle_ms = cfg::MODE_DEBOUNCE_MS + 20) {
    feed({0xB0, 22, cc22});
    stepAlive(settle_ms);
  }
};
