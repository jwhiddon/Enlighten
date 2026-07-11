// End-to-end pipeline harnesses: raw protocol data -> decoder -> sequencer
// -> safety filter, wired exactly like Enlighten.ino, stepped in 1 ms
// virtual-time ticks with the invariant checker watching every output.
#pragma once
#include <initializer_list>

#include "core/config.h"
#include "core/dmx_decoder.h"
#include "core/midi_decoder.h"
#include "core/midi_parser.h"
#include "core/safety.h"
#include "core/sequencer.h"
#include "sim.h"

// ---- DMX: drive the pipeline with a 24-byte channel frame -------------------
struct DmxPipeline {
  DmxDecoder dec;
  Sequencer seq;
  SafetySupervisor safety;
  InvariantChecker checker;
  uint8_t ch[DmxDecoder::NUM_CHANNELS] = {};
  HwInputs hw{true, true};
  TimeMs now;
  TimeMs last_packet;
  bool signal = true;  // false = console unplugged (packet age grows)
  uint16_t mask = 0;
  ShowInput last_in;

  explicit DmxPipeline(TimeMs start = 0, FaultCode boot = FaultCode::NONE)
      : now(start), last_packet(start) {
    safety.begin(boot, now);
  }

  void tick() {
    ++now;
    if (signal) last_packet = now;
    last_in = dec.decode(ch, elapsedMs(last_packet, now), now);
    seq.update(last_in, now);
    mask = safety.filter(seq.requestedMask(), last_in, hw, now);
    checker.observe(safety.state(), hw.estop_ok, mask, now);
  }

  void step(uint32_t ms) {
    while (ms--) tick();
  }

  // The documented operator arming procedure.
  void armFromConsole() {
    ch[0] = 0;
    ch[1] = 0;
    step(5);  // controller witnesses the de-asserted edge
    ch[0] = cfg::DMX_ARM_A;
    ch[1] = cfg::DMX_ARM_B;
    step(cfg::ARM_HOLD_MS + 10);
  }
};

// ---- MIDI: drive the pipeline with raw serial bytes -------------------------
struct MidiPipeline {
  MidiParser parser;
  MidiDecoder dec;
  Sequencer seq;
  SafetySupervisor safety;
  InvariantChecker checker;
  HwInputs hw{true, true};
  TimeMs now = 0;
  uint16_t mask = 0;

  MidiPipeline() { safety.begin(FaultCode::NONE, now); }

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
};
