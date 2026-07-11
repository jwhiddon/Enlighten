// Additional fuzzers: the MIDI parser under hostile bytes, the full
// pipeline under random wire garbage and hardware events — started just
// below the millis() wrap so every run also crosses the rollover — and
// sequencer mode-thrash.
#include <cstdio>

#include "core/midi_parser.h"
#include "framework.h"
#include "pipeline.h"

namespace {
struct XorShift32 {
  uint32_t s;
  explicit XorShift32(uint32_t seed) : s(seed) {}
  uint32_t next() {
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
  }
  uint8_t byte() { return (uint8_t)next(); }
  bool oneIn(uint32_t n) { return next() % n == 0; }
};
}  // namespace

TEST(fuzz_midi_parser_hostile_bytes) {
  MidiParser p;
  XorShift32 rng(0xDEADBEEF);
  MidiEvent ev;
  for (uint32_t i = 0; i < 500000; ++i) {
    if (p.feed(rng.byte(), &ev)) {
      // Every emitted event must be structurally valid.
      EXPECT_TRUE(ev.status >= 0x80);
      if (ev.status < 0xF0) {  // channel messages carry 7-bit data
        EXPECT_TRUE(ev.data1 < 128);
        EXPECT_TRUE(ev.data2 < 128);
      }
      if (testfw::failures()) return;  // don't spam 500k failures
    }
  }
}

TEST(fuzz_full_midi_pipeline_across_rollover) {
  // Random wire garbage straight into the parser plus random hardware
  // events, crossing the 2^32 wrap mid-run.  The safety invariants must
  // hold against ANY byte stream.
  MidiPipeline p(0xFFFFFFFFu - 250000);
  XorShift32 rng(0xFEEDF00D);
  for (uint32_t i = 0; i < 500000; ++i) {
    if (rng.oneIn(4)) p.feed({rng.byte()});     // hostile wire bytes
    if (rng.oneIn(20)) p.feed({0xFE});          // intermittent heartbeats
    if (rng.oneIn(300)) p.hw.arm_key = !p.hw.arm_key;
    if (rng.oneIn(5000)) p.hw.estop_ok = false;
    if (!p.hw.estop_ok && rng.oneIn(400)) p.hw.estop_ok = true;
    // Occasionally clear a lockout the legitimate way so ARMED paths get
    // exercised again.
    if (p.safety.state() == SafetyState::FAULT_LOCKOUT && rng.oneIn(200)) {
      p.hw.estop_ok = true;
      p.feed({0xFE, 0xB0, 20, 0, 0xB0, 21, 0});
    }
    p.tick();
    if (!p.checker.ok()) break;
  }
  if (!p.checker.ok()) std::printf("pipeline fuzz: %s\n", p.checker.message());
  EXPECT_TRUE(p.checker.ok());
}

TEST(fuzz_sequencer_mode_thrash) {
  // Random mode changes every few ms must never produce an out-of-range
  // request or wedge the engine.
  Sequencer seq;
  ShowInput in;
  in.link_ok = true;
  in.rate = 128;
  XorShift32 rng(0xABCD1234);
  TimeMs now = 0;
  for (uint32_t i = 0; i < 300000; ++i) {
    if (rng.oneIn(20)) in.mode = (ModeId)(rng.next() % 12);
    if (rng.oneIn(15)) in.trigger_mask = (uint16_t)rng.next();
    if (rng.oneIn(50)) in.poof_ms = (uint16_t)rng.next();
    if (rng.oneIn(50)) in.rest_ms = (uint16_t)rng.next();
    if (rng.oneIn(50)) in.rate = rng.byte();
    if (rng.oneIn(100)) in.repeat = !in.repeat;
    seq.update(in, ++now);
    // Requests must always be a subset of the enabled triggers.
    EXPECT_EQ(seq.requestedMask() & ~in.trigger_mask, (uint16_t)0);
    if (testfw::failures()) return;
  }
}
