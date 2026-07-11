// Additional fuzzers: protocol parsers/decoders under hostile bytes, and
// the full DMX pipeline under random console behavior — started just below
// the millis() wrap so every run also crosses the rollover.
#include <cstdio>

#include "core/dmx_decoder.h"
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

TEST(fuzz_dmx_decoder_hostile_frames) {
  DmxDecoder dec;
  XorShift32 rng(0x12345678);
  uint8_t ch[DmxDecoder::NUM_CHANNELS];
  TimeMs now = 0;
  for (uint32_t i = 0; i < 200000; ++i) {
    for (auto& c : ch) c = rng.byte();
    uint32_t age = rng.oneIn(4) ? rng.next() % 2000 : 0;
    ShowInput s = dec.decode(ch, age, ++now);
    // Outputs must always be in range, whatever the frame contained.
    EXPECT_TRUE(s.poof_ms >= cfg::MIN_POOF_MS && s.poof_ms <= cfg::MAX_POOF_MS);
    EXPECT_TRUE(s.rest_ms >= cfg::MIN_REST_MS && s.rest_ms <= cfg::MAX_REST_MS);
    EXPECT_TRUE((uint8_t)s.mode <= (uint8_t)ModeId::CHASE_OUT_IN);
    if (!s.link_ok) {
      EXPECT_FALSE(s.arm_a);
      EXPECT_FALSE(s.arm_b);
    }
    if (testfw::failures()) return;
  }
}

TEST(fuzz_full_dmx_pipeline_across_rollover) {
  DmxPipeline p(0xFFFFFFFFu - 250000);  // wraps mid-run
  XorShift32 rng(0xFEEDF00D);
  for (uint32_t i = 0; i < 500000; ++i) {
    // Random console behavior: scribble on channels, yank cables, press
    // the E-stop, cycle the key.
    if (rng.oneIn(10)) p.ch[rng.next() % DmxDecoder::NUM_CHANNELS] = rng.byte();
    if (rng.oneIn(500)) p.signal = !p.signal;
    if (rng.oneIn(300)) p.hw.arm_key = !p.hw.arm_key;
    if (rng.oneIn(5000)) p.hw.estop_ok = false;
    if (!p.hw.estop_ok && rng.oneIn(400)) p.hw.estop_ok = true;
    // Occasionally clear a lockout the legitimate way.
    if (p.safety.state() == SafetyState::FAULT_LOCKOUT && rng.oneIn(200)) {
      p.hw.estop_ok = true;
      p.signal = true;
      p.ch[0] = 0;
      p.ch[1] = 0;
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
