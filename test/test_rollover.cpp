// The millis() wrap (~49.7 days) is where the legacy firmware could stick a
// solenoid open.  Every stateful component gets exercised straight through
// the 2^32 boundary here.
#include "core/duty_limiter.h"
#include "framework.h"
#include "pipeline.h"
#include "sim.h"

namespace {
constexpr TimeMs BEFORE_WRAP = 0xFFFFFFFFu - 30000;  // 30 s before the wrap
}

TEST(rollover_duty_limiter_limits_hold) {
  DutyLimiter d;
  TimeMs t = BEFORE_WRAP;
  uint32_t longest_run = 0, run = 0;
  uint32_t opens = 0;
  bool prev = false;
  TimeMs closed_at = 0;
  bool min_close_ok = true;
  for (uint32_t i = 0; i < 60000; ++i) {  // 60 s of continuous demand
    bool open = d.evaluate(true, ++t);
    if (open) {
      if (!prev) {
        ++opens;
        if (opens > 1 && elapsedMs(closed_at, t) < cfg::MIN_CLOSE_MS)
          min_close_ok = false;
      }
      if (++run > longest_run) longest_run = run;
    } else {
      if (prev) closed_at = t;
      run = 0;
    }
    prev = open;
  }
  EXPECT_TRUE(longest_run <= cfg::MAX_OPEN_MS);
  EXPECT_TRUE(min_close_ok);
  EXPECT_TRUE(opens > 5);  // it kept working across the wrap
}

TEST(rollover_full_sim_raw_fire) {
  Sim s;
  s.now = BEFORE_WRAP;
  s.begin();
  s.armFully();
  s.in.mode = ModeId::RAW;
  s.in.trigger_mask = 0xFFFF;
  s.in.poof_ms = 200;
  s.in.rest_ms = 100;
  s.in.repeat = true;

  uint32_t fired_ticks_after_wrap = 0;
  for (uint32_t i = 0; i < 60000; ++i) {
    s.tick();
    if (s.now < 100000 && s.last_mask != 0) ++fired_ticks_after_wrap;
  }
  EXPECT_TRUE(s.checker.ok());
  EXPECT_TRUE(fired_ticks_after_wrap > 0);  // kept firing after the wrap
  EXPECT_EQ(s.safety.state(), SafetyState::ARMED);  // no spurious fault
}

TEST(rollover_chase_keeps_stepping) {
  Sequencer seq;
  ShowInput in;
  in.link_ok = true;
  in.mode = ModeId::CHASE_UP;
  in.trigger_mask = 0xFFFF;
  in.poof_ms = 100;
  in.rest_ms = 100;
  in.rate = 128;
  in.repeat = true;

  TimeMs now = BEFORE_WRAP;
  uint32_t steps_before = 0, steps_after = 0;
  uint16_t prev = 0;
  for (uint32_t i = 0; i < 60000; ++i) {
    seq.update(in, ++now);
    uint16_t m = seq.requestedMask();
    if (m != 0 && m != prev) {
      if (now > BEFORE_WRAP) ++steps_before;
      if (now < 100000) ++steps_after;
    }
    prev = m;
  }
  // ~5 steps/s on each side of the wrap; the wrap must not stall the chase.
  EXPECT_TRUE(steps_before > 50);
  EXPECT_TRUE(steps_after > 50);
}

TEST(rollover_end_to_end_midi_pipeline) {
  MidiPipeline p(BEFORE_WRAP);
  p.armViaCc();
  EXPECT_EQ(p.safety.state(), SafetyState::ARMED);
  p.feed({0xB0, 26, 127});  // repeat
  p.feed({0xB0, 23, 64});   // mid poof duration
  p.holdNotes();
  p.selectMode(115);  // FIRE_ALL band

  uint32_t fired_after_wrap = 0;
  for (uint32_t i = 0; i < 60000; ++i) {
    p.feed({0xFE});
    p.tick();
    if (p.now < 100000 && p.mask != 0) ++fired_after_wrap;
  }
  EXPECT_TRUE(p.checker.ok());
  EXPECT_TRUE(fired_after_wrap > 0);
  EXPECT_EQ(p.safety.state(), SafetyState::ARMED);
}

TEST(rollover_mode_debounce_unaffected) {
  ModeSelect m;
  TimeMs now = 0xFFFFFFFFu - 60;  // debounce interval spans the wrap
  for (uint32_t i = 0; i < cfg::MODE_DEBOUNCE_MS + 20; ++i) m.update(250, ++now);
  EXPECT_EQ(m.current(), ModeId::RAW);
}
