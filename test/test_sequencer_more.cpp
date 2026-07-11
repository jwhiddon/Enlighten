// Additional sequencer coverage: remaining chase variants, one-shot cycle
// lengths, rate scaling, and trigger gating in every mode family.
#include <vector>

#include "core/sequencer.h"
#include "framework.h"

namespace {

struct SeqH {
  Sequencer seq;
  ShowInput in;
  TimeMs now = 0;

  SeqH() {
    in.link_ok = true;
    in.trigger_mask = 0xFFFF;
    in.poof_ms = 100;
    in.rest_ms = 100;
    in.rate = 128;
    in.repeat = true;
  }

  void step(uint32_t ms) {
    for (uint32_t i = 0; i < ms; ++i) {
      ++now;
      seq.update(in, now);
    }
  }

  std::vector<uint16_t> collectSteps(uint32_t ms) {
    std::vector<uint16_t> steps;
    uint16_t prev = 0;
    for (uint32_t i = 0; i < ms; ++i) {
      ++now;
      seq.update(in, now);
      uint16_t m = seq.requestedMask();
      if (m != 0 && m != prev) steps.push_back(m);
      prev = m;
    }
    return steps;
  }
};

constexpr uint16_t B(int i) { return (uint16_t)(1u << i); }

}  // namespace

TEST(chase_down_up_bounce_order) {
  SeqH h;
  h.in.mode = ModeId::CHASE_DOWN_UP;
  auto steps = h.collectSteps(4000);
  uint16_t expect[] = {B(7), B(6), B(5), B(4), B(3), B(2), B(1), B(0),
                       B(1), B(2), B(3), B(4), B(5), B(6), B(7), B(6)};
  EXPECT_TRUE(steps.size() >= 16);
  for (int i = 0; i < 16 && i < (int)steps.size(); ++i)
    EXPECT_EQ(steps[i], expect[i]);
}

TEST(chase_in_out_bounce_order) {
  SeqH h;
  h.in.mode = ModeId::CHASE_IN_OUT;
  auto steps = h.collectSteps(3000);
  // pairs: (0,7)(1,6)(2,5)(3,4)(2,5)(1,6)(0,7)(1,6)...
  uint16_t p0 = B(0) | B(7), p1 = B(1) | B(6), p2 = B(2) | B(5), p3 = B(3) | B(4);
  uint16_t expect[] = {p0, p1, p2, p3, p2, p1, p0, p1};
  EXPECT_TRUE(steps.size() >= 8);
  for (int i = 0; i < 8 && i < (int)steps.size(); ++i)
    EXPECT_EQ(steps[i], expect[i]);
}

TEST(chase_out_in_bounce_order) {
  SeqH h;
  h.in.mode = ModeId::CHASE_OUT_IN;
  auto steps = h.collectSteps(3000);
  uint16_t p0 = B(0) | B(7), p1 = B(1) | B(6), p2 = B(2) | B(5), p3 = B(3) | B(4);
  uint16_t expect[] = {p3, p2, p1, p0, p1, p2, p3, p2};
  EXPECT_TRUE(steps.size() >= 8);
  for (int i = 0; i < 8 && i < (int)steps.size(); ++i)
    EXPECT_EQ(steps[i], expect[i]);
}

TEST(one_shot_up_down_full_cycle_length) {
  SeqH h;
  h.in.mode = ModeId::CHASE_UP_DOWN;
  h.in.repeat = false;
  auto steps = h.collectSteps(10000);
  // 0..7 (8 steps) + 6..0 (7 steps) = 15, endpoints fired once.
  EXPECT_EQ(steps.size(), (size_t)15);
  EXPECT_EQ(h.seq.requestedMask(), (uint16_t)0);
}

TEST(one_shot_fire_all_single_poof) {
  SeqH h;
  h.in.mode = ModeId::FIRE_ALL;
  h.in.repeat = false;
  auto steps = h.collectSteps(5000);
  EXPECT_EQ(steps.size(), (size_t)1);
  EXPECT_EQ(steps[0], (uint16_t)0xFFFF);
  EXPECT_EQ(h.seq.requestedMask(), (uint16_t)0);
}

TEST(alternate_respects_trigger_enables) {
  SeqH h;
  h.in.mode = ModeId::ALTERNATE;
  h.in.trigger_mask = 0x00FF;  // only poofers 1-8
  auto steps = h.collectSteps(1000);
  EXPECT_TRUE(steps.size() >= 2);
  EXPECT_EQ(steps[0], (uint16_t)0x0055);
  EXPECT_EQ(steps[1], (uint16_t)0x00AA);
}

TEST(rate_scales_chase_tempo) {
  SeqH slow, fast;
  slow.in.mode = fast.in.mode = ModeId::CHASE_UP;
  slow.in.rest_ms = fast.in.rest_ms = 1000;
  slow.in.rate = 128;  // 1.0x
  fast.in.rate = 255;  // maximum tempo (rest floors at MIN_REST_MS)
  auto s = slow.collectSteps(10000);
  auto f = fast.collectSteps(10000);
  EXPECT_TRUE(f.size() > s.size() * 3);
}

TEST(all_triggers_disabled_chase_is_silent_but_recovers) {
  SeqH h;
  h.in.mode = ModeId::CHASE_UP;
  h.in.trigger_mask = 0;
  auto silent = h.collectSteps(3000);
  EXPECT_EQ(silent.size(), (size_t)0);
  h.in.trigger_mask = 0xFFFF;  // re-enable mid-run
  auto resumed = h.collectSteps(2000);
  EXPECT_TRUE(resumed.size() >= 5);
}

TEST(rest_duration_hostile_input_clamped) {
  SeqH h;
  h.in.mode = ModeId::FIRE_ALL;
  h.in.rest_ms = 65535;
  h.in.rate = 0;  // slowest: rest x2 — must still clamp to MAX_REST_MS
  auto steps = h.collectSteps(2 * (cfg::MAX_POOF_MS + cfg::MAX_REST_MS) + 1000);
  EXPECT_TRUE(steps.size() >= 2);  // second poof arrived within the clamp
}

TEST(switching_between_chases_restarts_cleanly) {
  SeqH h;
  h.in.mode = ModeId::CHASE_UP;
  h.step(350);  // mid-pattern
  h.in.mode = ModeId::CHASE_DOWN;
  auto steps = h.collectSteps(1000);
  EXPECT_TRUE(steps.size() >= 1);
  EXPECT_EQ(steps[0], B(7));  // CHASE_DOWN starts at its own start index
}
