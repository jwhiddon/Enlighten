#include <vector>

#include "core/sequencer.h"
#include "framework.h"

namespace {

struct SeqHarness {
  Sequencer seq;
  ShowInput in;
  TimeMs now = 0;

  SeqHarness() {
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

  // Run and record each distinct nonzero mask (one entry per step fired).
  std::vector<uint16_t> collectSteps(uint32_t ms) {
    std::vector<uint16_t> steps;
    uint16_t prev = 0;
    for (uint32_t i = 0; i < ms; ++i) {
      ++now;
      seq.update(in, now);
      uint16_t m = seq.requestedMask();
      if (m != 0 && m != prev) steps.push_back(m);
      if (m != 0) prev = m;
      else prev = 0;
    }
    return steps;
  }
};

constexpr uint16_t B(int i) { return (uint16_t)(1u << i); }

}  // namespace

TEST(chase_up_order) {
  SeqHarness h;
  h.in.mode = ModeId::CHASE_UP;
  auto steps = h.collectSteps(2000);
  EXPECT_TRUE(steps.size() >= 9);
  for (int i = 0; i < 8; ++i) EXPECT_EQ(steps[i], B(i));
  EXPECT_EQ(steps[8], B(0));  // wraps with repeat on
}

TEST(chase_down_order) {
  SeqHarness h;
  h.in.mode = ModeId::CHASE_DOWN;
  auto steps = h.collectSteps(2000);
  EXPECT_TRUE(steps.size() >= 8);
  for (int i = 0; i < 8; ++i) EXPECT_EQ(steps[i], B(7 - i));
}

TEST(chase_up_down_bounces_single_endpoint_fire) {
  SeqHarness h;
  h.in.mode = ModeId::CHASE_UP_DOWN;
  auto steps = h.collectSteps(4000);
  // 0 1 2 3 4 5 6 7 6 5 4 3 2 1 0 1 ...
  uint16_t expect[] = {B(0), B(1), B(2), B(3), B(4), B(5), B(6), B(7),
                       B(6), B(5), B(4), B(3), B(2), B(1), B(0), B(1)};
  EXPECT_TRUE(steps.size() >= 16);
  for (int i = 0; i < 16 && i < (int)steps.size(); ++i)
    EXPECT_EQ(steps[i], expect[i]);
}

TEST(chase_in_fires_mirrored_pairs) {
  SeqHarness h;
  h.in.mode = ModeId::CHASE_IN;
  auto steps = h.collectSteps(2000);
  uint16_t expect[] = {(uint16_t)(B(0) | B(7)), (uint16_t)(B(1) | B(6)),
                       (uint16_t)(B(2) | B(5)), (uint16_t)(B(3) | B(4))};
  EXPECT_TRUE(steps.size() >= 5);
  for (int i = 0; i < 4; ++i) EXPECT_EQ(steps[i], expect[i]);
  EXPECT_EQ(steps[4], expect[0]);  // repeats from the outside
}

TEST(chase_out_fires_mirrored_pairs_inside_out) {
  SeqHarness h;
  h.in.mode = ModeId::CHASE_OUT;
  auto steps = h.collectSteps(2000);
  uint16_t expect[] = {(uint16_t)(B(3) | B(4)), (uint16_t)(B(2) | B(5)),
                       (uint16_t)(B(1) | B(6)), (uint16_t)(B(0) | B(7))};
  EXPECT_TRUE(steps.size() >= 4);
  for (int i = 0; i < 4; ++i) EXPECT_EQ(steps[i], expect[i]);
}

TEST(one_shot_stops_then_repeat_resumes) {
  SeqHarness h;
  h.in.mode = ModeId::CHASE_UP;
  h.in.repeat = false;
  auto steps = h.collectSteps(5000);
  EXPECT_EQ(steps.size(), (size_t)8);          // exactly one pass
  EXPECT_EQ(h.seq.requestedMask(), (uint16_t)0);  // idle afterwards
  h.in.repeat = true;  // toggling repeat resumes the pattern
  h.step(5);
  EXPECT_TRUE(h.seq.requestedMask() != 0);
}

TEST(alternate_evens_odds) {
  SeqHarness h;
  h.in.mode = ModeId::ALTERNATE;
  auto steps = h.collectSteps(1500);
  EXPECT_TRUE(steps.size() >= 3);
  EXPECT_EQ(steps[0], (uint16_t)0x5555);
  EXPECT_EQ(steps[1], (uint16_t)0xAAAA);
  EXPECT_EQ(steps[2], (uint16_t)0x5555);
}

TEST(fire_all_respects_enables) {
  SeqHarness h;
  h.in.mode = ModeId::FIRE_ALL;
  h.in.trigger_mask = 0x00FF;  // only poofers 1-8 enabled
  h.step(10);
  EXPECT_EQ(h.seq.requestedMask(), (uint16_t)0x00FF);
}

TEST(disabling_poofer_mid_step_drops_it_immediately) {
  SeqHarness h;
  h.in.mode = ModeId::FIRE_ALL;
  h.step(10);
  EXPECT_EQ(h.seq.requestedMask(), (uint16_t)0xFFFF);
  h.in.trigger_mask = 0xFFFE;  // poofer 1 disabled mid-poof
  h.step(1);
  EXPECT_EQ(h.seq.requestedMask(), (uint16_t)0xFFFE);
}

TEST(mode_change_clears_requests) {
  SeqHarness h;
  h.in.mode = ModeId::FIRE_ALL;
  h.step(10);
  EXPECT_TRUE(h.seq.requestedMask() != 0);
  h.in.mode = ModeId::OFF;
  h.step(1);
  EXPECT_EQ(h.seq.requestedMask(), (uint16_t)0);
}

TEST(raw_poof_bounded_and_release_semantics) {
  SeqHarness h;
  h.in.mode = ModeId::RAW;
  h.in.repeat = false;
  h.in.trigger_mask = B(2);
  h.in.poof_ms = 100;
  h.step(50);
  EXPECT_EQ(h.seq.requestedMask(), B(2));  // firing
  h.step(100);
  EXPECT_EQ(h.seq.requestedMask(), (uint16_t)0);  // poof ended at 100 ms
  h.step(2000);
  EXPECT_EQ(h.seq.requestedMask(), (uint16_t)0);  // held trigger: no re-fire
  h.in.trigger_mask = 0;  // release...
  h.step(5);
  h.in.trigger_mask = B(2);  // ...and press again
  h.step(5);
  EXPECT_EQ(h.seq.requestedMask(), B(2));  // re-fires after release
}

TEST(raw_repeat_refires_after_rest) {
  SeqHarness h;
  h.in.mode = ModeId::RAW;
  h.in.repeat = true;
  h.in.trigger_mask = B(0);
  h.in.poof_ms = 100;
  h.in.rest_ms = 100;
  uint32_t fired_ms = 0;
  for (int i = 0; i < 1000; ++i) {
    h.step(1);
    if (h.seq.requestedMask() & B(0)) ++fired_ms;
  }
  // ~100 ms on / ~100 ms off duty over 1 s => several hundred ms total.
  EXPECT_TRUE(fired_ms >= 300 && fired_ms <= 600);
}

TEST(hostile_poof_duration_clamped) {
  SeqHarness h;
  h.in.mode = ModeId::RAW;
  h.in.trigger_mask = B(0);
  h.in.poof_ms = 65535;
  uint32_t run = 0;
  for (int i = 0; i < 5000; ++i) {
    h.step(1);
    if (h.seq.requestedMask() & B(0)) ++run;
    else break;
  }
  EXPECT_TRUE(run <= cfg::MAX_POOF_MS);
}
