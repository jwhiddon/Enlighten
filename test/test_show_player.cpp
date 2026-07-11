#include <cstring>

#include "core/safety.h"
#include "core/show_player.h"
#include "framework.h"
#include "sim.h"

namespace {
const char* DEMO_SHOW =
    "# ENLIGHTEN SHOW v1\n"
    "# poofers=8 duration_ms=1000\n"
    "100 0001\n"
    "300 0000\n"
    "400 8003\n"
    "700 0000\n"
    "1000 0000\n";

struct PlayerHarness {
  ShowPlayer p;
  const char* data = DEMO_SHOW;
  size_t pos = 0;
  TimeMs now = 1000;  // playback clock is relative to start()

  void pump() {
    while (p.wantsData()) {
      if (data[pos] == 0) {
        p.feedEof();
        break;
      }
      p.feed(data[pos++]);
    }
  }
  void step(uint32_t ms) {
    while (ms--) {
      ++now;
      pump();
      p.update(now);
    }
  }
};
}  // namespace

TEST(player_replays_timeline) {
  PlayerHarness h;
  h.p.start(h.now);
  h.step(50);
  EXPECT_EQ(h.p.mask(), (uint16_t)0);
  h.step(100);  // t=150
  EXPECT_EQ(h.p.mask(), (uint16_t)0x0001);
  h.step(200);  // t=350
  EXPECT_EQ(h.p.mask(), (uint16_t)0x0000);
  h.step(100);  // t=450
  EXPECT_EQ(h.p.mask(), (uint16_t)0x8003);
  h.step(300);  // t=750
  EXPECT_EQ(h.p.mask(), (uint16_t)0x0000);
  h.step(300);  // past the end
  EXPECT_FALSE(h.p.playing());  // finished cleanly
}

TEST(player_position_tracks) {
  PlayerHarness h;
  h.p.start(h.now);
  h.step(444);
  EXPECT_EQ(h.p.positionMs(), (uint32_t)444);
}

TEST(player_stop_clears_immediately) {
  PlayerHarness h;
  h.p.start(h.now);
  h.step(150);
  EXPECT_EQ(h.p.mask(), (uint16_t)0x0001);
  h.p.stop();
  EXPECT_EQ(h.p.mask(), (uint16_t)0);
  EXPECT_FALSE(h.p.playing());
}

TEST(player_truncated_file_never_leaves_request_hanging) {
  // File ends mid-poof with no closing line: end of data = all off.
  ShowPlayer p;
  const char* bad = "100 FFFF\n";
  TimeMs now = 0;
  p.start(now);
  for (const char* c = bad; *c; ++c) p.feed(*c);
  p.feedEof();
  for (int i = 0; i < 150; ++i) p.update(++now);
  EXPECT_EQ(p.mask(), (uint16_t)0);
  EXPECT_FALSE(p.playing());
}

TEST(player_ignores_garbage_lines) {
  ShowPlayer p;
  const char* junk =
      "# comment\n"
      "\n"
      "not a line at all\n"
      "12garbage\n"
      "50 0002\n"
      "80 0000\n";
  TimeMs now = 0;
  p.start(now);
  for (const char* c = junk; *c; ++c) p.feed(*c);
  p.feedEof();
  for (int i = 0; i < 60; ++i) p.update(++now);
  EXPECT_EQ(p.mask(), (uint16_t)0x0002);
}

TEST(player_streams_larger_than_ring) {
  // 40 events > the 8-entry ring: streaming must still replay all of them.
  char big[2048];
  int n = 0;
  for (int i = 0; i < 20; ++i) {
    n += std::snprintf(big + n, sizeof(big) - n, "%d %04X\n", 100 + i * 100,
                       1 << (i % 8));
    n += std::snprintf(big + n, sizeof(big) - n, "%d 0000\n", 150 + i * 100);
  }
  ShowPlayer p;
  size_t pos = 0;
  TimeMs now = 0;
  p.start(now);
  uint32_t poofs_seen = 0;
  uint16_t prev = 0;
  for (int t = 0; t < 2300; ++t) {
    while (p.wantsData() && big[pos]) p.feed(big[pos++]);
    if (!big[pos]) p.feedEof();
    p.update(++now);
    if (p.mask() && p.mask() != prev) ++poofs_seen;
    prev = p.mask();
  }
  EXPECT_EQ(poofs_seen, (uint32_t)20);
  EXPECT_FALSE(p.playing());
}

TEST(player_requests_gated_by_safety) {
  // Playback while DISARMED produces zero output — the player is a request
  // source, not an authority.
  ShowPlayer p;
  SafetySupervisor safety;
  InvariantChecker checker;
  ShowInput in;  // no link, no arm: SAFE at best
  HwInputs hw{true, true};
  TimeMs now = 0;
  safety.begin(FaultCode::NONE, now);
  p.start(now);
  const char* show = "10 FFFF\n5000 0000\n";
  for (const char* c = show; *c; ++c) p.feed(*c);
  p.feedEof();
  for (int t = 0; t < 3000; ++t) {
    ++now;
    p.update(now);
    uint16_t out = safety.filter(p.mask(), in, hw, now);
    checker.observe(safety.state(), hw.estop_ok, out, now);
    EXPECT_EQ(out, (uint16_t)0);
    if (testfw::failures()) return;
  }
  EXPECT_TRUE(checker.ok());
}
