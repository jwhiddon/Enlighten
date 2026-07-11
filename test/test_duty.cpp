#include "core/config.h"
#include "core/duty_limiter.h"
#include "framework.h"

TEST(duty_first_open_immediate) {
  DutyLimiter d;
  EXPECT_TRUE(d.evaluate(true, 1000));
}

TEST(duty_never_wants_never_opens) {
  DutyLimiter d;
  for (TimeMs t = 0; t < 5000; ++t) EXPECT_FALSE(d.evaluate(false, t));
}

TEST(duty_max_open_enforced) {
  DutyLimiter d;
  TimeMs t = 0;
  EXPECT_TRUE(d.evaluate(true, t));
  uint32_t open_ms = 0;
  while (d.evaluate(true, ++t) && open_ms < 10000) ++open_ms;
  // Forced closed at exactly the cap despite continuous demand.
  EXPECT_EQ(open_ms, (uint32_t)(cfg::MAX_OPEN_MS - 1));
  // And it stays closed for at least MIN_CLOSE_MS.
  TimeMs closed_at = t;
  while (!d.evaluate(true, ++t) && t < closed_at + 60000) {
  }
  EXPECT_TRUE(elapsedMs(closed_at, t) >= cfg::MIN_CLOSE_MS);
}

TEST(duty_min_close_enforced) {
  DutyLimiter d;
  TimeMs t = 100;
  EXPECT_TRUE(d.evaluate(true, t));
  t += 100;
  EXPECT_FALSE(d.evaluate(false, t));  // voluntary close after 100 ms
  // Re-request immediately: must be held closed MIN_CLOSE_MS.
  TimeMs closed_at = t;
  while (!d.evaluate(true, ++t)) {
  }
  EXPECT_EQ(elapsedMs(closed_at, t), (uint32_t)cfg::MIN_CLOSE_MS);
}

TEST(duty_survives_large_time_jumps) {
  DutyLimiter d;
  TimeMs t = 0;
  EXPECT_TRUE(d.evaluate(true, t));
  // Caller stalls for 100 s while the channel was open: on resume the
  // continuous-open cap must close it immediately.
  t += 100000;
  EXPECT_FALSE(d.evaluate(true, t));
  // After the stall all bucket history has aged out; accounting is sane.
  t += cfg::DUTY_WINDOW_MS + 2000;
  d.evaluate(false, t);
  EXPECT_EQ(d.windowOpenMs(), (uint32_t)0);
  EXPECT_TRUE(d.evaluate(true, t + 1));
}

TEST(duty_window_budget_enforced_and_drains) {
  DutyLimiter d;
  TimeMs t = 0;
  uint32_t total_open_first_window = 0;
  // Hammer the channel continuously for one full window.
  for (; t < cfg::DUTY_WINDOW_MS; ++t) {
    if (d.evaluate(true, t)) ++total_open_first_window;
  }
  // Cannot meaningfully exceed the budget (tick-granularity tolerance).
  EXPECT_TRUE(total_open_first_window <= cfg::MAX_OPEN_PER_WINDOW_MS + 10);
  // Budget used: with demand still present the channel is now mostly shut.
  // After enough quiet time the window drains and it may open again.
  for (; t < 3u * cfg::DUTY_WINDOW_MS; ++t) d.evaluate(false, t);
  EXPECT_EQ(d.windowOpenMs(), (uint32_t)0);
  EXPECT_TRUE(d.evaluate(true, t));
}
