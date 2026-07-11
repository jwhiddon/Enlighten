#include "core/timebase.h"
#include "framework.h"

TEST(time_reached_basic) {
  EXPECT_TRUE(timeReached(100, 100));
  EXPECT_TRUE(timeReached(100, 101));
  EXPECT_FALSE(timeReached(100, 99));
}

TEST(time_reached_across_wrap) {
  // Deadline computed just before the 32-bit wrap lands just after it.
  TimeMs base = 0xFFFFFFF0u;
  TimeMs deadline = base + 100;  // wraps to 0x54
  EXPECT_EQ(deadline, (TimeMs)0x54);
  EXPECT_FALSE(timeReached(deadline, base));       // 100 ms early
  EXPECT_FALSE(timeReached(deadline, base + 99));  // 1 ms early (now wrapped)
  EXPECT_TRUE(timeReached(deadline, base + 100));  // exactly due
  EXPECT_TRUE(timeReached(deadline, base + 500));  // past due
}

TEST(elapsed_across_wrap) {
  EXPECT_EQ(elapsedMs(0xFFFFFFF0u, 0x10u), (uint32_t)0x20);
  EXPECT_EQ(elapsedMs(5, 5), (uint32_t)0);
}
