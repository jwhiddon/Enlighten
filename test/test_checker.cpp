// Meta-tests: prove the InvariantChecker actually catches each class of
// violation.  A guard rail that can't fail is not a guard rail.
#include "framework.h"
#include "sim.h"

TEST(checker_accepts_clean_timeline) {
  InvariantChecker c;
  TimeMs now = 0;
  // 400 ms open / 600 ms closed at 40% off-time = within every limit.
  for (int cycle = 0; cycle < 20; ++cycle) {
    for (int i = 0; i < 400; ++i) c.observe(SafetyState::ARMED, true, 1, ++now);
    for (int i = 0; i < 4600; ++i) c.observe(SafetyState::ARMED, true, 0, ++now);
  }
  EXPECT_TRUE(c.ok());
}

TEST(checker_catches_output_while_not_armed) {
  InvariantChecker c;
  c.observe(SafetyState::SAFE, true, 0x0001, 1);
  EXPECT_FALSE(c.ok());
}

TEST(checker_catches_output_with_estop) {
  InvariantChecker c;
  c.observe(SafetyState::ARMED, false, 0x0001, 1);
  EXPECT_FALSE(c.ok());
}

TEST(checker_catches_overlong_open) {
  InvariantChecker c;
  TimeMs now = 0;
  for (int i = 0; i < (int)cfg::MAX_OPEN_MS + 50; ++i)
    c.observe(SafetyState::ARMED, true, 0x0001, ++now);
  EXPECT_FALSE(c.ok());
}

TEST(checker_catches_min_close_violation) {
  InvariantChecker c;
  TimeMs now = 0;
  for (int i = 0; i < 100; ++i) c.observe(SafetyState::ARMED, true, 1, ++now);
  for (int i = 0; i < 10; ++i) c.observe(SafetyState::ARMED, true, 0, ++now);
  c.observe(SafetyState::ARMED, true, 1, ++now);  // reopened after only 10 ms
  EXPECT_FALSE(c.ok());
}

TEST(checker_catches_duty_budget_violation) {
  InvariantChecker c;
  TimeMs now = 0;
  // 400 open / 60 closed passes MAX_OPEN and MIN_CLOSE but racks up ~87%
  // duty — far past the 30% sliding budget.
  for (int cycle = 0; cycle < 30 && c.ok(); ++cycle) {
    for (int i = 0; i < 400; ++i) c.observe(SafetyState::ARMED, true, 1, ++now);
    for (int i = 0; i < 60; ++i) c.observe(SafetyState::ARMED, true, 0, ++now);
  }
  EXPECT_FALSE(c.ok());
}
