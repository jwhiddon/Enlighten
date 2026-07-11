// Additional SafetySupervisor edge cases beyond the core contract.
#include "core/safety.h"
#include "framework.h"
#include "sim.h"

TEST(begin_is_one_shot) {
  Sim s;
  s.begin(FaultCode::NONE);
  EXPECT_EQ(s.safety.state(), SafetyState::SAFE);
  s.safety.begin(FaultCode::WATCHDOG_RESET, s.now);  // must be ignored
  EXPECT_EQ(s.safety.state(), SafetyState::SAFE);
}

TEST(brownout_boot_fault_code) {
  Sim s;
  s.begin(FaultCode::BROWNOUT_RESET);
  EXPECT_EQ(s.safety.state(), SafetyState::FAULT_LOCKOUT);
  EXPECT_EQ(s.safety.fault(), FaultCode::BROWNOUT_RESET);
}

TEST(reported_fault_locks_out_immediately) {
  Sim s;
  s.begin();
  s.armFully();
  s.in.mode = ModeId::FIRE_ALL;
  s.in.trigger_mask = 0xFFFF;
  s.in.poof_ms = 300;
  s.in.repeat = true;
  s.step(50);
  EXPECT_TRUE(s.last_mask != 0);
  s.safety.reportFault(FaultCode::DECODER_INVALID);
  s.tick();
  EXPECT_EQ(s.last_mask, (uint16_t)0);
  EXPECT_EQ(s.safety.state(), SafetyState::FAULT_LOCKOUT);
  EXPECT_EQ(s.safety.fault(), FaultCode::DECODER_INVALID);
  EXPECT_TRUE(s.checker.ok());
}

TEST(estop_before_begin_keeps_outputs_closed) {
  SafetySupervisor s;
  ShowInput in;
  in.link_ok = true;
  in.arm_a = in.arm_b = true;
  HwInputs hw{false, true};  // E-stop asserted during boot
  EXPECT_EQ(s.filter(0xFFFF, in, hw, 1), (uint16_t)0);
  EXPECT_EQ(s.state(), SafetyState::BOOT_SELFTEST);  // begin() decides boot faults
}

TEST(arm_key_drop_while_pending_returns_safe) {
  Sim s;
  s.begin();
  s.in.link_ok = true;
  s.step(5);
  s.in.arm_a = s.in.arm_b = true;
  s.step(100);
  EXPECT_EQ(s.safety.state(), SafetyState::ARM_PENDING);
  s.hw.arm_key = false;
  s.step(1);
  EXPECT_EQ(s.safety.state(), SafetyState::SAFE);
  EXPECT_TRUE(s.checker.ok());
}

TEST(partial_handshake_never_arms) {
  Sim s;
  s.begin();
  s.in.link_ok = true;
  s.step(5);
  s.in.arm_a = true;  // only one of the two values
  s.step(10000);
  EXPECT_EQ(s.safety.state(), SafetyState::SAFE);
  s.in.arm_a = false;
  s.in.arm_b = true;
  s.step(10000);
  EXPECT_EQ(s.safety.state(), SafetyState::SAFE);
  EXPECT_TRUE(s.checker.ok());
}

TEST(lockout_requires_link_to_clear) {
  // The fault code must stay visible until an operator is actually present
  // (link up) with the arm values down — a dead console can't clear it.
  Sim s;
  s.begin(FaultCode::WATCHDOG_RESET);
  s.in.link_ok = false;
  s.in.arm_a = s.in.arm_b = false;
  s.step(5000);
  EXPECT_EQ(s.safety.state(), SafetyState::FAULT_LOCKOUT);
  s.in.link_ok = true;
  s.step(5);
  EXPECT_EQ(s.safety.state(), SafetyState::SAFE);
}

TEST(rearm_after_disarm_works_repeatedly) {
  Sim s;
  s.begin();
  for (int round = 0; round < 5; ++round) {
    s.armFully();
    EXPECT_EQ(s.safety.state(), SafetyState::ARMED);
    s.in.arm_a = s.in.arm_b = false;  // disarm
    s.step(5);
    EXPECT_EQ(s.safety.state(), SafetyState::SAFE);
  }
  EXPECT_TRUE(s.checker.ok());
}
