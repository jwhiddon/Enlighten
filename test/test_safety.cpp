#include "core/safety.h"
#include "framework.h"
#include "sim.h"

TEST(fresh_supervisor_is_safe) {
  SafetySupervisor s;
  ShowInput in;
  in.link_ok = true;
  in.arm_a = in.arm_b = true;
  HwInputs hw{true, true};
  EXPECT_EQ(s.filter(0xFFFF, in, hw, 0), (uint16_t)0);
  EXPECT_EQ(s.state(), SafetyState::BOOT_SELFTEST);
}

TEST(boot_fault_locks_out) {
  Sim s;
  s.begin(FaultCode::WATCHDOG_RESET);
  EXPECT_EQ(s.safety.state(), SafetyState::FAULT_LOCKOUT);
  EXPECT_EQ(s.safety.fault(), FaultCode::WATCHDOG_RESET);
  s.in.link_ok = true;
  s.in.arm_a = s.in.arm_b = true;  // console trying to arm changes nothing
  s.in.mode = ModeId::FIRE_ALL;
  s.in.trigger_mask = 0xFFFF;
  s.step(2000);
  EXPECT_EQ(s.last_mask, (uint16_t)0);
  EXPECT_EQ(s.safety.state(), SafetyState::FAULT_LOCKOUT);
  // Clearing requires link up with BOTH arm values de-asserted...
  s.in.arm_a = s.in.arm_b = false;
  s.step(2);
  EXPECT_EQ(s.safety.state(), SafetyState::SAFE);
  // ...after which a normal full handshake arms.
  s.in.arm_a = s.in.arm_b = true;
  s.step(cfg::ARM_HOLD_MS + 5);
  EXPECT_EQ(s.safety.state(), SafetyState::ARMED);
  EXPECT_TRUE(s.checker.ok());
}

TEST(arm_requires_rising_edge) {
  Sim s;
  s.begin();
  // Arm values present from the very first tick (saved console scene):
  s.in.link_ok = true;
  s.in.arm_a = s.in.arm_b = true;
  s.step(5000);
  EXPECT_EQ(s.safety.state(), SafetyState::SAFE);  // never arms
  // Operator drops the values, then raises them: now it arms.
  s.in.arm_a = s.in.arm_b = false;
  s.step(5);
  s.in.arm_a = s.in.arm_b = true;
  s.step(cfg::ARM_HOLD_MS + 5);
  EXPECT_EQ(s.safety.state(), SafetyState::ARMED);
  EXPECT_TRUE(s.checker.ok());
}

TEST(arm_requires_continuous_hold) {
  Sim s;
  s.begin();
  s.in.link_ok = true;
  s.step(5);  // edge witnessed (values de-asserted)
  s.in.arm_a = s.in.arm_b = true;
  s.step(cfg::ARM_HOLD_MS - 10);
  EXPECT_EQ(s.safety.state(), SafetyState::ARM_PENDING);
  s.in.arm_b = false;  // 1-tick dropout at 490 ms
  s.step(1);
  EXPECT_EQ(s.safety.state(), SafetyState::SAFE);
  s.in.arm_b = true;
  s.step(cfg::ARM_HOLD_MS - 10);
  EXPECT_EQ(s.safety.state(), SafetyState::ARM_PENDING);  // timer restarted
  s.step(20);
  EXPECT_EQ(s.safety.state(), SafetyState::ARMED);
  EXPECT_TRUE(s.checker.ok());
}

TEST(link_blip_with_held_arm_values_cannot_rearm) {
  Sim s;
  s.begin();
  s.armFully();
  EXPECT_EQ(s.safety.state(), SafetyState::ARMED);
  // Signal blip while the console keeps the arm values in the scene:
  s.in.link_ok = false;
  s.step(10);
  EXPECT_EQ(s.safety.state(), SafetyState::SAFE);
  s.in.link_ok = true;  // link returns, arm values still asserted
  s.step(5000);
  EXPECT_EQ(s.safety.state(), SafetyState::SAFE);  // must NOT auto-rearm
  // Operator explicitly cycles the arm values: arms again.
  s.in.arm_a = s.in.arm_b = false;
  s.step(5);
  s.in.arm_a = s.in.arm_b = true;
  s.step(cfg::ARM_HOLD_MS + 5);
  EXPECT_EQ(s.safety.state(), SafetyState::ARMED);
  EXPECT_TRUE(s.checker.ok());
}

TEST(signal_loss_closes_next_tick) {
  Sim s;
  s.begin();
  s.armFully();
  s.in.mode = ModeId::FIRE_ALL;
  s.in.trigger_mask = 0xFFFF;
  s.in.poof_ms = 300;
  s.in.repeat = true;
  s.step(50);
  EXPECT_TRUE(s.last_mask != 0);  // firing
  s.in.link_ok = false;           // deadman trips
  s.tick();
  EXPECT_EQ(s.last_mask, (uint16_t)0);
  EXPECT_EQ(s.safety.state(), SafetyState::SAFE);
  EXPECT_TRUE(s.checker.ok());
}

TEST(disarm_is_immediate) {
  Sim s;
  s.begin();
  s.armFully();
  s.in.mode = ModeId::RAW;
  s.in.trigger_mask = 0x0001;
  s.in.poof_ms = 300;
  s.step(20);
  EXPECT_TRUE(s.last_mask != 0);
  s.in.arm_a = false;  // operator pulls the arm fader
  s.tick();
  EXPECT_EQ(s.last_mask, (uint16_t)0);
  EXPECT_EQ(s.safety.state(), SafetyState::SAFE);
  EXPECT_TRUE(s.checker.ok());
}

TEST(estop_overrides_every_state) {
  // From ARMED while firing:
  {
    Sim s;
    s.begin();
    s.armFully();
    s.in.mode = ModeId::FIRE_ALL;
    s.in.trigger_mask = 0xFFFF;
    s.in.poof_ms = 300;
    s.in.repeat = true;
    s.step(50);
    EXPECT_TRUE(s.last_mask != 0);
    s.hw.estop_ok = false;
    s.tick();
    EXPECT_EQ(s.last_mask, (uint16_t)0);
    EXPECT_EQ(s.safety.state(), SafetyState::FAULT_LOCKOUT);
    EXPECT_EQ(s.safety.fault(), FaultCode::ESTOP_ASSERTED);
    // Releasing the E-stop alone does NOT re-enable anything.
    s.hw.estop_ok = true;
    s.step(1000);
    EXPECT_EQ(s.safety.state(), SafetyState::FAULT_LOCKOUT);
    // Only de-asserting the arm values acknowledges the fault...
    s.in.arm_a = s.in.arm_b = false;
    s.step(2);
    EXPECT_EQ(s.safety.state(), SafetyState::SAFE);
    EXPECT_TRUE(s.checker.ok());
  }
  // From SAFE and ARM_PENDING:
  {
    Sim s;
    s.begin();
    s.in.link_ok = true;
    s.step(5);
    s.hw.estop_ok = false;
    s.tick();
    EXPECT_EQ(s.safety.state(), SafetyState::FAULT_LOCKOUT);
  }
  {
    Sim s;
    s.begin();
    s.in.link_ok = true;
    s.step(5);
    s.in.arm_a = s.in.arm_b = true;
    s.step(100);
    EXPECT_EQ(s.safety.state(), SafetyState::ARM_PENDING);
    s.hw.estop_ok = false;
    s.tick();
    EXPECT_EQ(s.safety.state(), SafetyState::FAULT_LOCKOUT);
  }
}

TEST(buggy_decoder_poof_cannot_exceed_hard_cap) {
  Sim s;
  s.begin();
  s.armFully();
  s.in.mode = ModeId::RAW;
  s.in.trigger_mask = 0x0001;
  s.in.poof_ms = 65535;  // hostile/buggy input
  s.in.rest_ms = 65535;
  s.in.repeat = true;
  s.step(30000);
  EXPECT_TRUE(s.checker.ok());  // includes the MAX_OPEN_MS invariant
}

TEST(loop_stall_latches_fault) {
  Sim s;
  s.begin();
  s.armFully();
  EXPECT_EQ(s.safety.state(), SafetyState::ARMED);
  // Simulate a stalled loop: next filter call arrives 50 ms late.
  s.now += 50;
  s.seq.update(s.in, s.now);
  s.last_mask = s.safety.filter(s.seq.requestedMask(), s.in, s.hw, s.now);
  EXPECT_EQ(s.safety.state(), SafetyState::FAULT_LOCKOUT);
  EXPECT_EQ(s.safety.fault(), FaultCode::LOOP_OVERRUN);
  EXPECT_EQ(s.last_mask, (uint16_t)0);
}

TEST(loop_health_gate) {
  SafetySupervisor s;
  EXPECT_FALSE(s.consumeLoopHealth());  // no filter yet: don't feed the dog
  ShowInput in;
  HwInputs hw{true, true};
  s.filter(0, in, hw, 0);
  EXPECT_TRUE(s.consumeLoopHealth());
  EXPECT_FALSE(s.consumeLoopHealth());  // consumed
}

TEST(duty_budget_holds_under_hammering) {
  Sim s;
  s.begin();
  s.armFully();
  s.in.mode = ModeId::RAW;
  s.in.trigger_mask = 0xFFFF;
  s.in.poof_ms = 500;
  s.in.rest_ms = 45;
  s.in.repeat = true;
  s.step(60000);  // a full minute of maximum demand on all 16 channels
  EXPECT_TRUE(s.checker.ok());  // sliding-window duty invariant held
}
