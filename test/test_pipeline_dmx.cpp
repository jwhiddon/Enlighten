// End-to-end DMX scenarios: 24-byte channel frames through the full
// decoder -> sequencer -> safety pipeline, as wired in Enlighten.ino.
#include <vector>

#include "framework.h"
#include "pipeline.h"

TEST(e2e_dmx_arm_and_raw_fire) {
  DmxPipeline p;
  p.armFromConsole();
  EXPECT_EQ(p.safety.state(), SafetyState::ARMED);

  p.ch[2] = 250;  // MODE -> RAW band
  p.step(cfg::MODE_DEBOUNCE_MS + 20);
  p.ch[3] = 255;  // POOF_MS -> 500 ms
  p.ch[8] = 255;  // trigger poofer 1
  p.step(10);
  EXPECT_EQ(p.mask, (uint16_t)0x0001);

  // Poof ends at the cap; with repeat OFF a held channel must not refire.
  p.step(600);
  EXPECT_EQ(p.mask, (uint16_t)0);
  p.step(3000);
  EXPECT_EQ(p.mask, (uint16_t)0);

  // Release below the OFF threshold, press again: fires again.
  p.ch[8] = 0;
  p.step(cfg::MIN_CLOSE_MS + 10);
  p.ch[8] = 255;
  p.step(10);
  EXPECT_EQ(p.mask, (uint16_t)0x0001);
  EXPECT_TRUE(p.checker.ok());
}

TEST(e2e_dmx_saved_scene_cannot_arm_at_patch_in) {
  DmxPipeline p;
  // Console comes up with the arm scene already active.
  p.ch[0] = cfg::DMX_ARM_A;
  p.ch[1] = cfg::DMX_ARM_B;
  p.step(10000);
  EXPECT_EQ(p.safety.state(), SafetyState::SAFE);
  EXPECT_TRUE(p.checker.ok());
}

TEST(e2e_dmx_signal_loss_and_recovery) {
  DmxPipeline p;
  p.armFromConsole();
  p.ch[2] = 230;  // FIRE_ALL
  p.ch[6] = 255;  // repeat
  p.ch[3] = 128;
  for (int i = 8; i < 24; ++i) p.ch[i] = 255;
  p.step(cfg::MODE_DEBOUNCE_MS + 100);
  EXPECT_TRUE(p.mask != 0);  // firing

  p.signal = false;  // cable pulled mid-show
  p.step(cfg::DMX_TIMEOUT_MS + 5);
  EXPECT_EQ(p.mask, (uint16_t)0);
  EXPECT_EQ(p.safety.state(), SafetyState::SAFE);

  // Signal returns with the arm scene still active: must stay disarmed.
  p.signal = true;
  p.step(10000);
  EXPECT_EQ(p.safety.state(), SafetyState::SAFE);

  // Operator cycles the arm values: arms again.
  p.armFromConsole();
  EXPECT_EQ(p.safety.state(), SafetyState::ARMED);
  EXPECT_TRUE(p.checker.ok());
}

TEST(e2e_dmx_estop_lockout_and_acknowledge) {
  DmxPipeline p;
  p.armFromConsole();
  p.ch[2] = 230;  // FIRE_ALL
  p.ch[6] = 255;
  for (int i = 8; i < 24; ++i) p.ch[i] = 255;
  p.step(cfg::MODE_DEBOUNCE_MS + 100);
  EXPECT_TRUE(p.mask != 0);

  p.hw.estop_ok = false;  // mushroom pressed
  p.tick();
  EXPECT_EQ(p.mask, (uint16_t)0);
  EXPECT_EQ(p.safety.state(), SafetyState::FAULT_LOCKOUT);

  // Release the E-stop: still locked out (console still holds arm values).
  p.hw.estop_ok = true;
  p.step(2000);
  EXPECT_EQ(p.safety.state(), SafetyState::FAULT_LOCKOUT);

  // Operator drops the arm values: acknowledged -> SAFE -> re-armable.
  p.ch[0] = 0;
  p.ch[1] = 0;
  p.step(5);
  EXPECT_EQ(p.safety.state(), SafetyState::SAFE);
  p.armFromConsole();
  EXPECT_EQ(p.safety.state(), SafetyState::ARMED);
  EXPECT_TRUE(p.checker.ok());
}

TEST(e2e_dmx_chase_pattern_order) {
  DmxPipeline p;
  p.armFromConsole();
  p.ch[2] = 30;   // CHASE_UP
  p.ch[3] = 40;   // ~104 ms poof
  p.ch[4] = 0;    // min rest
  p.ch[6] = 255;  // repeat
  for (int i = 8; i < 16; ++i) p.ch[i] = 255;  // enable poofers 1-8

  std::vector<uint16_t> steps;
  uint16_t prev = 0;
  for (int t = 0; t < 4000; ++t) {
    p.tick();
    if (p.mask != 0 && p.mask != prev) steps.push_back(p.mask);
    prev = p.mask;
  }
  EXPECT_TRUE(steps.size() >= 8);
  for (int i = 0; i < 8 && i < (int)steps.size(); ++i)
    EXPECT_EQ(steps[i], (uint16_t)(1u << i));
  EXPECT_TRUE(p.checker.ok());
}

TEST(e2e_dmx_disabled_poofer_is_silent_gap) {
  DmxPipeline p;
  p.armFromConsole();
  p.ch[2] = 30;   // CHASE_UP
  p.ch[6] = 255;  // repeat
  for (int i = 8; i < 16; ++i) p.ch[i] = 255;
  p.ch[10] = 0;  // poofer 3 disabled

  bool fired3 = false;
  for (int t = 0; t < 6000; ++t) {
    p.tick();
    if (p.mask & (1u << 2)) fired3 = true;
  }
  EXPECT_FALSE(fired3);
  EXPECT_TRUE(p.checker.ok());
}

TEST(e2e_dmx_arm_key_gates_everything) {
  DmxPipeline p;
  p.hw.arm_key = false;  // keyswitch open
  p.armFromConsole();
  EXPECT_EQ(p.safety.state(), SafetyState::SAFE);  // console alone can't arm
  p.hw.arm_key = true;
  p.armFromConsole();
  EXPECT_EQ(p.safety.state(), SafetyState::ARMED);
  p.hw.arm_key = false;  // key pulled while armed
  p.tick();
  EXPECT_EQ(p.safety.state(), SafetyState::SAFE);
  EXPECT_TRUE(p.checker.ok());
}
