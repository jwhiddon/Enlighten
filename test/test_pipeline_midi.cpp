// End-to-end MIDI scenarios: raw serial bytes through parser -> decoder ->
// sequencer -> safety, as wired in Enlighten.ino.
#include "framework.h"
#include "pipeline.h"

TEST(e2e_midi_arm_and_note_fire) {
  MidiPipeline p;
  p.armViaCc();
  EXPECT_EQ(p.safety.state(), SafetyState::ARMED);

  p.feed({0xB0, 22, 125});  // CC22=125 -> mode raw value 250 -> RAW
  p.stepAlive(cfg::MODE_DEBOUNCE_MS + 20);

  p.feed({0x90, 36, 127});  // note C2 -> poofer 1
  p.stepAlive(10);
  EXPECT_EQ(p.mask, (uint16_t)0x0001);

  // Default poof (~100 ms) ends even while the note is held.
  p.stepAlive(200);
  EXPECT_EQ(p.mask, (uint16_t)0);

  p.feed({0x80, 36, 0});  // release
  p.stepAlive(300);        // let the default rest (~200 ms) elapse too
  p.feed({0x90, 36, 127});  // retrigger
  p.stepAlive(10);
  EXPECT_EQ(p.mask, (uint16_t)0x0001);
  EXPECT_TRUE(p.checker.ok());
}

TEST(e2e_midi_all_notes_off_is_panic) {
  MidiPipeline p;
  p.armViaCc();
  p.feed({0xB0, 22, 125});
  p.stepAlive(cfg::MODE_DEBOUNCE_MS + 20);
  p.feed({0x90, 36, 127, 38, 127});  // running status: two notes
  p.stepAlive(10);
  EXPECT_TRUE(p.mask != 0);

  p.feed({0xB0, 123, 0});  // all-notes-off
  p.stepAlive(2);
  EXPECT_EQ(p.mask, (uint16_t)0);
  EXPECT_EQ(p.safety.state(), SafetyState::SAFE);  // it also disarms
  EXPECT_TRUE(p.checker.ok());
}

TEST(e2e_midi_cable_pull_disarms_via_active_sensing) {
  MidiPipeline p;
  p.armViaCc();
  EXPECT_EQ(p.safety.state(), SafetyState::ARMED);
  // Cable pulled: heartbeats stop, only time passes.
  p.step(cfg::MIDI_AS_TIMEOUT_MS + 10);
  EXPECT_EQ(p.safety.state(), SafetyState::SAFE);
  EXPECT_EQ(p.mask, (uint16_t)0);
  // Plugging back in with the CCs still latched must not re-arm: the CC
  // values are decoder state, but link returning doesn't recreate the edge.
  p.stepAlive(10000);
  EXPECT_EQ(p.safety.state(), SafetyState::SAFE);
  // Explicitly cycling the arm values re-arms.
  p.feed({0xB0, 20, 0});
  p.stepAlive(5);
  p.feed({0xB0, 20, cfg::MIDI_ARM_A});
  p.stepAlive(cfg::ARM_HOLD_MS + 10);
  EXPECT_EQ(p.safety.state(), SafetyState::ARMED);
  EXPECT_TRUE(p.checker.ok());
}

TEST(e2e_latched_cc_state_cannot_arm_at_connect) {
  // Controller connects with the arm CCs already in its saved state.
  MidiPipeline p;
  p.feed({0xB0, 20, cfg::MIDI_ARM_A});
  p.feed({0xB0, 21, cfg::MIDI_ARM_B});
  p.stepAlive(5000);
  EXPECT_EQ(p.safety.state(), SafetyState::SAFE);  // rising edge required
  // Cycle one value: arms.
  p.feed({0xB0, 20, 0});
  p.stepAlive(5);
  p.feed({0xB0, 20, cfg::MIDI_ARM_A});
  p.stepAlive(cfg::ARM_HOLD_MS + 10);
  EXPECT_EQ(p.safety.state(), SafetyState::ARMED);
  EXPECT_TRUE(p.checker.ok());
}

TEST(e2e_cc_sweep_through_arm_value_cannot_arm) {
  MidiPipeline p;
  p.stepAlive(5);
  p.feed({0xB0, 21, cfg::MIDI_ARM_B});
  for (uint8_t v = 0; v < 128; ++v) {  // knob sweep passes 85 for ~1 ms
    p.feed({0xB0, 20, v});
    p.stepAlive(1);
  }
  p.stepAlive(cfg::ARM_HOLD_MS + 100);
  EXPECT_EQ(p.safety.state(), SafetyState::SAFE);
  EXPECT_TRUE(p.checker.ok());
}

TEST(e2e_arm_key_gates_everything) {
  MidiPipeline p;
  p.hw.arm_key = false;
  p.armViaCc();
  EXPECT_EQ(p.safety.state(), SafetyState::SAFE);  // CCs alone can't arm
  p.hw.arm_key = true;
  p.feed({0xB0, 20, 0});  // cycle for a fresh edge
  p.stepAlive(5);
  p.feed({0xB0, 20, cfg::MIDI_ARM_A});
  p.stepAlive(cfg::ARM_HOLD_MS + 10);
  EXPECT_EQ(p.safety.state(), SafetyState::ARMED);
  p.hw.arm_key = false;  // key pulled while armed
  p.tick();
  EXPECT_EQ(p.safety.state(), SafetyState::SAFE);
  EXPECT_TRUE(p.checker.ok());
}

TEST(e2e_estop_lockout_and_acknowledge) {
  MidiPipeline p;
  p.armViaCc();
  p.feed({0xB0, 26, 127});  // repeat on
  p.feed({0xB0, 23, 64});   // ~266 ms poof: still open at the check below
  p.holdNotes();
  p.selectMode(115);  // FIRE_ALL band
  p.stepAlive(50);
  EXPECT_TRUE(p.mask != 0);

  p.hw.estop_ok = false;  // mushroom pressed
  p.tick();
  EXPECT_EQ(p.mask, (uint16_t)0);
  EXPECT_EQ(p.safety.state(), SafetyState::FAULT_LOCKOUT);

  p.hw.estop_ok = true;  // release alone re-enables nothing
  p.stepAlive(2000);
  EXPECT_EQ(p.safety.state(), SafetyState::FAULT_LOCKOUT);

  // Acknowledging a lockout requires BOTH arm values down.
  p.feed({0xB0, 20, 0});
  p.feed({0xB0, 21, 0});
  p.stepAlive(5);
  EXPECT_EQ(p.safety.state(), SafetyState::SAFE);
  p.feed({0xB0, 20, cfg::MIDI_ARM_A});
  p.feed({0xB0, 21, cfg::MIDI_ARM_B});
  p.stepAlive(cfg::ARM_HOLD_MS + 10);
  EXPECT_EQ(p.safety.state(), SafetyState::ARMED);
  EXPECT_TRUE(p.checker.ok());
}

TEST(e2e_chase_pattern_order) {
  MidiPipeline p;
  p.armViaCc();
  p.feed({0xB0, 23, 20});   // ~104 ms poof
  p.feed({0xB0, 24, 0});    // min rest
  p.feed({0xB0, 26, 127});  // repeat
  p.holdNotes(1, 8);
  p.selectMode(15);  // CHASE_UP band

  uint16_t prev = 0;
  int idx = 0;
  bool order_ok = true;
  int seen = 0;
  for (int t = 0; t < 4000; ++t) {
    p.feed({0xFE});
    p.tick();
    if (p.mask != 0 && p.mask != prev) {
      if (p.mask != (uint16_t)(1u << (idx % 8))) order_ok = false;
      ++idx;
      ++seen;
    }
    prev = p.mask;
  }
  EXPECT_TRUE(seen >= 8);
  EXPECT_TRUE(order_ok);
  EXPECT_TRUE(p.checker.ok());
}

TEST(e2e_disabled_poofer_is_silent_gap) {
  MidiPipeline p;
  p.armViaCc();
  p.feed({0xB0, 26, 127});
  p.holdNotes(1, 8);
  p.feed({0x80, (uint8_t)(cfg::MIDI_NOTE_FIRST + 2), 0});  // poofer 3 off
  p.selectMode(15);  // CHASE_UP

  bool fired3 = false;
  for (int t = 0; t < 6000; ++t) {
    p.feed({0xFE});
    p.tick();
    if (p.mask & (1u << 2)) fired3 = true;
  }
  EXPECT_FALSE(fired3);
  EXPECT_TRUE(p.checker.ok());
}

TEST(e2e_midi_velocity_gate_end_to_end) {
  MidiPipeline p;
  p.armViaCc();
  p.feed({0xB0, 22, 125});
  p.stepAlive(cfg::MODE_DEBOUNCE_MS + 20);
  p.feed({0x90, 36, 40});  // below the velocity gate
  p.stepAlive(50);
  EXPECT_EQ(p.mask, (uint16_t)0);
  p.feed({0x90, 36, 64});  // at the gate
  p.stepAlive(10);
  EXPECT_EQ(p.mask, (uint16_t)0x0001);
  EXPECT_TRUE(p.checker.ok());
}

TEST(e2e_midi_sysex_storm_does_not_confuse_pipeline) {
  MidiPipeline p;
  p.armViaCc();
  p.feed({0xB0, 22, 125});
  p.stepAlive(cfg::MODE_DEBOUNCE_MS + 20);
  // A SysEx dump interleaved with realtime, followed by a note.
  p.feed({0xF0, 0x7E, 0x00, 0x01, 0x02, 0x03, 0xF8, 0x04, 0xF7});
  p.feed({0x90, 36, 127});
  p.stepAlive(10);
  EXPECT_EQ(p.mask, (uint16_t)0x0001);
  EXPECT_TRUE(p.checker.ok());
}
