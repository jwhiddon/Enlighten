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
