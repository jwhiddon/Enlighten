#include "core/midi_decoder.h"
#include "framework.h"

namespace {
struct MidiHarness {
  MidiDecoder dec;
  TimeMs now = 0;

  void ev(uint8_t status, uint8_t d1, uint8_t d2) {
    dec.onByte(now);
    MidiEvent e{status, d1, d2};
    dec.onEvent(e, now);
  }
  ShowInput snap() { return dec.snapshot(now); }
};
}  // namespace

TEST(midi_note_gates) {
  MidiHarness h;
  h.ev(0x90, 36, 100);  // poofer 1 on
  EXPECT_EQ(h.snap().trigger_mask, (uint16_t)0x0001);
  h.ev(0x90, 51, 127);  // poofer 16 on
  EXPECT_EQ(h.snap().trigger_mask, (uint16_t)0x8001);
  h.ev(0x80, 36, 0);  // note off
  EXPECT_EQ(h.snap().trigger_mask, (uint16_t)0x8000);
  h.ev(0x90, 51, 0);  // note-on velocity 0 == off
  EXPECT_EQ(h.snap().trigger_mask, (uint16_t)0x0000);
}

TEST(midi_low_velocity_ignored) {
  MidiHarness h;
  h.ev(0x90, 36, 63);  // below the velocity gate: no half-triggers
  EXPECT_EQ(h.snap().trigger_mask, (uint16_t)0x0000);
}

TEST(midi_notes_out_of_range_ignored) {
  MidiHarness h;
  h.ev(0x90, 35, 127);
  h.ev(0x90, 52, 127);
  EXPECT_EQ(h.snap().trigger_mask, (uint16_t)0x0000);
}

TEST(midi_wrong_channel_ignored) {
  MidiHarness h;
  h.ev(0x91, 36, 127);  // channel 2
  h.ev(0xB1, 20, 85);
  ShowInput s = h.snap();
  EXPECT_EQ(s.trigger_mask, (uint16_t)0x0000);
  EXPECT_FALSE(s.arm_a);
}

TEST(midi_arm_ccs) {
  MidiHarness h;
  h.ev(0xB0, 20, cfg::MIDI_ARM_A);
  h.ev(0xB0, 21, cfg::MIDI_ARM_B);
  ShowInput s = h.snap();
  EXPECT_TRUE(s.arm_a);
  EXPECT_TRUE(s.arm_b);
  h.ev(0xB0, 20, 0);  // value leaves the magic: disarm
  s = h.snap();
  EXPECT_FALSE(s.arm_a);
  EXPECT_TRUE(s.arm_b);
}

TEST(midi_all_notes_off_clears_gates_and_disarms) {
  MidiHarness h;
  h.ev(0xB0, 20, cfg::MIDI_ARM_A);
  h.ev(0xB0, 21, cfg::MIDI_ARM_B);
  h.ev(0x90, 36, 127);
  h.ev(0x90, 40, 127);
  ShowInput s = h.snap();
  EXPECT_TRUE(s.arm_a && s.arm_b);
  EXPECT_TRUE(s.trigger_mask != 0);
  h.ev(0xB0, 123, 0);  // all notes off
  s = h.snap();
  EXPECT_EQ(s.trigger_mask, (uint16_t)0);
  EXPECT_FALSE(s.arm_a);
  EXPECT_FALSE(s.arm_b);
}

TEST(midi_active_sensing_deadman) {
  MidiHarness h;
  h.now = 1000;
  h.ev(0xFE, 0, 0);  // active sensing observed
  h.now = 1300;
  EXPECT_TRUE(h.snap().link_ok);
  h.now = 1000 + cfg::MIDI_AS_TIMEOUT_MS + 1;
  EXPECT_FALSE(h.snap().link_ok);  // cable pulled
}

TEST(midi_keepalive_without_active_sensing) {
  MidiHarness h;
  EXPECT_FALSE(h.snap().link_ok);  // nothing ever received
  h.now = 500;
  h.ev(0x90, 36, 127);
  h.now = 500 + cfg::MIDI_KEEPALIVE_MS - 1;
  EXPECT_TRUE(h.snap().link_ok);
  h.now = 500 + cfg::MIDI_KEEPALIVE_MS + 1;
  EXPECT_FALSE(h.snap().link_ok);
}

TEST(midi_link_loss_drops_arm) {
  MidiHarness h;
  h.now = 100;
  h.ev(0xB0, 20, cfg::MIDI_ARM_A);
  h.ev(0xB0, 21, cfg::MIDI_ARM_B);
  EXPECT_TRUE(h.snap().arm_a);
  h.now = 100 + cfg::MIDI_KEEPALIVE_MS + 1;
  ShowInput s = h.snap();
  EXPECT_FALSE(s.link_ok);
  EXPECT_FALSE(s.arm_a);
  EXPECT_FALSE(s.arm_b);
}

TEST(midi_mode_cc_scaled_into_bands) {
  MidiHarness h;
  h.ev(0xB0, 22, 125);  // 125*2 = 250 -> RAW band
  ShowInput s;
  for (uint32_t i = 0; i < cfg::MODE_DEBOUNCE_MS + 10; ++i) {
    ++h.now;
    h.dec.onByte(h.now);  // keep link alive
    s = h.snap();
  }
  EXPECT_EQ(s.mode, ModeId::RAW);
}
