#include <vector>

#include "core/midi_parser.h"
#include "framework.h"

namespace {
std::vector<MidiEvent> feedAll(MidiParser& p, std::initializer_list<uint8_t> bytes) {
  std::vector<MidiEvent> out;
  MidiEvent ev;
  for (uint8_t b : bytes)
    if (p.feed(b, &ev)) out.push_back(ev);
  return out;
}
}  // namespace

TEST(midi_note_on_basic) {
  MidiParser p;
  auto evs = feedAll(p, {0x90, 36, 100});
  EXPECT_EQ(evs.size(), (size_t)1);
  EXPECT_EQ(evs[0].status, (uint8_t)0x90);
  EXPECT_EQ(evs[0].data1, (uint8_t)36);
  EXPECT_EQ(evs[0].data2, (uint8_t)100);
}

TEST(midi_running_status) {
  MidiParser p;
  auto evs = feedAll(p, {0x90, 36, 100, 37, 101, 38, 102});
  EXPECT_EQ(evs.size(), (size_t)3);
  EXPECT_EQ(evs[1].status, (uint8_t)0x90);
  EXPECT_EQ(evs[1].data1, (uint8_t)37);
  EXPECT_EQ(evs[2].data1, (uint8_t)38);
}

TEST(midi_realtime_interleaved_mid_message) {
  MidiParser p;
  // Clock (0xF8) arrives between the status and data bytes of a note-on.
  auto evs = feedAll(p, {0x90, 0xF8, 36, 0xFE, 100});
  EXPECT_EQ(evs.size(), (size_t)3);
  EXPECT_EQ(evs[0].status, (uint8_t)0xF8);
  EXPECT_EQ(evs[1].status, (uint8_t)0xFE);
  EXPECT_EQ(evs[2].status, (uint8_t)0x90);  // note assembled undisturbed
  EXPECT_EQ(evs[2].data1, (uint8_t)36);
  EXPECT_EQ(evs[2].data2, (uint8_t)100);
}

TEST(midi_sysex_skipped) {
  MidiParser p;
  auto evs = feedAll(p, {0xF0, 0x7E, 0x01, 0x02, 0xF7, 0x90, 40, 90});
  EXPECT_EQ(evs.size(), (size_t)1);
  EXPECT_EQ(evs[0].data1, (uint8_t)40);
}

TEST(midi_one_byte_messages) {
  MidiParser p;
  auto evs = feedAll(p, {0xC0, 5, 0xD0, 60});  // program change, ch pressure
  EXPECT_EQ(evs.size(), (size_t)2);
  EXPECT_EQ(evs[0].status, (uint8_t)0xC0);
  EXPECT_EQ(evs[0].data1, (uint8_t)5);
  EXPECT_EQ(evs[1].status, (uint8_t)0xD0);
}

TEST(midi_orphan_data_ignored) {
  MidiParser p;
  auto evs = feedAll(p, {36, 100, 55});
  EXPECT_EQ(evs.size(), (size_t)0);
}

TEST(midi_system_common_cancels_running_status) {
  MidiParser p;
  // 0xF3 (song select, 1 data byte we don't assemble) cancels running status;
  // following data bytes must not produce events.
  auto evs = feedAll(p, {0x90, 36, 100, 0xF3, 5, 37, 100});
  EXPECT_EQ(evs.size(), (size_t)1);
}

TEST(midi_cc_message) {
  MidiParser p;
  auto evs = feedAll(p, {0xB0, 20, 85});
  EXPECT_EQ(evs.size(), (size_t)1);
  EXPECT_EQ(evs[0].status, (uint8_t)0xB0);
  EXPECT_EQ(evs[0].data1, (uint8_t)20);
  EXPECT_EQ(evs[0].data2, (uint8_t)85);
}
