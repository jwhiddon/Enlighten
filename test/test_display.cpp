#include <cstring>

#include "core/display_model.h"
#include "framework.h"

namespace {
struct Screen {
  char buf[DISPLAY_CHARS + 1];
  ShowInput in;

  Screen() {
    in.link_ok = true;
    in.poof_ms = 150;
    in.rest_ms = 200;
    in.rate = 128;
  }

  void render(SafetyState st, FaultCode f = FaultCode::NONE,
              Protocol p = Protocol::MIDI, uint16_t mask = 0, bool key = true,
              TimeMs now = 0, bool bench = false,
              const char* play = nullptr, uint32_t pos = 0) {
    renderDisplay(buf, st, f, p, in, mask, key, now, bench, play, pos);
  }
  bool line(int r, const char* s) {
    return std::memcmp(buf + r * DISPLAY_COLS, s, DISPLAY_COLS) == 0;
  }
};
}  // namespace

TEST(display_always_80_printable_chars) {
  Screen s;
  SafetyState states[] = {SafetyState::BOOT_SELFTEST, SafetyState::SAFE,
                          SafetyState::ARM_PENDING, SafetyState::ARMED,
                          SafetyState::FAULT_LOCKOUT};
  for (auto st : states) {
    for (int f = 0; f <= 9; ++f) {
      s.in.mode = ModeId::CHASE_DOWN;
      s.render(st, (FaultCode)f, Protocol::DMX, 0xA5A5, false, 12345, false,
               "LONGNAME.SHW", 3600000);
      EXPECT_EQ(std::strlen(s.buf), (size_t)DISPLAY_CHARS);
      for (int i = 0; i < DISPLAY_CHARS; ++i)
        EXPECT_TRUE(s.buf[i] >= 0x20 && s.buf[i] < 0x7F);
      if (testfw::failures()) return;
    }
  }
}

TEST(display_boot) {
  Screen s;
  s.render(SafetyState::BOOT_SELFTEST);
  EXPECT_TRUE(s.line(0, "ENLIGHTEN v2        "));
  EXPECT_TRUE(s.line(1, "SELF-TEST...        "));
  s.render(SafetyState::BOOT_SELFTEST, FaultCode::NONE, Protocol::MIDI, 0,
           true, 0, /*bench=*/true);
  EXPECT_TRUE(s.line(0, "ENLIGHTEN BENCH     "));
}

TEST(display_safe_shows_what_is_missing) {
  Screen s;
  s.in.mode = ModeId::CHASE_UP;

  s.in.link_ok = false;
  s.render(SafetyState::SAFE, FaultCode::NONE, Protocol::MIDI);
  EXPECT_TRUE(s.line(0, "SAFE          [MIDI]"));
  EXPECT_TRUE(s.line(1, "NO SIGNAL           "));

  s.in.link_ok = true;
  s.render(SafetyState::SAFE, FaultCode::NONE, Protocol::MIDI, 0,
           /*key=*/false);
  EXPECT_TRUE(s.line(1, "TURN KEY TO ARM     "));

  s.render(SafetyState::SAFE, FaultCode::NONE, Protocol::MIDI);
  EXPECT_TRUE(s.line(1, "ARM: CC20+CC21      "));
  EXPECT_TRUE(s.line(2, "MODE: CHASE UP      "));

  s.render(SafetyState::SAFE, FaultCode::NONE, Protocol::DMX);
  EXPECT_TRUE(s.line(0, "SAFE           [DMX]"));
  EXPECT_TRUE(s.line(1, "ARM: SEND 85+170    "));
}

TEST(display_bench_screens) {
  Screen s;
  s.render(SafetyState::SAFE, FaultCode::NONE, Protocol::MIDI, 0, true, 0,
           /*bench=*/true);
  EXPECT_TRUE(s.line(0, "BENCH SAFE     [USB]"));
  EXPECT_TRUE(s.line(1, "TYPE: arm           "));
}

TEST(display_armed_mode_bar_and_timing) {
  Screen s;
  s.in.mode = ModeId::CHASE_DOWN;
  s.in.poof_ms = 500;
  s.in.rest_ms = 2000;
  s.in.repeat = true;
  s.in.rate = 128;
  s.render(SafetyState::ARMED, FaultCode::NONE, Protocol::MIDI, 0x8001);
  EXPECT_TRUE(s.line(0, "ARMED     CHASE DOWN"));
  EXPECT_TRUE(s.line(1, "  [#..............#]"));
  EXPECT_TRUE(s.line(2, "POOF  500  REST 2000"));
  EXPECT_TRUE(s.line(3, "REPEAT ON   RATE 128"));

  s.in.repeat = false;
  s.in.rate = 5;
  s.render(SafetyState::ARMED, FaultCode::NONE, Protocol::MIDI, 0);
  EXPECT_TRUE(s.line(1, "  [................]"));
  EXPECT_TRUE(s.line(3, "REPEAT OFF  RATE   5"));
}

TEST(display_playback_line) {
  Screen s;
  s.in.mode = ModeId::RAW;
  s.render(SafetyState::ARMED, FaultCode::NONE, Protocol::MIDI, 0x0001, true,
           100, false, "OPENER.SHW", 83000);
  EXPECT_TRUE(s.line(0, "ARMED            RAW"));
  EXPECT_TRUE(s.line(1, "  [#...............]"));
  EXPECT_TRUE(s.line(3, "> 01:23 OPENER.SHW  "));
}

TEST(display_lockout_full_instructions) {
  Screen s;
  s.render(SafetyState::FAULT_LOCKOUT, FaultCode::ESTOP_ASSERTED);
  EXPECT_TRUE(s.line(0, "FAULT LOCKOUT      6"));
  EXPECT_TRUE(s.line(1, "ESTOP PRESSED       "));
  EXPECT_TRUE(s.line(2, "CLEAR: LINK UP +    "));
  EXPECT_TRUE(s.line(3, "ARM VALUES DOWN     "));

  s.render(SafetyState::FAULT_LOCKOUT, FaultCode::WATCHDOG_RESET);
  EXPECT_TRUE(s.line(0, "FAULT LOCKOUT      1"));
  EXPECT_TRUE(s.line(1, "WATCHDOG RESET      "));
}
