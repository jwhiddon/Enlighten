#include "core/channel_leds.h"
#include "framework.h"

namespace {
struct Leds {
  ChannelLedColor c[cfg::NUM_POOFERS];
  void render(SafetyState st, uint16_t enabled, uint16_t firing,
              uint16_t requested, uint16_t playback = 0) {
    channelLedColors(c, st, enabled, firing, requested, playback);
  }
};
}  // namespace

TEST(leds_dark_unless_armed) {
  Leds l;
  SafetyState states[] = {SafetyState::BOOT_SELFTEST, SafetyState::SAFE,
                          SafetyState::ARM_PENDING, SafetyState::FAULT_LOCKOUT};
  for (auto st : states) {
    l.render(st, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF);
    for (int i = 0; i < cfg::NUM_POOFERS; ++i)
      EXPECT_EQ((int)l.c[i], (int)ChannelLedColor::OFF);
  }
}

TEST(leds_live_fire_is_red_playback_fire_is_blue) {
  Leds l;
  // ch1 firing live, ch2 firing from the SD player.
  l.render(SafetyState::ARMED, 0x0003, /*firing=*/0x0003,
           /*requested=*/0x0003, /*playback=*/0x0002);
  EXPECT_EQ((int)l.c[0], (int)ChannelLedColor::RED);
  EXPECT_EQ((int)l.c[1], (int)ChannelLedColor::BLUE);
}

TEST(leds_throttled_is_amber) {
  Leds l;
  // ch3 requested but the filter is holding it closed (duty budget).
  l.render(SafetyState::ARMED, 0x0004, /*firing=*/0x0000,
           /*requested=*/0x0004);
  EXPECT_EQ((int)l.c[2], (int)ChannelLedColor::AMBER);
}

TEST(leds_enabled_is_green_idle_is_off) {
  Leds l;
  l.render(SafetyState::ARMED, /*enabled=*/0x0001, 0, 0);
  EXPECT_EQ((int)l.c[0], (int)ChannelLedColor::GREEN);
  EXPECT_EQ((int)l.c[1], (int)ChannelLedColor::OFF);
}

TEST(leds_priority_firing_over_throttle_over_enable) {
  Leds l;
  l.render(SafetyState::ARMED, /*enabled=*/0x0001, /*firing=*/0x0001,
           /*requested=*/0x0001);
  EXPECT_EQ((int)l.c[0], (int)ChannelLedColor::RED);
  l.render(SafetyState::ARMED, 0x0001, 0x0000, 0x0001);
  EXPECT_EQ((int)l.c[0], (int)ChannelLedColor::AMBER);
}
