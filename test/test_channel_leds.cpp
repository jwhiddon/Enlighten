#include "core/channel_leds.h"
#include "framework.h"

TEST(leds_dark_unless_armed) {
  for (TimeMs t = 0; t < 32; ++t) {
    EXPECT_EQ(channelLedMask(SafetyState::SAFE, 0xFFFF, 0xFFFF, t), (uint16_t)0);
    EXPECT_EQ(channelLedMask(SafetyState::FAULT_LOCKOUT, 0xFFFF, 0xFFFF, t),
              (uint16_t)0);
    EXPECT_EQ(channelLedMask(SafetyState::ARM_PENDING, 0xFFFF, 0xFFFF, t),
              (uint16_t)0);
    EXPECT_EQ(channelLedMask(SafetyState::BOOT_SELFTEST, 0xFFFF, 0xFFFF, t),
              (uint16_t)0);
  }
}

TEST(leds_firing_is_solid) {
  // A firing channel is lit at EVERY tick — solid, not dimmed.
  for (TimeMs t = 0; t < 64; ++t) {
    uint16_t m = channelLedMask(SafetyState::ARMED, 0x0000, 0x0005, t);
    EXPECT_EQ(m & 0x0005, (uint16_t)0x0005);
  }
}

TEST(leds_armed_enabled_is_dim_duty) {
  // An enabled-but-idle channel is lit exactly 1 tick in 8.
  uint32_t lit = 0;
  for (TimeMs t = 0; t < 800; ++t)
    if (channelLedMask(SafetyState::ARMED, 0x0002, 0x0000, t) & 0x0002) ++lit;
  EXPECT_EQ(lit, (uint32_t)100);  // 12.5% of 800
}

TEST(leds_mixed_states_per_channel) {
  // ch1 firing (solid), ch2 enabled (dim), ch3 neither (off).
  uint32_t lit1 = 0, lit2 = 0, lit3 = 0;
  for (TimeMs t = 0; t < 160; ++t) {
    uint16_t m = channelLedMask(SafetyState::ARMED, 0x0003, 0x0001, t);
    if (m & 0x0001) ++lit1;
    if (m & 0x0002) ++lit2;
    if (m & 0x0004) ++lit3;
  }
  EXPECT_EQ(lit1, (uint32_t)160);  // solid
  EXPECT_EQ(lit2, (uint32_t)20);   // dim
  EXPECT_EQ(lit3, (uint32_t)0);    // off
}
