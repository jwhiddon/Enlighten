#include <cstring>

#include "core/dmx_decoder.h"
#include "framework.h"

namespace {
struct DmxHarness {
  DmxDecoder dec;
  uint8_t ch[DmxDecoder::NUM_CHANNELS] = {};
  TimeMs now = 0;

  ShowInput decode(uint32_t age = 0) { return dec.decode(ch, age, ++now); }
  ShowInput settle(uint32_t ms, uint32_t age = 0) {
    ShowInput out;
    for (uint32_t i = 0; i < ms; ++i) out = decode(age);
    return out;
  }
};
}  // namespace

TEST(dmx_link_timeout) {
  DmxHarness h;
  EXPECT_TRUE(h.decode(0).link_ok);
  EXPECT_TRUE(h.decode(cfg::DMX_TIMEOUT_MS - 1).link_ok);
  EXPECT_FALSE(h.decode(cfg::DMX_TIMEOUT_MS).link_ok);
}

TEST(dmx_arm_windows) {
  DmxHarness h;
  h.ch[0] = 85;
  h.ch[1] = 170;
  ShowInput s = h.decode();
  EXPECT_TRUE(s.arm_a);
  EXPECT_TRUE(s.arm_b);
  h.ch[0] = 80;  // low edge of the +/-5 window
  h.ch[1] = 175; // high edge
  s = h.decode();
  EXPECT_TRUE(s.arm_a);
  EXPECT_TRUE(s.arm_b);
  h.ch[0] = 79;
  h.ch[1] = 176;
  s = h.decode();
  EXPECT_FALSE(s.arm_a);
  EXPECT_FALSE(s.arm_b);
  // Blackout (0) and full (255) must never look armed.
  h.ch[0] = 0;
  h.ch[1] = 255;
  s = h.decode();
  EXPECT_FALSE(s.arm_a);
  EXPECT_FALSE(s.arm_b);
}

TEST(dmx_stale_buffer_cannot_arm) {
  DmxHarness h;
  h.ch[0] = 85;
  h.ch[1] = 170;
  ShowInput s = h.decode(cfg::DMX_TIMEOUT_MS + 1);  // link down
  EXPECT_FALSE(s.link_ok);
  EXPECT_FALSE(s.arm_a);
  EXPECT_FALSE(s.arm_b);
}

TEST(dmx_trigger_thresholds_and_dead_zone) {
  DmxHarness h;
  h.ch[8] = 255;  // poofer 1 on
  ShowInput s = h.decode();
  EXPECT_EQ(s.trigger_mask, (uint16_t)0x0001);
  h.ch[8] = 150;  // dead zone: hold ON
  s = h.decode();
  EXPECT_EQ(s.trigger_mask, (uint16_t)0x0001);
  h.ch[8] = 99;  // off
  s = h.decode();
  EXPECT_EQ(s.trigger_mask, (uint16_t)0x0000);
  h.ch[8] = 150;  // dead zone: hold OFF
  s = h.decode();
  EXPECT_EQ(s.trigger_mask, (uint16_t)0x0000);
  h.ch[8] = 200;  // exact on threshold
  h.ch[23] = 200; // poofer 16
  s = h.decode();
  EXPECT_EQ(s.trigger_mask, (uint16_t)0x8001);
}

TEST(dmx_timing_map_endpoints) {
  DmxHarness h;
  ShowInput s = h.decode();
  EXPECT_EQ(s.poof_ms, cfg::MIN_POOF_MS);
  EXPECT_EQ(s.rest_ms, cfg::MIN_REST_MS);
  h.ch[3] = 255;
  h.ch[4] = 255;
  s = h.decode();
  EXPECT_EQ(s.poof_ms, cfg::MAX_POOF_MS);
  EXPECT_EQ(s.rest_ms, cfg::MAX_REST_MS);
}

TEST(dmx_rate_and_repeat) {
  DmxHarness h;
  ShowInput s = h.decode();
  EXPECT_EQ(s.rate, (uint8_t)128);  // unpatched channel = 1.0x
  EXPECT_FALSE(s.repeat);
  h.ch[5] = 200;
  h.ch[6] = 128;
  s = h.decode();
  EXPECT_EQ(s.rate, (uint8_t)200);
  EXPECT_TRUE(s.repeat);
}

TEST(dmx_mode_through_debounce) {
  DmxHarness h;
  h.ch[2] = 250;  // RAW band
  ShowInput s = h.settle(cfg::MODE_DEBOUNCE_MS + 10);
  EXPECT_EQ(s.mode, ModeId::RAW);
  // Link loss holds the mode rather than resetting it.
  s = h.decode(cfg::DMX_TIMEOUT_MS + 1);
  EXPECT_EQ(s.mode, ModeId::RAW);
}
