#include <cstring>

#include "core/panel_ui.h"
#include "framework.h"

namespace {
bool lineIs(const char* buf, int r, const char* s) {
  return std::memcmp(buf + r * DISPLAY_COLS, s, DISPLAY_COLS) == 0;
}
}  // namespace

TEST(stats_fires_and_flame_time) {
  ShowStats s;
  TimeMs now = 0;
  s.update(0x0000, now);
  for (int i = 0; i < 100; ++i) s.update(0x0003, ++now);  // 2 open, 100 ms
  for (int i = 0; i < 50; ++i) s.update(0x0000, ++now);
  for (int i = 0; i < 100; ++i) s.update(0x0001, ++now);  // retrigger ch1
  EXPECT_EQ(s.fires[0], (uint16_t)2);
  EXPECT_EQ(s.fires[1], (uint16_t)1);
  EXPECT_EQ(s.flame_ms, (uint32_t)(2 * 100 + 100));
  s.reset();
  EXPECT_EQ(s.fires[0], (uint16_t)0);
  EXPECT_EQ(s.flame_ms, (uint32_t)0);
}

TEST(pager_cycles_and_wraps) {
  PanelUi ui;
  TimeMs now = 0;
  EXPECT_EQ(ui.page(now), DisplayPage::STATUS);
  ui.onDispPress(++now);
  EXPECT_EQ(ui.page(now), DisplayPage::STATS);
  ui.onDispPress(++now);
  EXPECT_EQ(ui.page(now), DisplayPage::DIAG);
  ui.onDispPress(++now);
  EXPECT_EQ(ui.page(now), DisplayPage::MODE_SELECT);
  ui.onDispPress(++now);
  EXPECT_EQ(ui.page(now), DisplayPage::STATUS);
}

TEST(pager_auto_returns_to_status) {
  PanelUi ui;
  TimeMs now = 1000;
  ui.onDispPress(now);
  EXPECT_EQ(ui.page(now + 100), DisplayPage::STATS);
  EXPECT_EQ(ui.page(now + cfg::DISPLAY_PAGE_TIMEOUT_MS - 1), DisplayPage::STATS);
  EXPECT_EQ(ui.page(now + cfg::DISPLAY_PAGE_TIMEOUT_MS), DisplayPage::STATUS);
  // SEL presses also count as activity.
  ui.onDispPress(now + 60000);
  ui.onSelPress(now + 80000);
  EXPECT_EQ(ui.page(now + 80000 + 5000), DisplayPage::STATS);
}

TEST(sel_cycles_panel_mode_with_auto) {
  PanelUi ui;
  TimeMs now = 0;
  ModeId m;
  EXPECT_FALSE(ui.modeOverride(&m));  // boots in AUTO
  // Navigate to the MODE page; SEL cycles through every mode then AUTO.
  ui.onDispPress(++now);
  ui.onDispPress(++now);
  ui.onDispPress(++now);
  EXPECT_EQ(ui.page(now), DisplayPage::MODE_SELECT);
  ui.onSelPress(++now);
  EXPECT_TRUE(ui.modeOverride(&m));
  EXPECT_EQ(m, ModeId::OFF);
  for (int i = 0; i < 11; ++i) ui.onSelPress(++now);
  EXPECT_TRUE(ui.modeOverride(&m));
  EXPECT_EQ(m, ModeId::CHASE_OUT_IN);
  ui.onSelPress(++now);  // wraps back to AUTO
  EXPECT_FALSE(ui.modeOverride(&m));
}

TEST(sel_on_stats_page_requests_reset_once) {
  PanelUi ui;
  TimeMs now = 0;
  ui.onDispPress(++now);  // STATS
  EXPECT_FALSE(ui.consumeStatsReset());
  ui.onSelPress(++now);
  EXPECT_TRUE(ui.consumeStatsReset());
  EXPECT_FALSE(ui.consumeStatsReset());  // one-shot
  // SEL elsewhere does not request a reset.
  ui.onDispPress(++now);  // DIAG
  ui.onSelPress(++now);
  EXPECT_FALSE(ui.consumeStatsReset());
}

TEST(stats_page_render) {
  PanelUi ui;
  ShowStats s;
  TimeMs now = 0;
  s.update(0, now);
  for (int hit = 0; hit < 18; ++hit) {
    for (int i = 0; i < 100; ++i) s.update(0x0040, ++now);  // poofer 7
    for (int i = 0; i < 100; ++i) s.update(0x0000, ++now);
  }
  for (int i = 0; i < 200; ++i) s.update(0x0001, ++now);  // poofer 1 once

  char buf[DISPLAY_CHARS + 1];
  AuxPageInfo info;
  renderAuxPage(buf, DisplayPage::STATS, ui, s, info);
  EXPECT_EQ(std::strlen(buf), (size_t)DISPLAY_CHARS);
  EXPECT_TRUE(lineIs(buf, 0, "STATS      SEL=RESET"));
  EXPECT_TRUE(lineIs(buf, 1, "FIRES      19       "));
  EXPECT_TRUE(lineIs(buf, 2, "FLAME       2 s     "));  // 2000 ms = 2 s
  EXPECT_TRUE(lineIs(buf, 3, "BUSIEST P 7 x    18 "));
}

TEST(diag_page_render) {
  PanelUi ui;
  ShowStats s;
  char buf[DISPLAY_CHARS + 1];
  AuxPageInfo info;
  info.last_fault = FaultCode::WATCHDOG_RESET;
  info.sd_ok = true;
  info.link_ok = false;
  info.uptime_ms = (1u * 3600 + 23u * 60 + 45u) * 1000u;  // 1:23:45
  renderAuxPage(buf, DisplayPage::DIAG, ui, s, info);
  EXPECT_TRUE(lineIs(buf, 0, "DIAG          [MIDI]"));
  EXPECT_TRUE(lineIs(buf, 1, "UP   1:23:45        "));
  EXPECT_TRUE(lineIs(buf, 2, "FLT WATCHDOG RESET  "));
  EXPECT_TRUE(lineIs(buf, 3, "SD OK        LINK --"));
}

TEST(mode_page_render) {
  PanelUi ui;
  ShowStats s;
  char buf[DISPLAY_CHARS + 1];
  AuxPageInfo info;
  info.active_mode = ModeId::CHASE_UP;
  renderAuxPage(buf, DisplayPage::MODE_SELECT, ui, s, info);
  EXPECT_TRUE(lineIs(buf, 0, "MODE SELECT         "));
  EXPECT_TRUE(lineIs(buf, 1, "ACTIVE: CHASE UP    "));
  EXPECT_TRUE(lineIs(buf, 2, "PANEL:  AUTO        "));
  EXPECT_TRUE(lineIs(buf, 3, "SEL BUTTON = CHANGE "));

  // After one SEL on the mode page: override shows OFF.
  TimeMs now = 0;
  ui.onDispPress(++now);
  ui.onDispPress(++now);
  ui.onDispPress(++now);
  ui.onSelPress(++now);
  renderAuxPage(buf, DisplayPage::MODE_SELECT, ui, s, info);
  EXPECT_TRUE(lineIs(buf, 2, "PANEL:  OFF         "));
}
