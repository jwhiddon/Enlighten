#include "panel_ui.h"

// ---- ShowStats ----------------------------------------------------------------

void ShowStats::update(uint16_t mask, TimeMs now) {
  if (has_last_) {
    uint32_t dt = elapsedMs(last_, now);
    uint16_t m = mask;
    uint8_t open = 0;
    while (m) {
      open = (uint8_t)(open + (m & 1u));
      m >>= 1;
    }
    flame_ms += (uint32_t)open * dt;
  }
  uint16_t rising = (uint16_t)(mask & ~prev_mask_);
  for (uint8_t i = 0; i < cfg::NUM_POOFERS; ++i)
    if ((rising >> i) & 1u && fires[i] != 0xFFFF) ++fires[i];
  prev_mask_ = mask;
  last_ = now;
  has_last_ = true;
}

void ShowStats::reset() {
  for (auto& f : fires) f = 0;
  flame_ms = 0;
  prev_mask_ = 0;
}

// ---- PanelUi -------------------------------------------------------------------

void PanelUi::onDispPress(TimeMs now) {
  last_press_ = now;
  switch (page_) {
    case DisplayPage::STATUS:      page_ = DisplayPage::STATS; break;
    case DisplayPage::STATS:       page_ = DisplayPage::DIAG; break;
    case DisplayPage::DIAG:        page_ = DisplayPage::MODE_SELECT; break;
    case DisplayPage::MODE_SELECT: page_ = DisplayPage::STATUS; break;
  }
}

void PanelUi::onSelPress(TimeMs now) {
  last_press_ = now;
  switch (page_) {
    case DisplayPage::MODE_SELECT:
      mode_idx_ = (uint8_t)((mode_idx_ + 1) % 13);  // AUTO + 12 modes
      break;
    case DisplayPage::STATS:
      stats_reset_ = true;
      break;
    default:
      break;
  }
}

DisplayPage PanelUi::page(TimeMs now) {
  if (page_ != DisplayPage::STATUS &&
      elapsedMs(last_press_, now) >= cfg::DISPLAY_PAGE_TIMEOUT_MS)
    page_ = DisplayPage::STATUS;
  return page_;
}

bool PanelUi::modeOverride(ModeId* out) const {
  if (mode_idx_ == 0) return false;
  *out = (ModeId)(mode_idx_ - 1);
  return true;
}

bool PanelUi::consumeStatsReset() {
  bool r = stats_reset_;
  stats_reset_ = false;
  return r;
}

// ---- Aux page rendering ---------------------------------------------------------

void renderAuxPage(char* out, DisplayPage page, const PanelUi& ui,
                   const ShowStats& stats, const AuxPageInfo& info) {
  char* l1 = out;
  char* l2 = out + DISPLAY_COLS;
  char* l3 = out + 2 * DISPLAY_COLS;
  char* l4 = out + 3 * DISPLAY_COLS;

  switch (page) {
    case DisplayPage::STATS: {
      dispPut(l1, "STATS");
      dispOverlayRight(l1, "SEL=RESET");

      uint32_t total = 0;
      uint8_t busiest = 0;
      for (uint8_t i = 0; i < cfg::NUM_POOFERS; ++i) {
        total += stats.fires[i];
        if (stats.fires[i] > stats.fires[busiest]) busiest = i;
      }
      dispPut(l2, "FIRES");
      dispNumTo(l2 + 7, total, 6);
      dispPut(l3, "FLAME         s");
      dispNumTo(l3 + 7, stats.flame_ms / 1000, 6);
      dispPut(l4, "BUSIEST P   x");
      dispNumTo(l4 + 9, (uint32_t)busiest + 1, 2);
      dispNumTo(l4 + 14, stats.fires[busiest], 5);
      break;
    }

    case DisplayPage::DIAG: {
      dispPut(l1, "DIAG");
      dispOverlayRight(l1, info.bench ? "[USB]" : "[MIDI]");
      // Uptime HHH:MM:SS
      uint32_t s = info.uptime_ms / 1000;
      dispPut(l2, "UP    :  :");
      dispNumTo(l2 + 3, s / 3600, 3);
      char mm = (char)('0' + (s / 60) % 60 / 10);
      l2[7] = mm;
      l2[8] = (char)('0' + (s / 60) % 10);
      l2[10] = (char)('0' + (s % 60) / 10);
      l2[11] = (char)('0' + s % 10);

      char line[DISPLAY_COLS + 1];
      dispPut(line, "FLT ");
      line[DISPLAY_COLS] = 0;
      const char* fn = dispFaultName(info.last_fault);
      for (uint8_t i = 0; fn[i] && 4 + i < DISPLAY_COLS; ++i) line[4 + i] = fn[i];
      dispPut(l3, line);

      dispPut(l4, info.sd_ok ? "SD OK" : "SD --");
      dispOverlayRight(l4, info.link_ok ? "LINK OK" : "LINK --");
      break;
    }

    case DisplayPage::MODE_SELECT: {
      dispPut(l1, "MODE SELECT");
      char line[DISPLAY_COLS + 1];
      dispPut(line, "ACTIVE: ");
      line[DISPLAY_COLS] = 0;
      const char* an = dispModeName(info.active_mode);
      for (uint8_t i = 0; an[i] && 8 + i < DISPLAY_COLS; ++i) line[8 + i] = an[i];
      dispPut(l2, line);

      dispPut(line, "PANEL:  ");
      ModeId ov;
      const char* pn = ui.modeOverride(&ov) ? dispModeName(ov) : "AUTO";
      for (uint8_t i = 0; pn[i] && 8 + i < DISPLAY_COLS; ++i) line[8 + i] = pn[i];
      dispPut(l3, line);

      dispPut(l4, "SEL BUTTON = CHANGE");
      break;
    }

    case DisplayPage::STATUS:
    default:
      // STATUS is rendered by renderDisplay(); blank defensively.
      dispPut(l1, "");
      dispPut(l2, "");
      dispPut(l3, "");
      dispPut(l4, "");
      break;
  }

  out[DISPLAY_CHARS] = 0;
}
