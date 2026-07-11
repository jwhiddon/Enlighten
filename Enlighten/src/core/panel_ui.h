// Operator panel UI: the DISP button cycles the display through info
// pages; the SEL button is context-sensitive input.
//
//   Page        SEL action
//   STATUS      -
//   STATS       reset the show statistics
//   DIAG        -
//   MODE_SELECT cycle the panel mode override (AUTO -> OFF -> ... -> OUT/IN)
//
// Aux pages auto-return to STATUS after DISPLAY_PAGE_TIMEOUT_MS of button
// inactivity — the safety status screen is never far away.
//
// The mode override is choreography-only input: it swaps ShowInput.mode
// before the sequencer and touches nothing in the safety chain.
#pragma once
#include "config.h"
#include "display_text.h"
#include "hw_inputs.h"
#include "show_input.h"
#include "timebase.h"

enum class DisplayPage : uint8_t { STATUS = 0, STATS, DIAG, MODE_SELECT };

// Running show statistics, fed the FILTERED output mask every loop.
struct ShowStats {
  uint16_t fires[cfg::NUM_POOFERS] = {};
  uint32_t flame_ms = 0;

  void update(uint16_t mask, TimeMs now);
  void reset();

 private:
  uint16_t prev_mask_ = 0;
  TimeMs last_ = 0;
  bool has_last_ = false;
};

class PanelUi {
 public:
  void onDispPress(TimeMs now);  // cycle pages
  void onSelPress(TimeMs now);   // context action for the current page

  // Current page, applying the idle auto-return to STATUS.
  DisplayPage page(TimeMs now);

  // True if the operator selected a panel mode (writes it to *out);
  // false = AUTO (follow the console/decoder mode).
  bool modeOverride(ModeId* out) const;

  // One-shot: SEL was pressed on the STATS page since the last call.
  bool consumeStatsReset();

 private:
  DisplayPage page_ = DisplayPage::STATUS;
  TimeMs last_press_ = 0;
  uint8_t mode_idx_ = 0;  // 0 = AUTO, 1..12 = ModeId 0..11
  bool stats_reset_ = false;
};

struct AuxPageInfo {
  FaultCode last_fault = FaultCode::NONE;
  bool bench = false;
  bool sd_ok = false;
  bool link_ok = false;
  ModeId active_mode = ModeId::OFF;
  TimeMs uptime_ms = 0;
};

// Renders the STATS / DIAG / MODE_SELECT pages (STATUS is renderDisplay).
void renderAuxPage(char* out, DisplayPage page, const PanelUi& ui,
                   const ShowStats& stats, const AuxPageInfo& info);
