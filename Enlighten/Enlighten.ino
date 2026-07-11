// Enlighten — NFPA 160-aligned flame-effect (poofer) controller.
//
// This file is deliberately thin glue.  All logic lives in src/core/
// (portable, host-unit-tested; see test/) and src/boards/ (hardware).
//
// Pipeline, once per loop:
//   input bytes -> decoder -> ShowInput -> Sequencer -> requested mask
//   requested mask + hardware inputs -> SafetySupervisor::filter()  <- FINAL authority
//   filtered mask -> Board::writeOutputs()                          <- ONLY output call
//
// Safety rules embodied here:
//   * Board::writeOutputs() is called from exactly ONE place, with the
//     SafetySupervisor's return value.  Never bypass it.
//   * The watchdog is fed ONLY when the safety filter actually ran.
//
// Docs: docs/SAFETY.md, docs/DMX_MAP.md, docs/MIDI_MAP.md, docs/HARDWARE.md

#include "src/board.h"
#include "src/core/bench_console.h"
#include "src/core/channel_leds.h"
#include "src/core/config.h"
#include "src/core/display_model.h"
#include "src/core/dmx_decoder.h"
#include "src/core/midi_decoder.h"
#include "src/core/midi_parser.h"
#include "src/core/panel_ui.h"
#include "src/core/safety.h"
#include "src/core/sequencer.h"
#include "src/core/show_input.h"
#include "src/core/show_player.h"
#include "src/core/status_led.h"

static SafetySupervisor g_safety;
static Sequencer g_sequencer;
static DmxDecoder g_dmx;
static MidiParser g_midi_parser;
static MidiDecoder g_midi;
static BenchConsole g_bench_console;
static Protocol g_protocol = Protocol::MIDI;
static bool g_bench = false;  // D5 jumper at boot: USB console, no show HW

// SD-card standalone playback (PLAY button cycles/stops *.SHW files).
static ShowPlayer g_player;
static bool g_sd_ok = false;
static char g_show_name[16] = {};

// Panel UI: DISP cycles display pages, SEL is page-context input.
static PanelUi g_ui;
static ShowStats g_stats;

// Signal the selected protocol on the LED before entering service:
// 1 blink = DMX, 2 blinks = MIDI.  Runs before the watchdog is enabled.
static void blinkProtocol(Protocol p) {
  uint8_t n = (p == Protocol::MIDI) ? 2 : 1;
  for (uint8_t i = 0; i < n; ++i) {
    Board::setStatusLed(true);
    Board::bootDelayMs(150);
    Board::setStatusLed(false);
    Board::bootDelayMs(150);
  }
}

static FaultCode bootSelfTest(bool bench) {
  // Reset-cause faults first: a watchdog or brownout reset means the last
  // session ended abnormally — boot into lockout and say so.
  switch (Board::resetCause()) {
    case ResetCause::WATCHDOG: {
      FaultCode persisted = Board::recoverPersistedFault();
      return persisted != FaultCode::NONE ? persisted
                                          : FaultCode::WATCHDOG_RESET;
    }
    case ResetCause::BROWNOUT:
      return FaultCode::BROWNOUT_RESET;
    default:
      break;
  }

  if (!Board::selfTestOutputs()) return FaultCode::SELFTEST_OUTPUTS;

  // E-stop loop must be intact at boot (a broken wire reads as pressed).
  // Bench mode: no interlock hardware is connected, so skip this check.
  if (!bench && !Board::readHwInputs().estop_ok)
    return FaultCode::SELFTEST_ESTOP;

  // Timebase sanity: millis() must advance.
  TimeMs t0 = Board::millisNow();
  Board::bootDelayMs(5);
  if (Board::millisNow() == t0) return FaultCode::SELFTEST_RAM;

  return FaultCode::NONE;
}

void setup() {
  Board::init();  // outputs forced closed before anything else

  g_bench = Board::benchSelected();

  // Bring up the (optional) operator LCD and show the boot screen.
  Board::displayBegin();
  {
    char screen[DISPLAY_CHARS + 1];
    ShowInput boot_in;
    renderDisplay(screen, SafetyState::BOOT_SELFTEST, FaultCode::NONE,
                  Protocol::MIDI, boot_in, 0, false, Board::millisNow(),
                  g_bench);
    for (uint8_t i = 0; i < DISPLAY_CHARS + 8; ++i)
      Board::displayService(screen);
  }

  FaultCode boot_fault = bootSelfTest(g_bench);

  if (g_bench) {
    // Bench: USB serial console instead of a show protocol (3 boot blinks).
    for (uint8_t i = 0; i < 3; ++i) {
      Board::setStatusLed(true);
      Board::bootDelayMs(150);
      Board::setStatusLed(false);
      Board::bootDelayMs(150);
    }
    Board::benchBegin();
    Board::benchPrint("\nENLIGHTEN BENCH MODE - sequencer test console\n");
    Board::benchPrint("Interlocks are SIMULATED. NEVER connect fuel.\n");
    Board::benchPrint(BenchConsole::helpText());
    Board::benchPrint("\n> ");
  } else {
    g_protocol = Board::readProtocolSelect();
    blinkProtocol(g_protocol);
    if (g_protocol == Protocol::DMX) {
      Board::dmxBegin();
    } else {
      Board::midiBegin();
    }
  }

  g_sd_ok = Board::sdBegin();  // optional hardware; absent is fine

  g_safety.begin(boot_fault, Board::millisNow());
  if (boot_fault != FaultCode::NONE) Board::persistFault(boot_fault);

  Board::watchdogEnable();
}

// Debounced press-edge detector for a panel button.
struct ButtonEdge {
  bool prev = false;
  TimeMs last_change = 0;

  bool pressedEdge(bool raw, TimeMs now) {
    if (raw == prev || elapsedMs(last_change, now) < 30) return false;
    prev = raw;
    last_change = now;
    return raw;
  }
};

// PLAY button: start next *.SHW / stop the current one.
static void servicePlayButton(TimeMs now) {
  static ButtonEdge edge;
  if (!edge.pressedEdge(Board::playButtonPressed(), now)) return;

  if (g_player.playing()) {
    g_player.stop();
    Board::showClose();
  } else if (g_sd_ok &&
             Board::showOpenNext(g_show_name, sizeof(g_show_name))) {
    g_player.start(now);
  }
}

// DISP / SEL buttons: display pages and page-context input.
static void serviceUiButtons(TimeMs now) {
  static ButtonEdge disp, sel;
  if (disp.pressedEdge(Board::dispButtonPressed(), now)) g_ui.onDispPress(now);
  if (sel.pressedEdge(Board::selButtonPressed(), now)) g_ui.onSelPress(now);
  if (g_ui.consumeStatsReset()) g_stats.reset();
}

void loop() {
  TimeMs now = Board::millisNow();

  // 1. Decode the active input source into the common show-command model.
  ShowInput in;
  if (g_bench) {
    int16_t b;
    char reply[96];
    while ((b = Board::benchReadByte()) >= 0) {
      char echo[2] = {(char)b, 0};  // local echo for the terminal
      Board::benchPrint(b == '\r' ? "\r\n" : echo);
      if (g_bench_console.feed((char)b, now, reply, sizeof(reply))) {
        Board::benchPrint(reply);
        Board::benchPrint("\n> ");
      }
    }
    in = g_bench_console.snapshot(now);
  } else if (g_protocol == Protocol::DMX) {
    uint8_t ch[DmxDecoder::NUM_CHANNELS];
    for (uint8_t i = 0; i < DmxDecoder::NUM_CHANNELS; ++i)
      ch[i] = Board::dmxRead((uint16_t)(i + 1));
    in = g_dmx.decode(ch, Board::dmxAgeMs(), now);
  } else {
    int16_t b;
    MidiEvent ev;
    while ((b = Board::midiReadByte()) >= 0) {
      g_midi.onByte(now);
      if (g_midi_parser.feed((uint8_t)b, &ev)) g_midi.onEvent(ev, now);
    }
    in = g_midi.snapshot(now);
  }

  // 1a. Panel input: display pages + panel mode override (choreography
  //     only — the safety chain is untouched by panel input).
  serviceUiButtons(now);
  {
    ModeId panel_mode;
    if (g_ui.modeOverride(&panel_mode)) in.mode = panel_mode;
  }

  // 1b. SD show playback: pump file bytes, advance the playback clock.
  //     The player's mask is a REQUEST like any other — the safety filter
  //     still gates it, so playback fires nothing unless armed.
  servicePlayButton(now);
  while (g_player.wantsData()) {
    int16_t b = Board::showReadByte();
    if (b < 0) {
      g_player.feedEof();
      Board::showClose();
      break;
    }
    g_player.feed((char)b);
  }
  g_player.update(now);

  // 2. Advance the effect engine (advisory only).
  g_sequencer.update(in, now);

  // 3. The safety filter decides what actually opens...  In bench mode the
  //    interlock inputs are SIMULATED (nothing is wired); everything else —
  //    arming edge/hold, duty limits, watchdog — remains fully enforced.
  HwInputs hw;
  if (g_bench) {
    hw.estop_ok = true;
    hw.arm_key = true;
  } else {
    hw = Board::readHwInputs();
  }
  uint16_t requested =
      (uint16_t)(g_sequencer.requestedMask() | g_player.mask());
  uint16_t mask = g_safety.filter(requested, in, hw, now);

  // 4. ...and its output goes straight to the ports.  Only call site.
  Board::writeOutputs(mask);

  Board::setStatusLed(
      statusLedOn(g_safety.state(), g_safety.fault(), mask != 0, now));

  // Per-solenoid panel LEDs: solid = valve open (filtered truth),
  // dim = armed + enabled, dark = disarmed.
  Board::writeChannelLeds(
      channelLedMask(g_safety.state(), in.trigger_mask, mask, now));

  // Show statistics (STATS page), fed the filtered output — the truth.
  g_stats.update(mask, now);

  // Operator LCD: render the desired screen (cheap) and let the budgeted
  // writer converge the panel one character per loop.
  {
    char screen[DISPLAY_CHARS + 1];
    DisplayPage pg = g_ui.page(now);
    if (pg == DisplayPage::STATUS) {
      renderDisplay(screen, g_safety.state(), g_safety.fault(), g_protocol,
                    in, mask, hw.arm_key, now, g_bench,
                    g_player.playing() ? g_show_name : nullptr,
                    g_player.positionMs());
    } else {
      AuxPageInfo info;
      info.last_fault = g_safety.fault();
      info.proto = g_protocol;
      info.bench = g_bench;
      info.sd_ok = g_sd_ok;
      info.link_ok = in.link_ok;
      info.active_mode = in.mode;
      info.uptime_ms = now;
      renderAuxPage(screen, pg, g_ui, g_stats, info);
    }
    Board::displayService(screen);
  }

  // 5. Feed the watchdog only if the safety filter actually ran this pass.
  if (g_safety.consumeLoopHealth()) Board::watchdogFeed();
}
