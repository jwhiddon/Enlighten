// Bench-mode serial console: drives the sequencer from a USB terminal for
// bench testing WITHOUT the show hardware (no console, no E-stop loop, no
// keyswitch).  Selected by the bench jumper at boot.
//
// What bench mode does NOT relax: the SafetySupervisor and duty limits are
// fully active.  `arm` still goes through the edge + 500 ms hold, outputs
// still flow only through filter(), and 2 minutes of console silence is a
// deadman that disarms the rig.  The ONLY simulated things are the
// hardware interlock inputs and the signal source.
//
// NEVER connect fuel in bench mode.
#pragma once
#include "config.h"
#include "show_input.h"
#include "timebase.h"

class BenchConsole {
 public:
  // Feed one received byte.  Returns true when a full command line was
  // executed; `reply` (capacity `cap`, >= 96 recommended) then holds a
  // single-line response for the operator.
  bool feed(char c, TimeMs now, char* reply, unsigned cap);

  // The console's view of the show input, deadman applied.
  ShowInput snapshot(TimeMs now);

  static const char* helpText();

 private:
  void execute(TimeMs now, char* reply, unsigned cap);

  char line_[40];
  uint8_t len_ = 0;
  bool any_activity_ = false;
  TimeMs last_activity_ = 0;

  bool armed_ = false;
  TimeMs arm_effective_ = 0;  // arm values assert only after a short delay
                              // so the supervisor witnesses the OFF edge
  ModeId mode_ = ModeId::RAW;
  uint16_t poof_ = 150;
  uint16_t rest_ = 200;
  uint8_t rate_ = 128;
  bool repeat_ = false;
  uint16_t hold_mask_ = 0;
  TimeMs pulse_until_[cfg::NUM_POOFERS] = {};
  bool pulse_active_[cfg::NUM_POOFERS] = {};
};
