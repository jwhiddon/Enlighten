// Hardware panel inputs sampled by the board layer each loop, plus the
// boot-time protocol selection.
#pragma once
#include <stdint.h>

enum class Protocol : uint8_t {
  DMX,   // default / failsafe (select jumper open)
  MIDI,  // select jumper to GND
};

struct HwInputs {
  bool estop_ok = false;  // true = NC E-stop loop intact (not pressed, not broken)
  bool arm_key = false;   // true = panel keyswitch permits arming
};
