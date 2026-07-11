// Hardware panel inputs sampled by the board layer each loop.
#pragma once
#include <stdint.h>

struct HwInputs {
  bool estop_ok = false;  // true = NC E-stop loop intact (not pressed, not broken)
  bool arm_key = false;   // true = panel keyswitch permits arming
};
