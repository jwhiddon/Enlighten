// Latched fault codes.  The numeric value doubles as the status-LED blink
// count in FAULT_LOCKOUT, so keep values small, stable, and documented in
// docs/SAFETY.md.
#pragma once
#include <stdint.h>

enum class FaultCode : uint8_t {
  NONE             = 0,
  WATCHDOG_RESET   = 1,  // firmware hung; watchdog forced a hardware reset
  BROWNOUT_RESET   = 2,  // supply dipped below the brownout threshold
  SELFTEST_RAM     = 3,
  SELFTEST_OUTPUTS = 4,  // output-port readback mismatch at boot
  SELFTEST_ESTOP   = 5,  // E-stop loop open at boot (pressed or broken wire)
  ESTOP_ASSERTED   = 6,
  LOOP_OVERRUN     = 7,  // main loop exceeded its time budget
  DUTY_EXCEEDED    = 8,  // reserved: repeated duty-budget violations
  DECODER_INVALID  = 9,  // reserved: persistent invalid input frames
};
