// Status LED patterns.  Pure: returns whether the LED should be lit at
// `now`; the board layer does the pin write.
//
//   BOOT_SELFTEST   solid on
//   SAFE            slow 1 Hz blink
//   ARM_PENDING     double-blink burst
//   ARMED idle      fast 4 Hz blink (deliberately conspicuous)
//   ARMED firing    solid on
//   FAULT_LOCKOUT   N blinks (N = fault code), 1 s pause, repeat
#pragma once
#include "faults.h"
#include "safety.h"
#include "timebase.h"

bool statusLedOn(SafetyState state, FaultCode fault, bool firing, TimeMs now);
