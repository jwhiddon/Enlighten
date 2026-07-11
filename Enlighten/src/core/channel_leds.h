// Per-solenoid panel LEDs (16 on the controller box):
//
//   solid  = the solenoid is OPEN right now (driven from the FILTERED
//            output mask, so the panel shows what the valves are actually
//            doing — never just what was requested)
//   dim    = system is ARMED and this poofer is enabled/requested
//            (software PWM: ~12.5% duty clocked by the fast main loop)
//   off    = disarmed / lockout / not enabled
//
// Pure function; the board layer writes the returned mask to the LED
// ports.  Indicators only — nothing reads them back.
#pragma once
#include "safety.h"
#include "timebase.h"

uint16_t channelLedMask(SafetyState state, uint16_t enabled_mask,
                        uint16_t firing_mask, TimeMs now);
