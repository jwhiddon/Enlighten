// Operator display model: renders the desired 20x4 character screen as a
// pure function of system state.  Portable and host-tested; the board
// layer's budgeted writer (Board::displayService) converges the physical
// LCD toward this text a character at a time so the 5 ms loop budget is
// never violated.
//
// Layout (20 columns x 4 rows, row-major in out[0..79], out[80] = NUL):
//
//   BOOT     ENLIGHTEN v2        SAFE     SAFE          [MIDI]
//            SELF-TEST...                 ARM: CC20+CC21
//                                         MODE: CHASE UP
//
//   ARMED    ARMED       CHASE UP LOCKOUT FAULT LOCKOUT      6
//              [##..............]         ESTOP PRESSED
//            POOF  500  REST 2000         CLEAR: LINK UP +
//            REPEAT ON   RATE 128         ARM VALUES DOWN
//            (line 4 shows "> MM:SS <file>" during SD playback)
#pragma once
#include "display_text.h"
#include "faults.h"
#include "hw_inputs.h"
#include "safety.h"
#include "show_input.h"
#include "timebase.h"

// `out` must hold DISPLAY_CHARS + 1 bytes; always space-padded, always
// NUL-terminated, only printable ASCII.  `bench` marks bench mode loudly.
// Pass `play_name` (+ position) while SD playback is running.
void renderDisplay(char* out, SafetyState state, FaultCode fault,
                   Protocol proto, const ShowInput& in, uint16_t firing_mask,
                   bool arm_key, TimeMs now, bool bench = false,
                   const char* play_name = nullptr, uint32_t play_pos_ms = 0);
