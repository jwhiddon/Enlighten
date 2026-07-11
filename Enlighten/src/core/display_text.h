// Shared text helpers and dimensions for the 20x4 operator display.
#pragma once
#include <stdint.h>

#include "faults.h"
#include "show_input.h"

constexpr uint8_t DISPLAY_COLS = 20;
constexpr uint8_t DISPLAY_ROWS = 4;
constexpr uint8_t DISPLAY_CHARS = DISPLAY_COLS * DISPLAY_ROWS;

// Copy s into a 20-column row, space-padded, truncated if oversized.
void dispPut(char* row, const char* s);

// Overlay s right-aligned onto an already-filled row.
void dispOverlayRight(char* row, const char* s);

// Right-aligned decimal into dst[0..width-1], space-padded.
void dispNumTo(char* dst, uint32_t v, uint8_t width);

const char* dispModeName(ModeId m);
const char* dispFaultName(FaultCode f);
