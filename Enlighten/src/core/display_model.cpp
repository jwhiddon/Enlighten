#include "display_model.h"

namespace {

// Copy s into a 20-column row, space-padded, truncated if oversized.
void put(char* row, const char* s) {
  uint8_t i = 0;
  while (i < DISPLAY_COLS && s[i]) {
    row[i] = s[i];
    ++i;
  }
  while (i < DISPLAY_COLS) row[i++] = ' ';
}

// Overlay s right-aligned onto an already-filled row.
void overlayRight(char* row, const char* s) {
  uint8_t len = 0;
  while (s[len]) ++len;
  if (len > DISPLAY_COLS) len = DISPLAY_COLS;
  uint8_t start = (uint8_t)(DISPLAY_COLS - len);
  for (uint8_t i = 0; i < len; ++i) row[start + i] = s[i];
}

// Right-aligned decimal into dst[0..width-1], space-padded.
void numTo(char* dst, uint16_t v, uint8_t width) {
  for (int8_t i = (int8_t)width - 1; i >= 0; --i) {
    if (v || i == (int8_t)width - 1) {
      dst[i] = (char)('0' + v % 10);
      v /= 10;
    } else {
      dst[i] = ' ';
    }
  }
}

const char* modeName(ModeId m) {
  switch (m) {
    case ModeId::OFF:           return "OFF";
    case ModeId::RAW:           return "RAW";
    case ModeId::FIRE_ALL:      return "FIRE ALL";
    case ModeId::ALTERNATE:     return "ALTERNATE";
    case ModeId::CHASE_UP:      return "CHASE UP";
    case ModeId::CHASE_DOWN:    return "CHASE DOWN";
    case ModeId::CHASE_UP_DOWN: return "UP/DOWN";
    case ModeId::CHASE_DOWN_UP: return "DOWN/UP";
    case ModeId::CHASE_IN:      return "CHASE IN";
    case ModeId::CHASE_OUT:     return "CHASE OUT";
    case ModeId::CHASE_IN_OUT:  return "IN/OUT";
    case ModeId::CHASE_OUT_IN:  return "OUT/IN";
  }
  return "?";
}

const char* faultName(FaultCode f) {
  switch (f) {
    case FaultCode::NONE:             return "NO FAULT";
    case FaultCode::WATCHDOG_RESET:   return "WATCHDOG RESET";
    case FaultCode::BROWNOUT_RESET:   return "BROWNOUT RESET";
    case FaultCode::SELFTEST_RAM:     return "SELFTEST: TIMEBASE";
    case FaultCode::SELFTEST_OUTPUTS: return "SELFTEST: OUTPUTS";
    case FaultCode::SELFTEST_ESTOP:   return "SELFTEST: ESTOP";
    case FaultCode::ESTOP_ASSERTED:   return "ESTOP PRESSED";
    case FaultCode::LOOP_OVERRUN:     return "LOOP OVERRUN";
    case FaultCode::DUTY_EXCEEDED:    return "DUTY EXCEEDED";
    case FaultCode::DECODER_INVALID:  return "DECODER INVALID";
  }
  return "?";
}

void modeLine(char* row, ModeId m) {
  char line[DISPLAY_COLS + 1];
  const char* prefix = "MODE: ";
  uint8_t i = 0;
  while (prefix[i]) {
    line[i] = prefix[i];
    ++i;
  }
  const char* mn = modeName(m);
  for (uint8_t j = 0; mn[j] && i < DISPLAY_COLS; ++j) line[i++] = mn[j];
  line[i] = 0;
  put(row, line);
}

}  // namespace

void renderDisplay(char* out, SafetyState state, FaultCode fault,
                   Protocol proto, const ShowInput& in, uint16_t firing_mask,
                   bool arm_key, TimeMs now, bool bench,
                   const char* play_name, uint32_t play_pos_ms) {
  (void)now;
  char* l1 = out;
  char* l2 = out + DISPLAY_COLS;
  char* l3 = out + 2 * DISPLAY_COLS;
  char* l4 = out + 3 * DISPLAY_COLS;
  put(l3, "");
  put(l4, "");

  const char* tag = bench ? "[USB]"
                          : (proto == Protocol::MIDI ? "[MIDI]" : "[DMX]");

  switch (state) {
    case SafetyState::BOOT_SELFTEST:
      put(l1, bench ? "ENLIGHTEN BENCH" : "ENLIGHTEN v2");
      put(l2, "SELF-TEST...");
      break;

    case SafetyState::SAFE:
      put(l1, bench ? "BENCH SAFE" : "SAFE");
      overlayRight(l1, tag);
      // The most useful thing to tell an operator: what is missing.
      if (bench) put(l2, in.link_ok ? "TYPE: arm" : "OPEN TERM 115200 8N1");
      else if (!in.link_ok) put(l2, "NO SIGNAL");
      else if (!arm_key) put(l2, "TURN KEY TO ARM");
      else if (proto == Protocol::MIDI) put(l2, "ARM: CC20+CC21");
      else put(l2, "ARM: SEND 85+170");
      modeLine(l3, in.mode);
      break;

    case SafetyState::ARM_PENDING:
      put(l1, bench ? "BENCH ARMING..." : "ARMING...");
      if (!bench) overlayRight(l1, tag);
      put(l2, "HOLD ARM VALUES");
      modeLine(l3, in.mode);
      break;

    case SafetyState::ARMED: {
      put(l1, "ARMED");
      overlayRight(l1, modeName(in.mode));

      // Live poofer bar, column i = poofer i+1.
      l2[0] = ' ';
      l2[1] = ' ';
      l2[2] = '[';
      for (uint8_t c = 0; c < 16; ++c)
        l2[3 + c] = ((firing_mask >> c) & 1u) ? '#' : '.';
      l2[19] = ']';

      char line[DISPLAY_COLS + 1];
      put(line, "POOF       REST");
      line[DISPLAY_COLS] = 0;
      numTo(line + 5, in.poof_ms, 4);
      numTo(line + 16, in.rest_ms, 4);
      put(l3, line);

      if (play_name) {
        // "> MM:SS <file>"
        uint8_t i = 0;
        line[i++] = '>';
        line[i++] = ' ';
        uint32_t s = play_pos_ms / 1000;
        line[i++] = (char)('0' + (s / 600) % 10);
        line[i++] = (char)('0' + (s / 60) % 10);
        line[i++] = ':';
        line[i++] = (char)('0' + (s % 60) / 10);
        line[i++] = (char)('0' + s % 10);
        line[i++] = ' ';
        for (uint8_t j = 0; play_name[j] && i < DISPLAY_COLS; ++j)
          line[i++] = play_name[j];
        line[i] = 0;
        put(l4, line);
      } else {
        put(line, "REPEAT      RATE");
        line[DISPLAY_COLS] = 0;
        line[7] = 'O';
        line[8] = in.repeat ? 'N' : 'F';
        if (!in.repeat) line[9] = 'F';
        numTo(line + 17, in.rate, 3);
        put(l4, line);
      }
      break;
    }

    case SafetyState::FAULT_LOCKOUT:
      put(l1, "FAULT LOCKOUT");
      l1[19] = (char)('0' + ((uint8_t)fault <= 9 ? (uint8_t)fault : 9));
      put(l2, faultName(fault));
      put(l3, "CLEAR: LINK UP +");
      put(l4, "ARM VALUES DOWN");
      break;
  }

  out[DISPLAY_CHARS] = 0;
}
