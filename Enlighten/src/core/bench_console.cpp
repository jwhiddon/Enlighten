#include "bench_console.h"

namespace {

void copyTo(char* dst, unsigned cap, const char* src) {
  unsigned i = 0;
  while (src[i] && i + 1 < cap) {
    dst[i] = src[i];
    ++i;
  }
  dst[i] = 0;
}

char lower(char c) { return (c >= 'A' && c <= 'Z') ? (char)(c + 32) : c; }

bool is(const char* tok, const char* word) {
  unsigned i = 0;
  for (; tok[i] && word[i]; ++i)
    if (tok[i] != word[i]) return false;
  return tok[i] == 0 && word[i] == 0;
}

long toNum(const char* s) {
  long v = 0;
  bool any = false;
  while (*s >= '0' && *s <= '9') {
    v = v * 10 + (*s++ - '0');
    any = true;
    if (v > 100000) break;
  }
  return any ? v : -1;
}

uint16_t clampU16(long v, uint16_t lo, uint16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return (uint16_t)v;
}

struct ModeEntry {
  const char* name;
  ModeId id;
};
const ModeEntry MODES[] = {
    {"off", ModeId::OFF},           {"raw", ModeId::RAW},
    {"all", ModeId::FIRE_ALL},      {"alt", ModeId::ALTERNATE},
    {"up", ModeId::CHASE_UP},       {"down", ModeId::CHASE_DOWN},
    {"updown", ModeId::CHASE_UP_DOWN}, {"downup", ModeId::CHASE_DOWN_UP},
    {"in", ModeId::CHASE_IN},       {"out", ModeId::CHASE_OUT},
    {"inout", ModeId::CHASE_IN_OUT}, {"outin", ModeId::CHASE_OUT_IN},
};

}  // namespace

const char* BenchConsole::helpText() {
  return "cmds: arm disarm stop status | mode "
         "off/raw/all/alt/up/down/updown/downup/in/out/inout/outin | "
         "poof N rest N rate N repeat on/off | fire N hold N release [N]";
}

bool BenchConsole::feed(char c, TimeMs now, char* reply, unsigned cap) {
  any_activity_ = true;
  last_activity_ = now;
  // Both CR and LF terminate a line (PuTTY sends CR, monitors send LF;
  // a CRLF pair yields one command plus an ignored empty line).
  if (c != '\n' && c != '\r') {
    if (len_ + 1 < (uint8_t)sizeof(line_)) line_[len_++] = lower(c);
    return false;
  }
  line_[len_] = 0;
  len_ = 0;
  if (line_[0] == 0) return false;
  execute(now, reply, cap);
  return true;
}

void BenchConsole::execute(TimeMs now, char* reply, unsigned cap) {
  // Split "cmd arg"
  char* arg = line_;
  while (*arg && *arg != ' ') ++arg;
  if (*arg == ' ') {
    *arg++ = 0;
    while (*arg == ' ') ++arg;
  }
  const char* cmd = line_;

  if (is(cmd, "help")) {
    copyTo(reply, cap, helpText());
    return;
  }

  if (is(cmd, "arm")) {
    armed_ = true;
    // Delay assertion so the supervisor sees the values OFF first (edge),
    // then the normal 500 ms hold applies.
    arm_effective_ = now + 50;
    copyTo(reply, cap, "arming (edge + 500ms hold)... LED goes fast-blink");
    return;
  }
  if (is(cmd, "disarm") || is(cmd, "x")) {
    armed_ = false;
    hold_mask_ = 0;
    for (auto& p : pulse_active_) p = false;
    copyTo(reply, cap, "disarmed, all triggers cleared");
    return;
  }
  if (is(cmd, "stop")) {
    hold_mask_ = 0;
    for (auto& p : pulse_active_) p = false;
    copyTo(reply, cap, "all triggers cleared (still armed)");
    return;
  }

  if (is(cmd, "mode")) {
    for (const auto& m : MODES) {
      if (is(arg, m.name)) {
        mode_ = m.id;
        copyTo(reply, cap, "mode set");
        return;
      }
    }
    copyTo(reply, cap, "unknown mode (try: help)");
    return;
  }

  if (is(cmd, "poof")) {
    poof_ = clampU16(toNum(arg), cfg::MIN_POOF_MS, cfg::MAX_POOF_MS);
    copyTo(reply, cap, "poof set (clamped 30-500ms)");
    return;
  }
  if (is(cmd, "rest")) {
    rest_ = clampU16(toNum(arg), cfg::MIN_REST_MS, cfg::MAX_REST_MS);
    copyTo(reply, cap, "rest set (clamped 45-2000ms)");
    return;
  }
  if (is(cmd, "rate")) {
    rate_ = (uint8_t)clampU16(toNum(arg), 0, 255);
    copyTo(reply, cap, "rate set (128 = 1.0x)");
    return;
  }
  if (is(cmd, "repeat")) {
    repeat_ = is(arg, "on") || is(arg, "1");
    copyTo(reply, cap, repeat_ ? "repeat on" : "repeat off");
    return;
  }

  if (is(cmd, "fire") || is(cmd, "hold") || is(cmd, "release")) {
    long n = toNum(arg);
    if (is(cmd, "release") && n < 0) {
      hold_mask_ = 0;
      copyTo(reply, cap, "all holds released");
      return;
    }
    if (n < 1 || n > cfg::NUM_POOFERS) {
      copyTo(reply, cap, "poofer number must be 1-16");
      return;
    }
    uint8_t i = (uint8_t)(n - 1);
    uint16_t b = (uint16_t)(1u << i);
    if (is(cmd, "fire")) {
      // Level-pulse the trigger just long enough for one full poof.
      pulse_active_[i] = true;
      pulse_until_[i] = now + poof_ + 10;
      copyTo(reply, cap, "poof!");
    } else if (is(cmd, "hold")) {
      hold_mask_ |= b;
      copyTo(reply, cap, "held (duty limits will throttle)");
    } else {
      hold_mask_ = (uint16_t)(hold_mask_ & ~b);
      copyTo(reply, cap, "released");
    }
    return;
  }

  if (is(cmd, "status")) {
    // Compact fixed-format status line.
    char* p = reply;
    unsigned left = cap;
    auto append = [&](const char* s) {
      while (*s && left > 1) {
        *p++ = *s++;
        --left;
      }
      *p = 0;
    };
    append(armed_ ? "armed " : "disarmed ");
    for (const auto& m : MODES)
      if (m.id == mode_) {
        append("mode:");
        append(m.name);
      }
    char num[24];
    auto appendNum = [&](const char* label, uint32_t v) {
      append(label);
      char tmp[12];
      int n = 0;
      if (v == 0) tmp[n++] = '0';
      while (v) {
        tmp[n++] = (char)('0' + v % 10);
        v /= 10;
      }
      int j = 0;
      while (n) num[j++] = tmp[--n];
      num[j] = 0;
      append(num);
    };
    appendNum(" poof:", poof_);
    appendNum(" rest:", rest_);
    appendNum(" rate:", rate_);
    append(repeat_ ? " repeat:on" : " repeat:off");
    return;
  }

  copyTo(reply, cap, "? (try: help)");
}

ShowInput BenchConsole::snapshot(TimeMs now) {
  // Deadman: console silence disarms.  Re-arming afterwards produces a
  // fresh OFF->ON edge, so the supervisor accepts it.
  bool alive =
      any_activity_ && elapsedMs(last_activity_, now) < cfg::BENCH_KEEPALIVE_MS;
  if (!alive) armed_ = false;

  ShowInput s;
  s.link_ok = alive;
  bool arm_now = armed_ && timeReached(arm_effective_, now);
  s.arm_a = alive && arm_now;
  s.arm_b = alive && arm_now;
  s.mode = mode_;
  s.poof_ms = poof_;
  s.rest_ms = rest_;
  s.rate = rate_;
  s.repeat = repeat_;

  uint16_t pulses = 0;
  for (uint8_t i = 0; i < cfg::NUM_POOFERS; ++i) {
    if (pulse_active_[i]) {
      if (timeReached(pulse_until_[i], now)) pulse_active_[i] = false;
      else pulses |= (uint16_t)(1u << i);
    }
  }
  s.trigger_mask = (uint16_t)(hold_mask_ | pulses);
  return s;
}
