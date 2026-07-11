#include "dmx_decoder.h"

namespace {
// ch[] indices (0-based) for the 1-based DMX channels in docs/DMX_MAP.md.
constexpr uint8_t CH_ARM_A = 0;
constexpr uint8_t CH_ARM_B = 1;
constexpr uint8_t CH_MODE = 2;
constexpr uint8_t CH_POOF_MS = 3;
constexpr uint8_t CH_REST_MS = 4;
constexpr uint8_t CH_RATE = 5;
constexpr uint8_t CH_OPTIONS = 6;
// ch[7] reserved
constexpr uint8_t CH_TRIGGER_BASE = 8;  // ch 9..24 -> poofers 1..16

inline bool inArmWindow(uint8_t v, uint8_t center) {
  return v >= (uint8_t)(center - cfg::DMX_ARM_TOL) &&
         v <= (uint8_t)(center + cfg::DMX_ARM_TOL);
}
}  // namespace

ShowInput DmxDecoder::decode(const uint8_t* ch, uint32_t age_ms, TimeMs now) {
  ShowInput out;
  out.link_ok = age_ms < cfg::DMX_TIMEOUT_MS;

  // Arm values only count while the link is alive — stale buffer contents
  // must never look like an arming request.
  out.arm_a = out.link_ok && inArmWindow(ch[CH_ARM_A], cfg::DMX_ARM_A);
  out.arm_b = out.link_ok && inArmWindow(ch[CH_ARM_B], cfg::DMX_ARM_B);

  // Hold the previous mode and triggers while the link is down.
  if (out.link_ok) {
    out.mode = mode_.update(ch[CH_MODE], now);
    for (uint8_t i = 0; i < cfg::NUM_POOFERS; ++i) {
      uint8_t v = ch[CH_TRIGGER_BASE + i];
      uint16_t b = (uint16_t)(1u << i);
      if (v >= cfg::TRIG_ON_AT) trig_state_ |= b;
      else if (v < cfg::TRIG_OFF_BELOW) trig_state_ = (uint16_t)(trig_state_ & ~b);
      // 100-199: dead zone, hold previous state (fade-through-safe)
    }
  } else {
    out.mode = mode_.current();
  }
  out.trigger_mask = trig_state_;

  out.poof_ms = (uint16_t)(cfg::MIN_POOF_MS +
      (uint32_t)ch[CH_POOF_MS] * (cfg::MAX_POOF_MS - cfg::MIN_POOF_MS) / 255u);
  out.rest_ms = (uint16_t)(cfg::MIN_REST_MS +
      (uint32_t)ch[CH_REST_MS] * (cfg::MAX_REST_MS - cfg::MIN_REST_MS) / 255u);
  out.rate = ch[CH_RATE] == 0 ? (uint8_t)128 : ch[CH_RATE];  // 0 (unpatched) = 1.0x
  out.repeat = ch[CH_OPTIONS] >= 128;
  return out;
}
