#include "midi_decoder.h"

namespace {
// cc_[] slots (CC number - 20)
constexpr uint8_t CC_ARM_A = 0;   // CC20: must be MIDI_ARM_A
constexpr uint8_t CC_ARM_B = 1;   // CC21: must be MIDI_ARM_B
constexpr uint8_t CC_MODE = 2;    // CC22: mode selector (scaled x2 into bands)
constexpr uint8_t CC_POOF = 3;    // CC23
constexpr uint8_t CC_REST = 4;    // CC24
constexpr uint8_t CC_RATE = 5;    // CC25
constexpr uint8_t CC_REPEAT = 6;  // CC26: >= 64 = repeat
}  // namespace

void MidiDecoder::onByte(TimeMs now) {
  any_byte_ = true;
  last_byte_ = now;
}

void MidiDecoder::onEvent(const MidiEvent& ev, TimeMs now) {
  if (ev.status >= 0xF8) {
    if (ev.status == 0xFE) {
      seen_active_sensing_ = true;
      last_active_sensing_ = now;
    }
    return;
  }

  if ((ev.status & 0x0F) != cfg::MIDI_CHANNEL) return;
  uint8_t type = ev.status & 0xF0;

  if (type == 0x90 || type == 0x80) {
    uint8_t note = ev.data1;
    if (note < cfg::MIDI_NOTE_FIRST ||
        note >= cfg::MIDI_NOTE_FIRST + cfg::NUM_POOFERS)
      return;
    uint16_t b = (uint16_t)(1u << (note - cfg::MIDI_NOTE_FIRST));
    if (type == 0x90 && ev.data2 >= cfg::MIDI_VEL_GATE) {
      gates_ |= b;
    } else if (type == 0x80 || ev.data2 == 0) {
      gates_ = (uint16_t)(gates_ & ~b);
    }
    // note-on velocity 1..(gate-1): ignored entirely — no half-triggers.
    return;
  }

  if (type == 0xB0) {
    uint8_t cc = ev.data1;
    if (cc >= CC_BASE && cc < CC_BASE + CC_COUNT) {
      cc_[cc - CC_BASE] = ev.data2;
    } else if (cc == 120 || cc == 123) {
      // All-sound-off / all-notes-off: clear every gate AND disarm.
      gates_ = 0;
      cc_[CC_ARM_A] = 0xFF;
      cc_[CC_ARM_B] = 0xFF;
    }
  }
}

ShowInput MidiDecoder::snapshot(TimeMs now) {
  ShowInput out;

  bool link;
  if (seen_active_sensing_) {
    link = elapsedMs(last_active_sensing_, now) <= cfg::MIDI_AS_TIMEOUT_MS;
  } else {
    link = any_byte_ && elapsedMs(last_byte_, now) <= cfg::MIDI_KEEPALIVE_MS;
  }
  out.link_ok = link;

  out.arm_a = link && cc_[CC_ARM_A] == cfg::MIDI_ARM_A;
  out.arm_b = link && cc_[CC_ARM_B] == cfg::MIDI_ARM_B;

  // CC values are 7-bit; scale into the shared 0-255 mode-band domain.
  uint8_t mode_raw = cc_[CC_MODE] == 0xFF ? 0 : (uint8_t)(cc_[CC_MODE] << 1);
  out.mode = link ? mode_.update(mode_raw, now) : mode_.current();

  uint8_t poof = cc_[CC_POOF] == 0xFF ? 19 : cc_[CC_POOF];   // default ~100 ms
  uint8_t rest = cc_[CC_REST] == 0xFF ? 10 : cc_[CC_REST];   // default ~200 ms
  out.poof_ms = (uint16_t)(cfg::MIN_POOF_MS +
      (uint32_t)poof * (cfg::MAX_POOF_MS - cfg::MIN_POOF_MS) / 127u);
  out.rest_ms = (uint16_t)(cfg::MIN_REST_MS +
      (uint32_t)rest * (cfg::MAX_REST_MS - cfg::MIN_REST_MS) / 127u);
  out.rate = cc_[CC_RATE] == 0xFF ? (uint8_t)128 : (uint8_t)(cc_[CC_RATE] << 1);
  out.repeat = cc_[CC_REPEAT] != 0xFF && cc_[CC_REPEAT] >= 64;
  out.trigger_mask = gates_;
  return out;
}
