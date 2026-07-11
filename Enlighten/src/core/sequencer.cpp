#include "sequencer.h"

namespace {
inline uint16_t clampU16(uint16_t v, uint16_t lo, uint16_t hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
inline uint16_t bit16(uint8_t i) { return (uint16_t)(1u << i); }
}  // namespace

const Sequencer::ChaseParams* Sequencer::chaseParams(ModeId m) {
  // {first, end, dir, mirrored, bounce}; chases run on poofers 0-7,
  // mirrored modes pair index i with (7 - i): (0,7) (1,6) (2,5) (3,4).
  static const ChaseParams kUp{0, 7, 1, false, false};
  static const ChaseParams kDown{7, 0, -1, false, false};
  static const ChaseParams kUpDown{0, 7, 1, false, true};
  static const ChaseParams kDownUp{7, 0, -1, false, true};
  static const ChaseParams kIn{0, 3, 1, true, false};
  static const ChaseParams kOut{3, 0, -1, true, false};
  static const ChaseParams kInOut{0, 3, 1, true, true};
  static const ChaseParams kOutIn{3, 0, -1, true, true};
  static const ChaseParams kAll{0, 0, 1, false, false};   // single step
  static const ChaseParams kAlt{0, 1, 1, false, false};   // two steps

  switch (m) {
    case ModeId::CHASE_UP:      return &kUp;
    case ModeId::CHASE_DOWN:    return &kDown;
    case ModeId::CHASE_UP_DOWN: return &kUpDown;
    case ModeId::CHASE_DOWN_UP: return &kDownUp;
    case ModeId::CHASE_IN:      return &kIn;
    case ModeId::CHASE_OUT:     return &kOut;
    case ModeId::CHASE_IN_OUT:  return &kInOut;
    case ModeId::CHASE_OUT_IN:  return &kOutIn;
    case ModeId::FIRE_ALL:      return &kAll;
    case ModeId::ALTERNATE:     return &kAlt;
    default:                    return nullptr;
  }
}

bool Sequencer::isStepMode(ModeId m) const { return chaseParams(m) != nullptr; }

void Sequencer::reset() {
  mask_ = 0;
  phase_ = Phase::IDLE;
  raw_firing_ = 0;
  raw_need_release_ = 0;
  for (uint8_t i = 0; i < cfg::NUM_POOFERS; ++i) {
    raw_deadline_[i] = 0;
    raw_next_ok_[i] = 0;
  }
}

uint16_t Sequencer::stepPattern() const {
  switch (mode_) {
    case ModeId::FIRE_ALL:
      return 0xFFFF;
    case ModeId::ALTERNATE:
      return index_ ? 0xAAAA : 0x5555;  // odds / evens
    default: {
      uint16_t p = bit16(index_);
      if (params_ && params_->mirrored)
        p |= bit16((uint8_t)(cfg::NUM_CHASE_POOFERS - 1 - index_));
      return p;
    }
  }
}

void Sequencer::startCycle(const ShowInput& in, TimeMs now) {
  (void)in;
  index_ = params_->first;
  dir_ = params_->dir;
  phase_ = Phase::POOF;
  deadline_ = now + poof_eff_;
}

void Sequencer::advanceStep(const ShowInput& in, TimeMs now) {
  if (!params_->bounce) {
    if (index_ == params_->end) {
      if (in.repeat) {
        index_ = params_->first;
      } else {
        phase_ = Phase::DONE;
        return;
      }
    } else {
      index_ = (uint8_t)(index_ + dir_);
    }
  } else {
    if (index_ == params_->end && dir_ == params_->dir) {
      // Outbound endpoint reached (it already fired once): turn around.
      dir_ = (int8_t)-params_->dir;
      index_ = (uint8_t)(index_ + dir_);
    } else if (index_ == params_->first && dir_ != params_->dir) {
      // Back home after the return leg: cycle complete.
      if (!in.repeat) {
        phase_ = Phase::DONE;
        return;
      }
      dir_ = params_->dir;
      index_ = (uint8_t)(index_ + dir_);
    } else {
      index_ = (uint8_t)(index_ + dir_);
    }
  }
  phase_ = Phase::POOF;
  deadline_ = now + poof_eff_;
}

void Sequencer::stepUpdate(const ShowInput& in, TimeMs now) {
  switch (phase_) {
    case Phase::IDLE:
      startCycle(in, now);
      break;
    case Phase::POOF:
      if (timeReached(deadline_, now)) {
        phase_ = Phase::REST;
        deadline_ = now + rest_eff_;
      }
      break;
    case Phase::REST:
      if (timeReached(deadline_, now)) advanceStep(in, now);
      break;
    case Phase::DONE:
      // One-shot finished.  Turning repeat on resumes; re-selecting the
      // mode (via a mode change and back) restarts.
      if (in.repeat) startCycle(in, now);
      break;
  }
  // Live gating: a poofer disabled mid-step closes immediately.
  mask_ = (phase_ == Phase::POOF) ? (uint16_t)(stepPattern() & in.trigger_mask) : 0;
}

void Sequencer::rawUpdate(const ShowInput& in, TimeMs now) {
  for (uint8_t i = 0; i < cfg::NUM_POOFERS; ++i) {
    uint16_t b = bit16(i);
    bool want = (in.trigger_mask & b) != 0;

    if (raw_firing_ & b) {
      // End the poof at its deadline, or immediately on trigger release.
      if (!want || timeReached(raw_deadline_[i], now)) {
        raw_firing_ = (uint16_t)(raw_firing_ & ~b);
        raw_next_ok_[i] = now + rest_eff_;
        if (want && !in.repeat) raw_need_release_ |= b;
      }
    } else {
      if (!want) {
        raw_need_release_ = (uint16_t)(raw_need_release_ & ~b);
      } else if (!(raw_need_release_ & b) && timeReached(raw_next_ok_[i], now)) {
        raw_firing_ |= b;
        raw_deadline_[i] = now + poof_eff_;
      }
    }
  }
  mask_ = raw_firing_;
}

void Sequencer::update(const ShowInput& in, TimeMs now) {
  if (in.mode != mode_) {
    reset();
    mode_ = in.mode;
    params_ = chaseParams(mode_);
  }

  poof_eff_ = clampU16(in.poof_ms, cfg::MIN_POOF_MS, cfg::MAX_POOF_MS);
  // rate scales the rest (tempo): 128 = 1.0x, 255 ~= 2x speed, 0 = half speed.
  uint32_t rest = (uint32_t)clampU16(in.rest_ms, cfg::MIN_REST_MS, cfg::MAX_REST_MS);
  rest = rest * (uint32_t)(256 - in.rate) / 128u;
  rest_eff_ = clampU16((uint16_t)(rest > 0xFFFFu ? 0xFFFFu : rest),
                       cfg::MIN_REST_MS, cfg::MAX_REST_MS);

  switch (mode_) {
    case ModeId::OFF:
      mask_ = 0;
      break;
    case ModeId::RAW:
      rawUpdate(in, now);
      break;
    default:
      if (params_) {
        stepUpdate(in, now);
      } else {
        mask_ = 0;
      }
      break;
  }
}
