// The effect engine.  One parametric step machine covers every chase mode
// (the legacy firmware had eight near-identical copy-pasted functions);
// RAW / FIRE_ALL / ALTERNATE share the same timing core.
//
// The sequencer's output is ADVISORY: requestedMask() feeds the
// SafetySupervisor, which independently clamps every channel.  The
// sequencer never touches hardware.
#pragma once
#include "config.h"
#include "show_input.h"
#include "timebase.h"

class Sequencer {
 public:
  void update(const ShowInput& in, TimeMs now);
  uint16_t requestedMask() const { return mask_; }
  void reset();

 private:
  struct ChaseParams {
    uint8_t first;   // starting index
    uint8_t end;     // far endpoint
    int8_t dir;      // initial step direction
    bool mirrored;   // also fire poofer (7 - index)
    bool bounce;     // ping-pong instead of wrap
  };

  enum class Phase : uint8_t { IDLE, POOF, REST, DONE };

  static const ChaseParams* chaseParams(ModeId m);
  bool isStepMode(ModeId m) const;
  uint16_t stepPattern() const;      // pattern for the current step (ungated)
  void startCycle(const ShowInput& in, TimeMs now);
  void advanceStep(const ShowInput& in, TimeMs now);
  void stepUpdate(const ShowInput& in, TimeMs now);
  void rawUpdate(const ShowInput& in, TimeMs now);

  ModeId mode_ = ModeId::OFF;
  uint16_t mask_ = 0;

  // step-engine state (chases, ALTERNATE, FIRE_ALL)
  Phase phase_ = Phase::IDLE;
  TimeMs deadline_ = 0;
  uint8_t index_ = 0;
  int8_t dir_ = 1;
  const ChaseParams* params_ = nullptr;

  // clamped/derived timing, recomputed each update
  uint16_t poof_eff_ = cfg::MIN_POOF_MS;
  uint16_t rest_eff_ = cfg::MIN_REST_MS;

  // RAW-mode per-channel state
  uint16_t raw_firing_ = 0;
  uint16_t raw_need_release_ = 0;  // one-shot: wait for trigger drop
  TimeMs raw_deadline_[cfg::NUM_POOFERS] = {};
  TimeMs raw_next_ok_[cfg::NUM_POOFERS] = {};
};
