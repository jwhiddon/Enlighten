#include "safety.h"

void SafetySupervisor::begin(FaultCode boot_fault, TimeMs now) {
  (void)now;
  if (state_ != SafetyState::BOOT_SELFTEST) return;  // begin() is one-shot
  if (boot_fault != FaultCode::NONE) {
    lockout(boot_fault);
  } else {
    enterSafe();
  }
}

void SafetySupervisor::enterSafe() {
  state_ = SafetyState::SAFE;
  arm_edge_ok_ = false;  // a fresh wrong->right transition is now required
}

void SafetySupervisor::lockout(FaultCode f) {
  state_ = SafetyState::FAULT_LOCKOUT;
  fault_ = f;
}

void SafetySupervisor::reportFault(FaultCode f) {
  lockout(f);
}

void SafetySupervisor::stepStateMachine(const ShowInput& in,
                                        const HwInputs& hw, TimeMs now) {
  // E-stop dominates every state, every loop.
  if (!hw.estop_ok) {
    if (state_ != SafetyState::BOOT_SELFTEST) lockout(FaultCode::ESTOP_ASSERTED);
    return;
  }

  const bool handshake = in.link_ok && in.arm_a && in.arm_b;

  switch (state_) {
    case SafetyState::BOOT_SELFTEST:
      break;  // only begin() leaves this state

    case SafetyState::SAFE:
      // The de-asserted edge must be OBSERVED over a live link.  Decoders
      // zero their arm flags when the link is down, and a dead link is not
      // evidence that the operator de-asserted anything — without this
      // qualifier, unplugging and replugging a console whose scene still
      // holds the arm values would auto-re-arm the system.
      if (in.link_ok && !(in.arm_a && in.arm_b)) arm_edge_ok_ = true;
      if (arm_edge_ok_ && handshake && hw.arm_key) {
        state_ = SafetyState::ARM_PENDING;
        pending_since_ = now;
      }
      break;

    case SafetyState::ARM_PENDING:
      if (!(handshake && hw.arm_key)) {
        enterSafe();
        // If the arm VALUES were de-asserted (vs. link/key loss with values
        // still held), that de-assertion is itself the witnessed edge.
        if (in.link_ok && !(in.arm_a && in.arm_b)) arm_edge_ok_ = true;
      } else if (elapsedMs(pending_since_, now) >= cfg::ARM_HOLD_MS) {
        state_ = SafetyState::ARMED;
      }
      break;

    case SafetyState::ARMED:
      // Deadman: signal loss or any arm condition dropping disarms in the
      // same loop iteration.
      if (!(handshake && hw.arm_key)) {
        enterSafe();
        if (in.link_ok && !(in.arm_a && in.arm_b)) arm_edge_ok_ = true;
      }
      break;

    case SafetyState::FAULT_LOCKOUT:
      // Clearing a fault requires the operator present (link up) with both
      // arm values explicitly de-asserted, so the fault code stays visible
      // on the LED until someone acknowledges it from the console.
      if (in.link_ok && !in.arm_a && !in.arm_b) enterSafe();
      break;
  }
}

uint16_t SafetySupervisor::filter(uint16_t requested, const ShowInput& in,
                                  const HwInputs& hw, TimeMs now) {
  filtered_since_health_ = true;

  // Self-check: if the loop stalled between filter calls, something is
  // blocking the pipeline — latch a fault rather than trust stale timing.
  if (have_last_filter_ &&
      elapsedMs(last_filter_, now) > cfg::LOOP_BUDGET_MS &&
      state_ != SafetyState::BOOT_SELFTEST) {
    lockout(FaultCode::LOOP_OVERRUN);
  }
  last_filter_ = now;
  have_last_filter_ = true;

  stepStateMachine(in, hw, now);

  if (state_ != SafetyState::ARMED) {
    // Keep duty accounting live so open time accrued just before a disarm
    // still counts against the budget.
    for (uint8_t i = 0; i < cfg::NUM_POOFERS; ++i) duty_[i].evaluate(false, now);
    return 0;
  }

  uint16_t out = 0;
  for (uint8_t i = 0; i < cfg::NUM_POOFERS; ++i) {
    bool want = (requested >> i) & 1u;
    if (duty_[i].evaluate(want, now)) out |= (uint16_t)(1u << i);
  }
  return out;
}

bool SafetySupervisor::consumeLoopHealth() {
  bool ok = filtered_since_health_;
  filtered_since_health_ = false;
  return ok;
}
