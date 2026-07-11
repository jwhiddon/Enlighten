// SafetySupervisor — the FINAL authority over the outputs.
//
// filter() is the single decision point between everything upstream
// (decoders, sequencer) and the port write.  It returns 0 in every state
// except ARMED, and while ARMED it clamps every channel through its
// DutyLimiter.  A freshly constructed supervisor is safe by definition.
//
// State machine (see docs/SAFETY.md):
//
//   BOOT_SELFTEST --begin(ok)--> SAFE
//   BOOT_SELFTEST --begin(fault)--> FAULT_LOCKOUT
//   SAFE --(edge-latched handshake + key + link, held ARM_HOLD_MS)--> ARMED
//   ARMED --(any arm condition or link drops)--> SAFE   (same loop)
//   any  --(E-stop asserted / reported fault)--> FAULT_LOCKOUT
//   FAULT_LOCKOUT --(E-stop ok, link ok, both arm values de-asserted)--> SAFE
//
// The rising-edge latch: arming values must be seen NOT-asserted after every
// entry into SAFE before they can arm — a console scene saved with the arm
// values in it can never arm the system at patch-in or after a signal blip.
#pragma once
#include "config.h"
#include "duty_limiter.h"
#include "faults.h"
#include "hw_inputs.h"
#include "show_input.h"
#include "timebase.h"

enum class SafetyState : uint8_t {
  BOOT_SELFTEST = 0,
  SAFE,           // disarmed idle; outputs forced closed
  ARM_PENDING,    // handshake seen; hold timer running
  ARMED,          // requests honored, subject to per-channel duty limits
  FAULT_LOCKOUT,  // latched fault; explicit clear + re-arm cycle required
};

class SafetySupervisor {
 public:
  // Leave BOOT_SELFTEST: NONE -> SAFE, anything else -> FAULT_LOCKOUT.
  void begin(FaultCode boot_fault, TimeMs now);

  // The one and only output decision.  Call exactly once per loop; the
  // return value must go straight to Board::writeOutputs().
  uint16_t filter(uint16_t requested, const ShowInput& in, const HwInputs& hw,
                  TimeMs now);

  // Latch an externally detected fault (decoder corruption, etc.).
  void reportFault(FaultCode f);

  SafetyState state() const { return state_; }
  FaultCode fault() const { return fault_; }

  // True if filter() ran since the previous call — the watchdog-feed gate.
  // A code path that skips the safety filter starves the watchdog.
  bool consumeLoopHealth();

 private:
  void stepStateMachine(const ShowInput& in, const HwInputs& hw, TimeMs now);
  void enterSafe();
  void lockout(FaultCode f);

  SafetyState state_ = SafetyState::BOOT_SELFTEST;
  FaultCode fault_ = FaultCode::NONE;
  bool arm_edge_ok_ = false;   // saw handshake de-asserted since SAFE entry
  TimeMs pending_since_ = 0;
  TimeMs last_filter_ = 0;
  bool have_last_filter_ = false;
  bool filtered_since_health_ = false;
  DutyLimiter duty_[cfg::NUM_POOFERS];
};
