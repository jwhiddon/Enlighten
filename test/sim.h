// Virtual-time simulator: drives the core pipeline exactly like
// Enlighten.ino does, in 1 ms ticks, with an invariant checker that runs
// every tick and a recorded per-ms output history for sliding-window
// assertions.
#pragma once
#include <cstdint>
#include <cstring>

#include "core/config.h"
#include "core/safety.h"
#include "core/sequencer.h"
#include "core/show_input.h"
#include "core/timebase.h"

// Checks the non-negotiable safety properties on every observed output.
// Usable standalone (e.g. by the fuzzer, which drives filter() directly).
class InvariantChecker {
 public:
  void observe(SafetyState state, bool estop_ok, uint16_t mask, TimeMs now);
  bool ok() const { return ok_; }
  const char* message() const { return msg_; }

 private:
  void fail(const char* m);

  bool ok_ = true;
  char msg_[160] = {};
  bool has_prev_ = false;
  uint16_t prev_mask_ = 0;
  TimeMs prev_now_ = 0;
  uint32_t open_run_ms_[cfg::NUM_POOFERS] = {};
  bool ever_open_[cfg::NUM_POOFERS] = {};
  TimeMs closed_at_[cfg::NUM_POOFERS] = {};
  // ring of per-ms output masks for the sliding duty window
  static constexpr uint32_t RING = cfg::DUTY_WINDOW_MS;
  uint16_t ring_[RING] = {};
  uint32_t ring_pos_ = 0;
  uint32_t ring_fill_ = 0;
  uint32_t duty_ms_[cfg::NUM_POOFERS] = {};  // rolling sums over the ring
};

// Full-pipeline simulator (decoderless: you set ShowInput fields directly).
class Sim {
 public:
  Sim() { in.rate = 128; }

  void begin(FaultCode boot_fault = FaultCode::NONE) {
    safety.begin(boot_fault, now);
  }

  // Advance `ms` milliseconds, one tick at a time.
  void step(uint32_t ms) {
    for (uint32_t i = 0; i < ms; ++i) tick();
  }

  void tick() {
    now += 1;
    seq.update(in, now);
    last_mask = safety.filter(seq.requestedMask(), in, hw, now);
    checker.observe(safety.state(), hw.estop_ok, last_mask, now);
  }

  // Convenience: assert the full arming handshake and run it to completion.
  void armFully() {
    hw.estop_ok = true;
    hw.arm_key = true;
    in.link_ok = true;
    in.arm_a = false;
    in.arm_b = false;
    step(5);  // witness the de-asserted edge
    in.arm_a = true;
    in.arm_b = true;
    step(cfg::ARM_HOLD_MS + 5);
  }

  SafetySupervisor safety;
  Sequencer seq;
  InvariantChecker checker;
  ShowInput in;
  HwInputs hw{true, true};
  TimeMs now = 0;
  uint16_t last_mask = 0;
};
