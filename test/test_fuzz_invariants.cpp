// Fuzz the SafetySupervisor directly with random requested masks, random
// ShowInput, and random hardware inputs, asserting the safety invariants on
// every single tick.  This bypasses the sequencer on purpose: the filter
// must hold even against a hostile/buggy upstream.
#include <cstdio>

#include "core/safety.h"
#include "framework.h"
#include "sim.h"

namespace {
struct XorShift32 {
  uint32_t s;
  explicit XorShift32(uint32_t seed) : s(seed) {}
  uint32_t next() {
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
  }
  // true with probability 1/n
  bool oneIn(uint32_t n) { return next() % n == 0; }
};
}  // namespace

TEST(fuzz_filter_invariants) {
  constexpr uint32_t TICKS = 2000000;  // 2M ms of virtual time

  SafetySupervisor safety;
  InvariantChecker checker;
  XorShift32 rng(0xC0FFEE01);

  ShowInput in;
  HwInputs hw{true, true};
  TimeMs now = 0;

  safety.begin(FaultCode::NONE, now);

  for (uint32_t t = 0; t < TICKS; ++t) {
    ++now;

    // Mutate inputs with various probabilities.
    if (rng.oneIn(50)) in.link_ok = !in.link_ok;
    if (rng.oneIn(20)) in.arm_a = !in.arm_a;
    if (rng.oneIn(20)) in.arm_b = !in.arm_b;
    if (rng.oneIn(200)) hw.arm_key = !hw.arm_key;
    if (rng.oneIn(5000)) hw.estop_ok = false;
    if (!hw.estop_ok && rng.oneIn(300)) hw.estop_ok = true;

    // Random hostile request mask, frequently all-on.
    uint16_t requested = rng.oneIn(3) ? 0xFFFF : (uint16_t)rng.next();

    uint16_t mask = safety.filter(requested, in, hw, now);
    checker.observe(safety.state(), hw.estop_ok, mask, now);
    if (!checker.ok()) break;

    // Occasionally clear a lockout the legitimate way so ARMED gets
    // exercised again.
    if (safety.state() == SafetyState::FAULT_LOCKOUT && rng.oneIn(100)) {
      hw.estop_ok = true;
      in.link_ok = true;
      in.arm_a = in.arm_b = false;
    }
  }

  if (!checker.ok()) std::printf("fuzz invariant: %s\n", checker.message());
  EXPECT_TRUE(checker.ok());
}
