#include <cstring>

#include "core/bench_console.h"
#include "core/safety.h"
#include "framework.h"
#include "sim.h"

namespace {
// Bench-mode pipeline: console -> sequencer -> safety, as Enlighten.ino
// wires it, with the interlocks simulated exactly as bench mode does.
struct BenchRig {
  BenchConsole con;
  Sequencer seq;
  SafetySupervisor safety;
  InvariantChecker checker;
  TimeMs now = 0;
  uint16_t mask = 0;
  char reply[96] = {};

  BenchRig() { safety.begin(FaultCode::NONE, now); }

  void type(const char* cmd) {  // sends "cmd\n"
    for (const char* p = cmd; *p; ++p) con.feed(*p, now, reply, sizeof(reply));
    con.feed('\n', now, reply, sizeof(reply));
  }

  void step(uint32_t ms) {
    HwInputs hw{true, true};  // bench mode simulates the interlocks
    while (ms--) {
      ++now;
      ShowInput in = con.snapshot(now);
      seq.update(in, now);
      mask = safety.filter(seq.requestedMask(), in, hw, now);
      checker.observe(safety.state(), hw.estop_ok, mask, now);
    }
  }
};
}  // namespace

TEST(bench_arm_goes_through_real_edge_and_hold) {
  BenchRig r;
  r.step(5);
  r.type("arm");
  r.step(100);
  EXPECT_EQ(r.safety.state(), SafetyState::ARM_PENDING);  // hold running
  r.step(cfg::ARM_HOLD_MS);
  EXPECT_EQ(r.safety.state(), SafetyState::ARMED);
  EXPECT_TRUE(r.checker.ok());
}

TEST(bench_fire_pulses_one_poof) {
  BenchRig r;
  r.step(5);
  r.type("arm");
  r.step(cfg::ARM_HOLD_MS + 100);
  r.type("poof 200");
  r.type("fire 3");
  r.step(20);
  EXPECT_EQ(r.mask, (uint16_t)0x0004);
  r.step(300);
  EXPECT_EQ(r.mask, (uint16_t)0);  // ended, no repeat while trigger dropped
  EXPECT_TRUE(r.checker.ok());
}

TEST(bench_hold_is_throttled_by_duty_limits) {
  BenchRig r;
  r.step(5);
  r.type("arm");
  r.step(cfg::ARM_HOLD_MS + 100);
  r.type("poof 500");
  r.type("repeat on");
  r.type("hold 1");
  uint32_t open_ms = 0;
  for (int i = 0; i < 10000; ++i) {
    r.step(1);
    if (r.mask & 1) ++open_ms;
  }
  EXPECT_TRUE(open_ms <= cfg::MAX_OPEN_PER_WINDOW_MS + 10);
  EXPECT_TRUE(r.checker.ok());  // bench mode cannot bypass the limits
}

TEST(bench_modes_drive_sequencer) {
  BenchRig r;
  r.step(5);
  r.type("arm");
  r.step(cfg::ARM_HOLD_MS + 100);
  r.type("mode up");
  r.type("repeat on");
  r.type("hold 1");  // enables are per-poofer in pattern modes
  for (int i = 2; i <= 8; ++i) {
    char cmd[16];
    std::snprintf(cmd, sizeof(cmd), "hold %d", i);
    r.type(cmd);
  }
  uint32_t fired = 0;
  uint16_t prev = 0;
  for (int i = 0; i < 3000; ++i) {
    r.step(1);
    if (r.mask && r.mask != prev) ++fired;
    prev = r.mask;
  }
  EXPECT_TRUE(fired >= 5);  // chase is stepping
  EXPECT_TRUE(r.checker.ok());
}

TEST(bench_stop_and_disarm) {
  BenchRig r;
  r.step(5);
  r.type("arm");
  r.step(cfg::ARM_HOLD_MS + 100);
  r.type("repeat on");
  r.type("hold 1");
  r.step(50);
  EXPECT_TRUE(r.mask != 0);
  r.type("stop");
  r.step(5);
  EXPECT_EQ(r.mask, (uint16_t)0);
  EXPECT_EQ(r.safety.state(), SafetyState::ARMED);  // stop keeps arm
  r.type("disarm");
  r.step(5);
  EXPECT_EQ(r.safety.state(), SafetyState::SAFE);
  EXPECT_TRUE(r.checker.ok());
}

TEST(bench_console_silence_deadman) {
  BenchRig r;
  r.step(5);
  r.type("arm");
  r.step(cfg::ARM_HOLD_MS + 100);
  EXPECT_EQ(r.safety.state(), SafetyState::ARMED);
  r.step(cfg::BENCH_KEEPALIVE_MS + 10);  // walk away from the bench
  EXPECT_EQ(r.safety.state(), SafetyState::SAFE);
  // Typing arm again re-arms (fresh edge).
  r.type("arm");
  r.step(cfg::ARM_HOLD_MS + 100);
  EXPECT_EQ(r.safety.state(), SafetyState::ARMED);
  EXPECT_TRUE(r.checker.ok());
}

TEST(bench_command_parsing_details) {
  BenchConsole c;
  char reply[96];
  TimeMs now = 100;
  auto run = [&](const char* cmd) {
    for (const char* p = cmd; *p; ++p) c.feed(*p, now, reply, sizeof(reply));
    return c.feed('\n', now, reply, sizeof(reply));
  };
  EXPECT_TRUE(run("HELP"));  // case-insensitive
  EXPECT_TRUE(std::strstr(BenchConsole::helpText(), "mode") != nullptr);

  run("poof 9999");  // clamped
  run("status");
  EXPECT_TRUE(std::strstr(reply, "poof:500") != nullptr);
  run("rest 1");
  run("status");
  EXPECT_TRUE(std::strstr(reply, "rest:45") != nullptr);

  run("fire 17");  // out of range
  EXPECT_TRUE(std::strstr(reply, "1-16") != nullptr);
  run("bogus");
  EXPECT_TRUE(std::strstr(reply, "?") != nullptr);

  // Unknown/garbage never asserts triggers.
  ShowInput s = c.snapshot(now);
  EXPECT_EQ(s.trigger_mask, (uint16_t)0);
  EXPECT_FALSE(s.arm_a);
}
