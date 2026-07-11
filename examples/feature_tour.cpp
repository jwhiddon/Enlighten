// ENLIGHTEN FEATURE TOUR
//
// A narrated, host-runnable walk through every feature of the firmware,
// driving the REAL core (the exact code flashed to the Mega) with raw
// MIDI bytes through the same pipeline harness the tests use.  Build & run:
//
//   examples\run_demo.ps1      (Windows)
//   examples/run_demo.sh       (POSIX)
//
// Output: a timeline.  Poofer bars show all 16 channels, poofer 1 leftmost:
//   [#.......|........]  = poofer 1 open       [...##...|........] = 4+5 open
#include <cstdio>

#include "pipeline.h"

namespace {

bool g_all_ok = true;
unsigned long g_ticks = 0;

const char* stateName(SafetyState s) {
  switch (s) {
    case SafetyState::BOOT_SELFTEST: return "BOOT_SELFTEST";
    case SafetyState::SAFE:          return "SAFE";
    case SafetyState::ARM_PENDING:   return "ARM_PENDING";
    case SafetyState::ARMED:         return "ARMED";
    case SafetyState::FAULT_LOCKOUT: return "FAULT_LOCKOUT";
  }
  return "?";
}

const char* ledDesc(SafetyState s, FaultCode f, bool firing) {
  static char buf[64];
  switch (s) {
    case SafetyState::BOOT_SELFTEST: return "solid on";
    case SafetyState::SAFE:          return "slow 1 Hz blink";
    case SafetyState::ARM_PENDING:   return "double-blink burst";
    case SafetyState::ARMED:
      return firing ? "solid on (firing)" : "fast 4 Hz blink";
    case SafetyState::FAULT_LOCKOUT:
      std::snprintf(buf, sizeof(buf), "%d blinks + pause (fault code)",
                    (int)f);
      return buf;
  }
  return "?";
}

void maskBar(uint16_t m, char* out) {
  int pos = 0;
  out[pos++] = '[';
  for (int i = 0; i < 16; ++i) {
    if (i == 8) out[pos++] = '|';
    out[pos++] = ((m >> i) & 1) ? '#' : '.';
  }
  out[pos++] = ']';
  out[pos] = 0;
}

void section(const char* title) {
  std::printf("\n============================================================\n");
  std::printf(" %s\n", title);
  std::printf("============================================================\n");
}

void note(const char* text) { std::printf("  -- %s\n", text); }

// Ticks the pipeline and prints state transitions and output changes.
struct Narrator {
  MidiPipeline& p;
  SafetyState ls;
  uint16_t lm;
  TimeMs start;
  bool show_masks = true;
  bool heartbeat = true;  // send Active Sensing each tick (device present)

  explicit Narrator(MidiPipeline& pp)
      : p(pp), ls(pp.safety.state()), lm(pp.mask), start(pp.now) {}

  void stamp() {
    std::printf("  t=%7lums  ", (unsigned long)elapsedMs(start, p.now));
  }

  void run(uint32_t ms) {
    while (ms--) {
      if (heartbeat) p.feed({0xFE});
      p.tick();
      ++g_ticks;
      if (p.safety.state() != ls) {
        stamp();
        std::printf("STATE %-13s -> %-13s LED: %s\n", stateName(ls),
                    stateName(p.safety.state()),
                    ledDesc(p.safety.state(), p.safety.fault(), p.mask != 0));
        ls = p.safety.state();
      }
      if (show_masks && p.mask != lm) {
        char b[20];
        maskBar(p.mask, b);
        stamp();
        std::printf("%s%s\n", b, p.mask == 0 ? "  (all closed)" : "");
        lm = p.mask;
      }
    }
  }

  // Quiet run that just counts distinct firing steps.
  uint32_t countSteps(uint32_t ms) {
    bool saved = show_masks;
    show_masks = false;
    uint32_t steps = 0;
    uint16_t prev = p.mask;
    while (ms--) {
      if (heartbeat) p.feed({0xFE});
      p.tick();
      ++g_ticks;
      if (p.mask != 0 && p.mask != prev) ++steps;
      prev = p.mask;
    }
    show_masks = saved;
    lm = p.mask;
    ls = p.safety.state();
    return steps;
  }
};

void check(bool ok, const char* what) {
  std::printf("  %s %s\n", ok ? "[OK]  " : "[FAIL]", what);
  if (!ok) g_all_ok = false;
}

void cycleArm(MidiPipeline& p, Narrator& n) {
  p.feed({0xB0, 20, 0});
  n.run(5);
  p.feed({0xB0, 20, cfg::MIDI_ARM_A});
  p.feed({0xB0, 21, cfg::MIDI_ARM_B});
  n.run(cfg::ARM_HOLD_MS + 10);
}

}  // namespace

int main() {
  std::printf("==============================================================\n");
  std::printf(" ENLIGHTEN FEATURE TOUR - real firmware core, simulated clock\n");
  std::printf("==============================================================\n");

  // ---------------------------------------------------------------- 1
  section("1. Boot: clean start, and what a watchdog reset looks like");
  {
    MidiPipeline p;
    Narrator n(p);
    note("clean power-on: supervisor enters SAFE");
    n.run(3);
    check(p.safety.state() == SafetyState::SAFE, "boots to SAFE, outputs closed");

    // Mid-show crash: the controller still has the arm CCs latched.
    MidiPipeline pw(0, FaultCode::WATCHDOG_RESET);
    pw.feed({0xB0, 20, cfg::MIDI_ARM_A});
    pw.feed({0xB0, 21, cfg::MIDI_ARM_B});
    Narrator nw(pw);
    note("boot after a firmware hang mid-show (watchdog reset): LED blinks 1");
    nw.run(2000);
    check(pw.safety.state() == SafetyState::FAULT_LOCKOUT,
          "locked out; the latched arm CCs cannot clear or re-arm it");
    note("operator acknowledges: drops BOTH arm values");
    pw.feed({0xB0, 20, 0});
    pw.feed({0xB0, 21, 0});
    nw.run(5);
    check(pw.safety.state() == SafetyState::SAFE, "fault acknowledged -> SAFE");
  }

  // ---------------------------------------------------------------- 2
  section("2. Arming security: everything that must NOT arm, then the real thing");
  {
    note("attempt 1: correct CC handshake but the panel keyswitch is OFF");
    MidiPipeline p;
    Narrator n(p);
    p.hw.arm_key = false;
    n.run(5);
    p.feed({0xB0, 20, cfg::MIDI_ARM_A});
    p.feed({0xB0, 21, cfg::MIDI_ARM_B});
    n.run(2000);
    check(p.safety.state() == SafetyState::SAFE, "protocol alone cannot arm");
    p.hw.arm_key = true;

    note("attempt 2: controller connects with the arm CCs already latched");
    MidiPipeline scene;
    scene.feed({0xB0, 20, cfg::MIDI_ARM_A});
    scene.feed({0xB0, 21, cfg::MIDI_ARM_B});
    Narrator ns(scene);
    ns.run(3000);
    check(scene.safety.state() == SafetyState::SAFE,
          "latched state never arms (rising edge required)");

    note("attempt 3: a knob sweep passes through 85 (~1 ms < 500 ms hold)");
    for (int v = 0; v <= 127; ++v) {
      p.feed({0xB0, 20, (uint8_t)v});
      n.run(1);
    }
    n.run(600);
    check(p.safety.state() == SafetyState::SAFE, "sweep cannot arm (500 ms hold)");

    note("the real procedure: values down -> CC20=85 + CC21=106 held 500 ms");
    cycleArm(p, n);
    check(p.safety.state() == SafetyState::ARMED, "deliberate handshake arms");
  }

  // ---------------------------------------------------------------- 3
  section("3. Mode banding: a mode-knob sweep cannot glitch-fire");
  {
    MidiPipeline p;
    Narrator n(p);
    p.armViaCc();
    p.holdNotes();  // every poofer enabled!
    note("sweeping CC22 (mode) 0 -> 127 with all poofers enabled");
    bool fired = false;
    for (int v = 0; v <= 127; ++v) {
      p.feed({0xB0, 22, (uint8_t)v});
      n.run(2);
      if (p.mask != 0) fired = true;
    }
    check(!fired, "no poofer fired during the sweep (guards + 150 ms debounce)");
  }

  // ---------------------------------------------------------------- 4
  section("4. Effect modes (poof ~100 ms, min rest, all poofers enabled)");
  {
    MidiPipeline p;
    p.armViaCc();
    p.feed({0xB0, 23, 20});   // ~104 ms poof
    p.feed({0xB0, 24, 0});    // min rest
    p.feed({0xB0, 26, 127});  // repeat
    p.holdNotes();
    Narrator n(p);

    note("CHASE_UP: poofers 1..8 in order");
    p.feed({0xB0, 22, 15});
    n.run(cfg::MODE_DEBOUNCE_MS + 1300);

    note("CHASE_IN: mirrored pairs, outside -> in: (1,8)(2,7)(3,6)(4,5)");
    p.feed({0xB0, 22, 55});
    n.run(cfg::MODE_DEBOUNCE_MS + 700);

    note("CHASE_UP_DOWN: ping-pong, endpoints fire once");
    p.feed({0xB0, 22, 35});
    n.run(cfg::MODE_DEBOUNCE_MS + 2300);

    note("ALTERNATE: evens / odds");
    p.feed({0xB0, 22, 95});
    n.run(cfg::MODE_DEBOUNCE_MS + 500);

    note("FIRE_ALL: everything at once (still duty-limited per channel)");
    p.feed({0xB0, 22, 115});
    n.run(cfg::MODE_DEBOUNCE_MS + 400);
    check(p.checker.ok(), "invariants held through every mode");
  }

  // ---------------------------------------------------------------- 5
  section("5. Choreography controls: rate CC and one-shot vs repeat");
  {
    MidiPipeline p;
    p.armViaCc();
    p.feed({0xB0, 22, 15});   // CHASE_UP
    p.feed({0xB0, 23, 20});   // ~104 ms poof
    p.feed({0xB0, 24, 64});   // ~1 s rest
    p.feed({0xB0, 26, 127});  // repeat
    p.holdNotes(1, 8);
    Narrator n(p);
    n.run(cfg::MODE_DEBOUNCE_MS + 5);

    p.feed({0xB0, 25, 64});  // rate 1.0x
    uint32_t slow = n.countSteps(5000);
    p.feed({0xB0, 25, 127});  // max tempo
    uint32_t fast = n.countSteps(5000);
    std::printf("  rate 64 (1.0x): %lu steps in 5 s;  rate 127: %lu steps in 5 s\n",
                (unsigned long)slow, (unsigned long)fast);
    check(fast > slow * 2, "RATE CC scales chase tempo");

    note("repeat OFF: FIRE_ALL is a one-shot");
    p.feed({0xB0, 22, 5});  // pass through OFF while reconfiguring
    n.countSteps(cfg::MODE_DEBOUNCE_MS + 200);
    p.feed({0xB0, 26, 0});  // repeat off
    p.holdNotes();
    p.feed({0xB0, 22, 115});
    uint32_t shots = n.countSteps(cfg::MODE_DEBOUNCE_MS + 4000);
    std::printf("  one-shot produced %lu poof(s) in 4 s\n", (unsigned long)shots);
    check(shots == 1, "one poof, then idle until re-triggered");
  }

  // ---------------------------------------------------------------- 6
  section("6. The physical backstop: duty limits vs a held trigger");
  {
    MidiPipeline p;
    p.armViaCc();
    p.feed({0xB0, 22, 125});  // RAW
    p.feed({0xB0, 23, 127});  // poof maxed: 500 ms
    p.feed({0xB0, 26, 127});  // repeat: refire as fast as allowed
    Narrator n(p);
    n.run(cfg::MODE_DEBOUNCE_MS + 5);
    note("holding poofer 1's note at full velocity for one 10 s budget window...");
    p.feed({0x90, cfg::MIDI_NOTE_FIRST, 127});
    uint32_t open_ms = 0;
    for (int i = 0; i < 10000; ++i) {
      n.run(1);
      if (p.mask & 1) ++open_ms;
    }
    std::printf("  total open time: %lu ms of 10000 ms demanded (%.0f%%)\n",
                (unsigned long)open_ms, 100.0 * open_ms / 10000.0);
    check(open_ms <= cfg::MAX_OPEN_PER_WINDOW_MS + 10,
          "duty budget capped a held note at 30% of the window");
    note("...and as the window slides, the budget refills (still 30% max):");
    n.run(2500);
    check(p.checker.ok(), "every open <= 500 ms with >= 50 ms forced closes");
  }

  // ---------------------------------------------------------------- 7
  section("7. Deadman: cable pull mid-show (Active Sensing stops)");
  {
    MidiPipeline p;
    p.armViaCc();
    p.feed({0xB0, 22, 115});  // FIRE_ALL
    p.feed({0xB0, 26, 127});
    p.feed({0xB0, 23, 64});
    p.holdNotes();
    Narrator n(p);
    n.run(cfg::MODE_DEBOUNCE_MS + 300);

    note("MIDI cable pulled while firing...");
    n.heartbeat = false;
    uint32_t ms_to_safe = 0;
    while ((p.mask != 0 || p.safety.state() == SafetyState::ARMED) &&
           ms_to_safe < 5000) {
      n.run(1);
      ++ms_to_safe;
    }
    std::printf("  outputs closed and disarmed %lu ms after the last byte\n",
                (unsigned long)ms_to_safe);
    check(ms_to_safe <= cfg::MIDI_AS_TIMEOUT_MS + 2,
          "deadman within the Active Sensing timeout");

    note("cable restored - the controller still has the arm CCs latched");
    n.heartbeat = true;
    n.run(3000);
    check(p.safety.state() == SafetyState::SAFE,
          "NO auto-re-arm on reconnection (operator must cycle the values)");
  }

  // ---------------------------------------------------------------- 8
  section("8. E-stop: overrides everything, latches, requires acknowledgment");
  {
    MidiPipeline p;
    p.armViaCc();
    p.feed({0xB0, 22, 115});
    p.feed({0xB0, 26, 127});
    p.feed({0xB0, 23, 64});
    p.holdNotes();
    Narrator n(p);
    n.run(cfg::MODE_DEBOUNCE_MS + 300);

    note("E-STOP PRESSED mid-fire");
    p.hw.estop_ok = false;
    n.run(3);
    check(p.mask == 0 && p.safety.state() == SafetyState::FAULT_LOCKOUT,
          "closed and locked out in the same loop");

    note("E-stop released: still locked out (release alone re-enables nothing)");
    p.hw.estop_ok = true;
    n.run(1500);
    check(p.safety.state() == SafetyState::FAULT_LOCKOUT, "lockout is latched");

    note("operator drops both arm values to acknowledge, then re-arms");
    p.feed({0xB0, 20, 0});
    p.feed({0xB0, 21, 0});
    n.run(5);
    cycleArm(p, n);
    check(p.safety.state() == SafetyState::ARMED, "full handshake re-arms");
    check(p.checker.ok(), "invariants held throughout");
  }

  // ---------------------------------------------------------------- 9
  section("9. Playing the rig: chords, velocity gate, and panic");
  {
    MidiPipeline p;
    Narrator n(p);
    n.run(5);
    cycleArm(p, n);
    p.feed({0xB0, 22, 125});  // RAW
    n.run(cfg::MODE_DEBOUNCE_MS + 10);

    note("playing a chord: notes 36, 39, 43 -> poofers 1, 4, 8");
    p.feed({0x90, 36, 127, 39, 127, 43, 127});  // running status
    n.run(150);

    note("velocity 40 (below the 64 gate): ignored - no half-triggers");
    p.feed({0x90, 37, 40});
    n.run(100);
    check((p.mask & 0x0002) == 0, "soft note did not fire");

    note("CC123 (all notes off) = panic: gates cleared AND disarmed");
    p.feed({0x90, 36, 127});
    n.run(20);
    p.feed({0xB0, 123, 0});
    n.run(5);
    check(p.mask == 0 && p.safety.state() == SafetyState::SAFE, "panic works");
    check(p.checker.ok(), "invariants held");
  }

  // ---------------------------------------------------------------- 10
  section("10. The 49.7-day clock rollover (the legacy firmware's nemesis)");
  {
    MidiPipeline p(0xFFFFFFFFu - 5000);  // 5 s before millis() wraps
    p.armViaCc();
    p.feed({0xB0, 22, 115});
    p.feed({0xB0, 26, 127});
    p.feed({0xB0, 23, 64});
    p.holdNotes();
    Narrator n(p);
    n.run(cfg::MODE_DEBOUNCE_MS + 5);
    uint32_t fired_after_wrap = 0;
    for (int i = 0; i < 15000; ++i) {
      n.countSteps(1);
      if (p.now < 100000 && (p.mask != 0)) ++fired_after_wrap;
    }
    std::printf("  ran 15 s straight through the 2^32 wrap; %lu ms fired after it\n",
                (unsigned long)fired_after_wrap);
    check(p.safety.state() == SafetyState::ARMED && fired_after_wrap > 0,
          "show continues seamlessly across the wrap");
    check(p.checker.ok(), "no stuck solenoid, no spurious fault");
  }

  // ---------------------------------------------------------------- fin
  std::printf("\n==============================================================\n");
  std::printf(" TOUR COMPLETE - %lu ms of virtual time simulated\n", g_ticks);
  std::printf(" Invariants watched on every tick: outputs closed unless ARMED,\n");
  std::printf(" E-stop always wins, <=500 ms opens, >=50 ms closes, 30%% duty.\n");
  std::printf(" Overall: %s\n", g_all_ok ? "ALL CHECKS PASSED" : "CHECKS FAILED");
  std::printf("==============================================================\n");
  return g_all_ok ? 0 : 1;
}
