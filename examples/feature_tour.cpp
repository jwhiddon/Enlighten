// ENLIGHTEN FEATURE TOUR
//
// A narrated, host-runnable walk through every feature of the firmware,
// driving the REAL core (the exact code flashed to the Mega) through the
// same pipeline harness the tests use.  Build & run:
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

// Ticks a pipeline and prints state transitions and output changes.
template <typename P>
struct Narrator {
  P& p;
  SafetyState ls;
  uint16_t lm;
  TimeMs start;
  bool show_masks = true;

  explicit Narrator(P& pp)
      : p(pp), ls(pp.safety.state()), lm(pp.mask), start(pp.now) {}

  void stamp() {
    std::printf("  t=%7lums  ", (unsigned long)elapsedMs(start, p.now));
  }

  template <typename F>
  void runWith(uint32_t ms, F pre) {
    while (ms--) {
      pre();
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

  void run(uint32_t ms) {
    runWith(ms, [] {});
  }

  // Quiet run that just counts distinct firing steps.
  uint32_t countSteps(uint32_t ms) {
    bool saved = show_masks;
    show_masks = false;
    uint32_t steps = 0;
    uint16_t prev = p.mask;
    while (ms--) {
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

}  // namespace

int main() {
  std::printf("==============================================================\n");
  std::printf(" ENLIGHTEN FEATURE TOUR - real firmware core, simulated clock\n");
  std::printf("==============================================================\n");

  // ---------------------------------------------------------------- 1
  section("1. Boot: self-test, and what a watchdog reset looks like");
  {
    DmxPipeline p;
    Narrator<DmxPipeline> n(p);
    note("clean power-on: supervisor enters SAFE");
    n.run(3);
    check(p.safety.state() == SafetyState::SAFE, "boots to SAFE, outputs closed");

    DmxPipeline pw(0, FaultCode::WATCHDOG_RESET);
    // Mid-show crash: the console still has the arm scene up.
    pw.ch[0] = cfg::DMX_ARM_A; pw.ch[1] = cfg::DMX_ARM_B;
    Narrator<DmxPipeline> nw(pw);
    note("boot after a firmware hang mid-show (watchdog reset): LED blinks 1");
    nw.run(2000);
    check(pw.safety.state() == SafetyState::FAULT_LOCKOUT,
          "locked out; the held arm scene cannot clear or re-arm it");
    note("operator acknowledges: drops both arm values at the console");
    pw.ch[0] = 0; pw.ch[1] = 0;
    nw.run(5);
    check(pw.safety.state() == SafetyState::SAFE, "fault acknowledged -> SAFE");
  }

  // ---------------------------------------------------------------- 2
  section("2. Arming security: everything that must NOT arm, then the real thing");
  {
    DmxPipeline p;
    Narrator<DmxPipeline> n(p);

    note("attempt 1: correct DMX handshake but the panel keyswitch is OFF");
    p.hw.arm_key = false;
    p.ch[0] = cfg::DMX_ARM_A; p.ch[1] = cfg::DMX_ARM_B;
    n.run(2000);
    check(p.safety.state() == SafetyState::SAFE, "protocol alone cannot arm");
    p.ch[0] = 0; p.ch[1] = 0;
    p.hw.arm_key = true;
    n.run(5);

    note("attempt 2: console patched in with the arm scene already saved");
    DmxPipeline scene;
    scene.ch[0] = cfg::DMX_ARM_A; scene.ch[1] = cfg::DMX_ARM_B;
    Narrator<DmxPipeline> ns(scene);
    ns.run(3000);
    check(scene.safety.state() == SafetyState::SAFE,
          "saved scene never arms (rising edge required)");

    note("attempt 3: fader sweep passes through the arm window (~11 ms < 500 ms hold)");
    p.ch[1] = cfg::DMX_ARM_B;
    for (int v = 0; v <= 255; ++v) { p.ch[0] = (uint8_t)v; n.run(1); }
    p.ch[0] = 0; p.ch[1] = 0;
    n.run(5);
    check(p.safety.state() == SafetyState::SAFE, "sweep cannot arm (500 ms hold)");

    note("the real procedure: values down -> 85 + 170 held 500 ms, key ON");
    p.ch[0] = cfg::DMX_ARM_A; p.ch[1] = cfg::DMX_ARM_B;
    n.run(cfg::ARM_HOLD_MS + 10);
    check(p.safety.state() == SafetyState::ARMED, "deliberate handshake arms");
  }

  // ---------------------------------------------------------------- 3
  section("3. Mode banding: a fader sweep cannot glitch-fire a mode");
  {
    DmxPipeline p;
    Narrator<DmxPipeline> n(p);
    p.armFromConsole();
    for (int i = 8; i < 24; ++i) p.ch[i] = 255;  // everything enabled!
    note("sweeping the MODE fader 0 -> 255 with all poofers enabled");
    bool fired = false;
    for (int v = 0; v <= 255; ++v) {
      p.ch[2] = (uint8_t)v;
      n.run(1);
      if (p.mask != 0) fired = true;
    }
    p.ch[2] = 255;
    check(!fired, "no poofer fired during the sweep (guards + 150 ms debounce)");
  }

  // ---------------------------------------------------------------- 4
  section("4. Effect modes (poof 100 ms / rest 45 ms, all poofers enabled)");
  {
    DmxPipeline p;
    p.armFromConsole();
    p.ch[3] = 38;   // ~100 ms poof
    p.ch[4] = 0;    // min rest
    p.ch[6] = 255;  // repeat
    for (int i = 8; i < 24; ++i) p.ch[i] = 255;
    Narrator<DmxPipeline> n(p);

    note("CHASE_UP: poofers 1..8 in order");
    p.ch[2] = 30;
    n.run(cfg::MODE_DEBOUNCE_MS + 1300);

    note("CHASE_IN: mirrored pairs, outside -> in: (1,8)(2,7)(3,6)(4,5)");
    p.ch[2] = 110;
    n.run(cfg::MODE_DEBOUNCE_MS + 700);

    note("CHASE_UP_DOWN: ping-pong, endpoints fire once");
    p.ch[2] = 70;
    n.run(cfg::MODE_DEBOUNCE_MS + 2300);

    note("ALTERNATE: evens / odds");
    p.ch[2] = 190;
    n.run(cfg::MODE_DEBOUNCE_MS + 500);

    note("FIRE_ALL: everything at once (still duty-limited per channel)");
    p.ch[2] = 230;
    n.run(cfg::MODE_DEBOUNCE_MS + 400);
    check(p.checker.ok(), "invariants held through every mode");
  }

  // ---------------------------------------------------------------- 5
  section("5. Choreography controls: rate fader and one-shot vs repeat");
  {
    DmxPipeline p;
    p.armFromConsole();
    p.ch[2] = 30;   // CHASE_UP
    p.ch[3] = 38;
    p.ch[4] = 128;  // ~1 s rest
    p.ch[6] = 255;
    for (int i = 8; i < 16; ++i) p.ch[i] = 255;
    Narrator<DmxPipeline> n(p);
    n.run(cfg::MODE_DEBOUNCE_MS + 5);

    p.ch[5] = 128;  // rate 1.0x
    uint32_t slow = n.countSteps(5000);
    p.ch[5] = 255;  // max tempo
    uint32_t fast = n.countSteps(5000);
    std::printf("  rate 128 (1.0x): %lu steps in 5 s;  rate 255: %lu steps in 5 s\n",
                (unsigned long)slow, (unsigned long)fast);
    check(fast > slow * 2, "RATE fader scales chase tempo");

    note("repeat OFF: FIRE_ALL is a one-shot");
    p.ch[2] = 10;  // pass through OFF while reconfiguring
    n.countSteps(cfg::MODE_DEBOUNCE_MS + 200);
    p.ch[6] = 0;  // repeat off
    for (int i = 8; i < 24; ++i) p.ch[i] = 255;
    p.ch[2] = 230;
    uint32_t shots = n.countSteps(cfg::MODE_DEBOUNCE_MS + 4000);
    std::printf("  one-shot produced %lu poof(s) in 4 s\n", (unsigned long)shots);
    check(shots == 1, "one poof, then idle until re-triggered");
  }

  // ---------------------------------------------------------------- 6
  section("6. The physical backstop: duty limits vs a held RAW trigger");
  {
    DmxPipeline p;
    p.armFromConsole();
    p.ch[2] = 250;  // RAW
    p.ch[3] = 255;  // poof fader maxed: 500 ms
    p.ch[6] = 255;  // repeat: refire as fast as allowed
    Narrator<DmxPipeline> n(p);
    n.run(cfg::MODE_DEBOUNCE_MS + 5);
    note("holding poofer 1's trigger at full for one 10-second budget window...");
    p.ch[8] = 255;
    uint32_t open_ms = 0;
    for (int i = 0; i < 10000; ++i) {
      n.run(1);
      if (p.mask & 1) ++open_ms;
    }
    std::printf("  total open time: %lu ms of 10000 ms demanded (%.0f%%)\n",
                (unsigned long)open_ms, 100.0 * open_ms / 10000.0);
    check(open_ms <= cfg::MAX_OPEN_PER_WINDOW_MS + 10,
          "duty budget capped a maxed-out trigger at 30% of the window");
    note("...and as the window slides, the budget refills (still 30% max):");
    n.run(2500);  // watch a refill open appear
    check(p.checker.ok(), "every open <= 500 ms with >= 50 ms forced closes");
  }

  // ---------------------------------------------------------------- 7
  section("7. Deadman: cable pull mid-show");
  {
    DmxPipeline p;
    p.armFromConsole();
    p.ch[2] = 230; p.ch[6] = 255; p.ch[3] = 128;
    for (int i = 8; i < 24; ++i) p.ch[i] = 255;
    Narrator<DmxPipeline> n(p);
    n.run(cfg::MODE_DEBOUNCE_MS + 300);

    note("DMX cable pulled while firing...");
    p.signal = false;
    uint32_t ms_to_safe = 0;
    while ((p.mask != 0 || p.safety.state() == SafetyState::ARMED) &&
           ms_to_safe < 5000) {
      n.run(1);
      ++ms_to_safe;
    }
    std::printf("  outputs closed and disarmed %lu ms after the last packet\n",
                (unsigned long)ms_to_safe);
    check(ms_to_safe <= cfg::DMX_TIMEOUT_MS + 2, "deadman within the timeout");

    note("cable restored - console scene still holds the arm values");
    p.signal = true;
    n.run(3000);
    check(p.safety.state() == SafetyState::SAFE,
          "NO auto-re-arm on reconnection (operator must cycle the values)");
  }

  // ---------------------------------------------------------------- 8
  section("8. E-stop: overrides everything, latches, requires acknowledgment");
  {
    DmxPipeline p;
    p.armFromConsole();
    p.ch[2] = 230; p.ch[6] = 255; p.ch[3] = 128;
    for (int i = 8; i < 24; ++i) p.ch[i] = 255;
    Narrator<DmxPipeline> n(p);
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

    note("operator drops the arm values to acknowledge, then re-arms");
    p.ch[0] = 0; p.ch[1] = 0;
    n.run(5);
    p.armFromConsole();
    check(p.safety.state() == SafetyState::ARMED, "full handshake re-arms");
    check(p.checker.ok(), "invariants held throughout");
  }

  // ---------------------------------------------------------------- 9
  section("9. MIDI: same safety core, played from a keyboard");
  {
    MidiPipeline m;
    Narrator<MidiPipeline> n(m);
    auto alive = [&] { m.feed({0xFE}); };  // device sends Active Sensing

    note("arming via CC20=85, CC21=106 (7-bit MIDI arm values)");
    n.runWith(5, alive);
    m.feed({0xB0, 20, cfg::MIDI_ARM_A});
    m.feed({0xB0, 21, cfg::MIDI_ARM_B});
    n.runWith(cfg::ARM_HOLD_MS + 10, alive);
    check(m.safety.state() == SafetyState::ARMED, "MIDI handshake arms");

    m.feed({0xB0, 22, 125});  // RAW mode
    n.runWith(cfg::MODE_DEBOUNCE_MS + 10, alive);

    note("playing a chord: notes 36, 39, 43 -> poofers 1, 4, 8");
    m.feed({0x90, 36, 127, 39, 127, 43, 127});  // running status
    n.runWith(150, alive);

    note("velocity 40 (below the 64 gate): ignored - no half-triggers");
    m.feed({0x90, 37, 40});
    n.runWith(100, alive);
    check((m.mask & 0x0002) == 0, "soft note did not fire");

    note("CC123 (all notes off) = panic: gates cleared AND disarmed");
    m.feed({0x90, 36, 127});
    n.runWith(20, alive);
    m.feed({0xB0, 123, 0});
    n.runWith(5, alive);
    check(m.mask == 0 && m.safety.state() == SafetyState::SAFE, "panic works");

    note("re-arm, then pull the MIDI cable (Active Sensing stops)");
    m.feed({0xB0, 20, 0});
    n.runWith(5, alive);
    m.feed({0xB0, 20, cfg::MIDI_ARM_A});
    m.feed({0xB0, 21, cfg::MIDI_ARM_B});
    n.runWith(cfg::ARM_HOLD_MS + 10, alive);
    n.run(cfg::MIDI_AS_TIMEOUT_MS + 20);  // no more heartbeats
    check(m.safety.state() == SafetyState::SAFE,
          "cable pull disarms in ~330 ms via Active Sensing");
    check(m.checker.ok(), "invariants held on the MIDI path");
  }

  // ---------------------------------------------------------------- 10
  section("10. The 49.7-day clock rollover (the legacy firmware's nemesis)");
  {
    DmxPipeline p(0xFFFFFFFFu - 5000);  // 5 s before millis() wraps
    p.armFromConsole();
    p.ch[2] = 230; p.ch[6] = 255; p.ch[3] = 128;
    for (int i = 8; i < 24; ++i) p.ch[i] = 255;
    Narrator<DmxPipeline> n(p);
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
