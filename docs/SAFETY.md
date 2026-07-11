# Enlighten Safety Design

Enlighten drives 16 propane poofer solenoids. This document describes the
firmware safety architecture, maps it to NFPA 160 (Standard for the Use of
Flame Effects Before an Audience) concepts, and — critically — lists what
firmware **cannot** provide and must exist in hardware.

> Firmware is one layer of a flame-effect safety system, never the only one.
> Review docs/HARDWARE.md for the required electrical/mechanical safeguards.

## Architecture: one authority, one output path

```
MIDI bytes → MidiDecoder ──┐
Bench console / SD player ─┴→ ShowInput → Sequencer → requested mask
HwInputs (E-stop, arm key) ─────────────┐              │
                      SafetySupervisor::filter()   ← FINAL authority
                                        │
                                        ▼
                      Board::writeOutputs(mask)    ← only output call site
```

Grep-auditable invariants:

* `PORTA`/`PORTC` appear **only** in `src/boards/mega2560/board.cpp`.
* `Board::writeOutputs()` is called from **one** place (`Enlighten.ino`),
  always with `SafetySupervisor::filter()`'s return value.
* `filter()` returns 0 in every state except ARMED. A freshly constructed
  supervisor is safe by definition (unit-tested).

## Safety state machine

```
BOOT_SELFTEST ──ok──────────→ SAFE ──handshake held 500 ms──→ ARM_PENDING → ARMED
      │                        ↑                                            │
      └──fault──→ FAULT_LOCKOUT┘←──── E-stop / faults (from any state) ←────┘
```

| State | Outputs | Meaning |
|---|---|---|
| BOOT_SELFTEST | forced closed | startup checks running |
| SAFE | forced closed | disarmed idle |
| ARM_PENDING | forced closed | handshake seen; 500 ms hold timer running |
| ARMED | requests honored **through per-channel duty limits** | firing permitted |
| FAULT_LOCKOUT | forced closed | latched fault; explicit clear required |

### Arming (supervised, deliberate)

ALL of the following, simultaneously:

1. Physical **arm keyswitch** closed (panel key).
2. **E-stop loop intact** (NC circuit; a broken wire reads as pressed).
3. **Signal present** (MIDI heartbeat alive).
4. Two-value handshake: CC20 = 85 and CC21 = 106 — two distinct mid-scale
   magic values that a knob sweep, all-zeros, or all-full state can never
   produce together.
5. Held continuously for **500 ms**.
6. **Rising edge**: the supervisor must have seen the arm values
   de-asserted **over a live link** since it last entered SAFE. A console
   scene saved with the arm values in it can NEVER arm the system at
   patch-in, and a signal blip or cable pull while armed requires the
   operator to explicitly cycle the arm values — a dead link is not
   evidence of de-assertion.

### Disarm / deadman (continuous permission)

* Either arm value changing, the keyswitch opening, or signal loss
  (Active Sensing lapse >330 ms / 2 s byte silence) → SAFE **in the same
  loop iteration** (sub-millisecond).
* E-stop assertion → FAULT_LOCKOUT in the same iteration. Releasing the
  E-stop alone re-enables nothing.

### Fault lockout and acknowledgment

FAULT_LOCKOUT is latched. The LED blinks the fault code (N blinks + 1 s
pause). Clearing requires E-stop released **and** the console link up with
both arm values explicitly de-asserted — so the fault code stays visible
until an operator acknowledges it — followed by a full fresh arm handshake.

Fault codes (`src/core/faults.h`):

| Blinks | Code | Cause |
|---|---|---|
| 1 | WATCHDOG_RESET | firmware hang forced a hardware reset |
| 2 | BROWNOUT_RESET | supply dipped below the brownout threshold |
| 3 | SELFTEST_RAM | timebase failed to advance at boot |
| 4 | SELFTEST_OUTPUTS | output-port readback mismatch at boot |
| 5 | SELFTEST_ESTOP | E-stop loop open at boot (pressed or broken wire) |
| 6 | ESTOP_ASSERTED | E-stop pressed in service |
| 7 | LOOP_OVERRUN | main loop exceeded its 5 ms budget |

## Independent physical duty limits (DutyLimiter)

Enforced per channel inside the supervisor, regardless of what any decoder
or the sequencer requests — including hostile/corrupt input:

* **MAX_OPEN_MS = 500** — absolute cap on continuous open time.
* **MIN_CLOSE_MS = 50** — forced close between opens.
* **Duty budget** — at most 3000 ms of open time in any sliding 10 s window
  (30%), implemented with 11 one-second buckets so enforcement is always
  conservative for a true sliding window.

## Watchdog supervision

* `wdt_enable(WDTO_120MS)` after self-test; fed at exactly one place (end
  of `loop()`), and **only if the safety filter ran** since the last feed.
  A code path that skips the filter starves the watchdog.
* The supervisor also latches LOOP_OVERRUN if >5 ms elapses between filter
  calls.
* Reset cause is captured in `.init3` (before the runtime starts) and a
  watchdog/brownout reset boots directly into FAULT_LOCKOUT with the cause
  on the LED. The legacy behavior of using resets for signal recovery is
  gone: signal loss is a disarm, not a reboot.

## Boot self-test

1. Reset cause check (watchdog/brownout → lockout).
2. Output-port readback (stuck-low driver bit → lockout).
3. E-stop loop intact.
4. Timebase advances.
5. Boot signature on the LED: 2 blinks (normal service) or 3 (bench mode).

## NFPA 160 concept mapping

NFPA 160 is a standard about the *whole effect*: fuel train, supervision,
operators, and documentation — not just the controller. This table maps the
controller-relevant concepts to their mechanism. Compliance of the overall
effect requires the hardware items and operational controls too, plus AHJ
(authority having jurisdiction) approval.

| NFPA 160 concept | Mechanism | Where |
|---|---|---|
| Emergency stop, independent of software | Hardwired NC E-stop in series with valve supply **plus** monitored pin | HARDWARE.md + `SafetySupervisor` (test: `estop_overrides_every_state`) |
| Deadman / continuous operator permission | Arm handshake + keyswitch + signal deadman; loss → safe same loop | `SafetySupervisor` (tests: `signal_loss_closes_next_tick`, `disarm_is_immediate`) |
| Supervised, deliberate arming | Two-value handshake, 500 ms hold, rising edge, physical key | `SafetySupervisor` (tests: `arm_requires_rising_edge`, `arm_requires_continuous_hold`) |
| Bounded effect duration | MAX_OPEN_MS + duty budget independent of inputs | `DutyLimiter` (tests: `duty_*`, `buggy_decoder_poof_cannot_exceed_hard_cap`) |
| Failsafe on power loss / crash | Active-low outputs + external pull-ups + watchdog + boot-to-lockout | board layer + HARDWARE.md |
| Pre-operation checkout | Boot self-test; bench checklist | this doc + HARDWARE.md |
| Pilot/flame supervision | **NOT in firmware — required external hardware** | HARDWARE.md |
| Fuel pressure limits (high/low cutoffs) | **NOT in firmware — required external hardware** | HARDWARE.md |
| Manual fuel shutoff | **NOT in firmware — required external hardware** | HARDWARE.md |

## Executable safety contract

`test/` holds the proof — 140 tests across five layers:

1. **Unit tests** per module (timing, duty limiting, state machine,
   sequencer, mode banding, MIDI parser/decoder, display, bench console,
   SD show player, panel UI, status LED patterns).
2. **End-to-end pipeline scenarios** (`test_pipeline_midi.cpp`): raw MIDI
   byte streams through parser → decoder → sequencer → safety exactly as
   `Enlighten.ino` wires them — arming procedures, E-stop/acknowledge
   cycles, cable pulls, latched-state connect, knob sweeps.
3. **Rollover tests** (`test_rollover.cpp`): every stateful component
   driven straight through the 2^32 millis() wrap.
4. **Fuzzers**: 2M ticks against the safety filter with hostile request
   masks, 500k hostile bytes against the MIDI parser, 500k ticks of
   random wire garbage and hardware events against the full pipeline
   (crossing the rollover), and mode-thrash against the sequencer — all
   asserting the invariants every tick.
5. **Meta-tests** (`test_checker.cpp`) proving the invariant checker itself
   catches each violation class — the guard rails are tested too.

Run with `test/run_tests.ps1` (Windows) or `test/run_tests.sh`.
