# Enlighten

Firmware and support files for the Enlighten Poofer Project — a 16-channel
propane flame-effect ("poofer") controller with NFPA 160-aligned safeguards,
driven by **DMX or MIDI** (selectable at boot).

## Layout

| Path | What |
|---|---|
| `Enlighten/` | The firmware sketch (Arduino Mega 2560). Thin `.ino` shell; all logic in `src/core/` (portable, host-tested) and `src/boards/` (hardware) |
| `test/` | Host-side unit tests — the executable safety contract. `run_tests.ps1` / `run_tests.sh` (needs g++) |
| `examples/` | **Feature tour**: narrated, runnable walkthrough of every feature on a simulated clock — `examples/run_demo.ps1` |
| `tools/` | **Show tools**: `seqgen` composes duty-limit-verified shows (`--seconds`, `--poofers`) as `.show` + playable `.mid`; `showplay` previews/validates them |
| `docs/` | **[MANUAL.md](docs/MANUAL.md) — start here** · [SAFETY.md](docs/SAFETY.md) · [DMX_MAP.md](docs/DMX_MAP.md) · [MIDI_MAP.md](docs/MIDI_MAP.md) · [HARDWARE.md](docs/HARDWARE.md) |
| `Freestyler/` | Freestyler fixture profile for the v2 DMX map |
| `legacy/` | The original single-file EnlightenDMX sketch (frozen) |

## Safety model (short version)

One pipeline, one authority: protocol decoders produce a common command
model, the sequencer proposes, and the **SafetySupervisor** disposes — it is
the only code that decides what opens, and it enforces supervised arming
(two-value handshake + physical keyswitch + 500 ms hold + rising edge),
deadman signal supervision, E-stop lockout, watchdog supervision, and
per-channel duty limits (max 500 ms open, forced 50 ms close, 30% duty
budget) no matter what the inputs ask for. **Read
[docs/SAFETY.md](docs/SAFETY.md) and [docs/HARDWARE.md](docs/HARDWARE.md)
before deploying** — required E-stop/keyswitch wiring and fuel-train
hardware are documented there.

## Building

```
arduino-cli core install arduino:avr
arduino-cli lib install DMXSerial SdFat
arduino-cli compile --fqbn arduino:avr:mega Enlighten
```

MIDI is the primary protocol (bare boot); DMX needs the D4 jumper. With
an SD module, generated shows play standalone — no PC at the show
([MANUAL §7.4](docs/MANUAL.md)).

## Testing

```
test\run_tests.ps1     # Windows (g++ via MinGW/WinLibs)
test/run_tests.sh      # POSIX
```

116 tests: per-module units, end-to-end DMX/MIDI pipeline scenarios (real
channel frames and byte streams through the exact production wiring),
millis()-rollover coverage, five fuzzers (safety filter, MIDI parser, DMX
decoder, full pipeline across the clock wrap, sequencer mode-thrash), and
meta-tests that prove the invariant checker itself catches violations. The
invariants — never open unless armed, never open past the duty limits,
E-stop always wins — are asserted on every simulated millisecond.

## Dependencies

* [Arduino DMXSerial library](https://github.com/mathertel/DMXSerial)
