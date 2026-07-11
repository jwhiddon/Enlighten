# Enlighten Operator & Builder Manual

Step-by-step instructions for building, flashing, commissioning, and
operating the Enlighten 16-channel flame-effect controller. Reference
detail lives in the companion docs:

* [SAFETY.md](SAFETY.md) — the safety architecture and NFPA 160 mapping
* [HARDWARE.md](HARDWARE.md) — electrical requirements and bench checklist
* [MIDI_MAP.md](MIDI_MAP.md) — the control protocol map

> ⚠️ **This controller operates propane flame effects.** Firmware is one
> layer of the safety system. Do not connect fuel until every step of
> §5 (Commissioning) passes, the fuel-train hardware in §2.3 is in place,
> and your effect has whatever approval your venue/AHJ requires. NFPA 160
> also expects a trained operator with sightlines to the effect and the
> audience at all times.

---

## 1. System overview

```
MIDI keyboard / DAW ──DIN cable──> Arduino Mega 2560 ──> 16-relay board ──> solenoid valves
SD card (.SHW shows) ────────────────────┘  │
                                            ├─ E-stop (also hardwired to valve power)
                                            ├─ Arm keyswitch
                                            └─ Status LED + LCD + panel buttons
```

The controller will not fire unless ALL of these are true:
physical key ON, E-stop circuit intact, control signal alive, the arming
handshake sent and held, and the request within the per-channel duty
limits (max 500 ms open, 50 ms forced close, 30% duty per 10 s).

---

## 2. Hardware build

### 2.1 Parts

| Qty | Item |
|---|---|
| 1 | Arduino Mega 2560 |
| 1 | 16-channel relay board (or MOSFET board), **active-low inputs** |
| 16 | Fuel-rated solenoid valves (per your fuel-train design) |
| 1 | NC mushroom-head E-stop (2 NC contact blocks) |
| 1 | Panel keyswitch (NC to GND when enabled) |
| 1 | 6N138 optocoupler + 220 Ω/470 Ω resistors + 1N4148 (MIDI input, if used) |
| 1 | NEMA 4/4X enclosure, glands, strain reliefs |
| — | External pull-up resistors (10 kΩ) for the relay inputs if the board lacks them |

### 2.2 Wiring

**Solenoid outputs (active LOW — pin high = valve closed):**

| Poofer | Mega pin | Port bit | | Poofer | Mega pin | Port bit |
|---|---|---|---|---|---|---|
| 1 | D22 | PA0 | | 9 | D37 | PC0 |
| 2 | D23 | PA1 | | 10 | D36 | PC1 |
| 3 | D24 | PA2 | | 11 | D35 | PC2 |
| 4 | D25 | PA3 | | 12 | D34 | PC3 |
| 5 | D26 | PA4 | | 13 | D33 | PC4 |
| 6 | D27 | PA5 | | 14 | D32 | PC5 |
| 7 | D28 | PA6 | | 15 | D31 | PC6 |
| 8 | D29 | PA7 | | 16 | D30 | PC7 |

**Verify the relay board polarity and idle-state behavior**: the firmware
ships configured for **active-low** boards (`OUTPUTS_ACTIVE_LOW` in
`src/core/config.h`); flip it to `false` for typical active-high
MOSFET/SSR boards. Either way, with the Mega unplugged every channel must
stay OFF — add 10 kΩ pull-ups (active-low) or pull-downs (active-high) to
the driver inputs if not. This is what keeps the valves closed during
resets — do not skip it. If you switch to solid-state drivers, remember
semiconductors tend to fail SHORTED: the hardwired E-stop path and a
normally-closed master fuel valve matter even more.

**Control inputs:**

| Mega pin | Function | Wiring |
|---|---|---|
| D2 | E-stop monitor | One NC contact block: D2 → switch → GND. Circuit closed = OK. |
| D3 | Arm keyswitch | D3 → keyswitch → GND. Closed = arming permitted. |
| D5 | Bench mode select | Jumper to GND at boot = bench/test mode (§5b). Remove for shows. |
| D6 | PLAY button (optional) | Momentary to GND: start next / stop SD show (§7.4). |
| D7 | DISP button (optional) | Momentary to GND: cycle display pages (§2.6). |
| D8 | SEL button (optional) | Momentary to GND: page-context input (§2.6). |
| D50–D53 | SD card module (optional) | Standard SPI wiring: MISO→D50, MOSI→D51, SCK→D52, CS→D53, 5 V module or 3.3 V with level shifting. |
| D13 | Status LED | Onboard; optionally wire a panel LED (with resistor) in parallel. |
| D20/D21 | Operator LCD (optional) | **20×4** HD44780 with PCF8574 I²C backpack: SDA→D20, SCL→D21, 5 V, GND. Default address 0x27 (0x3F for PCF8574A — set in `board_pins.h`). |
| D11/D12 | Panel LED chain (optional) | 16 APA102/SK9822 tri-color LEDs daisy-chained: D11→DI (data), D12→CI (clock), plus 5 V + GND. LED 1 first in the chain. |
| USB (RX0/TX0) | Bench console (§5b) | Free in normal service. |
| D19 (RX1) | MIDI in | Via the 6N138 circuit below. |

**E-stop — two independent paths (required):** the SECOND NC contact block
goes **in series with the relay-board coil supply** (or the solenoid valve
common). Pressing the mushroom removes valve power no matter what the
electronics do; the D2 path lets the firmware latch and report it.

**MIDI input circuit (standard):** DIN pin 4 → 220 Ω → 6N138 anode (pin 2);
DIN pin 5 → 6N138 cathode (pin 3); 1N4148 reversed across pins 2–3;
6N138 pin 8 → 5 V, pin 5 → GND, pin 6 → Mega D19 with a 470 Ω pull-up to
5 V. Leave the DIN shield unconnected at the receiver.

### 2.3 Fuel-train hardware (firmware cannot substitute)

Per [HARDWARE.md](HARDWARE.md) and NFPA 160: pilot/flame supervision, high
AND low fuel-pressure cutoffs, a manual shutoff at the operator position,
and appropriate accumulator plumbing. The controller assumes these exist.

### 2.4 Fuses

Set the Mega's brownout detector to 4.3 V (BODLEVEL efuse) so a sagging
supply resets cleanly. With an ISP programmer:
`avrdude -c usbasp -p m2560 -U efuse:w:0xFC:m` (verify against your
current fuse settings first).

---

## 3. Firmware installation

### 3.1 Install the toolchain (once)

```
winget install ArduinoSA.CLI          # or download from arduino.github.io
arduino-cli core install arduino:avr
arduino-cli lib install SdFat
```

### 3.2 Build and flash

```
cd <repo>
arduino-cli compile --fqbn arduino:avr:mega Enlighten
arduino-cli upload  --fqbn arduino:avr:mega -p COM5 Enlighten   # your port
```

Find the port with `arduino-cli board list`.

### 3.3 Run the test suite (recommended before every flash)

```
test\run_tests.ps1        # 116 tests; requires g++ (MinGW/WinLibs)
examples\run_demo.ps1     # narrated feature tour on a simulated clock
```

### 3.4 Configuration

Every tunable is in
[Enlighten/src/core/config.h](../Enlighten/src/core/config.h): timing
limits, duty budget, arm values, timeouts, MIDI note map. Safety-relevant
values are guarded by `static_assert`s — the build will refuse plainly
unsafe combinations, but treat ANY change to `MAX_OPEN_MS`,
`MIN_CLOSE_MS`, or the duty budget as a safety review, and re-run the test
suite afterwards.

---

### 2.5 Operator display (optional)

A **20×4** character LCD on an I²C backpack gives the operator full status
at a glance — the rig runs identically without one (it is probed at boot).
The four lines show everything simultaneously, no alternating screens:

```
ARMED       CHASE UP        FAULT LOCKOUT      6
  [##..............]        ESTOP PRESSED
POOF  500  REST 2000        CLEAR: LINK UP +
REPEAT ON   RATE 128        ARM VALUES DOWN
```

* **Safe**: `SAFE [MIDI]` + what's missing (`NO SIGNAL` / `TURN KEY TO
  ARM` / `ARM: CC20+CC21`) + the selected mode.
* **Armed**: mode, live 16-poofer bar, poof/rest times, repeat/rate — or
  `> 01:23 OPENER.SHW` on line 4 during SD playback.
* **Fault**: code number, cause in words, and the full two-line clear
  instruction, all visible at once.

The display driver is failure-isolated: it writes at most one character
per loop (never threatening the 5 ms safety budget), the I²C bus has a
hard timeout, and any display/bus fault silently disables the panel for
the session — a flaky LCD can never stall the show.

### 2.6 Display pages and panel input (DISP / SEL buttons)

**DISP** cycles the LCD through four pages; any aux page auto-returns to
STATUS after 30 s so the safety screen is never far away. **SEL** acts on
the current page:

| Page | Shows | SEL does |
|---|---|---|
| STATUS | the normal state screen (§2.5) | — |
| STATS | total fires, flame seconds (fuel proxy), busiest poofer | reset counters |
| DIAG | uptime, last fault, SD and signal status | — |
| MODE SELECT | active mode + panel override | cycle override: AUTO → OFF → RAW → … |

The MODE SELECT page gives the box its own mode control: pick a pattern
with SEL and it overrides the console/CC mode until you cycle back to
AUTO. This is choreography-only input — arming, deadman, E-stop, and duty
limits are completely unaffected by the panel buttons.

### 2.7 Per-solenoid panel LEDs (optional, tri-color)

16 APA102/SK9822 addressable RGB LEDs on a two-wire chain (wiring in
§2.2), one per solenoid:

| Color | Meaning |
|---|---|
| **Dark** | Disarmed / lockout — the dark panel reads "rig is cold" across the room |
| **Green** | ARMED and this poofer enabled (in the pattern / trigger held) |
| **Red** | Solenoid **open right now**, fired live (MIDI/panel) |
| **Blue** | Solenoid open right now, fired by SD playback |
| **Amber** | Requested but held closed by the duty limiter — the "why is it quiet" state, now visible |

Colors derive from the safety filter's actual output (never merely the
request), so the panel always shows what the valves are doing.

## 4. Understanding the status LED

| Pattern | Meaning |
|---|---|
| Solid on (briefly at boot) | Self-test running |
| 2 quick blinks after self-test | Normal (MIDI) service starting |
| 3 quick blinks after self-test | **Bench mode** (D5 jumper in) |
| Slow 1 Hz blink | SAFE (disarmed) — normal idle |
| Double-blink burst | ARM_PENDING (handshake hold timer running) |
| Fast 4 Hz blink | **ARMED** — treat the effect as live |
| Solid on (in service) | ARMED and FIRING |
| N blinks, 1 s pause, repeat | FAULT LOCKOUT — count the blinks: |

| Blinks | Fault | Typical cause |
|---|---|---|
| 1 | Watchdog reset | firmware hung and hardware-reset itself |
| 2 | Brownout reset | supply voltage dipped |
| 3 | Self-test: timebase | hardware fault |
| 4 | Self-test: outputs | shorted/stuck output driver — do not use |
| 5 | Self-test: E-stop | E-stop pressed at boot, or broken E-stop wiring |
| 6 | E-stop asserted | mushroom pressed in service |
| 7 | Loop overrun | firmware stall — report it |

**Clearing any lockout:** fix the cause (e.g. release the E-stop), then
with the console/keyboard connected, set BOTH arm values off. The LED
returns to the slow SAFE blink. Then re-arm normally.

---

## 5. Commissioning (no fuel — do all of this first)

Work through the bench checklist in [HARDWARE.md](HARDWARE.md) §Bench
verification. In summary, verify: relays stay closed through power-ups and
resets; the E-stop faults at boot when disconnected (5 blinks) and locks
out in service (6 blinks); arming is impossible without the keyswitch;
killing the control signal mid-"fire" closes everything within 0.5 s and
does NOT re-arm on reconnection; a held trigger visibly throttles (≤0.5 s
pulses, forced closes); and 20 rapid power cycles never chatter a relay.
Only after every item passes — and the fuel train's own safeties are
verified — proceed to a supervised low-pressure fuel test.

---

## 5b. Bench mode: testing the Mega + sequencer standalone

To exercise the controller and sequencer with **no show hardware at all** —
no console, no E-stop loop, no keyswitch — jumper **D5 to GND** and power
up. The LED blinks 3× at boot, the LCD reads `BENCH SAFE [USB]`, and the
USB port becomes an interactive console (115200 baud — Arduino Serial
Monitor or PuTTY):

```
arm                     go through the real edge + 500 ms hold
mode up                 off/raw/all/alt/up/down/updown/downup/in/out/inout/outin
poof 200  rest 300  rate 128  repeat on
hold 1 .. hold 8        per-poofer enables (patterns) / held triggers (raw)
fire 3                  single poof on poofer 3 (raw mode)
stop / disarm / status / help
```

What bench mode simulates: the E-stop and keyswitch inputs, and the signal
source. What it does **not** relax: arming still requires the edge and
500 ms hold (typed as `arm`), all outputs still pass through the safety
filter and duty limits, the watchdog stays armed, and **2 minutes of
console silence disarms the rig** (walk-away deadman). The self-test skips
only the E-stop wiring check.

Rules: bench mode is for relay/valve **dry** testing and sequencer
development. **Never connect fuel with the bench jumper in.** Pull the D5
jumper before show wiring — the 3-blink boot pattern and the LCD banner
exist so a forgotten jumper is obvious (the rig also ignores MIDI
entirely in bench mode, so it cannot run a show by accident).

## 6. DMX (removed)

DMX support was removed when MIDI became the sole control protocol —
it freed the USB port, ~0.5 KB of RAM, and a class of UART conflicts.
If a DMX venue ever requires it, the complete v2 DMX implementation
(decoder, channel map, Freestyler profile, tests) lives at the git tag
**`v2-last-dmx`**, and the retired docs are in [legacy/](../legacy/).

---

## 7. Operating with MIDI

### 7.1 Setup

Connect DIN MIDI to the 6N138 input (the LED double-blinks at boot when
entering normal service). Use MIDI channel 1.

If your keyboard/interface supports **Active Sensing**, enable it: it
gives you a ~⅓-second cable-pull deadman. If it doesn't, keep any MIDI
traffic flowing at least every 2 s while armed (e.g. re-send CC20 = 85
once a second from your DAW) — silence disarms the rig.

### 7.2 Arming and playing

1. Key to ARM.
2. Send **CC20 = 85** and **CC21 = 106** (map them to two buttons/pads on
   your controller). Hold ARMED after the 500 ms hold — fast LED blink.
3. Select a mode with **CC22** (0–127, see the band table in MIDI_MAP; 125 =
   RAW). CC23/24/25/26 = poof, rest, rate, repeat.
4. In RAW mode, notes **C2–D#3 (36–51)** fire poofers 1–16 like drum pads —
   velocity ≥ 64 fires, softer notes are ignored entirely. In pattern
   modes, held notes are the per-poofer enables.
5. **Panic:** CC120 or CC123 (all-notes-off — most controllers have a
   panic button) closes everything AND disarms.
6. Disarm: send anything else on CC20/21, hit panic, turn the key, or
   pull the cable.

---

### 7.3 Playing generated shows

`tools/seqgen` composes complete choreographed shows of any length for any
poofer count and writes them as playable MIDI files:

```
tools\build.ps1
tools\build\seqgen --seconds 60 --poofers 16 --out myshow
tools\build\showplay myshow.show          # preview + verify on screen
```

Load `myshow.mid` in any DAW with MIDI-out to the rig, **arm by hand**
(the file intentionally contains no arm messages), and press play. Every
generated show is pre-verified against the duty limits so nothing gets
clipped in performance. Full details: [tools/README.md](../tools/README.md).

### 7.4 Standalone SD-card playback (no PC at show time)

With an SD module (§2.2) the rig plays generated shows entirely on its
own — the laptop stays home:

1. Generate shows with `tools/seqgen` and copy the `.show` files to a
   FAT-formatted card using **8.3 names with a `.SHW` extension** (e.g.
   `OPENER.SHW`, `FINALE.SHW`).
2. Arm normally (keyswitch + MIDI CC handshake — arming is never on the
   card).
3. Press the **PLAY button**: the next `.SHW` on the card starts; the LCD
   alternates `PLAY <name>` / `PLAY MM:SS` over the live poofer bar.
   Press again to stop. Repeated presses cycle through the files.
4. Live MIDI notes still work during playback (they merge with the show).

Safety posture is unchanged: the card is a *request* source only. Disarm,
E-stop, signal-loss deadman, and per-channel duty limits all still gate
playback, and a file that ends abruptly closes every valve. A missing or
unreadable card simply disables the PLAY button.

## 8. Show-day run sheet

**Before doors:** walk the fuel train per your own checklist; verify the
E-stop physically kills valve power (hardwired path) with the controller
running; boot the controller and confirm the protocol blink and slow SAFE
blink; arm, fire a single test poofer, disarm; confirm the key is OFF and
pocketed until showtime.

**During the show:** the operator keeps the key, one hand's reach from the
E-stop, with full sightlines. Arm only for the effect; disarm between
numbers. The fast-blink LED means live — treat it like a loaded weapon.

**After:** disarm, key off, close manual fuel valves, bleed lines per your
fuel procedure, then power down the controller.

**If anything ever behaves unexpectedly** — a fault code you didn't
expect, a poofer that fires when it shouldn't, a lockout you can't explain
— stop using the rig, keep fuel off, and diagnose on the bench (§9). The
test suite and feature tour reproduce nearly every behavior off-hardware.

---

## 9. Troubleshooting

| Symptom | Check |
|---|---|
| Won't arm, LED stays slow-blink | Key on D3 actually closing to GND? CC20 exactly 85 and CC21 exactly 106, both held ≥500 ms? Were the values already latched when you connected (cycle them off/on)? Signal alive? |
| 5 blinks at every boot | E-stop pressed, or the D2 loop is open — check the wiring; a broken wire reads as pressed (by design). |
| 6 blinks in service | E-stop was pressed. Release, drop both arm values, re-arm. |
| 1 blink at boot | Watchdog reset — the firmware hung. Note what preceded it and report; the rig is safe (that's the point) but the cause should be found. |
| 4 blinks at boot | Output self-test failed — a driver pin reads wrong. Do not use until found. |
| Poofer never fires | Note velocity ≥64 (softer is ignored by design)? In a chase, poofers 9–16 don't participate (patterns run on 1–8). Duty budget spent (wait a few seconds)? |
| Poofer fires short | Poof time is clamped to 500 ms max, and the duty limiter force-closes at 500 ms regardless of the fader. |
| Mode won't change | Values within 2 of a band boundary hold the current mode; move to a band center and hold ≥150 ms. |
| Random disarms on MIDI | Active-Sensing gaps or >2 s of silence. Enable AS or add a keepalive. |
| Relays chatter at power-up | Missing pull-ups on the relay inputs — fix before anything else (§2.2). |

For anything deeper: `test\run_tests.ps1` (116 tests) verifies the logic,
and `examples\run_demo.ps1` replays every feature on a simulated clock so
you can compare expected vs. observed behavior.
