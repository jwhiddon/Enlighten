# Enlighten Logic Shield — Design Intent (Rev A)

A consolidation of every external connection the firmware currently
expects into one PCB that stacks on the Arduino Mega 2560, in place of
today's point-to-point wiring. This is a schematic-level spec, not yet
fabricated — precise enough to transcribe directly into KiCad (or hand to
a PCB designer), including a wire-by-wire appendix for that purpose.

**Status:** design intent only. Treat the current point-to-point wiring
as the reference until it's had a few shows of burn-in; fab this once
the pin map has held steady. All pins below are pulled from
[board_pins.h](../Enlighten/src/boards/mega2560/board_pins.h) and
[config.h](../Enlighten/src/core/config.h) as of this writing — if either
changes, this doc and the shield disagree.

## Design decision: logic only — no relay/valve switching on the shield

The shield carries **signal-level connections exclusively**. Solenoid
drive stays on the existing external relay/SSR board, connected by a
single keyed ribbon header. Rationale: keeping switching current, coil
flyback transients, and any mains-side wiring physically off the same
board as the MIDI opto input, I²C LCD, and LED data lines is the
difference between "a relay board fault is a relay board problem" and "a
relay board fault takes the logic down with it." It also means a shorted
output channel is a swap of the external board, not a full shield
respin. If you'd rather have one integrated board, that's a legitimate
alternative — it just trades isolation and repairability for compactness.

## Block diagram

```
                    +---------------------------------------------+
                    |     ENLIGHTEN LOGIC SHIELD (Rev A)           |
                    |     Mega 2560 shield footprint (NOT Uno)     |
                    +---------------------------------------------+

  MIDI DIN-5 in --->[ 6N138 opto isolation ]-------------------> D19 (RX1)

  ESTOP    (2-pin terminal, NC loop) --------------------------> D2
  ARM_KEY  (2-pin terminal, NC loop) --------------------------> D3
  BENCH    (2-pin terminal, jumper)  --------------------------> D5
  PLAY     (2-pin terminal, momentary) ------------------------> D6
  DISP     (2-pin terminal, momentary) ------------------------> D7
  SEL      (2-pin terminal, momentary) ------------------------> D8
       (no shield-side pull resistors needed — firmware uses
        INPUT_PULLUP on all six; terminals are signal + GND only)

  LCD header   (4-pin: 5V/GND/SDA/SCL) ------------------------> D20/D21
  LED header   (4-pin: 5V/GND/DI/CI)   ------------------------> D11/D12
  microSD hdr  (6-pin: 5V/GND/MISO/MOSI/SCK/CS) ----------------> D50-D53

  SOLENOID OUT — 20-pin shrouded box header, keyed, ribbon cable
     16x logic signal (PORTA D22-29, PORTC D30-37) + 2x GND + 2x +5V ref
     -----------------------------------------------------------> external
                                                        relay/SSR board

  ****************************************************************
  *  The E-STOP HARDWIRED (fuel-power) PATH DOES NOT ROUTE        *
  *  THROUGH THIS SHIELD.  It runs directly in series with the    *
  *  relay board's coil supply per docs/HARDWARE.md.  The D2      *
  *  terminal above is the firmware MONITOR TAP ONLY.             *
  ****************************************************************
```

## Subsystem detail

### MIDI input (opto-isolated, standard spec)

DIN-5 jack, pins 4 and 5 only:

- DIN pin 4 → 220 Ω → 6N138 pin 2 (anode)
- DIN pin 5 → 6N138 pin 3 (cathode)
- 1N4148 reverse-biased across pins 2–3 (transient protection)
- 6N138 pin 8 → 5 V, pin 5 → GND
- 6N138 pin 6 (output) → 470 Ω pull-up to 5 V, and → D19 (RX1)
- DIN shield pin: unconnected at the receiver (per spec)

Matches the circuit already documented in [MANUAL.md](MANUAL.md) §2.2 —
this just moves it from dead-bug perfboard onto the shield.

### E-stop / keyswitch / panel buttons

Six 2-pin pluggable terminal blocks (5.08 mm pitch), one per signal:
`ESTOP`, `ARM_KEY`, `BENCH`, `PLAY`, `DISP`, `SEL`. Each is just
signal + GND — no resistors needed, since the firmware enables
`INPUT_PULLUP` on all six internally.

Layout requirement: physically separate the `ESTOP` and `ARM_KEY`
terminals from the four convenience buttons (their own silkscreened
box), and print the warning banner from the block diagram directly next
to the `ESTOP` terminal — the point is that nobody assembling or
repairing the box mistakes the D2 monitor tap for the safety-critical
hardwired interrupt.

### Operator LCD (I²C)

4-pin header: 5 V, GND, SDA (D20), SCL (D21). No level shifting needed
(backpack and Mega are both 5 V logic). Add DNP (do-not-populate)
footprints for 4.7 kΩ pull-ups to 5 V on SDA/SCL — most PCF8574 backpacks
already include their own; only populate if driving a bare HD44780 or the
bus needs more drive.

### Panel LED chain (APA102/SK9822)

4-pin header: 5 V, GND, DI (D11), CI (D12). Populate small ceramic
decoupling (0.1 µF) at the connector by default. DNP footprints for a
330 Ω series resistor on DI (ringing suppression on long runs) and a bulk
1000 µF electrolytic across 5 V/GND — populate either only if you see
glitching, which is unlikely at 16 LEDs and the shipped brightness
(10/31).

### SD card

Use an off-the-shelf microSD breakout with its own 3.3 V regulator and
level shifters (SD logic is 3.3 V; raw modules are cheap and common — do
not wire a bare card socket directly to 5 V logic). Shield provides a
6-pin header matching it: 5 V, GND, MISO (D50), MOSI (D51), SCK (D52),
CS (D53). SPI pins are fixed in hardware on the Mega; CS is firmware-fixed
to D53 (`SD_CS` in `board_pins.h`).

### Solenoid output header

20-pin shrouded box header (2×10, 0.1"), keyed so it can't be seated
backwards, wired via ribbon cable to the external relay/SSR board:
16 signal pins in port order (D22…D29, D30…D37) + 2× GND + 2× +5 V
reference (for opto-isolated relay boards that want a logic-side supply).
No relay coil or valve current present on the shield — see the design
decision above. Output polarity (`OUTPUTS_ACTIVE_LOW` in `config.h`) is a
firmware concept only; the header pinout is identical either way.

### Power and protection

Shield draws 5 V from the Mega header. Add a ~500 mA resettable polyfuse
in series with the shield's 5 V input — sized above the combined draw of
the LCD backpack, LED chain, SD module, and opto (comfortably under
200 mA in practice), just to protect the Mega's onboard regulator from a
wiring-fault short on the logic side. This is a protection nicety, not a
safety-critical path — solenoid power is already a separate, externally
supplied circuit.

## Mechanical

- Use the **Mega-specific shield footprint**, not an Uno-only footprint —
  Mega shields need the extra header row for D22–D53, which many generic
  "Arduino shield" footprints omit or misalign.
- Stackable (tall) female headers if you want the option to add another
  shield later; not required otherwise.
- Silkscreen zones matching the block diagram: SAFETY I/O (E-stop/key),
  PANEL I/O (buttons/LCD/LEDs), DATA (MIDI/SD), SOLENOID OUT — reduces
  assembly and field-repair mistakes.

## Bill of materials (approximate — verify current pricing/availability)

| Qty | Part | Notes |
|---|---|---|
| 1 | Mega-footprint prototype/blank shield PCB | JLCPCB/OSHPark, ~1 week turnaround |
| 1 | 6N138 (or PC900) optocoupler, DIP-8 | MIDI input |
| 1 | 1N4148 switching diode | MIDI protection |
| 2 | 220 Ω, 470 Ω resistors, 1/4 W | MIDI circuit |
| 1 | 5-pin DIN-180 female jack, PCB mount | MIDI input |
| 6 | 2-pin 5.08 mm pluggable terminal blocks | ESTOP/ARM_KEY/BENCH/PLAY/DISP/SEL |
| 1 | 4-pin header (or JST-XH) | LCD |
| 1 | 4-pin header (or JST-XH) | LED chain |
| 1 | 6-pin header | microSD breakout |
| 1 | microSD breakout module (3.3 V reg + level shift) | off-the-shelf |
| 1 | 20-pin 2.54 mm shrouded box header + IDC ribbon | solenoid output |
| 1 | 500 mA resettable polyfuse | 5 V protection |
| — | 0.1 µF ceramic decoupling caps | LCD/LED/SD headers |

Rough total: parts well under $20; PCB fab a few dollars per board in
small quantity plus shipping/lead time.

## Appendix: wire-by-wire netlist (for direct KiCad transcription)

| From (shield connector) | To (Mega pin) | Function |
|---|---|---|
| MIDI opto output (6N138 pin 6) | D19 (RX1) | MIDI data in |
| ESTOP terminal | D2 | E-stop monitor (NC loop) |
| ARM_KEY terminal | D3 | Keyswitch (NC loop) |
| BENCH terminal | D5 | Bench-mode select jumper |
| PLAY terminal | D6 | SD play/stop button |
| DISP terminal | D7 | Display page button |
| SEL terminal | D8 | Page-context button |
| LCD header SDA | D20 | I²C data |
| LCD header SCL | D21 | I²C clock |
| LED header DI | D11 | APA102 data |
| LED header CI | D12 | APA102 clock |
| SD header MISO | D50 | SPI |
| SD header MOSI | D51 | SPI |
| SD header SCK | D52 | SPI |
| SD header CS | D53 | SPI chip select |
| Solenoid header pins 1–8 | D22–D29 | Poofers 1–8 (PORTA) |
| Solenoid header pins 9–16 | D30–D37 | Poofers 9–16 (PORTC) |
| Status LED | D13 | Onboard Mega LED; optionally paralleled to a panel-mount LED |
