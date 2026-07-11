# Enlighten Hardware Requirements

Firmware safeguards mean nothing if the electrical layer can defeat them.
This checklist is part of the safety design — see docs/SAFETY.md.

## Pin map (Arduino Mega 2560)

| Pin | Function | Wiring |
|---|---|---|
| PORTA (D22–D29) | Poofers 1–8 relay drive | **active LOW** (high = closed/safe) |
| PORTC (D37–D30) | Poofers 9–16 relay drive | active LOW |
| D2 | E-stop monitor | NC loop to GND; INPUT_PULLUP (open = stop) |
| D3 | Arm keyswitch | key closes to GND; INPUT_PULLUP |
| D5 | Bench mode | jumper to GND at boot = USB test console, interlocks simulated — never with fuel |
| D6 | PLAY button | momentary to GND: SD show start/next/stop |
| D7/D8 | DISP / SEL buttons | display page cycling and page-context input |
| D50–D53 | SD card (SPI) | standalone .SHW playback; CS = D53 |
| D13 | Status LED | see SAFETY.md for patterns |
| D20/D21 | Operator LCD (optional) | 20×4 HD44780 via PCF8574 I²C backpack, addr 0x27/0x3F |
| D11/D12 | Panel LED chain | APA102/SK9822 tri-color chain: data/clock (+5 V, GND); PORTL/PORTK now spare |
| RX0/TX0 (USB) | bench console / future telemetry | free in normal service |
| RX1 (D19) | MIDI in | via 6N138 optocoupler per MIDI spec |

## Required safety hardware (firmware cannot substitute)

1. **E-stop, two independent paths.** A NC mushroom-head E-stop wired:
   * **in series with the relay-board coil supply (or the solenoid valve
     common)** so pressing it removes valve power regardless of anything
     the firmware does — NFPA 160 expects the E-stop not to depend on
     software; and
   * **into D2** so the firmware also sees it, latches FAULT_LOCKOUT, and
     reports it on the LED.
   Broken wire = open circuit = stop (fail-safe).
2. **Arm keyswitch** on D3. Protocol data alone can never arm the system.
3. **Relay-board pull-ups.** Verify the relay inputs are pulled to the
   INACTIVE (valve closed) level so a resetting or tristated MCU keeps the
   valves closed. Add external pull-ups to 5 V if the board lacks them —
   this is what makes a watchdog reset safe.
4. **Brownout fuse.** Set the Mega's BOD to 4.3 V (`efuse` BODLEVEL) so a
   sagging supply gives a clean reset instead of undefined execution.
5. **Fuel-train safeties — external, not firmware:** pilot/flame
   supervision, high AND low fuel-pressure cutoff switches, a manual
   shutoff valve at the operator position, and accumulator-appropriate
   plumbing. NFPA 160 compliance of the overall effect requires these and
   AHJ approval.

## Enclosure and wiring (the "NEMA" part)

* **NEMA 4/4X enclosure** for outdoor propane service (hose-down/corrosion
  rated); glands/strain relief on every penetration.
* MIDI in through the standard 6N138 opto circuit (inherent isolation).
* Label the E-stop and arm key clearly at the operator position.
* Solenoid flyback protection (diodes on DC valve coils / RC snubbers on
  AC) if the relay board doesn't include it.
* Mechanical relays add ~10 ms actuation latency; for crisper poofs use
  low-side MOSFET drivers with flyback diodes — the firmware timing already
  supports it (same active-low drive).

## Bench verification checklist (before fuel is EVER connected)

1. Power-on: all relays stay closed through boot; LED solid then protocol
   blink(s) then slow SAFE blink.
2. Pull the E-stop wire off D2 → LED blinks 5 (SELFTEST_ESTOP) on next
   boot; in service → 6 blinks (ESTOP_ASSERTED), relays closed.
3. Arm without the keyswitch → stays SAFE. Arm with key + handshake → fast
   blink after 0.5 s.
4. Pull the MIDI cable while firing → all relays close within ~⅓ s
   (Active Sensing deadman), LED back to slow SAFE blink; reconnecting
   does NOT re-arm until the arm values are cycled.
5. Hold a RAW trigger continuously → relay opens ≤0.5 s at a time, forced
   50 ms closes, and total open time visibly throttles (duty budget).
6. Induced hang (test build with a deliberate `while(1)` behind a jumper)
   → watchdog resets within 120 ms, boots to LOCKOUT blinking 1.
7. Power-cycle 20× rapidly: relays never chatter open.

## Recommended parallel platform: RP2040 (Raspberry Pi Pico)

The core is portable; a Pico port is `src/boards/rp2040/board.cpp` plus a
build config — the safety logic and its tests are unchanged. What the ~$4
Pico buys: 264 KB SRAM, dual core (the safety filter can own a core), PIO
state machines for exotic protocols, native USB for diagnostics. It needs
5 V level shifting to the relay board. The Mega remains the primary
target.
