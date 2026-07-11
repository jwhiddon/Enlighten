# Enlighten DMX Channel Map (v2)

> DMX is the rig's secondary protocol: jumper D4 to GND at boot to select
> it (the LED blinks once to confirm). Bare D4 = MIDI.

24 channels, base-address relative. This map intentionally breaks from the
legacy firmware: arming now requires a two-value handshake, and triggers
have a dead zone so mid-fade values can't glitch.

| Ch | Name | Values | Notes |
|---|---|---|---|
| 1 | ARM_A | **85** (accepted 80–90) | both arm channels must hold their |
| 2 | ARM_B | **170** (accepted 165–175) | magic values for 500 ms to arm |
| 3 | MODE | banded, see below | 150 ms debounce + boundary guards |
| 4 | POOF_MS | 0–255 → 30–500 ms | poof duration |
| 5 | REST_MS | 0–255 → 45–2000 ms | rest between steps |
| 6 | RATE | 0–255 (128 = 1.0×) | master tempo; scales rest. 0 = treated as 128 |
| 7 | OPTIONS | bit 7 (≥128) = repeat | low bits reserved |
| 8 | reserved | — | future use |
| 9–24 | TRIGGER 1–16 | ≥200 = on, <100 = off | **100–199 = dead zone (hold)** |

## Arming

* Set ch1 = 85 AND ch2 = 170 simultaneously and hold; the controller arms
  after 500 ms (LED goes to fast 4 Hz blink).
* The controller must see the values **de-asserted first** — a scene saved
  with the arm values in it will not arm at patch-in. Cycle the values.
* Drop either channel out of its window to disarm instantly.
* In Freestyler, put the arm values behind a dedicated override/macro
  button — never on a fader you might sweep.

## Mode bands (ch3)

Values within 2 of an internal band boundary hold the current mode; a new
band must persist 150 ms to take effect.

| Range | Mode |
|---|---|
| 0–19 | OFF |
| 20–39 | CHASE_UP |
| 40–59 | CHASE_DOWN |
| 60–79 | CHASE_UP_DOWN |
| 80–99 | CHASE_DOWN_UP |
| 100–119 | CHASE_IN (mirrored pairs, outside → in) |
| 120–139 | CHASE_OUT (mirrored pairs, inside → out) |
| 140–159 | CHASE_IN_OUT |
| 160–179 | CHASE_OUT_IN |
| 180–199 | ALTERNATE (evens/odds) |
| 200–219 | reserved (OFF) |
| 220–239 | FIRE_ALL |
| 240–255 | RAW (per-poofer direct triggers) |

* Chase modes run on poofers 1–8; the trigger channels act as per-poofer
  enables (a disabled poofer is a silent gap in the pattern).
* Mirrored (IN/OUT) modes fire pairs: (1,8) (2,7) (3,6) (4,5).
* RAW: each trigger channel fires its poofer for POOF_MS; with repeat OFF
  the channel must drop below 100 before it can fire again; with repeat ON
  it refires after REST_MS while held.
* FIRE_ALL/ALTERNATE use all 16 poofers, gated by their trigger channels.

## Signal loss

No valid DMX for 500 ms = deadman trip: outputs close and the controller
disarms. It will NOT re-arm when the signal returns until the operator
cycles the arm values.
