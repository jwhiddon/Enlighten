# Enlighten MIDI Map

DIN MIDI (31250 baud) on Serial1/RX1 (Mega pin 19), MIDI channel 1.
MIDI is the rig's control protocol (the LED blinks twice during startup
when entering normal service).

## Notes → poofers

| Note | Poofer |
|---|---|
| 36 (C2) … 51 (D#3) | 1 … 16 |

* Note-on velocity ≥ 64 = trigger on; note-off or note-on velocity 0 =
  trigger off. Velocities 1–63 are ignored entirely (no half-triggers).
* In RAW mode a note fires its poofer for POOF_MS (like a drum pad). In
  pattern modes the held notes are the per-poofer enables.

## Control changes

| CC | Function | Values |
|---|---|---|
| 20 | ARM_A | must be **85** |
| 21 | ARM_B | must be **106** |
| 22 | MODE | 0–127, scaled ×2 into 20-wide bands (table below) |
| 23 | POOF_MS | 0–127 → 30–500 ms (default ~100 ms if never sent) |
| 24 | REST_MS | 0–127 → 45–2000 ms (default ~200 ms) |
| 25 | RATE | 0–127 scaled ×2 (64 = 1.0×) |
| 26 | REPEAT | ≥64 = on |
| 120 / 123 | All sound/notes off | clears every gate **and disarms** |

(Two distinct mid-scale magic values that a knob sweep, all-zeros, or
all-full state can never produce together by accident.)

## Mode bands (CC22, after the ×2 scaling; hold a value ≥150 ms)

| CC22 | Mode | | CC22 | Mode |
|---|---|---|---|---|
| 0–9 | OFF | | 60–69 | CHASE OUT |
| 10–19 | CHASE UP | | 70–79 | IN/OUT |
| 20–29 | CHASE DOWN | | 80–89 | OUT/IN |
| 30–39 | UP/DOWN | | 90–99 | ALTERNATE |
| 40–49 | DOWN/UP | | 110–119 | FIRE ALL |
| 50–59 | CHASE IN | | 120–127 | RAW |

Values within 1 of a band boundary hold the current mode (guard zones), so
a swept knob cannot glitch through modes. Chases run on poofers 1–8;
IN/OUT variants fire mirrored pairs (1+8, 2+7, 3+6, 4+5).

## Arming

Send CC20 = 85 and CC21 = 106 (both persist as decoder state) with the
panel keyswitch closed; the controller arms after the 500 ms hold. Same
rising-edge rule as always: the values must be seen de-asserted since the last
disarm. Sending any other value on either CC — or CC120/123 — disarms.

## Deadman over MIDI

MIDI is silent when idle, so continuous permission needs a heartbeat:

* If your device sends **Active Sensing (0xFE)** (most hardware does), its
  absence for 330 ms drops the link → instant disarm. Pulling the cable
  disarms in ~a third of a second.
* If not, ANY MIDI byte within 2 s keeps the link alive — script a
  keepalive (e.g. resend CC20=85 every second) or enable active sensing.
