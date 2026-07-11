# Enlighten MIDI Map

DIN MIDI (31250 baud) on Serial1/RX1 (Mega pin 19), MIDI channel 1.
MIDI is the **default protocol** — leave the PROTOCOL_SELECT pin (Mega
pin 4) open (the LED blinks twice during startup to confirm). Jumper D4
to GND only when a DMX console is used instead.

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
| 22 | MODE | 0–127, scaled ×2 into the DMX mode bands (see DMX_MAP.md) |
| 23 | POOF_MS | 0–127 → 30–500 ms (default ~100 ms if never sent) |
| 24 | REST_MS | 0–127 → 45–2000 ms (default ~200 ms) |
| 25 | RATE | 0–127 scaled ×2 (64 = 1.0×) |
| 26 | REPEAT | ≥64 = on |
| 120 / 123 | All sound/notes off | clears every gate **and disarms** |

(MIDI data bytes are 7-bit, so the DMX ARM_B value of 170 is impossible on
MIDI; 106 is the MIDI-side magic value.)

## Arming

Send CC20 = 85 and CC21 = 106 (both persist as decoder state) with the
panel keyswitch closed; the controller arms after the 500 ms hold. Same
rising-edge rule as DMX: the values must be seen de-asserted since the last
disarm. Sending any other value on either CC — or CC120/123 — disarms.

## Deadman over MIDI

MIDI is silent when idle, so continuous permission needs a heartbeat:

* If your device sends **Active Sensing (0xFE)** (most hardware does), its
  absence for 330 ms drops the link → instant disarm. Pulling the cable
  disarms in ~a third of a second.
* If not, ANY MIDI byte within 2 s keeps the link alive — script a
  keepalive (e.g. resend CC20=85 every second) or enable active sensing.
