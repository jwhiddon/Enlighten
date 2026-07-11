# Legacy firmware and retired components (frozen)

* `DMX_MAP.md` / `Freestyler/` — the v2 DMX channel map and Freestyler
  fixture profile, retired when MIDI became the sole control protocol.
  The full v2 DMX implementation (decoder, board glue, tests) lives at
  the git tag `v2-last-dmx`.

* `EnlightenDMX/` — the original single-file DMX sketch (with the 2026
  bug-fix pass: rollover-safe timing, RawDMX stuck-solenoid fixes, watchdog
  reset). Superseded by the unified `../Enlighten/` firmware; kept for
  reference and behavior parity checks.
* `EnlightenMIDI/` — was an empty placeholder; MIDI support now lives in
  the unified firmware (see ../docs/MIDI_MAP.md).

Do not deploy these — the legacy DMX map (single-channel arming, no E-stop,
no duty limiting) does not meet the safety design in ../docs/SAFETY.md.
