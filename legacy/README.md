# Legacy firmware (frozen)

* `EnlightenDMX/` — the original single-file DMX sketch (with the 2026
  bug-fix pass: rollover-safe timing, RawDMX stuck-solenoid fixes, watchdog
  reset). Superseded by the unified `../Enlighten/` firmware; kept for
  reference and behavior parity checks.
* `EnlightenMIDI/` — was an empty placeholder; MIDI support now lives in
  the unified firmware (see ../docs/MIDI_MAP.md).

Do not deploy these — the legacy DMX map (single-channel arming, no E-stop,
no duty limiting) does not meet the safety design in ../docs/SAFETY.md.
