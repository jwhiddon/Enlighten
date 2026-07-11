# Enlighten Feature Tour

A narrated, host-runnable walkthrough of **every firmware feature**, driving
the real core (the exact code flashed to the Mega) through the same
pipeline harness the test suite uses — no hardware needed.

```
examples\run_demo.ps1      # Windows (needs g++, same as the tests)
examples/run_demo.sh       # POSIX
```

It prints a timeline of state transitions, LED patterns, and a live
16-poofer bar (`[#..#...#|........]`, poofer 1 leftmost), and exits nonzero
if any demonstrated property fails — so it doubles as an executable
acceptance walkthrough.

## What it demonstrates

| # | Section | Features shown |
|---|---|---|
| 1 | Boot | self-test states, watchdog-reset boot to FAULT_LOCKOUT, LED fault code, operator acknowledgment |
| 2 | Arming security | keyswitch gating, saved-scene patch-in refused (rising edge), fader-sweep refused (500 ms hold), the real two-value handshake |
| 3 | Mode banding | full MODE fader sweep with all poofers enabled — zero glitch fires (guards + debounce) |
| 4 | Effect modes | CHASE_UP, mirrored CHASE_IN pairs, ping-pong CHASE_UP_DOWN, ALTERNATE, FIRE_ALL — visualized step by step |
| 5 | Choreography | RATE fader tempo scaling, one-shot vs repeat |
| 6 | Duty backstop | a held maxed-out RAW trigger throttled to 500 ms opens, 50 ms forced closes, 30% of the 10 s budget window, and the budget refilling as the window slides |
| 7 | Deadman | mid-show cable pull → closed and disarmed at the timeout; **no auto-re-arm** on reconnection |
| 8 | E-stop | same-loop shutdown, latched lockout, release ≠ re-enable, acknowledge + re-arm |
| 9 | MIDI | CC arming handshake, chord → poofers via running status, velocity gate, CC123 panic (clears + disarms), Active-Sensing cable-pull deadman |
| 10 | Rollover | a show running straight through the 49.7-day millis() wrap — the legacy firmware's stuck-solenoid scenario — without a hiccup |

Every tick of the tour also runs the test suite's `InvariantChecker`
(outputs closed unless ARMED, E-stop always wins, per-channel open/close/
duty limits), so the whole walkthrough is continuously verified while it
narrates.
