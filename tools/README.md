# Enlighten Show Tools

Generate aesthetically structured poofer shows and play/validate them —
on screen, or on the real rig over MIDI.

```
tools\build.ps1        # builds seqgen + showplay + showviz (needs g++)   [build.sh on POSIX]

seqgen --seconds 60 --poofers 16 [--seed 42] [--bpm 120] [--out myshow]
showplay myshow.show [--animate] [--speed 4]
showviz  myshow.show [--speed 2] [--turbo]
```

## seqgen — the sequence generator

Composes a show with a musical arc — a sparse intro, a developing middle
(sweeps, bounces, mirrors, call-and-response, alternation, sparkles), an
accelerating climax (overlapping waves, third-note chases, build-ups), and
a triple-hit finale with one big closing flame — all quantized to a groove
(`--bpm`) so it reads as intentional rather than random. `--seed` makes a
show reproducible; the seed is printed so you can regenerate one you liked.

Every generated show is **verified against the firmware's physical limits
before it is written** (max 500 ms open, min 50 ms close, 30% duty per
sliding 10 s window — the same constants and invariant checker the test
suite uses). The generator trims anything the duty limiter would clip, so
what you preview is exactly what the rig will do.

Outputs:

| File | Purpose |
|---|---|
| `<name>.show` | Text timeline (`<ms> <mask_hex>` state changes) for showplay and other tooling |
| `<name>.mid` | Standard MIDI File that plays the show on the REAL rig through the firmware's MIDI input |

## showplay — preview and validation

Prints (or `--animate`s in real time) the show as a 16-poofer bar
(`[#..#...#|........]`, poofer 1 leftmost), then replays every millisecond
through the real `SafetySupervisor` and reports:

* **VERBATIM** — the duty limiter would clip nothing; the rig plays it as designed;
* poofs per poofer, total flame time, invariant check result.

Exit code is nonzero if anything would be clipped — usable in scripts.

## showviz — animated flame simulation

Watch the show without any hardware: each poofer is a column whose flame
whooshes up while the solenoid is open and decays after it closes,
rendered at ~30 fps with ANSI colors (Windows Terminal / any VT-capable
terminal):

```
       .                   &
       *                   *
       ##                  ##
       @@                  @@
  ================================
    1   2   3   4   5   6   7   8
   11  12  13  12  13  12  13  13  fires

  TC 00:23.067 / 00:30.000  [==================      ]  x1.0
  open  2/8   poofs   99/139   flame  23.3s  duty(10s max)  17%  next  0.06s
```

The bottom block is the operator dashboard: **timecode** (current/total
with a progress bar), poofers **open right now**, **poofs fired vs. the
show total**, cumulative **flame time** (a fuel-consumption proxy), the
**worst per-poofer duty over the rolling 10 s window** (if this approaches
30% the firmware's limiter would start throttling — seqgen shows stay
comfortably under), the **countdown to the next cue**, and a per-poofer
fire counter row under the burner numbers.

`--speed` scales playback; `--turbo` skips frame delays for scripted runs.
Like showplay, it first replays the show through the real SafetySupervisor
and reports VERBATIM/CLIPPED and the invariant check in its summary; the
exit code reflects the verdict.

## Playing a show on the real rig (MIDI path)

1. Boot the controller in MIDI mode (D4 jumpered; LED double-blinks).
2. Load `<name>.mid` into any DAW/sequencer with a MIDI-out to the rig.
3. **Arm by hand** (key + CC20=85/CC21=106 from your controller) — the
   generated file deliberately contains **no arm messages**; pressing play
   while disarmed produces nothing, by design.
4. Press play. The file selects RAW mode (CC22), sets each flame's
   duration via CC23 before the notes that need it, sends notes 36..51
   for poofers 1..16, refreshes CC25 once a second as a keepalive (the
   firmware disarms after 2 s of MIDI silence), and parts with CC123
   (all-notes-off).
5. Disarm between runs. All the usual safeguards stay active during
   playback — E-stop, deadman, duty limits.

## Playing a show standalone (SD-card path — no PC)

Copy the `.show` file to a FAT SD card with an **8.3 filename and `.SHW`
extension** (`OPENER.SHW`), insert it in the controller, arm, and press
the PLAY button. See MANUAL §7.4 — all safety layers still gate playback.

DMX-only rig? Use the `.show` file as the cue timeline for your console's
show recorder, or script it into Freestyler; a direct DMX player would
need a PC-to-DMX interface and is a natural future addition.
