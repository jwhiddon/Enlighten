# Freestyler support files

Freestyler (http://www.freestylerdmx.be/) fixture profile for the Enlighten
poofer controller, **v2 channel map** (see ../docs/DMX_MAP.md).

Build the fixture in Freestyler's Fixture Creator with 24 channels:

| Ch | Name | Type | Notes |
|---|---|---|---|
| 1 | Arm A | preset button | value **85** (window 80–90) |
| 2 | Arm B | preset button | value **170** (window 165–175) |
| 3 | Mode | preset buttons | one button per band center: 10=OFF, 30=Chase Up, 50=Chase Down, 70=Up/Down, 90=Down/Up, 110=In, 130=Out, 150=In/Out, 170=Out/In, 190=Alternate, 230=Fire All, 250=Raw |
| 4 | Poof time | fader | 0–255 → 30–500 ms |
| 5 | Rest time | fader | 0–255 → 45–2000 ms |
| 6 | Rate | fader | 128 = 1.0× |
| 7 | Repeat | switch | ≥128 = on |
| 8 | (reserved) | — | leave at 0 |
| 9–24 | Poofer 1–16 | buttons | ON = 255, OFF = 0 (100–199 is a hold dead zone — don't park faders there) |

**Operator setup rules:**

* Put Arm A/Arm B on a dedicated override/macro button pair — never on
  faders, and never saved into a scene/chase. The controller requires the
  values to be seen OFF before they can arm (cycling the button after
  patch-in is expected).
* Program poofer triggers as full-on (255) button presses.
* The physical arm keyswitch and E-stop at the operator position are
  required regardless of anything in Freestyler.
