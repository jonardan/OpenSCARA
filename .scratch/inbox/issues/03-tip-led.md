# Tip LED: wire it and finish step 8 (scara led, M3/M5)

Status: ready-for-human
Category: enhancement

Step 8 of `docs/plan.md` was skipped on 2026-10-11: the code was written,
then reverted because the LED is not wired yet. Step 9 (`scara gcode`) went
ahead without it: M3/M5 (and M106/M107) are parsed but ignored with a
warning.

The step 8 work is saved as a patch against commit 6683380 (step 7):
`.scratch/inbox/issues/03-tip-led.patch`. It adds P9_23 as the gpio-leds LED
"tip" in the device tree (pulled down, off at boot), `led = tip` in
`tmc2209.conf`, `scara led on|off` (alone: prints the state) and the docs.
It touches scara.c, plan.md, software/README.md and the board readme, which
step 9 also changed, so expect to apply it with `git apply -3` and fix the
conflicts by hand.

To do:

1. Wire the LED: P9_23 -> 330 Ohm -> LED anode, cathode -> GND (P9_45).
   100 Ohm for a white or blue LED; a transistor (2N2222, 1 kOhm to the
   base) for anything brighter than ~4 mA.
2. Apply the patch, rebuild, flash; test `scara led on` / `off`, and that
   the LED is off after a reboot.
3. In `prog_run` (scara.c), replace the "M3/M5 ignored" warning with the
   LED: M3/M106 on, M5/M107 off, and LED off when the file ends, fails or
   is stopped with Ctrl-C (plan step 10).
4. Long-exposure photo of square.gcode with M3/M5 around the square.

## Comments
