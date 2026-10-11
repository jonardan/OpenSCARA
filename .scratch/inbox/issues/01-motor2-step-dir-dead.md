# Motor 2 does not move with STEP/DIR (turn/move), works over UART (run)

Status: needs-info
Closed: 2026-10-09
Category: bug

## Report (2026-10-09)

Driver 2: STEP on P9_28, DIR on P9_30 (both P9 header). User triple-checked
wiring, no shorts. `tmc2209ctl -m 2 run ...` (UART VACTUAL) turns the motor;
`tmc2209ctl -m 2 turn ...` (GPIO STEP/DIR) does nothing.

## Checked, found correct (static, on the build host)

- `tmc2209.conf` [motor2]: address 2, `step_line = P9_28 [spi1_cs0]`,
  `dir_line = P9_30 [spi1_d1]`.
- Kernel 6.18.21 `am335x-boneblack.dts` gpio3 line names: offset 16 =
  "P9_30 [spi1_d1]", offset 17 = "P9_28 [spi1_cs0]", so the names match exactly.
- Built `am335x-boneblack-tmc2209.dtb` (decompiled): `tmc2209-gpio-pins` muxes
  0x198 (P9_30) and 0x19c (P9_28) to mode 7, pull-down; it is hogged by
  `am33xx_pinmux` pinctrl-0. Both mcasp0 nodes and `/sound` are disabled, so the
  mcasp0-pins group (which puts 0x19c in mode 2) is never applied.
- uEnv.txt and extlinux.conf both load the tmc2209 dtb; the file is in boot.vfat.
- tmc2209ctl.c: config load, `select_motor`, `move_axes`, `find_line`,
  `request_line`, `set_line` treat every motor the same way. Nothing in the
  code is specific to motor 1. VACTUAL is zeroed and TOFF is set before stepping,
  same as for motor 1.

Conclusion: the software side is correct on paper. Next, check on the running
board whether the pins are muxed and toggling.

## Next checks on the board

1. `cat /proc/device-tree/model`, which should say "+ 2x TMC2209".
2. `mount -t debugfs none /sys/kernel/debug; grep -E "pin 10[23] " /sys/kernel/debug/pinctrl/44e10800.pinmux-pinctrl-single/pins`
   should show value 00000007 for both.
3. DIR test: `tmc2209ctl -m 2 turn 20 30` and then `turn -20 30`. Measure P9_30
   to GND during each move: about 3.3 V one way and 0 V the other.
4. Swap test: move driver 2 STEP/DIR wires to P9_16/P9_18, set motor2
   step_line/dir_line to the motor1 values, and run `-m 2 turn 1`.

## Comments

2026-10-09: user ran checks 1-3 and all passed: the right device tree is
booted, both pins are in mode 7, and P9_30 flips 3.3 V / 0 V with direction.
`-m 2 turn` reports "moved N usteps", yet the motor does not move. So
software stepping finishes and the BBB drives DIR (GPIO3_16). Still unproven:
does P9_28 (GPIO3_17) reach driver 2's STEP input? Next test: the driver
reads back its own pin levels (IOIN STEP bit 7, DIR bit 9; the `status`
decoding is correct). Swap step_line/dir_line in a copy of the config so
P9_28 is held at a steady level during a long move, then read `-m 2 status`
from a second shell.

2026-10-09: root cause found. Swapping the two TMC modules did not help
(the driver is fine). The sysfs blink test (shield removed) shows P9_30
toggles normally, but P9_28 stays at 3.25-3.3 V even when driven low:
something holds the pin high, or its output driver is damaged. Fix: STEP 2
moved to P9_12 (GPIO1_28, pad 0x878, mode 7). The user wanted an even-row
pin. Changed the DTS, tmc2209.conf and readme. The image is rebuilt and the
DTB is checked (pad 0x78 in mode 7, step_line = P9_12). Waiting for the user
to flash, rewire and test `-m 2 turn 1`.

2026-10-09: closed. User flashed the image and rewired STEP 2 to P9_12. Motor 2
now turns with `-m 2 turn`. Root cause: BBB pin P9_28 is stuck high (hardware).
