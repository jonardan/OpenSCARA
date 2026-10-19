# Plan: from endstops to G-code

Goal: the OpenSCARA arm reads G-code and moves its tip to X/Y positions.
First use: an LED on the tip, switched by M-codes, drawing in the air for
long-exposure photos. Later the tip tool can change; G-code keeps that easy.

Each step is small, ends with something to try on the BeagleBone, and is
done in its own thread. Do them in order; a step only relies on the ones
above it.

## Arm parameters

These live in `/etc/tmc2209.conf` (source: `software/package/tmc2209ctl/tmc2209.conf`),
not in the code. Values marked TBD come from the user and are filled in by
the step that first needs them.

| Key | Meaning | Value | Needed from step |
|---|---|---|---|
| `steps_per_rev` | full steps per motor revolution | 200 | 4 |
| `microsteps` | TMC2209 microstepping | 16 | 4 |
| `motor_teeth` (per motor) | GT2 teeth on the motor pulley | 20 / 20 | 4 |
| `joint_teeth` (per motor) | GT2 teeth on the joint pulley | 72 (shoulder) / 45 (elbow) | 4 |
| `home_angle` (per motor) | joint angle (deg) when its endstop clicks | 116.75 / 171 | 4 |
| `home_dir` (per motor) | direction to turn to reach the endstop (+1/−1) | 1 / 1 | 3 |
| `min_angle`, `max_angle` (per motor) | safe joint range (deg) | −109.25..111.75 / −171..166 | 4 |
| `endstop_line`, `endstop_nc` (per motor) | GPIO line, normally-closed switch | P9_14 / P9_22, 0 | 2 |
| `link1_mm` | shoulder axis to elbow axis | 195.4 | 5 |
| `link2_mm` | elbow axis to tool tip | 166.4 | 5 |
| `led_red`, `led_green`, `led_blue`, `led_common` | tip RGB LED channels in /sys/class/leds (P8_12, P8_13, P8_19 in the device tree), common cathode or anode | tip-red, tip-green, tip-blue, cathode | 8 |

Mechanics: motor 1 sits on the base and turns link 1 (shoulder). Motor 2
rides on link 1 and turns link 2 (elbow), so the elbow angle is relative to
link 1 and needs no belt-coupling correction.

With these values: shoulder 3.6:1, 200 × 16 × 72 / 20 / 360 = **32 microsteps
per degree**; elbow 2.25:1, 200 × 16 × 45 / 20 / 360 = **20 microsteps per
degree**. Reach: 29.0 mm to 361.8 mm from the shoulder axis.

Conventions (also go in `GLOSSARY.md` in step 4):

- **Joint angle** θ1: shoulder, measured from the +X axis, counter-clockwise
  positive. θ2: elbow, measured from the line of link 1 (0 = arm straight).
- **Origin**: the shoulder axis is (0, 0). X/Y in mm.
- **Home**: the pose where both endstops are pressed; after homing, joint
  angles are known exactly.

## Where the code goes

`tmc2209ctl` stays the low-level tool (driver setup, raw motor moves) and
gains the endstop and homing commands, since those are per-motor. Arm-level
commands (angles, X/Y, G-code) go in a new program `scara` in the same
package, so the driver code is shared rather than copied: step 4 first moves
the UART/GPIO/move code out of `tmc2209ctl.c` into `tmc2209.c` + `tmc2209.h`,
with no behaviour change. Pure math (IK, G-code parsing) goes in files that
also build and run on the PC, so it gets host tests (`make test`) before it
ever touches the arm.

Position between separate commands is kept in `/run/scara.pos` (joint
positions in microsteps, plus a "homed" flag). Disabling the motors or a
reboot clears "homed"; arm-level moves refuse to run until homed.

## Steps

### 1. Endstop pins in the device tree

Mux P9_14 (GPIO1_18) and P9_22 (GPIO0_2) as GPIO inputs with pull-up
(mode 7) in `am335x-boneblack-tmc2209.dts`; update `software/README.md`
(remove "planned").

Test on the BBB: wire one switch, then
`gpioget $(gpiofind "P9_14")` (or by chip/offset) prints 1 when released
and 0 when pressed; same for P9_22. If libgpiod tools are not in the image,
add `BR2_PACKAGE_LIBGPIOD_TOOLS`.

### 2. `tmc2209ctl endstop`

Read each motor's `endstop_line` (honouring `endstop_nc`) and print
`motor1: open / pressed`. Add a `watch` form that prints changes until
Ctrl-C.

Test: press each switch by hand and see the right motor's line change.
Swap the two wires in the config on purpose once to check the motor ↔
switch mapping is what you think it is.

### 3. Homing: `tmc2209ctl home`

For each selected motor: turn toward the switch (`home_dir`) at a slow
speed until it reads pressed, back off a few degrees, approach again at
a slower speed, stop on the click. Give up with an error after a maximum
travel (one joint revolution) so a broken switch can't wind the arm into
its hard stop. If the switch is already pressed at the start, move away
from it first.
The stepping loop checks the endstop between pulses, so this needs a
"move until" variant of `move_axes`. Writes `/run/scara.pos` with
the joint at `home_angle` (0 until it is measured in step 4), homed = 1.

Home one motor at a time: motor 2 (elbow) first, then motor 1 (shoulder),
so the arm is folded to a known pose before the shoulder swings.

Test: home from three different start poses; the arm stops at the same
place each time (mark it with tape). Unplug a switch: homing stops with an
error after the max travel.

### 4. Joint angles: `scara joint A1 A2`

Refactor first (driver code into `tmc2209.c/h`, no behaviour change; test
`tmc2209ctl turn` still works). Then the new `scara` program:

- microsteps per joint degree =
  `steps_per_rev × microsteps × joint_teeth / motor_teeth / 360`
- `scara joint A1 A2` moves both joints to absolute angles (deg) together,
  using the synchronised move, refusing angles outside `min_angle..max_angle`
  and refusing to move if not homed.
- `scara where` prints the joint angles.

Measuring the angles (the user doesn't know them yet, so they are measured
here rather than guessed):

- `scara jog J DEG` turns joint J by DEG relative to where it is, ignoring
  limits, and `scara where` prints the angle relative to home.
- `home_dir`: run `tmc2209ctl -m N turn 0.1` and see which way the joint
  moves; the direction that goes toward the switch is +1, otherwise −1.
- Shoulder `home_angle`: home, then jog the shoulder until link 1 points
  along the +X axis you chose (a line taped on the table). `home_angle` =
  minus the angle jogged.
- Elbow `home_angle`: home, then jog the elbow until link 2 lies straight in
  line with link 1 (0°). `home_angle` = minus the angle jogged.
- Limits: from home, jog each joint slowly toward each side until it is a
  few degrees from hitting something (cables, the other link, the frame);
  note the angle and keep a 5° margin.

Write the values into `tmc2209.conf` and re-home.

Test: home, then `scara joint` to 0°, 90°, back; check with a protractor or
a printed angle gauge. Do 20 back-and-forth moves and home again: the
switch should click at the same count (no lost steps); `scara where` before
re-homing shows how far off it was.

### 5. Kinematics (host-tested)

`kinematics.c`: forward (θ1, θ2 → x, y) and inverse (x, y → θ1, θ2) for
the two-link arm, with an elbow side choice (left/right-handed; default
the one that stays inside the joint limits), and "unreachable" for points
outside `|L1 − L2| ≤ r ≤ L1 + L2` or outside the joint limits.

Test on the PC: `make test` checks IK(FK(θ)) = θ over a grid of angles,
known points (straight arm → (L1+L2, 0)), and unreachable points.

Done: also `scara fk A1 A2` / `scara ik X Y [+|-]` (compute only), and
`scara where` already prints x, y, so step 6 only adds `goto`.

### 6. Move to a point: `scara goto X Y`

IK, then the joint move from step 4. `scara where` also prints x, y.

Test: tape paper under the arm, mark the origin and axes; `goto` a few
points (e.g. corners of a 100 mm square) with a pen or the LED, and
measure the error with a ruler. Errors that grow with distance point to a
wrong link length; rotated points point to a wrong `home_angle`.

Done: `scara goto X Y [+|-] [RPM]`; the square check (sides and
diagonals, no origin needed) is in the board readme.

### 7. Straight lines: `scara line X Y [F mm/s]`

A joint move from A to B traces a curve, not a line. Split the line into
short segments (≈1 mm), IK each point, and stream them through one stepping
loop at constant tip speed, accelerating only at the start and end of the
whole line (no stop at every segment). This is what makes drawings look
right in a long exposure.

Test: draw a 100 mm square with a pen; edges are straight and the arm does
not stutter at segment joins. Check the speed with a stopwatch over a long
line.

Done: `scara line X Y [F]` (F in mm/s, default `line_speed`; tip
acceleration `line_accel`), with the square and the stopwatch check in
the board readme.

### 8. Tip LED: `scara led on|off`

Add `led_line` to the device tree (output, pulled down so the LED is off
at boot) and to the config. LED through a transistor if it draws more than
a few mA (a BBB pin gives ~4–6 mA safely).

Test: `scara led on/off` from the shell; the LED is off after a reboot.

Done (2026-10-12, on P8_12 instead of P9_23): P8_12 is a gpio-leds LED
"tip" in the device tree rather than a GPIO line in the config, because a
line requested by `scara` goes back to an input when it exits, which would
switch the LED off. `led = tip` in the config names it; `scara led` alone
prints the state.

RGB (2026-10-19): red stays on P8_12 (on/off, no PWM there), green on
P8_13 and blue on P8_19 as pwm-leds on EHRPWM2 (0-255). `M150 R U B` sets
the colour in G-code, `scara led COLOR|R G B` by hand, `led_common = anode`
inverts the pins for a common anode LED.

### 9. G-code parser (host-tested)

`gcode.c`: parse one line into a command, separate from moving anything.
First set:

- `G0 X Y` rapid move (LED off), `G1 X Y F` straight line at feed F
  (mm/min, G-code convention)
- `G28` home
- `G90` / `G91` absolute / relative, `G21` mm (only mm supported)
- `M3` / `M5` LED on / off (spindle on/off, what most drawing tools emit);
  `M106` / `M107` as aliases
- `;` and `( )` comments, line numbers `N`, unknown codes → error with the
  line number, Z ignored with a warning

Test on the PC: `make test` with sample lines and a sample file.

Done, with part of step 10 pulled in: `scara gcode [-n] FILE` checks the
whole file (syntax, then every move planned from the previous one, first
unreachable point with its line number), then runs it with `goto`, `line`
and `home`. M3/M5 drive the LED since step 8.

### 10. Run G-code: `scara gcode FILE` (or `-` for stdin)

Execute the parsed commands with steps 3, 6, 7, 8. Before moving, check
every point of the file is reachable and report the first one that isn't
(line number) without moving. Print `ok` after each line so a PC-side
sender can stream it over the USB console. Ctrl-C: LED off, decelerate,
stop.

Test: a square, a circle made of G1 segments, and a short text from a
G-code generator (e.g. Inkscape's G-code export) with the LED; then a long
exposure photo.

Done: the file form came with step 9. `scara gcode -` now streams: each
line is checked from where the arm is, run, and answered `ok` (`error`
stops it). `/usr/share/scara/spiral.gcode` (G1 segments) replaces the circle test.
LED: M3/M5 set the tool, and the LED is on only while a G1 moves with the
tool on (off for G0 travel and G28, so travel moves leave no trace in the
photo). It goes off when the file ends, fails or is stopped (Ctrl-C,
SIGTERM, SIGHUP).

## Later (not planned in detail)

- `G2`/`G3` arcs (expanded into G1 segments).
- LED brightness/colour with PWM (`M3 S`).
- Continuous motion across G1 lines (look-ahead) for smoother curves.
  Done for files: G1 lines turning by at most `corner_deg` run as one
  path; sharper corners still stop. Next would be slowing down to a
  corner speed instead of stopping, and look-ahead when streaming.
- Other tip tools (pen lift on Z, servo).
