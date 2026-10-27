# 📖 Command reference

Everything `scara`, `tmc2209ctl` and the PC tools can do, and every key in
`/etc/tmc2209.conf`. For a first look at the project, start from the
[main README](../README.md); for the wiring, see
[software/README.md](../software/README.md).

- [How the arm sees the world](#how-the-arm-sees-the-world)
- [`scara`: arm commands](#scara-arm-commands) ([homing](#homing), [G-code](#g-code))
- [`tmc2209ctl`: motor and driver commands](#tmc2209ctl-motor-and-driver-commands)
- [Settings: `/etc/tmc2209.conf`](#settings-etctmc2209conf)
- [Drawing: from an SVG to the arm](#drawing-from-an-svg-to-the-arm)

## How the arm sees the world

- **X/Y** are millimetres from the shoulder axis. With the arm straight,
  the tip is on +X at `link1_mm + link2_mm` (361.8 mm).
- **Shoulder angle (A1)** is measured from the +X axis; **elbow angle
  (A2)** from the line of link 1, so `0 0` is the arm straight along +X.
  Counter-clockwise is positive, seen from above.
- **Elbow side**: most points can be reached with the elbow bent either
  way. `+` and `-` pick the side; without it `scara` takes one inside the
  limits, the closer to the current pose.
- **Homed**: every arm move needs `scara home` first. The position lives
  in `/run/scara.pos`, so a reboot, `tmc2209ctl disable`, `run` or `init`
  forget it.
- **Ctrl-C** decelerates any move to a stop.

## `scara`: arm commands

```
scara [-c CONF] COMMAND [ARGS]
```

`-c` (or the `TMC2209_CONF` environment variable) reads another config
file instead of `/etc/tmc2209.conf`. `scara help` prints the list.

| Command | What it does |
|---|---|
| `scara home [1\|2\|all]` | Homes the joints on their endstops, elbow first so the arm is folded before the shoulder swings. Prints the angles at the end. |
| `scara where` | Prints both joint angles and, once homed, the tip X/Y. |
| `scara jog J DEG [RPM]` | Turns joint J (1 shoulder, 2 elbow) by DEG degrees from where it is. **Ignores the limits**: it is the tool for measuring them. |
| `scara joint A1 A2 [RPM]` | Moves the shoulder to A1 and the elbow to A2 degrees (absolute), both together. Refuses angles outside `min_angle..max_angle`. |
| `scara fk A1 A2` | Only computes: the tip X/Y for those joint angles. |
| `scara ik X Y [+\|-]` | Only computes: the joint angles that put the tip on X/Y. |
| `scara goto X Y [+\|-] [RPM]` | Moves the tip to X/Y: `ik`, then the same move as `joint`. The joints move in a straight line, so the tip follows a curve. |
| `scara line X Y [F]` | Moves the tip to X/Y in a **straight line** at F mm/s (default `line_speed`). The elbow stays on its side, and every point is checked before moving. |
| `scara gcode [-n] FILE` | Runs a G-code file (below). `-n` only checks it. |
| `scara gcode [-n] -` | Streams G-code from stdin, answering `ok` or `error` per line. |
| `scara led [COLOR]` | Sets the tip RGB LED by hand: `off`, `on` (white), `red`, `green`, `blue`, `yellow`, `cyan`, `magenta`, `orange`, `purple`, `#rrggbb` or `R G B` (0-255). Alone it prints `led: R G B` or `led: off`. Red (P8_12) is on/off only. |
| `scara ir [F]` | Drives the tip with an IR remote (VS1838 receiver on P8_16). It first asks for the UP, DOWN, RIGHT and LEFT buttons and learns their codes. Then holding an arrow moves the tip smoothly in a straight line at F mm/s (default `line_speed`) and letting go decelerates it to a stop; at the reach or a joint limit it stops by itself. Seen from the remote, up is +X and right is +Y. `max_y_mm` is not enforced (you watch the table). Ctrl-C ends. Needs homing. `scara ir test` only prints the code of each button pressed. |

RPM is the speed of the motor that moves furthest (default `rpm`). If a
line would turn a motor faster than its `rpm`, the whole line runs slower
and `scara` prints `speed lowered to ...`.

Examples:

```bash
scara home
scara where
scara jog 1 -10 10          # shoulder 10 degrees clockwise at 10 rpm
scara joint 0 90            # shoulder to 0, elbow to 90 degrees
scara fk 0 90               # where would the tip be?
scara ik 200 100            # which angles reach X200 Y100?
scara ik 200 100 -          # same point, elbow bent the other way
scara goto 150 20 +
scara line 250 20           # straight edge at line_speed
scara line 250 120 10       # straight edge at 10 mm/s
```

### Homing

<p align="center"><img src="media/homing.gif" width="400" alt="scara home: the arm finds its endstops"></p>

`scara home` homes the elbow first, so the arm is folded before the
shoulder swings, then the shoulder. Each joint turns toward its switch at
`home_rpm`, backs off 5 degrees, comes back at `home_rpm / 5` and stops on
the click; that angle is `home_angle`. It gives up with an error after one
joint revolution without a click (switch unplugged or broken), and moves
off the switch first if it is already pressed.

### G-code

`scara gcode FILE` reads the whole file and checks every line first: a
syntax error, a point out of reach or past a joint limit, or a line that
passes too close to the shoulder is reported with its line number, and
nothing moves. Then it runs, printing each line.

| Code | Meaning |
|---|---|
| `G0 X Y` | move like `goto` |
| `G1 X Y F` | straight line like `line`; F is in **mm/min** and stays for the next lines |
| `G28` | home |
| `G90` / `G91` | absolute / relative coordinates |
| `G21` | millimetres |
| `M3` / `M5` (`M106` / `M107`) | tool on / off: the tip LED lights only while a `G1` draws after `M3`; it is off during `G0`, `G28` and after `M5`, and off when the file ends, fails or is stopped |
| `M150 R U B [P]` | tip LED colour, Marlin style: R red, U green, B blue, 0-255 each, a missing one is 0; P (0-255) scales all three. It stays until the next `M150` (white before the first) and recolours a line being drawn at once |
| `M2` / `M30` | end of program |
| `Z` | accepted and ignored with a warning (the arm has no Z) |

A line with only X/Y repeats the last G0/G1. Comments (`;` and `( )`),
`N` line numbers, `*` checksums and `%` lines are skipped. Anything else,
such as `G2` arcs or `G20` inches, stops the check with the line number.

Consecutive G1 lines at the same feed run as **one smooth path** while the
direction turns by at most `corner_deg` degrees, so a curve made of short
segments does not stutter. A sharper corner, a G0, a feed change or an
M3/M5 stops there. After `G28` the arm is folded, so start with a `G0`.

```bash
scara gcode -n /usr/share/scara/square.gcode   # check only
scara gcode /usr/share/scara/square.gcode      # 100 mm square, stops at each corner
scara gcode /usr/share/scara/spiral.gcode      # one continuous spiral around X200 Y-90
```

`scara gcode -` is for a sender on the PC that waits for an answer to each
line. Each line is checked from where the arm is, run, then answered `ok`;
a bad line is answered `error` (the reason on stderr) and stops it, and so
do Ctrl-C, Ctrl-D and `M2`. Streamed G1 lines can't be joined, because the
next one isn't there yet, so for drawings copy the file and run it.

```
scara gcode -
G28
G0 X200 Y-90                -> moves, then ok
G1 X200 Y80 F600            -> line at 10 mm/s, then ok
G1 X500                     -> error: out of reach, nothing moves
```

## `tmc2209ctl`: motor and driver commands

The low-level tool: one motor at a time in motor revolutions, with no
idea of angles or X/Y. Use it for setup and testing.

```
tmc2209ctl [-m 1|2|all] [-d DEV] [-a ADDR] [-b BAUD] [-c CONF] COMMAND [ARGS]
```

`-m` picks the motor (default 1); with `all` the command runs on every
motor and moves run at the same time. `-d`, `-a` and `-b` override the
UART device, address and baud rate from the config.

| Command | What it does |
|---|---|
| **Setup** | |
| `init` | Applies `/etc/tmc2209.conf` to the driver (motor off). Runs at boot. |
| `status` | Driver state, temperature, faults. |
| `motors` | The configured motors: address and pins. |
| `enable` / `disable` | Energises / releases the motor coils. |
| `current MA [HOLD%]` | RMS run current, and the hold current as a percent of it. |
| `microsteps N` | 1, 2, 4, 8, 16, 32, 64, 128 or 256. |
| `mode stealth\|spread` | stealthChop (quiet) or spreadCycle (more torque). |
| **Motion** | |
| `turn REVS [RPM]` | Exact move of REVS motor revolutions; negative is reverse. |
| `move USTEPS [RPM]` | Exact move in microsteps. |
| `sync R1 R2 [RPM]` | Motor 1 turns R1 and motor 2 R2 revolutions together; both finish at the same time. |
| `run RPM` | Spins continuously (timed by the driver over UART). |
| `stop` | Ramps a `run` down to zero. |
| **Endstops** (every motor, `-m` ignored) | |
| `endstop` | Prints each switch: released or pressed. |
| `endstop watch` | Prints every change until Ctrl-C. |
| `home [1\|2\|all]` | Drives each joint onto its switch, motor 2 first. Needs `home_dir`. |
| **Other** | |
| `read REG` | Reads a register, by name (like `CHOPCONF`) or number. |
| `write REG VALUE` | Writes a register. |
| `shell` | Interactive prompt for the commands above; `motor 1\|2\|all` picks the motor. |

```bash
tmc2209ctl -m all status
tmc2209ctl -m 2 turn -2.5 120    # motor 2: 2.5 revolutions backwards at 120 rpm
tmc2209ctl sync 5 -2 90          # motor 1 +5 rev and motor 2 -2 rev together
tmc2209ctl endstop watch
tmc2209ctl -m 2 current 800 30   # motor 2: 800 mA RMS, 30 % hold
tmc2209ctl -m all disable        # release both motors
```

## Settings: `/etc/tmc2209.conf`

On the board the file is `/etc/tmc2209.conf`; in the repo it is
[software/package/tmc2209ctl/tmc2209.conf](../software/package/tmc2209ctl/tmc2209.conf).
Changes on the board are kept on the SD card and read by the next
command, but a new image overwrites them, so copy anything that works
into the repo. Keys at the top apply to both motors; `[motor1]` (shoulder)
and `[motor2]` (elbow) override them for one motor.

| Key | Now | Meaning |
|---|---|---|
| `current_ma` | 600 | RMS run current, mA |
| `hold_percent` | 50 | standstill current, percent of run current |
| `microsteps` | 16 | microsteps per full step |
| `mode` | stealthchop | or `spreadcycle` for more torque at speed |
| `invert_dir` | 0 | 1 flips a motor's direction (then flip `home_dir` too) |
| `rpm` | 60 | default motor speed, and the cap `line` and G1 slow down to |
| `accel` | 2 | motor acceleration for `turn`, `move`, `joint`, `goto`, rev/s² |
| `home_rpm` | 20 | homing speed; the slow second touch is `home_rpm / 5` |
| `link1_mm` / `link2_mm` | 195.4 / 166.4 | shoulder axis to elbow axis, elbow axis to the tip |
| `line_speed` | 20 | tip speed for `line` and G1 without F, mm/s |
| `line_accel` | 400 | tip acceleration at the start and end of a line or path, mm/s² |
| `corner_deg` | 15 | G1 lines turning by at most this many degrees run as one path; 0 stops at every line |
| `max_y_mm` | 0 | the table is on the +Y side: the tip never goes past this Y (0 = the X axis). Points, lines and G1 past it are refused; a `goto`/`G0` whose joint move would swing the tip past it turns the shoulder and the elbow one at a time, or goes straight. `off` removes the limit |
| `led_red` / `led_green` / `led_blue` | tip-red / tip-green / tip-blue | the tip LED's channels in `/sys/class/leds` (P8_12, P8_13, P8_19 in the device tree) |
| `led_common` | cathode | `anode` for a common anode RGB LED (its long leg on 3.3 V): the pins are inverted |
| `home_dir` | 1 / 1 | 1 if a positive `turn` moves the joint toward its switch, -1 if away; 0 refuses to home |
| `home_angle` | 116.75 / 171 | joint angle when the switch clicks, measured with `scara jog` |
| `min_angle` / `max_angle` | -109.25..111.75 / -171..166 | safe joint range for `joint`, `goto`, `line`, G-code |
| `motor_teeth` / `joint_teeth` | 20:72 / 20:45 | GT2 pulley teeth, motor and joint |

The two-value rows are shoulder / elbow. The pins (`step_line`,
`dir_line`, `endstop_line`), `address` and the UART `device` are in the
same file and explained in [software/README.md](../software/README.md).
How to measure `home_dir`, `home_angle` and the limits is in the
[bring-up guide](../software/board/beaglebone/readme.txt).

## Drawing: from an SVG to the arm

Three steps from the PC; the full guide with Inkscape tips is
[docs/drawing.md](drawing.md). Both tools are Python 3 with the
standard library only.

```bash
python3 tools/svg2gcode.py drawing.svg -o drawing.gcode
python3 tools/bbb_send.py drawing.gcode
picocom -b 115200 /dev/ttyACM0
```

Then on the board:

```bash
scara gcode -n /root/drawing.gcode
scara gcode /root/drawing.gcode
```

### `tools/svg2gcode.py`

Turns the outlines of an SVG (paths with curves and arcs, lines,
rectangles, circles, ellipses, polygons) into G-code that `scara gcode`
accepts. Strokes are ordered nearest-first to cut the travel between them.

| Option | Default | Meaning |
|---|---|---|
| `-o FILE` | stdout | output G-code file |
| `--size` | 150 | longer side of the drawing, mm |
| `--center` | 200,-90 | centre of the drawing, X,Y in mm |
| `--rotate` | 0 | turn the drawing counter-clockwise, degrees |
| `--feed` | 1200 | drawing speed, mm/min (1200 = 20 mm/s) |
| `--step` | 1 | length of the straight pieces curves are cut into, mm |

```bash
python3 tools/svg2gcode.py logo.svg -o logo.gcode --size 120 --center 210,-100 --feed 900
```

A 150 mm drawing centred on X200 Y-90 fits comfortably; `scara gcode -n`
tells you if a bigger one doesn't.

### `tools/bbb_send.py`

Copies a text file to the board over the same mini-USB cable as the
console, then compares the `md5sum` on both sides. Close picocom first,
since only one program can hold the port. Files in `/root` stay on the SD
card across reboots.

```bash
python3 tools/bbb_send.py drawing.gcode                  # to /root/drawing.gcode
python3 tools/bbb_send.py drawing.gcode /root/a.gcode --port /dev/ttyACM1
```

### Example drawings

| File | What it draws |
|---|---|
| `/usr/share/scara/square.gcode` (on the board) | a 100 mm square, stopping at each corner |
| `/usr/share/scara/spiral.gcode` (on the board) | a spiral from 5 to 80 mm radius, one continuous move |
| [`tools/forme.gcode`](../tools/forme.gcode) | a spiral, a cross and a heart at 150 mm/s, each one smooth stroke |
| [`tools/disegno.gcode`](../tools/disegno.gcode) | a paper plane, made from [`tools/plane-svgrepo-com.svg`](../tools/plane-svgrepo-com.svg) |
