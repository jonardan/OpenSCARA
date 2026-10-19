BeagleBone Black + 2x BIGTREETECH TMC2209 (UART mode)
=====================================================

Two TMC2209 V1.3 modules, each driving one stepper motor. Both share ONE
UART bus (UART1) and are told apart by their address, which is set with
the MS1/MS2 pins. Each motor has its own STEP and DIR GPIO.

Build
-----
  This repo is a Buildroot BR2_EXTERNAL tree. Run make from the Buildroot
  source directory (~/Desktop/buildroot-2026.05.3):

  make BR2_EXTERNAL=~/Desktop/robot-arm/software robotarm_defconfig   # once
  make

  Buildroot remembers the BR2_EXTERNAL path, so later builds are just make.

  After editing only the programs: make tmc2209ctl-rebuild all
  After editing only the DTS:       make linux-rebuild all

Result: output/images/sdcard.img

Flash and boot
--------------
  lsblk                                     # find the SD card, e.g. /dev/sdb
  sudo dd if=output/images/sdcard.img of=/dev/sdX bs=4M conv=fsync status=progress

  Insert the card into the unpowered BBB, hold the S2 (BOOT) button, plug in
  the mini-USB cable, release S2 after the LEDs start blinking.

  Console: the mini-USB port shows up on the PC as /dev/ttyACM0
    picocom -b 115200 /dev/ttyACM0          # login: root, no password

Pin summary
-----------

                        DRIVER 1 (motor 1)       DRIVER 2 (motor 2)
  UART address          0                        2
  MS1                   GND                      GND
  MS2                   GND                      3.3V (BBB P9_3)
  STEP                  P9_16  (GPIO1_19)        P9_12  (GPIO1_28) *
  DIR                   P9_18  (GPIO0_4)         P9_30  (GPIO3_16)
  PDN_UART (pin 4)      shared UART bus          shared UART bus
  EN, CLK               GND                      GND
  VIO                   P9_3 (3.3V)              P9_3 (3.3V)
  GND (logic)           P9_2                     P9_2 / P9_1

  Shared UART bus:  P9_24 UART1_TX --[1 kOhm]--+--> driver 1 PDN_UART
                    P9_26 UART1_RX ------------+--> driver 2 PDN_UART
  (ONE 1 kOhm resistor for both drivers; RX connects to the bus directly)

  * STEP 2 USED TO BE P9_28
    On the BeagleBone Black, P9_28 is wired to the HDMI chip as the audio
    data line (McASP0 AXR2). Even with HDMI audio disabled in this image's
    device tree, P9_28 stayed at ~3.3 V when driven low (measured with the
    shield removed), so motor 2 never saw a STEP pulse. STEP 2 is now on
    P9_12. HDMI audio stays disabled; HDMI video still works.

  UART address = MS1 + 2 x MS2   (pin to GND = 0, pin to 3.3V = 1)
      MS2   MS1   address
      GND   GND   0   <- driver 1
      GND   3.3V  1
      3.3V  GND   2   <- driver 2
      3.3V  3.3V  3
  The MS pins have internal pull-downs: an UNCONNECTED pin reads as GND.
  Tie the "high" pin to 3.3V (P9_3/P9_4), never to 5 V or 12 V.
  In UART mode MS1/MS2 do NOT set microstepping (the program does that).

Wiring (power everything OFF while wiring)
------------------------------------------

  1) SIGNALS (BBB P9 header -> TMC2209 left/logic headers)

  BEAGLEBONE BLACK P9                              DRIVER 1  (address 0)
                                                   +-----------------+
  P9_1  GND ●──────────────────────────┬───────────┤ 1  EN           |
                                       ├───────────┤ 2  MS1          |
                                       ├───────────┤ 3  MS2          |
  P9_24 UART1_TX ●──[ 1 kΩ ]──┐        │           |                 |
                              ├─ BUS ──┼───────────┤ 4  PDN_UART     |
  P9_26 UART1_RX ●────────────┘   │    │           | 5  (leave open) |
                                  │    ├───────────┤ 6  CLK          |
  P9_16 GPIO1_19 ●────────────────┼────┼───────────┤ 7  STEP         |
  P9_18 GPIO0_4  ●────────────────┼────┼───────────┤ 8  DIR          |
                                  │    │           +-----------------+
                                  │    │
                                  │    │           DRIVER 2  (address 2)
                                  │    │           +-----------------+
                                  │    ├───────────┤ 1  EN           |
                                  │    ├───────────┤ 2  MS1          |
  P9_4  3.3V     ●────────────────┼────┼───────────┤ 3  MS2          |
                                  └────┼───────────┤ 4  PDN_UART     |
                                       │           | 5  (leave open) |
                                       └───────────┤ 6  CLK          |
  P9_12 GPIO1_28 ●─────────────────────────────────┤ 7  STEP         |
  P9_30 GPIO3_16 ●─────────────────────────────────┤ 8  DIR          |
                                                   +-----------------+
  (P9_3 and P9_4 are both 3.3V; P9_3 feeds VIO below, P9_4 is handy for
   MS2 of driver 2. Either can be used for both.)

  2) POWER (each TMC2209 right/power header - same for both drivers)

                                                   RIGHT header (power)
                                                   +-----------------+
  12V PSU  V+ ●────────────────────┬───────────────┤ 1  VM           |
                              (+) ===  100 uF      |                 |
                              (-) ===  >= 25 V     |                 |
  12V PSU  V- ●────────────────────┴───────────────┤ 2  GND          |
                                   ┌───────────────┤ 3  A2           |
                 motor coil A ─────┴───────────────┤ 4  A1           |
                                   ┌───────────────┤ 5  B1           |
                 motor coil B ─────┴───────────────┤ 6  B2           |
  BBB P9_3  3.3V ●─────────────────────────────────┤ 7  VIO          |
  BBB P9_2  GND  ●─────────────────────────────────┤ 8  GND          |
                                                   +-----------------+
  Wire both drivers' VM/GND to the same 12 V supply (each driver with its
  own 100 uF capacitor next to it), both VIO to 3.3V, both logic GND to a
  BBB GND (P9_1, P9_2, P9_43..P9_46 are all GND).
  (The module GND pins are connected on the module, so the BBB GND and
   the 12 V supply V- end up common - that is required.)

  3) WHERE THE PINS ARE on the BeagleBone Black (top view)

        5V barrel jack   Ethernet
       +----[====]------[======]------+
       | P9  1 ● ● 2        P8 1 ● ● 2|   <- pins 1/2 at the jack end
       |     3 ● ● 4           3 ● ● 4|
       |       ...               ...  |
       |    11 ● ● 12   STEP 2 = 12   |
       |       ...                    |
       |    15 ● ● 16   STEP 1 = 16   |
       |    17 ● ● 18   DIR 1  = 18   |
       |       ...                    |
       |    23 ● ● 24   UART TX = 24  |
       |    25 ● ● 26   UART RX = 26  |
       |    27 ● ● 28   (do not use)  |
       |    29 ● ● 30   DIR 2  = 30   |
       |       ...                    |
       |    45 ● ● 46                 |
       +----------[mini-USB]----------+
  Trust the "P9" and "1 2 / 45 46" silkscreen printed next to the header.
  Check with a multimeter (BBB on USB): P9_3 to P9_1 must read 3.3 V.

  Full table:
    BBB P9_1  (GND)       -> both drivers: EN, CLK; driver 1: MS1, MS2;
                             driver 2: MS1
    BBB P9_2  (GND)       -> both drivers: GND (logic side, power header pin 8)
    BBB P9_3  (3.3V)      -> both drivers: VIO
    BBB P9_4  (3.3V)      -> driver 2: MS2
    BBB P9_16 (GPIO1_19)  -> driver 1: STEP
    BBB P9_18 (GPIO0_4)   -> driver 1: DIR
    BBB P9_12 (GPIO1_28)  -> driver 2: STEP
    BBB P9_30 (GPIO3_16)  -> driver 2: DIR
    BBB P9_24 (UART1_TX)  -> 1 kOhm resistor -> PDN_UART (pin 4) of BOTH
    BBB P9_26 (UART1_RX)  -> PDN_UART (pin 4) of BOTH, directly
    12V PSU  V+           -> VM of both drivers
    12V PSU  V- / COM     -> GND (power side, next to VM) of both drivers
    100 uF / 25 V (or more) electrolytic across VM and GND, at each module
    Motor coil A -> A1/A2, coil B -> B1/B2 (find pairs with an ohmmeter)

  Rules:
  * Never feed 12 V to any BBB pin. The BBB is powered by its mini-USB cable.
  * Never plug/unplug a motor while 12 V is on: it destroys the driver.
  * A TMC2209 only answers on UART when its VM (12 V) is present.
  * 3.3 V logic only: P9_3/P9_4, never P9_5..P9_8 (5 V) for VIO or MS2.
  * Never ground PDN_UART: the UART stops working ("no UART echo").
  * The two drivers must have different addresses.

P9 pins that do NOT work for STEP/DIR (for future changes)
-----------------------------------------------------------
  P9_1-4, 43-46        power / GND
  P9_5-10              5 V, power button, reset
  P9_32-40             analog (AIN0-6, max 1.8 V, inputs only)
  P9_19/20             I2C2 (cape EEPROM)
  P9_24/26             UART1 (the TMC bus)
  P9_25/28/29/31       HDMI audio (P9_28 also stuck high on our board)
  P9_41/42             each wired to two processor pins - avoid
  Endstops:   P9_14 (motor 1), P9_22 (motor 2)
  Still free: P9_11, 13, 15, 17, 21, 23, 27

Using it
--------
  -m selects the motor: -m 1 (default), -m 2 or -m all.

  tmc2209ctl motors               # list motors, addresses, pins
  tmc2209ctl -m all status        # check both drivers
  tmc2209ctl -m all init          # apply /etc/tmc2209.conf (done at boot too)

  tmc2209ctl turn 1               # motor 1: one revolution
  tmc2209ctl -m 2 turn -2.5 120   # motor 2: 2.5 rev backwards at 120 rpm
  tmc2209ctl -m all turn 3        # both: 3 revs, at the same time
  tmc2209ctl sync 5 -2 90         # together: motor 1 +5 rev, motor 2 -2 rev;
                                  #   the longer move runs at 90 rpm and
                                  #   both start and finish together

  tmc2209ctl endstop              # each switch: released or pressed
  tmc2209ctl endstop watch        # print every change until Ctrl-C
  tmc2209ctl home                 # home motor 2 (elbow), then motor 1
  tmc2209ctl home 2               # home only motor 2

  tmc2209ctl -m all run 200       # both spin continuously (hardware-timed)
  tmc2209ctl -m 2 run -60         # change only motor 2's speed/direction
  tmc2209ctl -m all stop

  tmc2209ctl -m 2 current 800 30  # motor 2: 800 mA RMS run, 30 % hold
  tmc2209ctl -m all disable       # release both motors

  tmc2209ctl shell                # interactive prompt:
    tmc1> motor 2                 #   choose motor 2
    tmc2> turn 1
    tmc2> motor all
    tmc*> status
    tmc*> sync 2 4

  Ctrl-C during turn/move/sync decelerates to a stop.
  Separate commands for different motors can also run at the same time
  (e.g. one in the background with &); the shared UART is locked so they
  don't collide.

Homing
------
  "home" turns each joint toward its switch at home_rpm, backs off 5
  degrees, comes back at home_rpm / 5 and stops on the click. It gives up
  with an error after one joint revolution (switch unplugged or broken),
  and moves off the switch first if it is already pressed. The result
  goes to /run/scara.pos; disable, run, init or a reboot clear "homed".

  First set home_dir for each motor in /etc/tmc2209.conf (homing refuses
  while it is 0). Check it safely, one motor at a time, with a hand on
  the 12V switch:

    tmc2209ctl -m 2 turn 0.05 10    # small, slow move of the elbow
    tmc2209ctl -m 2 turn -0.05 10   # and back

  If the positive turn moved the joint toward its switch, home_dir = 1,
  otherwise -1. Same with -m 1 for the shoulder. Then try "home 2" alone
  with the joint well away from the switch; if it runs away from the
  switch, cut the 12V and flip home_dir.

Joint angles: scara
-------------------
  "scara" works in joint angles (degrees) instead of motor turns. Joint 1
  is the shoulder (motor 1), joint 2 the elbow (motor 2). The shoulder
  angle is measured from the +X axis, the elbow from the line of link 1
  (0 = arm straight), counter-clockwise positive seen from above.

  scara home                      # home elbow, then shoulder
  scara where                     # print both joint angles
  scara joint 0 90                # shoulder to 0, elbow to 90 deg, together
  scara jog 1 -10 10              # shoulder 10 deg clockwise at 10 rpm

  "joint" needs both joints homed and refuses angles outside min_angle..
  max_angle; "jog" ignores the limits (it is for measuring them).

  Measuring home_angle and the limits (once, or after moving a switch).
  Keep a hand on the 12V switch and jog slowly (the last number is rpm):

  1. Direction: "scara jog 1 5 10" must turn link 1 counter-clockwise seen
     from above, and "scara jog 2 5 10" link 2 counter-clockwise relative
     to link 1. If one turns clockwise, set invert_dir = 1 in that motor's
     section and flip its home_dir (1 <-> -1).
  2. Tape a line on the table from the shoulder axis: that is +X.
  3. scara home, then jog the shoulder (scara jog 1 DEG 10, in small
     steps) until link 1 lies along the tape. scara where prints A1:
     home_angle of [motor1] = -A1.
  4. Jog the elbow until link 2 is straight in line with link 1. scara
     where prints A2: home_angle of [motor2] = -A2.
  5. Put both values in /etc/tmc2209.conf and run scara home again;
     scara where now shows the real angles.
  6. Limits: jog each joint slowly toward each side until it is a few
     degrees from hitting something (cables, the other link, the frame),
     read scara where, and keep a 5 degree margin: min_angle / max_angle.

  /etc is on the SD card, so the values survive a reboot; copy them into
  software/package/tmc2209ctl/tmc2209.conf so the next image has them.

X/Y: scara fk / ik
------------------
  X/Y are in mm from the shoulder axis; joint angles 0 0 put the tip at
  (link1_mm + link2_mm, 0). Two commands only compute, nothing moves:

  scara fk 0 90                   # tip X/Y for shoulder 0, elbow 90 deg
  scara ik 200 100                # joint angles that put the tip there
  scara ik 200 100 -              # same point with the elbow bent the other way

  Most points have two solutions, mirror images: the elbow bent counter-
  clockwise (+, elbow angle > 0) or clockwise (-). Without + or -, ik
  takes one inside the joint limits, the closer to the current pose when
  the arm is homed. Points too far, too close or only reachable outside
  min_angle..max_angle are an error. After homing, scara where also prints
  the tip X/Y. The math is tested on the PC:
  make -C software/package/tmc2209ctl/src test

Moving to X/Y: scara goto
-------------------------
  scara goto X Y [+|-] [RPM] does ik, then the same move as scara joint
  (both joints together, needs homing, refuses points outside the limits).
  Give + or - to keep the elbow on one side between moves.

  Checking the arm with a 100 mm square: tape paper under the tip with a
  pen (or the LED) on it, then

  scara home
  scara goto 150 -50 +            # mark the dot, then the next corner
  scara goto 250 -50 +
  scara goto 250 50 +
  scara goto 150 50 +

  Measure the four sides (100 mm) and both diagonals (141.4 mm); this
  needs no origin on the paper. Sides too long or too short, more so on
  the far edge (x 250) than the near one: link1_mm / link2_mm are off.
  Right sizes but the square turned, or the same point landing elsewhere
  with + and -: a home_angle is off (shoulder: turned; elbow: + and -
  disagree).

Straight lines: scara line
--------------------------
  scara line X Y [F] moves the tip from where it is to X/Y (mm) in a
  straight line at F mm/s (default line_speed in the config, 20). goto
  moves the joints, so the tip follows a curve; line splits the path into
  1 mm segments and streams them through one stepping loop, accelerating
  only at the start and the end (line_accel, mm/s^2). The elbow stays on
  the side it is on; every point is checked before moving, and a line
  that would leave the reach or the limits, or pass too close to the
  shoulder, is refused with the first bad point. If a motor would go over
  its rpm the whole line runs slower ("speed lowered to ...").

  Drawing the 100 mm square with straight edges:

  scara home
  scara goto 150 -50 +
  scara line 250 -50
  scara line 250 50
  scara line 150 50
  scara line 150 -50

  The edges should be straight (put a ruler on them) and the arm should
  not stutter halfway. Speed check: scara line 150 -150 then
  time "scara line 150 150 10": 300 mm at 10 mm/s is about 30 s.

G-code: scara gcode
-------------------
  scara gcode FILE runs a G-code file:
  G0 X Y is goto, G1 X Y F is line (F in mm/min, kept for the next
  lines; default line_speed), G28 homes, G90 / G91 absolute / relative,
  G21 mm. A line with only X/Y repeats the last G0/G1. Comments ; and
  ( ), N line numbers and % are skipped. M3 / M5 (M106 / M107) switch
  the tool on / off: the tip LED lights only while a G1 draws after M3,
  never during G0 or G28 (see "Tip LED" below). Z is ignored with a
  warning; anything else, like G2 arcs or G20 inches, stops with the
  line number.

  The whole file is checked first: a syntax error or a point out of reach
  is reported with its line, and nothing moves. scara gcode -n FILE only
  checks. Each line is printed as it runs; Ctrl-C stops the move and the
  file. After G28 the arm is folded, so the first move should be a G0: a
  G1 straight out of the home pose leaves the joint limits.

  The square again, from a file installed on the image:

  scara gcode -n /usr/share/scara/square.gcode
  scara gcode /usr/share/scara/square.gcode
  scara gcode /usr/share/scara/rgb.gcode   # each side a colour

  A spiral around X200 Y0, 5 to 80 mm radius in 5 turns, as 360 G1
  segments (about a minute at 20 mm/s). In a file,
  consecutive G1 lines at the same feed run as one path without slowing
  down at the joins, as long as the direction turns by at most corner_deg
  (15) degrees; sharper corners, like the square's, still stop:

  scara gcode /usr/share/scara/spiral.gcode

  scara gcode - streams stdin, for a sender on the PC that waits for an
  answer to each line before sending the next: every line is checked
  from where the arm is (reach, joint limits), run, then answered "ok".
  A bad line is answered "error" (the reason on stderr) and stops it;
  so does Ctrl-C. Ctrl-D or M2 ends it. Streamed lines can't be joined
  (the next one isn't there yet), so each G1 stops at its end. By hand:

  scara gcode -
  G28
  G0 X200 Y0                  -> moves, then ok
  G1 X200 Y80 F600            -> line at 10 mm/s, then ok
  G1 X500                     -> error: out of reach, nothing moves

  Drawings: docs/drawing.md turns an SVG into G-code
  (tools/svg2gcode.py) and copies it here over this USB cable
  (tools/bbb_send.py).
Tip LED: scara led
------------------
  An RGB LED (4 legs), one resistor per colour. Common cathode (the long
  leg to GND, the default):

  P8_12 o──[ 330 Ohm ]──|>|── red   ─┐
  P8_13 o──[ 150 Ohm ]──|>|── green ─┼──o P8_2 GND   (long leg)
  P8_19 o──[ 150 Ohm ]──|>|── blue  ─┘

  Common anode (the long leg to 3.3 V): the same resistors from each pin
  to its leg, the long leg to P9_3 (3.3 V), and in /etc/tmc2209.conf

  led_common = anode

  The resistors keep each pin at about 4 mA, the most a BBB pin should
  give: red drops about 2 V, green and blue about 2.7-3 V. A single red
  LED still works on P8_12 alone (330 Ohm to P8_2). For a brighter LED,
  drive each colour through an NPN transistor (e.g. 2N2222: pin --[1
  kOhm]-- base, emitter to GND, LED leg and its resistor from 5 V (P9_7)
  to the collector; common anode LED, led_common stays cathode).

  P8_12 has no PWM, so red is only on or off (any value above 0 is full
  red). P8_13 and P8_19 are EHRPWM2B / 2A at 10 kHz, so green and blue
  have 256 levels. The device tree makes them the kernel LEDs tip-red,
  tip-green and tip-blue (/sys/class/leds/), all off at boot. The kernel
  holds the pins, so the LED stays on after the command exits.

  scara led red                   # or green blue yellow cyan magenta
                                  # orange purple white (= on)
  scara led 0 255 128             # R G B, 0-255
  scara led '#00ff80'             # the same, as in the drawing page
  scara led off
  scara led                       # prints led: R G B, or off

  In G-code the LED follows M3 / M5, but only on the drawn lines: after
  M3 it lights at the start of each G1 and goes off for any G0 (travel)
  or G28; M5 switches it off. M150 R.. U.. B.. (0-255, as in Marlin; U is
  green, a missing one is 0) sets the colour of the next G1 lines, or of
  the one being drawn; it is white until the first M150. When the file ends, fails or is stopped
  with Ctrl-C (or the console closes), scara switches it off. The square
  has M3 / M5 around its four sides, so the LED is off while it goes to
  the first corner and on along the edges:

  scara gcode /usr/share/scara/square.gcode
  scara gcode /usr/share/scara/rgb.gcode   # each side a colour

Configuration: /etc/tmc2209.conf
--------------------------------
  Keys at the top apply to every motor (current, microsteps, speed, ...);
  [motor1] / [motor2] sections hold each motor's address and STEP/DIR
  pins and can override any other key for that motor, e.g.

    [motor2]
    address = 2
    step_line = P9_12
    dir_line = P9_30 [spi1_d1]
    current_ma = 800
    invert_dir = 1

  GPIO line names are the BeagleBone Black device tree names
  (listed by "tmc2209ctl motors").
