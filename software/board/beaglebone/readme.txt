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

  After editing only the program:  make tmc2209ctl-rebuild all
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
