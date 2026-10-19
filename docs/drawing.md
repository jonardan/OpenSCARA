# Drawing with the arm: SVG to G-code to the board

Three steps, all from the PC:

    python3 tools/svg2gcode.py drawing.svg -o drawing.gcode    # 1. convert
    python3 tools/bbb_send.py drawing.gcode                     # 2. copy to /root/
    picocom -b 115200 /dev/ttyACM0                              # 3. run it:
      scara gcode -n /root/drawing.gcode     # check only: reach, limits
      scara gcode /root/drawing.gcode        # draw (Ctrl-C stops)

Both tools are plain Python 3, standard library only.

## Or draw in the browser: `tools/disegna.html`

Open `tools/disegna.html` in any browser (PC or phone, works offline). It
shows the area the tip can reach with the joint limits of `tmc2209.conf`
(darker: both elbow sides, lighter: one side only), lets you draw in it
(freehand, line, polyline, rectangle, circle), each stroke in the LED
colour picked above the drawing (a palette with the `scara led` colours,
or "+" for any colour; "Colora" recolours a stroke you tap), and saves the drawing as
G-code in the same format as `svg2gcode.py`. Every stroke is checked
live with the same kinematics as `scara gcode -n` (a port of
`kinematics.c` and `line.c`), so a red stroke is one the board would
refuse. "Simula" plays the arm through the drawing; "Apri .gcode" loads
an existing file to look at it. The red dashed line is the table
(`min_y_mm`, 0 = the X axis): nothing can be drawn below it, and a G0
whose joint move would swing the tip below it runs as a straight line,
as on the board.

### Draw straight from the page (USB cable)

Open `tools/disegna.html` from the PC in **Chrome or Edge** (they have
Web Serial; Firefox and the page inside Claude don't), with the board on
the mini-USB cable and picocom closed. "Collega" asks for the port (the
board's `ttyACM0`), "Disegna sul robot" copies the drawing to
`/tmp/disegno.gcode` on the board (one line at a time, like
`bbb_send.py`, then checks the line count) and runs `scara gcode` on it,
showing which stroke it is on. "Ferma" sends Ctrl-C: the arm decelerates
and the LED goes off. The file starts with `G28`, so the arm homes first.
On Linux the user needs the `dialout` group (`sudo usermod -aG dialout
$USER`, then log out and in). The arm lengths and limits are editable
under "Parametri del braccio" if you recalibrate.

## 1. Make the drawing (Inkscape)

- Any SVG works; the page size does not matter, the drawing is scaled to
  fit. Only the outlines are drawn: a filled shape becomes its border.
- Text: Extensions > Text > Hershey Text turns it into single-stroke
  letters, which is what a pen or the LED wants. Normal text must be
  converted first (Path > Object to Path), and then each letter is drawn
  as its outline. Text left as text is skipped with a warning.
- Photos and bitmaps: Path > Trace Bitmap first, then delete the bitmap.
- Lines, rectangles, circles, ellipses, polygons and all path commands
  (curves and arcs included) are understood; clones (`<use>`) are not,
  so Edit > Clone > Unlink Clone first.

## 2. Convert: `tools/svg2gcode.py`

    python3 tools/svg2gcode.py drawing.svg -o drawing.gcode
    python3 tools/svg2gcode.py logo.svg -o logo.gcode --size 120 --center 210,100 --feed 900

| option | default | meaning |
|---|---|---|
| `--size` | 150 | longer side of the drawing, mm |
| `--center` | 200,90 | centre of the drawing, X,Y in mm from the shoulder axis |
| `--rotate` | 0 | turn the drawing counter-clockwise, degrees |
| `--feed` | 1200 | drawing speed, mm/min (1200 = 20 mm/s, the `line_speed` default) |
| `--step` | 1 | length of the straight pieces curves are cut into, mm |

Orientation: seen from above with the arm pointing to the right (+X), the
drawing reads upright, like in any G-code viewer. To read it upright
standing behind the shoulder and looking along the arm (top of the
drawing toward +X), use `--rotate -90`.

The output uses only what `scara gcode` accepts: `G21 G90`, `G28`, then for
each stroke `G0` to its start, `M3`, `G1` pieces (F on the first), `M5`,
and `M2` at the end. Strokes are ordered nearest-first (and reversed when
that is shorter) to cut the travel. The tip LED lights only on the `G1`
lines between `M3` and `M5`, so the `G0` travel between strokes leaves no
trace in a long-exposure photo. `tools/disegna.html` writes the same
shape (its "M3/M5 ... M150" box, on by default), plus an
`M150 R.. U.. B..` before `M3` whenever the next stroke's colour differs
from the last one; "Apri .gcode" reads those colours back. Red is on/off
only on the board (P8_12 has no PWM), so any red above 0 lights it fully.

With a pen there is no lift (no Z): the G0 moves between strokes draw too.
The LED is the intended tool.

### Where the drawing fits

X/Y are mm from the shoulder axis, X along the straight arm. With
L1 = 195.4 and L2 = 166.4 and the joint limits in `tmc2209.conf`
(shoulder -109.25..111.75, elbow -171..166):

- The tip never goes below `min_y_mm` (0: the X axis, where the table
  is), so drawings sit above it. The default, a 150 mm square centred on
  X200 Y90 (X 125..275, Y 15..165), is well inside the reach.
- A bigger drawing has to stay in Y >= 0 and inside the reach; grow the
  size step by step, the base and the cables may limit it first.
- `scara gcode -n` on the board checks every point (reach, limits,
  passing too close to the shoulder) and names the first bad line without
  moving. A G1 stroke keeps the elbow on its side, so a drawing must fit
  with one elbow side.

The examples on the board (`/usr/share/scara/square.gcode`, `spiral.gcode`,
`rgb.gcode`) also sit above the X axis, around X200 Y70..90.

## 3. Copy to the board

### Over the USB console: `tools/bbb_send.py` (works with the current image)

    python3 tools/bbb_send.py drawing.gcode                  # -> /root/drawing.gcode
    python3 tools/bbb_send.py drawing.gcode /root/a.gcode --port /dev/ttyACM1

It uses the same mini-USB cable and `/dev/ttyACM0` as picocom, so close
picocom first. It logs in if the board asks, types the file into
`cat > FILE <<'...'` one line at a time (waiting for the shell's `> `
prompt before each, so nothing overflows), then compares `md5sum` on the
board with the PC's. Files on `/root` stay on the SD card across reboots.

### Other ways, also with the current image

- SD card: power off, put the card in the PC, copy onto the second
  (ext4, "rootfs") partition:
  `sudo cp drawing.gcode /media/$USER/rootfs/root/`, then `sync`, eject.
- Ethernet: cable from the BBB to the router, then on the PC
  `python3 -m http.server 8000` in the folder with the file, and on the
  board `udhcpc -i eth0` (gets an address) and
  `wget http://PC_IP:8000/drawing.gcode -O /root/drawing.gcode`.

### Streaming instead of copying

`scara gcode -` reads stdin and answers `ok` per line, for a PC sender.
Streamed G1 lines can't be joined (the next one isn't there yet), so every
piece of a curve stops at its end and curves stutter. For drawings, copy
the file and run it.

## Proposal: scp over USB (needs a new image)

Not done; it changes the image. Adding a USB network interface next to
the serial console and an SSH server would give `scp drawing.gcode
root@192.168.7.2:` with the same cable:

- `BR2_PACKAGE_DROPBEAR=y` in `robotarm_defconfig`, and a root password
  (`BR2_TARGET_GENERIC_ROOT_PASSWD`): dropbear refuses empty passwords.
- `S30usbgadget`: add an `ecm.usb0` function to the same gadget config as
  `acm.usb0` (module `usb_f_ecm`), then
  `ifconfig usb0 192.168.7.2 netmask 255.255.255.0 up`.
- On the PC the new `enx...`/`usb0` interface gets 192.168.7.1/24 (once,
  in NetworkManager: IPv4 manual, or `sudo ip addr add 192.168.7.1/24 dev usb0`).
