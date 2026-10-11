# Redo the 100 mm square check once the belts are tightened

Status: ready-for-human
Category: bug

Step 6 (`scara goto`), first square check on 2026-10-11: the computed tip
was within 0.05 mm of every corner, but on paper the two sides along Y
(x = 150 and x = 250, the ones the shoulder draws) came out a bit longer
than 100 mm, the X sides were right, and the whole square was slightly
rotated. jo's belts are loose and the pulley sometimes slips under the
belt, which is the likely cause.

To do: tighten the belts, re-home, redo the square from
`software/board/beaglebone/readme.txt` ("Moving to X/Y: scara goto") and
measure sides and diagonals. If the Y sides are still long and the square
still rotated, suspect the shoulder ratio (72/20 teeth, 32 microsteps/deg):
one scale error explains both. A pure rotation alone points to the shoulder
home_angle; `scara joint 0 0` should lie on the +X line.

## Comments
