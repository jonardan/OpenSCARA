; 100 mm square, like the one in the board readme (scara gcode square.gcode)
%
G21 G90          (mm, absolute)
G28              ; home
G0 X150 Y-50     ; first corner, rapid
M3               ; tool on: the LED lights on the G1 lines
G1 X250 Y-50 F1200
X250 Y50         ; still G1, still 1200 mm/min = 20 mm/s
N10 G1 X150 Y50
G1 X150 Y-50 Z0  ; Z: no Z axis, ignored with a warning
M5
G91              ; relative
G0 Y20
G90
M2
%
