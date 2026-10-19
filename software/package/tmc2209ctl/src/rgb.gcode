; RGB LED test: the 100 mm square, each side in its own colour
; (scara gcode /usr/share/scara/rgb.gcode; long-exposure photo)
G21 G90
G28
G0 X150 Y-50
M150 R255 U0 B0      ; red (P8_12: on/off only)
M3
G1 X250 Y-50 F1200
M150 R0 U255 B0      ; green: changes while the LED is on
G1 X250 Y50
M150 R0 U0 B255      ; blue
G1 X150 Y50
M150 R255 U64 B0     ; orange
G1 X150 Y-50
M5
M2
