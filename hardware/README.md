# 🖨️ OpenSCARA hardware

The 3D-printed parts of the [OpenSCARA](../README.md) arm, exported from
the Fusion design "robot claude" (v47).

## 🌟 Highlights

- 📦 **Ready to print**: one binary STL per part, in millimetres.
- ✏️ **Editable**: the full Fusion design is included, so you can adapt any part.
- 🔩 **Few bought parts**: two NEMA 17 motors, 20T GT2 pulleys and belts, endstop switches.

## 📁 Files

| Path | What it is |
|---|---|
| [`stl/`](stl/) | one STL per printable part |
| [`fusion/robot_claude.f3d`](fusion/robot_claude.f3d) | the editable Fusion design, with every body including the reference parts |

## 🧩 Parts

| Part | File |
| --- | --- |
| Base, bottom plate | [`stl/Base_PiastraFondo.stl`](stl/Base_PiastraFondo.stl) |
| Base, front support | [`stl/Base_SupportoAnteriore.stl`](stl/Base_SupportoAnteriore.stl) |
| Base, rear support | [`stl/Base_SupportoPosteriore.stl`](stl/Base_SupportoPosteriore.stl) |
| Shoulder, 72T pulley | [`stl/Spalla_Puleggia72T.stl`](stl/Spalla_Puleggia72T.stl) |
| Upper arm, motor side | [`stl/Braccio_LatoMotore.stl`](stl/Braccio_LatoMotore.stl) |
| Upper arm, front side | [`stl/Braccio_LatoAnteriore.stl`](stl/Braccio_LatoAnteriore.stl) |
| Upper arm, elbow motor plate | [`stl/Braccio_PiastraMotoreGomito.stl`](stl/Braccio_PiastraMotoreGomito.stl) |
| Elbow pin | [`stl/Gomito_Perno.stl`](stl/Gomito_Perno.stl) |
| Elbow M14 nut | [`stl/Gomito_DadoM14.stl`](stl/Gomito_DadoM14.stl) |
| Elbow spacer | [`stl/Gomito_Distanziale.stl`](stl/Gomito_Distanziale.stl) |
| Elbow washer | [`stl/Gomito_Rondella.stl`](stl/Gomito_Rondella.stl) |
| Forearm | [`stl/Avambraccio.stl`](stl/Avambraccio.stl) |

The file names are in Italian: *Base*, *Spalla* (shoulder), *Braccio*
(upper arm), *Gomito* (elbow), *Avambraccio* (forearm).

Bodies named `RIF_*` in the design (NEMA 17 motor, 20T motor pulley,
elbow endstop) are bought parts kept only as references, so they have no
STL.

## 🔧 Tensioning the shoulder belt

The shoulder belt tensions itself. The motor sits in slots in
`Base_SupportoPosteriore`, and two compression springs (7 mm outside,
5 mm inside, 30 mm long) in pockets above the motor push it away from
the shoulder. Each pocket is 21 mm deep, so a spring is compressed about
6 mm with the motor in the middle of its slots.

1. Push a spring into each pocket and hold it there with a pin through
   the 3.2 mm cross hole (an M3 screw, a 3 mm rod or a hex key).
2. Slide the motor in, put the belt on and fit the four motor screws:
   snug, then back them off about a quarter turn, so the motor can still
   slide.
3. Pull the pin out: the springs push the motor down and tighten the belt.

If the belt is still loose, add a washer on top of each spring; if the
motor gets stuck at the end of its slots, the springs are too strong.

## ➡️ Next

Once the arm is printed and assembled, wire it with the
🔌 [wiring guide](../software/README.md) and follow the
[installation steps](../README.md) in the main README.
