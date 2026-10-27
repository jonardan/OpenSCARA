# 🖨️ OpenSCARA hardware

The 3D-printed parts of the [OpenSCARA](../README.md) arm, exported from
the Fusion design "robot claude" (v46).

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

The shoulder motor sits in slots in `Base_SupportoPosteriore`, and a
block above the motor holds two M3 push screws (x = ±16 mm, 2.7 mm
holes, self-tapping in the plastic, M3×16 to M3×20). Moving the motor
away from the shoulder tightens the belt; the screws keep it from creeping
back.

1. Loosen the four motor screws a little, so the motor slides.
2. Turn both push screws in by the same amount until the belt is tight
   (it should twang, not flap).
3. Tighten the four motor screws again.

Back the push screws out before you slide the motor in or out.

## ➡️ Next

Once the arm is printed and assembled, wire it with the
🔌 [wiring guide](../software/README.md) and follow the
[installation steps](../README.md) in the main README.
