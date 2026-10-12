# Hardware

3D-printed parts of the SCARA arm, exported from the Fusion design "robot claude" (v41).

- `fusion/robot_claude.f3d`: the editable Fusion design (all bodies, including the reference parts).
- `stl/`: one binary STL per printable part, in millimetres.

| Part | File |
| --- | --- |
| Base, bottom plate | `stl/Base_PiastraFondo.stl` |
| Base, front support | `stl/Base_SupportoAnteriore.stl` |
| Base, rear support | `stl/Base_SupportoPosteriore.stl` |
| Shoulder, 72T pulley | `stl/Spalla_Puleggia72T.stl` |
| Upper arm, motor side | `stl/Braccio_LatoMotore.stl` |
| Upper arm, front side | `stl/Braccio_LatoAnteriore.stl` |
| Upper arm, elbow motor plate | `stl/Braccio_PiastraMotoreGomito.stl` |
| Elbow pin | `stl/Gomito_Perno.stl` |
| Elbow M14 nut | `stl/Gomito_DadoM14.stl` |
| Elbow spacer | `stl/Gomito_Distanziale.stl` |
| Elbow washer | `stl/Gomito_Rondella.stl` |
| Forearm | `stl/Avambraccio.stl` |

Bodies named `RIF_*` in the design (NEMA 17 motor, 20T motor pulley, elbow endstop) are bought parts kept only as
references, so they have no STL.
