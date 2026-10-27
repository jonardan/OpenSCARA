# SCARA IR per Flipper Zero

App per Flipper Zero che fa da telecomando di `scara ir`: le quattro frecce
muovono la punta, Indietro esce.

| Tasto    | Codice NEC (come lo vede il BBB) | Movimento con `scara ir` |
|----------|----------------------------------|--------------------------|
| Su       | `0x00860542`                     | +X                       |
| Giù      | `0x00860543`                     | -X                       |
| Destra   | `0x00860515`                     | +Y                       |
| Sinistra | `0x00860516`                     | -Y                       |

Sono gli stessi codici del telecomando di casa, quindi `scara ir` funziona
senza imparare tasti nuovi (se all'avvio chiede di premere SU/GIÙ/DESTRA/
SINISTRA, premi le frecce del Flipper). Finché tieni premuta una freccia il
Flipper manda il codice e poi i codici di ripetizione NEC, come il telecomando
vero: il braccio si muove in continuo e si ferma appena lasci il tasto. Una
freccia alla volta: le altre vengono ignorate finché non lasci la prima.

A sinistra c'è un piccolo SCARA visto dall'alto: segue le frecce e, mentre
il Flipper trasmette, il LED sulla punta si accende e lascia una scia (come
nel light painting). Se non tocchi niente per 3 secondi gira da solo
disegnando un otto.

Il LED IR del Flipper è in alto: puntalo verso il ricevitore VS1838 (P8_16).

## Installare

Il file da installare è `scara_ir.fap`.

**Con qFlipper:** collega il Flipper via USB, apri qFlipper, scheda *File
manager*, entra in `SD Card/apps/Infrared/` e trascinaci `scara_ir.fap`.

**Con la scheda SD:** copia `scara_ir.fap` nella cartella `apps/Infrared/`
della micro SD del Flipper.

Sul Flipper: *Apps → Infrared → SCARA IR*.

Se il Flipper dice che l'app è per un'altra versione dell'API, aggiorna il
firmware del Flipper (qFlipper → *Update*) oppure ricompila l'app.

## Compilare

GitHub la compila da sola (workflow `Flipper SCARA IR`): nella scheda
*Actions* apri l'ultima esecuzione e scarica l'artefatto `scara_ir-...`.

In locale, con [ufbt](https://pypi.org/project/ufbt/):

    pip install ufbt
    cd tools/flipper-scara
    ufbt              # crea dist/scara_ir.fap
    ufbt launch       # la installa e la avvia sul Flipper collegato via USB

`ufbt` scarica l'SDK del firmware ufficiale (release). Con un firmware
diverso (Momentum, Unleashed, ...) usa `ufbt update --index-url=...` del
firmware che hai.

Dopo ogni push l'ultimo `scara_ir.fap` compilato sta da solo sul ramo
`flipper-fap`: <https://github.com/jonardan/OpenSCARA/raw/flipper-fap/scara_ir.fap>.
