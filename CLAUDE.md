# Robot arm: BeagleBone Black + TMC2209

Buildroot BR2_EXTERNAL tree. Buildroot itself lives in ~/Desktop/buildroot-2026.05.3
(not versioned here). Build from that directory:

    make BR2_EXTERNAL=~/Desktop/robot-arm robotarm_defconfig   # once
    make                                                       # image: output/images/sdcard.img

Wiring and bring-up guide: board/beaglebone/readme.txt.
GPIO/UART pin assignment source of truth: package/tmc2209ctl/tmc2209.conf.

## Agent skills

### Issue tracker

Issues and specs live as local markdown files under `.scratch/`. See `docs/agents/issue-tracker.md`.

### Triage labels

The seven default label strings (bug, enhancement, needs-triage, needs-info, ready-for-agent, ready-for-human, wontfix). See `docs/agents/triage-labels.md`.

### Domain docs

Single-context: one `GLOSSARY.md` at the root, ADRs in `docs/adr/`. See `docs/agents/domain.md`.
