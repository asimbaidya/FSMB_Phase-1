
# Voltage Protector Device — FSMB Phase 1

**Author:** Asim Baidya
**Role applied for:** Embedded System Engineer
**Project:** FSMB Recruitment 2026 — Phase 1
**Wokwi project:** see `project_files/wokwi_project_link.txt`

A microcontroller-based voltage protector that disconnects an appliance when
the grid voltage or load current moves outside safe limits, and reconnects
automatically once the supply has been stable for a configurable period.
Designed and tested entirely in the Wokwi simulator; no mains voltage is
used at any point.

---

## Repository Structure

```
.
├── design_document.pdf
├── project_files
│   ├── diagram.json
│   ├── libraries.txt
│   └── wokwi_project_link.txt
├── README.md
├── schematics
│   └── sketch.png
├── src
│   └── voltage_protector.ino
└── test_evidence
    ├── state-error.png
    └── state-okey.png
```

## Live 

**Live:** open https://wokwi.com/projects/477494837539055617 and click **Run**.
