# Proven PV reference

Field-validated Boost firmware used as Boost safety guideline:

`cv58-stability-v14-cv-stable`

Restored into `cv58-boost-v14-forward-v82`:
- SoftStart→CC_MPPT→CV→DONE (PI)
- BMS-OPEN @ 56.30 V / I≤1.20 A (immediate + full-hold)
- HARD OVP / runaway / spike pre-cut (Boost-only paths)

Forward keeps its separate hardened BMS/OVP thresholds.
