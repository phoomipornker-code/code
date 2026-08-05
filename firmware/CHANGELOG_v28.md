# Changelog v28 — cv58-boost-v14-forward-v28

## Design point: 5 A @ D≈45%

Hardware is sized so Forward CC **~5 A** is expected near **D=45%** (`MAX_DUTY_FORWARD=460`).  
v27 already debounced AC brownout and unlocked climb past duty 280; v28 aims SoftStart/CC at that design point.

| Item | v28 |
|------|-----|
| Constant | `FWD_DESIGN_DUTY_FRAC = 0.45` tied to `FWD_TARGET_CC_CURRENT = 5 A` |
| SoftStart seed | Prefer ~90–100% of design D=45% (not a low mid-duty exit) |
| SoftStart ready | Near seed **and** I≥0.4 A, else timeout 2.5 s (no early exit on tiny I alone) |
| SoftStart slew | Faster ramp toward design duty (`6/5`) |
| CC feedforward | Floor duty from `forwardDutyFfForIref()` — 5 A ↔ D=45%, scaled by Iref/Vin/Vbat |
| Debug | Shows `(5A@45%)` next to Dmax |

Boost remains frozen to `cv58-stability-v14-cv-stable`.
