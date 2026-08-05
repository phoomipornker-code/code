# Changelog v17 — cv58-boost-v14-forward-v17

## Policy

- **Boost:** freeze proven `cv58-stability-v14-cv-stable` behavior (50 kHz, 6 A CC, SoftStart→CC_MPPT→CV→DONE).
- **Forward:** keep SoftStart→CC→CV→DONE, harden for real AC use.

## Changes vs v16

| Item | v16 | v17 |
|------|-----|-----|
| Boost PWM | 67 kHz (shared) | **50 kHz** (proven) |
| Forward PWM | 67 kHz | **67 kHz** (separate) |
| Boost CC | 5 A (shared) | **6 A** (proven) |
| Forward CC | 5 A | **5 A** (`FWD_TARGET_CC_CURRENT`) |
| `MIN_AC_VOLTAGE` | 140 V | **200 V** |
| BMS-open / spike preempt | Boost only | **Boost + Forward** |
| Forward SoftStart no-load | enter CC on timeout | **fault latch** if I ≈ 0 |
| Forward OC | soft duty cut | **latched shutdown** (Ibat / Iac) |
| Battery start window | none | **40.0 … 56.4 V** |
| OVP clear | auto after delay | **STOP** when V safe |

## Unchanged (Boost control)

MPPT / CV PI gains, taper, BMS preempt duty cap, hard OVP trip 57.8 V, FULL detect thresholds.
