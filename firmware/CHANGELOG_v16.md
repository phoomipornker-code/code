# Changelog v16 — cv58-forward-67khz-5a-v16

## Forward control rewrite

| Item | v15 | v16 |
|------|-----|-----|
| Forward loop | Dual PID `min(pid_cc, pid_cv)` | **State machine SoftStart → CC → CV → DONE** |
| Soft-start | Seed duty=10, no ramp | Ramp to seed ~80 raw, confirm on current or 2 s |
| CC | Competing with CV PID | Current PI + pre-CV taper from 54.8 V |
| CV | Same loop as CC via `min()` | Voltage outer PI → Iref, current inner PI, slew limits |
| FULL end | Mostly high-V stop 56.8 V | CV end-current (≥55.9 V & ≤0.5 A for 60 s) + high-V stop |
| Debug labels | `FORWARD` | `FWD_SOFT` / `FWD_CC` / `FWD_CV` / `FWD_DONE` |

## Unchanged (hardware align from v15)

- `PWM_FREQ = 67000`
- `TARGET_CC_CURRENT = 5.0 A`
- `TARGET_CV_VOLTAGE = 56.0 V`
- `MAX_DUTY_FORWARD = 460` (~45%, Nr=Np)
- Boost / MPPT path unchanged
