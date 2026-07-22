# Changelog v33 — cv58-boost-v14-forward-v33

## AC sag: freeze duty-up only (no pause)

Per field request: temporary AC sag must **not** pause PWM / zero duty / SoftStart reset.

| Was (v32) | Now (v33) |
|-----------|-----------|
| duty → 0 while AC low | keep current duty |
| SoftStart on recover | continue same phase |
| “PWM suspend” | **freeze duty-up** (`slewUp=0`, no climb-help/FF up) |

Still may **decrease** duty via normal safety cuts. Sustained AC loss ≥15 s → Auto-Shutdown.
