# Changelog v34 — cv58-boost-v14-forward-v34

## Slow duty open — Cin starvation / bridge ripple

Field: duty opened too fast → Cin could not supply → AC bridge ripple/sag.

| Item | Was | Now |
|------|-----|-----|
| SoftStart slew up | 6 | **1.5** |
| SoftStart time | 2.5 s | **5 s** |
| SoftStart seed | ~90% of CC duty | **~45%** of CC duty |
| CC slew up / far | 4 / 8 | **1.5 / 2.5** |
| Climb-help | 3 / 6 | **1 / 2** |
| Feedforward | snap to FF | **+2 raw/tick** max |
| Freeze climb | AC &lt; 95 V | also AC &lt; **115 V** (Cin stress) |

Still: freeze duty-up on sag (no pause). CC target **3 A**. Boost unchanged.
