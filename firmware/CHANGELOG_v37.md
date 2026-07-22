# Changelog v37 — cv58-boost-v14-forward-v37

## Enter CV earlier

Field stayed in `FWD_CC` at Vbat≈55.1–55.4 with tapering Iref (~1.7 A) —  
entry was still **55.50 / force 55.70**, so CV never triggered.

| Threshold | Was | Now |
|-----------|-----|-----|
| `FWD_CV_ENTRY_VOLTAGE` | 55.50 | **55.10** |
| `FWD_CV_FORCE_VOLTAGE` | 55.70 | **55.35** |
| `FWD_CV_EXIT_VOLTAGE` | 54.80 | **54.60** |

Entry uses `max(Vbat, Vfilt)` with 200 ms confirm. Still no PID; CC 3 A.
