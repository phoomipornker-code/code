# Changelog v27 — cv58-boost-v14-forward-v27

## Field log: FWD_CC stalled @ ~34% then instant OFF on AC brownout

From field debug (AC bridge ~144 V, Ibat ~0.46 A, duty raw=348) then sudden STANDBY with AC ~3 V:

| Issue | Fix in v27 |
|------|------------|
| AC undervoltage shut down **instantly** (`v_ac_in < MIN_AC`) | Debounce **2 s** like Boost PV collapse (`FWD_AC_COLLAPSE_CONFIRM_MS`) |
| CC climb-help stopped at duty_acc ≥ **280** (Boost-oriented) | Climb-help until near Forward Dmax (`allowed_max − 40`) |
| Slow climb while I ≪ 5 A | Faster slew when `iErr > 1 A` (`FWD_DUTY_SLEW_UP_CC_FAR = 8`) |
| Debug showed only `Iref_cv` in CC | Print `phase`, `Iref_cc`, `Iref_cv`, `Dmax`, `AC_LOW!` |

Boost remains frozen to `cv58-stability-v14-cv-stable`.

If after flash duty climbs toward **460** with AC stable but Ibat still ~0.5 A, power is likely limited by transformer turns / magnetics — software can only push to `MAX_DUTY_FORWARD`.
