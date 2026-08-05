# Changelog v31 — cv58-boost-v14-forward-v31

## Why it cut (field log)

```text
[WARN] ADC glitch filtered: PVraw=42.0mV BATraw=42.0mV ...
[WARN] AC bridge low 3.0V — confirm 2000ms before shutdown.
```

Not a real AC brownout. ADS voltage bus glitched: **BAT and AC both ~42 mV** (3 V on AC scale).  
BAT was held by the old glitch filter; **AC had no hold** → fake `v_ac_in=3 V` → AC-collapse path (2 s) → shutdown.

## Fix

- Hold last-good **AC raw** like BAT when raw collapses while charging
- Detect **multi-channel bus glitch** (AC+BAT both invalid and nearly equal)
- Glitch log now prints `PVraw ACraw BATraw` + `BUS!` when multi-ch

Forward CC still **3 A**. Boost unchanged.
