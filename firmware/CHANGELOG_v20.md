# Changelog v20 — cv58-boost-v14-forward-v20

## Fixes from field debug log

Observed:
```text
System: OFF | State: FULL_HOLD
PV V:0.0V I:0.9A   (ghost current)
AC V:~101V         (below old MIN_AC=120)
```

### Changes
- `MIN_AC_VOLTAGE`: **120 → 95 V** (AC 110 V post-bridge under load often ~100–110 V)
- Clear `charge_full_hold` whenever `system_ON` is false (stop stuck `OFF + FULL_HOLD`)
- BMS-open no longer sets `charge_full_hold` after shutdown (OVP latch is enough)
- Zero PV/AC current when voltage is 0 (remove Hall/offset ghost amps)
- ADC glitch hold only while `duty > 0` (stop false hold / WARN spam when idle)
- Debug label `FULL_HOLD` only if `system_ON && charge_full_hold`
