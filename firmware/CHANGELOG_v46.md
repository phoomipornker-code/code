# Changelog v46 — cv58-boost-v14-forward-v46

## Field: Forward CC healthy then sudden STANDBY (no CRITICAL)

```text
[FWD ] phase=CC … Iref=3.00A Ibat≈2.9A duty≈408  Vac≈139V Vbat≈54.2
[WARN] ADC glitch filtered: PVraw=0.4 … duty=0 Ib=2.26A
[DEBUG] System: OFF | Run: STANDBY | Duty: 0%
```

**What this means**
- CC looked normal (no AC sag flag, no BMS-OPEN, no OVP line).
- The WARN with `duty=0` is **after** PWM already off: Forward leaves PV sense at ~0, and residual `Ib` filter made the old code yell “PV glitch” — not the cause.
- Sudden `System: OFF` with **no** `[CRITICAL] …` almost always means **STOP ended the charge**. Old STOP used a single edge and printed nothing, so EMI/noise on GPIO26 looked like a mystery shutdown.

### v46 fixes
- While charging: **hold STOP ~350 ms** to end (rejects short EMI edges)
- Log: `[INFO] STOP held … — charge ended`
- Do not treat PV=0 as ADC glitch when mode is FORWARD
- Boost control unchanged; Forward CV still **55.9 V**

Flash until boot shows:

`[BOOT] Firmware: cv58-boost-v14-forward-v46`

If it stops again, look for either `STOP held` or a real `[CRITICAL]` line.
