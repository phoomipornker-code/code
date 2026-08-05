# Changelog v30 — cv58-boost-v14-forward-v30

## Field log: Forward CC ~3 A OK, then false ADC timeout

Charging was healthy (`Iref_cc=3A`, Ibat≈2.8–3.0 A, duty≈40%) then:

```text
[WARN] ADC glitch filtered: PVraw=0.0mV ...
[CRITICAL] ADC sample timeout. Auto-shutdown for safety.
```

| Cause | Fix |
|------|-----|
| PV=0 in FORWARD treated as glitch (PV relay open) | Skip PV glitch gate when `STATE_FORWARD` |
| `i2c_Mutex` held through BMS/Serial after ADS reads | Release mutex **immediately** after ADS sample |
| ADC mutex wait 50 ms vs LCD refresh | ADC wait **200 ms** |
| Stale trip at 700 ms | Warn at 800 ms; shutdown at **2500 ms** |

Forward CC remains **3 A**. Boost unchanged.
