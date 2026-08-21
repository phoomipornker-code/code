# cv58-boost-v14-forward-pi

ESP32 charger firmware: proven **Boost / PV** (`cv58-stability-v14-cv-stable`) plus a finished **Forward / AC** path using the same **PI** cascade.

## Flash

1. Arduino IDE → open this folder (`cv58-boost-v14-forward-pi`).
2. Board: ESP32 (LEDC `ledcAttach` API).
3. Libraries: `Adafruit ADS1X15`, `LiquidCrystal I2C`.
4. Upload. Boot log must show:

```
[BOOT] Firmware: cv58-boost-v14-forward-pi-v1
```

## What changed vs the PV-only sketch

| | Boost (PV) | Forward (AC) |
|---|---|---|
| PWM | 50 kHz GPIO 27 | **67 kHz** GPIO 14 |
| CC | 6.0 A | **5.0 A** (transformer rating) |
| CV | 56.0 V | 56.0 V |
| Dmax | 760 | **460 (~45%, Nr=Np)** |
| Control | SoftStart → CC+MPPT → CV → DONE | SoftStart → **CC PI** → **CV PI** → DONE |

The old Forward dual-PID (`min(CC PID, CV PID)` + Kd) is gone. Ki was not scaled by `dt`, the two integrators fought near 56 V, and derivative amplified Ibat noise.

Forward now matches Boost:

```
current PI  → duty          (CC)
voltage PI  → Iref
current PI  → duty          (CV, milder gains / slew)
```

Anti-windup is clamp-style: the integral is updated only when the unsaturated output is inside the PI limits (`control_pi.h`).

## Serial states

`FWD_SOFT` → `FWD_CC` → `FWD_CV` → `FWD_DONE` / `FULL_HOLD`

Debug line `[AC ]` prints `FWD Iref` and both PI integrators.

## Safety (Forward)

- AC below 140 V: 2 s confirm then shutdown; below 100 V PWM is cut immediately.
- BMS-open / Vbat spike preempt now runs on Forward as well as Boost.
- Ibat > 5.25 A or Iac > 2.5 A: duty is pulled down.
- Hard OVP 57.8 V, FULL at 55.9 V and I ≤ 0.5 A for 60 s, restart at 54.0 V.
