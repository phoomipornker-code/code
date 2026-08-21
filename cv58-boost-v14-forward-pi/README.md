# cv58-boost-v14-forward-pi

ESP32 charger firmware: proven **Boost / PV** plus a finished **Forward / AC** path using the same **PI** cascade.

Matched to pack BMS **HXYP-SH5-16S-20ATF** (16S LFP, same-port, cell OVP **3.65 V**, charge 15 A).

## Flash

1. Arduino IDE → open this folder (`cv58-boost-v14-forward-pi`).
2. Board: ESP32 (LEDC `ledcAttach` API).
3. Libraries: `Adafruit ADS1X15`, `LiquidCrystal I2C`.
4. Upload. Boot log must show:

```
[BOOT] Firmware: cv58-boost-v14-forward-pi-v3
```

Paste into Arduino IDE as a single sketch: copy only `cv58-boost-v14-forward-pi.ino` (helpers are inside the .ino). Do not `#include "control_pi.h"` — that file is only for host tests.

## Charge setpoints vs this BMS

| | Charger (this firmware) | HXYP BMS |
|---|---|---|
| Full / OVP | **CV 57.60 V** (3.60 V/cell) | 58.40 V (3.65 V/cell) |
| Why the gap | charger finishes first; FET does not open | backup only |
| CC | Boost 6 A / Forward 5 A | allows 15 A charge |
| Empty | restart ≤ 53.60 V | UVP 36.80 V (2.30 V/cell) |
| FULL HOLD | ≥ 57.40 V and I ≤ 0.50 A for 60 s | — |
| Hard OVP | 59.50 V (fly-up after FET open) | — |

56.0 V in v1 was why the pack felt not full (~90–95%). 57.6 V is the factory-like 3.60 V/cell point and sits in the passive-balance window. Do not set charger CV to 58.4 V — that races the BMS.

If one cell is high, the BMS can still open while the pack reads ~56–57 V. Jump detect remains active from 54.5 V.

## Control

| | Boost (PV) | Forward (AC) |
|---|---|---|
| PWM | 50 kHz GPIO 27 | **67 kHz** GPIO 14 |
| CC | 6.0 A | **5.0 A** (transformer rating) |
| CV | 57.6 V | 57.6 V |
| Dmax | 760 | **460 (~45%, Nr=Np)** |
| Control | SoftStart → CC+MPPT → CV → DONE | SoftStart → **CC PI** → **CV PI** → DONE |

```
current PI  → duty          (CC)
voltage PI  → Iref
current PI  → duty          (CV)
```
