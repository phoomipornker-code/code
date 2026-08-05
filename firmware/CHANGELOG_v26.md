# Changelog v26 — cv58-boost-v14-forward-v26

## Forward fixed to mirror proven Boost

Boost remains frozen to `cv58-stability-v14-cv-stable`.  
Forward now gets the **same safety + SoftStart pattern**:

| Item | Forward v26 |
|------|-------------|
| SoftStart | `forwardEstimateDutyRaw()` seed (like Boost estimate) + ready I≥0.4A / 2.5s |
| CC | PI 14/55 + taper + **AC sag backoff** (like PV collapse) |
| CV | same near-band / bleed / slew as Boost |
| BMS-open / spike / runaway | **Boost + Forward** |
| Outer clamps | Ibat/Iac soft cut + AC floor cut + BMS duty cap |

Still Forward-specific: no MPPT, 5 A CC, 67 kHz, duty max 460, AC 110 V bridge.
