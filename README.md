# code

ESP32 dual-path charger for 16S LiFePO4 with BMS **HXYP-SH5-16S-20ATF** (cell OVP 3.65 V → pack 58.4 V):

- **Boost / PV** — 50 kHz, 6 A CC, **57.6 V CV**, MPPT.
- **Forward / AC** — 67 kHz, 5 A CC, **57.6 V CV**, PI cascade SoftStart → CC → CV → DONE.

Charger CV is 0.80 V below the BMS so the charger terminates first. Flash [`cv58-boost-v14-forward-pi/`](cv58-boost-v14-forward-pi/).

MATLAB / Simulink for Forward (do not only retune the old Switch diagram): [`sim/README_FORWARD_SIMULINK.md`](sim/README_FORWARD_SIMULINK.md)

Host tests for the shared PI math:

```bash
g++ -std=c++17 -Wall -Wextra -O2 tests/test_control_pi.cpp -o /tmp/test_control_pi
/tmp/test_control_pi
```
