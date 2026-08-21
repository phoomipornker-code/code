# code

ESP32 dual-path charger for 16S LiFePO4:

- **Boost / PV** — proven `cv58-stability-v14-cv-stable` (50 kHz, 6 A CC, 56 V CV, MPPT).
- **Forward / AC** — finished with the same **PI** cascade (67 kHz, 5 A CC, 56 V CV). SoftStart → CC → CV → DONE.

Flash the sketch in [`cv58-boost-v14-forward-pi/`](cv58-boost-v14-forward-pi/).

Host tests for the shared PI math:

```bash
g++ -std=c++17 -Wall -Wextra -O2 tests/test_control_pi.cpp -o /tmp/test_control_pi
/tmp/test_control_pi
```
