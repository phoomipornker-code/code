# cv58-boost-v14-forward-v82

Boost restored from proven `cv58-stability-v14-cv-stable`  
Forward keeps SoftStart→CC→CV→DONE (no PID)

## Flash
Open this folder in Arduino IDE. See `FLASH.txt`.

## Key values
- Boost: SoftStart→CC_MPPT→CV→DONE · CC 6 A · CV 56.00 V · 50 kHz
- Forward: SoftStart→CC→CV→DONE · CC 3 A · CV 55.90 V · 67 kHz
- `CAL_SCALE_V_BAT = 41.85`
- Boost BMS-OPEN: 56.30 V / 1.20 A (v14)
- Forward BMS-OPEN: 56.80 V / 0.25 A (hardened)
