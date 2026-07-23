# Changelog v49 — cv58-boost-v14-forward-v49

## Field: LCD still dead while Serial/charging OK

v48 reduced contention but LCD task still fought ADS on the same I2C bus.
Under EMI (`ACraw/BATraw=42`) the display stayed blank/frozen.

### v49 fix
- **LCD is drawn from the ADC/PWM task** after each sample window (no cross-task I2C fight)
- Button task no longer writes the LCD (except standby bus recover)
- Soft I2C unlock (SCL clocks) before LCD reinit
- Boot splash: `BOOT OK` + firmware tag — confirms LCD alive after flash
- ADC task stack raised to 8 KB

Flash until boot shows:

`[BOOT] Firmware: cv58-boost-v14-forward-v49`

LCD should show `BOOT OK` then STANDBY / ACTIVE screens.
