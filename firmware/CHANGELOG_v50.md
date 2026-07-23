# Changelog v50 — cv58-boost-v14-forward-v50

## Field: LCD OK in STANDBY, goes dark only while charging

EMI from PWM/power stage corrupts the LCD I2C backpack (PCF8574) and often
**clears the backlight bit** — display looks “dead” only under charge.

### v50 fixes
- While duty is high: **brief PWM-off quiet window** (~0.8 ms) before LCD I2C
- Slower I2C clock (50 kHz) for those charge-time LCD writes
- Always re-assert `lcd.backlight()` after draws
- Sparse full redraw every 2 s + backlight keepalive every 1 s while charging
- Standby refresh unchanged (500 ms)

Flash until boot shows:

`[BOOT] Firmware: cv58-boost-v14-forward-v50`

LCD should stay lit during Forward/Boost charge (may update slower).
