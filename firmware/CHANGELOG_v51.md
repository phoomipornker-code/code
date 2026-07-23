# Changelog v51 — cv58-boost-v14-forward-v51

## Field: PWM twitch / control disturbance while charging

v50 briefly turned PWM off (~0.8 ms) every 1–2 s so LCD I2C could run.
That gated the power stage and disturbed current/voltage control (“กระตุก”).

### v51 policy
- **Never mute PWM for LCD**
- When `duty > 40`: **freeze LCD I2C** (keep last frame; no backlight keepalive pulses)
- LCD updates only in STANDBY / SoftStart / low duty, and again when charge ends
- Control stability > live display during high power

Flash until boot shows:

`[BOOT] Firmware: cv58-boost-v14-forward-v51`
