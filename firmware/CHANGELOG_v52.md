# Changelog v52 — cv58-boost-v14-forward-v52

## Field: PWM signal still cut/interrupted while charging

v51 stopped deliberate PWM mute, but signal cuts remained.

### Root cause found
LCD recover used `charge_active = system_ON && raw_duty > 0`.  
During Forward/Boost **entry** (`vTaskDelay(500)` with `raw_duty=0`) the button task could still run **`Wire.end()` / `lcd.init()`** — that glitches the shared I2C bus and can interrupt PWM/control.

### v52 fixes
- **No LCD I2C / no bus reinit while `system_ON`** (including SoftStart duty=0)
- Forward PWM: **no dither** (stable duty integer — no 1-LSB toggling)
- Debug Serial less often while charging (2 s)
- Boost path unchanged (still dither)

Flash until boot shows:

`[BOOT] Firmware: cv58-boost-v14-forward-v52`
