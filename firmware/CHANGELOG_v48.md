# Changelog v48 — cv58-boost-v14-forward-v48

## Field: LCD frozen while Forward CC still OK

Charging continued (~3 A, duty ~37%) with frequent I2C sense glitches:

```text
ACblip! ACraw=42.0
BATraw=42.0mV
PVraw≈BAT (mux cross-read → PV shows ~51 V)
```

**Cause:** ADS + LCD share one I2C bus. ADC held the mutex for a long multi-conversion burst; LCD refresh was every 180 ms and, after mutex fails, called `Wire.end()` / `lcd.init()` **while PWM was live**. EMI + bus reinit freezes the display (control task kept running — Serial still OK).

### v48 fixes
- Split ADS volt/curr reads into **two short mutex holds** (LCD can interleave)
- Drop per-sample `discard_first` in the hot loop (faster ADC, less bus time)
- LCD refresh **400 ms**; longer mutex wait; LCD task delay 50 ms
- **No LCD I2C reinit while charging** — recover only in standby
- Debug print interval **1 s** (less Serial load)

Flash until boot shows:

`[BOOT] Firmware: cv58-boost-v14-forward-v48`
