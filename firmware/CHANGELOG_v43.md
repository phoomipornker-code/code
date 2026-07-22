# Changelog v43 — cv58-boost-v14-forward-v43

## Field: false BMS-OPEN while CV holding 56 V

Log (healthy CV, then trip):

```text
[FWD ] phase=CV hold Vcv:56.00V Vf:56.02V Ibat:1.27A duty_acc:349.8 … (constV)
[CRITICAL] BMS-OPEN/preempt at raw=63.76 filt=56.99 I=1.10A step=7.76. PWM off.
[WARN] ADC glitch filtered: … BATraw=1705.3mV …
```

**Cause:** not a real BMS open. BAT ADC spiked (~+7.8 V in one sample) while charge current was still ~1.1 A. Old logic latched OVP on `v_bat≥56.30` / `step≥1.2` without requiring sustained open + collapsed current. Glitch filter only caught BAT-too-low, not BAT-too-high.

### v43 fixes
- Filter **BAT high spikes** (≥ +80 mV raw ≈ +3.3 V) while charging current still present (`BATspike!`)
- BMS-OPEN requires **current collapsed** + **~120 ms confirm** (ignore single-sample spikes)
- Forward fine CV / Boost frozen path otherwise unchanged

Flash until boot shows:

`[BOOT] Firmware: cv58-boost-v14-forward-v43`
