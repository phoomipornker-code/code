# Changelog v53 — cv58-boost-v14-forward-v53

## Show battery SOC on LCD while charging

User request: during charge, display pack **SOC**.

### Changes
- 16S LFP voltage→SOC estimate (piecewise) with small IR trim while charging
- Charge LCD layout focuses on **SOC % + bar**, Vbat, Ibat, phase, duty
- Sparse LCD refresh every **2 s** while `system_ON` (no PWM mute, no `Wire.end`)
- Standby also shows SOC next to battery voltage
- Still **no I2C bus reinit while charging**

Example charge screen:

```text
FORW CC   SOC: 78%
[########--]
BAT: 52.6V I:2.97A
IN:145.0V D: 37%
```

Flash until boot shows:

`[BOOT] Firmware: cv58-boost-v14-forward-v53`
