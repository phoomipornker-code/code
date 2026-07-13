# code

## MPPT + CC + CV controller (Boost, 16S LiFePO4)

Added sketch:

- `mppt_cc_cv_boost_16s.ino`
- `legacy_boost_dropin_patch.md` (drop-in patch for replacing BOOST section in an existing legacy sketch)

This sketch implements state-machine charging:

- `SOFTSTART -> CC_MPPT -> CV_HOLD -> CHARGE_DONE`
- plus `STANDBY` and `FAULT`

Target values currently set:

- CC = `6.0A`
- CV = `58.0V`
- 16S LiFePO4 thresholds (`CV enter 57.8V`, `recharge 54.0V`)
- PWM = `50kHz`

Before deploying, verify calibration constants in the sketch:

- `CAL_SCALE_*`
- `OFFSET_*`

and confirm ADS1115 channel mapping matches your wiring.
