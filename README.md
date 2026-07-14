# code

## MPPT + CC + CV controller (Boost, 16S LiFePO4)

Added sketch:

- `mppt_cc_cv_boost_16s.ino`
- `legacy_boost_dropin_patch.md` (drop-in patch for replacing BOOST section in an existing legacy sketch)
- `legacy_cv58_v5_exact_boost_replace.md` (exact replace guide matched to the user-provided `cv58-stability-v5` code)
- `cv58_stability_v6_full_boost_reworked.ino` (full legacy-style sketch with BOOST section fully reworked to MPPT->CC->CV)

This sketch implements state-machine charging:

- `SOFTSTART -> CC_MPPT -> CV_HOLD -> CHARGE_DONE`
- plus `STANDBY` and `FAULT`

Target values currently set:

- CC = `6.0A`
- CV = `56.0V` (~3.50V/cell, longevity / BMS-friendly)
- 16S LiFePO4 thresholds (`CV enter 55.6V`, `force CV 55.8V`, `recharge 54.0V`)
- PWM = `50kHz`
- Hard OVP = `57.8V` + early BMS-open preempt and near-zone duty cap

Before deploying, verify calibration constants in the sketch:

- `CAL_SCALE_*`
- `OFFSET_*`

and confirm ADS1115 channel mapping matches your wiring.

Recent control update:

- fixed startup deadlock where very low sampled PV power could clamp current reference and keep boost duty too low to ramp into real charging.
- added one-point PV voltage trim factor (`FIELD_TRIM_V_SOLAR`) to align ADC reading with multimeter measurements.
- confirmed BMS opens near 56.2–57V; charger CV set below BMS, duty capped near zone, PWM killed on jump.
- fixed CC low-current lock: removed measured-Ppv Iref ceiling; CC now seeks 6A and backs off only on PV collapse.
- stabilized CV loop: wider hysteresis, Iref slew, near-target gentle duty steps, freeze in deadband.
- CV IR-compensation + heavy Vfb LPF to reduce current/voltage chatter near 56V.
