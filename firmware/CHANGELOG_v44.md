# Changelog v44 — cv58-boost-v14-forward-v44

## Forward CV target lowered to 55.8 V

User request: hold CV slightly under 56 V (e.g. 55.8) to stay clearer of BMS/preempt / sense spikes.

### Changes
- Forward `TARGET_CV_VOLTAGE` **56.00 → 55.80**
- Forward FULL detect **55.70** (separate from Boost FULL 55.90)
- CV entry/force/exit / CC taper nudged down to match 55.8 hold
- **Boost CV remains 56.00** (frozen)

Flash until boot shows:

`[BOOT] Firmware: cv58-boost-v14-forward-v44`

Expected CV line: `phase=CV hold Vcv:55.80 … (constV)`
