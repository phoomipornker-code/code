# Changelog v45 — cv58-boost-v14-forward-v45

## Forward CV target 55.9 V

User request: change CV hold from 55.8 → **55.9 V** (still slightly under 56).

### Changes
- Forward `TARGET_CV_VOLTAGE` **55.80 → 55.90**
- Forward FULL detect **55.80**
- CV entry/force/exit / CC taper nudged to match 55.9 hold
- **Boost CV remains 56.00** (frozen)

Flash until boot shows:

`[BOOT] Firmware: cv58-boost-v14-forward-v45`

Expected CV line: `phase=CV hold Vcv:55.90 … (constV)`
