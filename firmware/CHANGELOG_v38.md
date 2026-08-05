# Changelog v38 — cv58-boost-v14-forward-v38

## CV entered but did not taper current

Field: `phase=CV`, Vbat≈55.89 V, Ibat≈2.8–3.2 A, duty held ~36%.  
Old CV logic **held** duty when near 56 V but still below target (`nearTarget` blocked both up and down).

### Fix
- Near 56 V: compute **iCap** that falls as V rises (3 A → 0.3 A)
- If Ibat > iCap → **step duty down**
- At/over 56 V → stronger duty cut
- Outer over-current in CV uses soft cut vs iCap (not harsh CC+0.25 dump that collapsed duty 372→141)

Debug shows `Icap_cv`.
