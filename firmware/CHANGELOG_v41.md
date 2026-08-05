# Changelog v41 — cv58-boost-v14-forward-v41

## Field log still showed `Icap_cv` + duty dump on AC=0

That debug string is **v38**. Please flash until boot shows:

`[BOOT] Firmware: cv58-boost-v14-forward-v41`

Expected CV line (v39+): `phase=CV hold Vcv:56.00 … (constV)` — **not** `Icap_cv`.

### v41 extras (on top of v40)
- If AC sense reads 0/low **but Ibat still flowing** → treat as ADC glitch, **do not** enter AC sag
- Lone AC collapse hold extended to **1.5 s** (and unlimited while charging current present)
- Still no AC-floor duty dump; CV remains **constant voltage** hold
