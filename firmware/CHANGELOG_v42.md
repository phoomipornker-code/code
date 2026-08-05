# Changelog v42 — cv58-boost-v14-forward-v42

## Field: duty 380→142 in CV near 56 V + request for finer control

Log showed Forward CV holding ~56 V at duty ~37%, then duty slammed to ~14%
(`raw≈142` ≈ `BMS_PREEMPT_DUTY_CAP_RAW=140`) while still below BMS-open.

Please flash until boot shows:

`[BOOT] Firmware: cv58-boost-v14-forward-v42`

Expected CV line: `phase=CV hold Vcv:56.00 … (constV)` — **not** `Icap_cv`.

### v42 changes
- **Finer step sizes** (raw/tick @ 20 ms): SoftStart/CC/CV up/down roughly halved; add near/fine down steps
- **Tighter hold bands**: CC ±0.10 A, CV ±0.05 V around setpoints
- **Forward CV BMS preempt**: no slam to 140 (or 80) near 55.95 V — hard cap only near `BMS_OPEN_DETECT_V` (56.30)
- Softer outer OVP / hard-I cuts in Forward CV so they do not fight fine const-V steps
- Boost path unchanged (frozen v14)
