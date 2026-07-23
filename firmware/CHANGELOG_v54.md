# Changelog v54 — cv58-boost-v14-forward-v54

## Field: BAT reads 4.37 V while still charging ~3 A

```text
[BAT] V: 4.37V  I: 3.71A  Vf: 4.37V  If: 2.79A
[RAW V mV] BAT(ch1): 104.4
[WARN] … BATraw=42.0mV … BUS!
```

**Cause:** ADS BAT channel glitched to ~104 mV. Old filter only rejected `<80 mV`, so 104 mV became “valid”, overwrote `last_valid`, and poisoned `Vf` to 4.37 V. CV/SOC then see a dead pack while current still flows.

### v54 fixes
- Plausible BAT window while charging: **850–1500 mV** (~35.6–62.8 V)
- Reject sudden BAT drops (`BATbad!`) and hold last **plausible** sample
- Never store out-of-range BAT into `last_valid` / filter
- If already poisoned, fall back to last good V / previous filt

Flash until boot shows:

`[BOOT] Firmware: cv58-boost-v14-forward-v54`

Expect `BATbad!` in WARN instead of sticky `Vbat≈4.4V` during CC.
