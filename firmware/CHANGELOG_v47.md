# Changelog v47 — cv58-boost-v14-forward-v47

## Field: HARD OVP at 91.61 V while BAT was ~53.8 V

```text
[FWD ] phase=CC … Vac≈156 Vbat≈53.8 Ibat=0.00 duty≈241
[CRITICAL] HARD OVP TRIP at 91.61V (trip=57.80V). Output disabled.
… Vbat raw≈53.75  Vf≈58.51 (filter ate the spike)
```

**Cause:** ADS BAT channel glitch — `91.61 V ≈ ACraw_mV × BAT_scale` (mux/cross-read), not a real pack over-voltage.  
Old BAT high-spike filter only ran when **I > 0.35 A**, so with **I=0** the spike passed and latched HARD OVP on one raw sample.

### v47 fixes
- Filter BAT up-spikes **even when I=0** (while ON / duty active)
- HARD OVP needs short confirm; reject single raw spikes when filt is still far below trip
- Forward CV still **55.9 V**; Boost frozen

Flash until boot shows:

`[BOOT] Firmware: cv58-boost-v14-forward-v47`

Expect `BATspike!` WARN instead of HARD OVP on a lone 90 V sample.
