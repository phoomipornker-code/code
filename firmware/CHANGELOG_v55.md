# Changelog v55 — cv58-boost-v14-forward-v55

## Field: FWD_CV duty→Dmax, then false HARD OVP after STOP

```text
[FWD ] phase=CV … Ibat:0.16A duty_acc:460.0 Dmax=460
[WARN] … BATraw=2408.6mV … BATspike!
[INFO] STOP held 350ms — charge ended (was FORWARD).
[CRITICAL] HARD OVP TRIP at 80.52V (filt=74.14 trip=57.80V).
```

**Cause:**
1. Near-full CV kept climbing duty toward Dmax while I≈0.15 A (V stuck ~55.63) → more ADS sense fly-ups (`BATspike!` / `BUS!`).
2. Spike/implausible BAT gates still required `system_ON` / charging current, so after STOP a ~100 V mux ghost could enter the filter (`Vf≈74`) and latch HARD OVP.

### v55 fixes
- **Always** reject BAT outside ~850–1500 mV when a plausible pack sample exists (including after STOP)
- Always block out-of-range `v_bat` from the filter; **heal** `Vf` if it leaves 35–62 V
- HARD OVP only if filt/raw stay in a physical 16S window (not filt=74 from spikes)
- Forward CV: if `Ifilt ≤ 0.40 A` and within ~0.6 V of target, **freeze duty-up** (stop climb to Dmax)

Boost path unchanged.

Flash until boot shows:

`[BOOT] Firmware: cv58-boost-v14-forward-v55`

Expect CV to hold duty near taper instead of flying to Dmax, and no HARD OVP latch from post-STOP mux ghosts.
