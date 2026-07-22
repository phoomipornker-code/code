# Changelog v40 — cv58-boost-v14-forward-v40

## AC blip 3 V dumped duty 339 → 116

Field: climbing CC, then `AC sag 3.0V`, then duty collapsed while AC recovered to ~170 V.

Cause: outer clamp  
`duty -= (2 + (95 − Vac) * 2.5)` at Vac=3 → **−232 raw/tick**  
violated “freeze duty-up only” policy.

### Fix
- **Remove** Forward AC-floor duty dump
- Hold last-good AC for **400 ms** on sudden AC collapse while BAT still valid (`ACblip!`)
- Real sustained sag still freezes climb; shutdown only after 15 s
