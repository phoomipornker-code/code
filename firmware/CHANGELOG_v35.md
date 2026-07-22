# Changelog v35 — cv58-boost-v14-forward-v35

## Forward without PID

Forward now uses **step / hysteresis** control (no current/voltage PI):

| Phase | Action |
|-------|--------|
| SoftStart | Ramp duty up by fixed step toward seed |
| CC | I low → step duty up; I high → step down; in band → hold |
| CV | V low → small up; V high → step down; deadband → hold |

Still: CC **3 A**, freeze duty-up on AC sag / AC&lt;115 V (Cin), slow steps.  
**Boost unchanged** (still PI SoftStart→CC_MPPT→CV).
