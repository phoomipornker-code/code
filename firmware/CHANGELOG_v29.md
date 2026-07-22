# Changelog v29 — cv58-boost-v14-forward-v29

## Forward CC setpoint: 5 A → 3 A

| Item | Value |
|------|--------|
| `FWD_TARGET_CC_CURRENT` | **3.0 A** |
| `FWD_BAT_CURRENT_HARD_A` | **3.75 A** (CC+0.75) |
| HW design (feedforward) | still **5 A @ D≈45%** via `FWD_DESIGN_I_AT_D45` |
| SoftStart / FF duty for 3 A | ~27% (`0.45 × 3/5`) |

Boost unchanged. Debug shows `CC=3A HW5A@45%`.
