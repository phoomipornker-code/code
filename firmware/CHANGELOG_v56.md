# Changelog v56 — cv58-boost-v14-forward-v56

## BAT voltage scale trim

```cpp
const float CAL_SCALE_V_BAT = 41.5;  // was 41.85
```

Field trim of the BAT ADS voltage scale. All BAT readings (`V`, `Vf`, CV/OVP compares) use this constant.

Boost path unchanged otherwise. Forward v55 ADC/OVP/CV taper fixes retained.

Flash until boot shows:

`[BOOT] Firmware: cv58-boost-v14-forward-v56`
