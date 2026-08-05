# Changelog v21 — cv58-boost-v14-forward-v21

## Debug shows operating mode

Serial debug line now includes selected path and run phase:

```text
[DEBUG] System: OFF | Mode: FORWARD | Run: STANDBY | Duty: 0%
[DEBUG] System: ON  | Mode: BOOST   | Run: BOOST_CCMP | Duty: 42%
[DEBUG] System: ON  | Mode: FORWARD | Run: FWD_CV | Duty: 28%
```

| Field | Meaning |
|-------|---------|
| `Mode` | โหมดที่เลือกด้วย STOP ตอน STANDBY (`BOOST` / `FORWARD`) |
| `Run` | เฟสทำงานจริง: `STANDBY`, `STARTING`, `BOOST_*`, `FWD_*`, `FULL_HOLD`, `OVP_LOCK` |
