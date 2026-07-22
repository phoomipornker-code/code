# Changelog v22 — cv58-boost-v14-forward-v22

## Debug prints every sensor channel

```text
[DEBUG] System: ON  | Mode: FORWARD | Run: FWD_CC | Duty: 28% (raw=286)
  [PV ] V:  0.00V  I:  0.00A  |P|:   0.0W  (mag I: 0.00A)
  [AC ] V:101.20V  I:  0.45A  |P|:  45.5W  (bridge DC)
  [BAT] V: 50.10V  I:  4.80A  Vf: 50.05V  If:  4.75A  Iabs: 4.80A
  [RAW V mV] PV(ch0):    1.5  AC(ch2): 1415.0  BAT(ch1): 1195.0
  [RAW I mV] PV(ch0): 1659.0  AC(ch1): 1668.0  BAT(ch2): 1760.0
  [I ZERO]   PV: 1659.7  AC: 1646.9  BAT: 1646.9  (boot offset mV)
  [FWD ] Iref_cv:0.50A duty_acc:286.0
```

| Line | Sensors |
|------|---------|
| `[PV]` / `[AC]` / `[BAT]` | ค่าหลังคาลิเบรตครบ 3 ช่อง V+I |
| `[BAT] Vf/If` | ค่ากรองแบตที่ใช้ในลูปควบคุม |
| `[RAW V/I mV]` | ADS1115 ดิบทั้ง 6 ช่อง |
| `[I ZERO]` | offset ศูนย์กระแสตอนบูต |
