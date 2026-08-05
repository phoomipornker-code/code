# Changelog v73 — cv58-boost-v14-forward-v73

## Serial สะอาด — ตัด WARN/INFO ที่รบกวน

รูปแบบที่ถูก (คงไว้):

```text
[BOOT] …
Tim           Iin        Vin     Iout    Vout    Duty
00:00:02     0.00       1.1    0.00   50.94     0
```

ตัดออกโดยค่าเริ่มต้น (`ENABLE_EVENT_LOG=false`, `ENABLE_DEBUG_STATUS=false`):
- `[WARN] ADC glitch…`
- `[WARN] AC sag…`
- `[INFO] Enter FORWARD…` / SoftStart done / CC→CV
- `[STAT] START…`

ยังแสดง: `[BOOT]`, ตารางทุก 1s, `[CRITICAL]`, Battery FULL

ถ้าต้องการ log เก่า: ตั้ง `ENABLE_EVENT_LOG = true` ใน `firmware.ino`

Flash: `[BOOT] cv58-boost-v14-forward-v73 | …`
