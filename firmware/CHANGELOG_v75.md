# Changelog v75 — cv58-boost-v14-forward-v75

## Serial: เริ่ม / หยุด / แรงดันเกิน / กระแสเกิน

พิมพ์เสมอ (แม้ `ENABLE_EVENT_LOG=false`) — ตารางยังเงียบ:

```text
[START] FORWARD SoftStart->CC->CV
[START] BOOST SoftStart->CC_MPPT->CV
[STOP] hold 350ms end (FORWARD)
[STOP] AC bridge lost (sustained). Auto-Shutdown.
[FULL] FORWARD CV V=… I=…
[OVP] HARD TRIP V=… filt=… lim=57.80. Output disabled.
[OC] FORWARD BAT I=… lim=3.75A (duty cut)
```

| แท็ก | ความหมาย |
|------|----------|
| `[START]` | เริ่มชาร์จ (เข้า SoftStart) |
| `[STOP]` | หยุด (กดค้าง / อินพุตหาย / ADC timeout / high-V) |
| `[FULL]` | แบตเต็ม |
| `[OVP]` | แรงดันเกิน (HARD / BMS-OPEN / SPIKE / RUNAWAY) |
| `[OC]` | กระแสเกิน hard limit (rate-limit ~1 s) |
| `[MODE]` | สลับโหมด STANDBY (จาก v74) |

Flash: `[BOOT] cv58-boost-v14-forward-v75 | …`
