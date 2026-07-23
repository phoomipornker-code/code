# Changelog v61 — cv58-boost-v14-forward-v61

## Field: ตัดชาร์จใกล้เต็ม (false BMS-OPEN → FULL_HOLD)

```text
[FWD ] phase=CV … Vf:55.21V Ibat:2.12A
[CRITICAL] BMS-OPEN/preempt at raw=56.38 filt=56.19 I=0.68A step=0.00. PWM off.
[DEBUG] … Run: FULL_HOLD …
```

**Cause:** `BMS_OPEN_CURRENT_MAX_A=1.20` ทำให้ I≈0.68 A ตอนใกล้เต็มถูกมองว่า “กระแสยุบ” + raw≥56.30 → ตัด PWM แล้วเข้า FULL ปลอม

### v61 fixes
- BMS-OPEN ต้อง I ≤ **0.25 A** (ไม่ใช่ 1.2 A)
- เกณฑ์ V เปิดจริง ≈ **56.80 V** และต้องมี filt สนับสนุน (หรือ jump ใหญ่)
- ช่วงใกล้เต็ม V≤56.6 + ยังมีกระแส → **ไม่** BMS-OPEN
- BMS-OPEN ไม่ตั้ง `FULL_HOLD` แล้ว (เป็น OVP latch ให้กด STOP เคลียร์)
- ชาร์จเต็มจริงยังใช้เงื่อนไขเดิม: V≥55.8 และ I≤0.5 A นาน 60 s

Flash until boot shows:

`[BOOT] cv58-boost-v14-forward-v61 | …`

หมายเหตุ: ล็อกที่มี `[DEBUG]` + RAW mV เป็นเฟิร์มแวร์เก่า — แฟลช v61 แล้วจะเหลือ `[STAT]` บรรทัดเดียว
