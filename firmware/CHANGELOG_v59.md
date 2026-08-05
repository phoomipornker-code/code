# Changelog v59 — cv58-boost-v14-forward-v59

## Field: LCD เพี้ยน / ตัวอักษรกตอนชาร์จ

รูปจากสนาม: จอเต็มไปด้วยสัญลักษณ์เพี้ยน (ทึบ, `/`, `_`) ระหว่างชาร์จ

**Cause:** custom CGRAM (`createChar`) เสียจาก EMI ขณะ PWM/กำลังสูง + จอ HD44780 หลุดซิงก์ — ไม่ควร `Wire.end` ตอน `system_ON` (v51/v52)

### v59 fixes
- รูปแบตใช้ **CGROM เท่านั้น**: `[████------]* 78%` ด้วย `0xFF` solid block — ไม่ใช้ `createChar`
- Soft resync ทุก ~12 s ตอนชาร์จ: `lcd.init()` + clear **โดยไม่** `Wire.end` / pin-bang
- ยังแสดงรูปแบต + SOC ตอนชาร์จ / FULL HOLD

ตัวอย่าง:

```text
FORW CV  CHARGE
[##########]*100%
BAT: 55.6V I:0.16A
IN:152.0V D: 44%
```

Flash until boot shows:

`[BOOT] cv58-boost-v14-forward-v59 | …`
