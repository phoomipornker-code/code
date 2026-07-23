# Changelog v57 — cv58-boost-v14-forward-v57

## LCD: รูปแบตตอนชาร์จ

User request: หน้าจอแสดงรูปแบตขณะชาร์จ

### Changes
- HD44780 custom characters: โครงแบตแนวนอน + ช่องเติมตาม SOC (5 ช่อง) + ขั้ว + สัญลักษณ์สายฟ้าเมื่อมีกระแส
- หน้าจอตอนชาร์จ / FULL HOLD ใช้ไอคอนแบตแทนแถบ `#---` เดิม
- โหลด CGRAM ใหม่ทุกครั้งหลัง `lcd.init()` (รวม standby I2C reinit)

ตัวอย่างตอนชาร์จ:

```text
FORW CV  CHARGE
[████░] ⚡  92%
BAT: 55.6V I:0.16A
IN:152.0V D: 44%
```

(บนจอจริงเป็น glyph custom ไม่ใช่ Unicode)

Flash until boot shows:

`[BOOT] Firmware: cv58-boost-v14-forward-v57`
