# Changelog v58 — cv58-boost-v14-forward-v58

## Serial: ลดดีบัก เหลือเฉพาะที่จำเป็น

User request: ลดข้อมูลดีบักบน Serial ให้แสดงเฉพาะที่จำเป็น

### Changes
- ตัด dump หลายบรรทัด (`[DEBUG]` + RAW mV + I ZERO + เส้นคั่น)
- สถานะเหลือ **หนึ่งบรรทัด** `[STAT]` ทุก 3 s ตอนชาร์จ / 5 s ตอน standby
- ลดอัตรา `[WARN] ADC glitch` (ทุก 5 s)
- ย่อข้อความ `[BOOT]` ให้สั้นลง
- คง `[CRITICAL]` / `[INFO]` สำคัญ (เข้าโหมด, STOP, FULL, OVP)

ตัวอย่างตอนชาร์จ:

```text
[STAT] F_CV D=44% BAT 55.63V/55.63V I=0.16A IN=152.0V
```

Flash until boot shows:

`[BOOT] cv58-boost-v14-forward-v58 | …`
