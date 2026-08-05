# Changelog v83 — cv58-boost-v14-forward-v83

## Boost CV ไม่นิ่ง (field)

Log ใน BOOST_CV:
- Vbat แกว่ง ~54.8 ↔ 55.5 V
- Ibat เหวี่ยง ~0.3 ↔ 1.6 A
- Duty ปีน 17% → 21%
- PV อ่อน/ผันผวน (~13–110 W) แต่ CV ขอกระแสสูง → ล่า

### แก้เฉพาะ BOOST_NEW_CV
- แคป `iReq` จาก Ppv กรองแล้ว (CV เท่านั้น — CC ไม่แคป)
- แคปกลางช่วงเมื่อ V ≥ 55.0 V (ไม่ดึงถึง 3.5 A)
- near-band กว้างขึ้น 0.35 → **0.80 V** (โหมดนุ่มเร็วขึ้น)
- Iref slew ช้าลง 0.08 → **0.04 A/tick**
- duty step / slew / PI ใน CV อ่อนลง
- CV exit ลดเป็น **54.20 V** (เดิม 54.80 — การแกว่งใกล้ 54.8 สตาร์ทตัวจับเวลาออก CV)

Forward ไม่เปลี่ยน

Boot: `[BOOT] cv58-boost-v14-forward-v83`
