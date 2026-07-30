# Changelog v82 — cv58-boost-v14-forward-v82

## Restore Boost safety from `cv58-stability-v14-cv-stable`

ผู้ใช้รายงานว่าส่วนควบคุม Boost มีปัญหา → คืน **BMS/OVP/runaway/spike ของ Boost** ให้ตรงโค้ดที่ชาร์จ PV สำเร็จ

### Boost (ตรง v14)
- SoftStart→CC_MPPT→CV→DONE (PI) ไม่เปลี่ยน
- BMS-OPEN: detect **56.30 V**, I≤**1.20 A**, ตัดทันที + `charge_full_hold`
- HARD OVP: ตัดทันทีตอน Boost
- Runaway / Spike pre-cut: ตรรกะ v14 (Boost เท่านั้น)
- `CAL_SCALE_V_BAT = 41.85` (ค่าจาก v14)

### Forward (ไม่แตะพฤติกรรมที่พิสูจน์แล้ว)
- SoftStart→CC→CV→DONE step/hysteresis
- BMS-OPEN แยกค่า: detect **56.80 V**, I≤**0.25 A**, confirm + near-full ignore
- HARD OVP / spike แบบ hardened ยังใช้กับ Forward

Flash จนเห็น:

`[BOOT] cv58-boost-v14-forward-v82`
