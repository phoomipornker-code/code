# Changelog v24 — cv58-boost-v14-forward-v24

## Forward control aligned with proven Boost

Forward SoftStart→CC→CV→DONE now uses the same control style as Boost v14:

| Item | Before | v24 (= Boost style) |
|------|--------|---------------------|
| SoftStart ready | I≥0.35 / timeout / no-load fault latch | **I≥0.4 A หรือ timeout 2.5 s** (ไม่มี no-load latch) |
| Curr PI | 8 / 35 | **14 / 55** |
| Volt PI | 0.70 / 0.35 | **0.85 / 0.45** |
| Iref slew / duty steps | 0.06 / 0.6–2.0 | **0.08 / 0.8–2.5** |
| Duty slew | 3 / 5 | **4 / 6** |
| CV near / bleed | milder | **เหมือน Boost** (nearCap 1.2, bleed 0.8+4·over) |
| Over-current | latched shutdown | **soft duty cut** แบบ Boost |

ยังต่างจาก Boost ตามฮาร์ดแวร์:
- ไม่มี MPPT (CC คงที่ 5 A)
- `MAX_DUTY_FORWARD = 460`, PWM 67 kHz
- อินพุต = บริดจ์ DC จาก AC 110 V
