# cv58-boost-v14-forward-v84

ควบคุมชุดเดียวกับ v81 (v75 field-proven) — **เปลี่ยนเฉพาะเป้า CV เป็น 58.00 V**

PWM / ADC / ปุ่ม / ลูป SoftStart→CC→CV→DONE ไม่ได้แก้

## แฟลช

1. เปิดโฟลเดอร์นี้ใน Arduino IDE
2. เปิด `cv58-boost-v14-forward-v84.ino`
3. Board: ESP32 Dev Module · Upload
4. Serial 115200 ต้องเห็น:

```text
[BOOT] cv58-boost-v14-forward-v84 | B_CC=6A F_CC=3A CV=58.00V DmaxF=460
```

## ค่าที่ขยับ (ให้ชาร์จถึง 58 V ได้)

เดิมค้างที่ ~55.9–56.0 V เพราะ CV, BMS-preempt, high-V stop และ HARD OVP อยู่ต่ำกว่า 58 V

| ค่า | v81 | v84 |
|-----|-----|-----|
| Forward / Boost CV | 55.90 / 56.00 | **58.00** |
| เข้า CV (Fwd / Boost) | 55.00 / 55.50 | 57.10 / 57.50 |
| BMS duty-cap zone | 55.95 | 58.20 (เหนือ CV — ไม่ตัดตอนโฮลด์ 58 V) |
| High-V FULL HOLD | 56.80 | 58.40 |
| HARD OVP | 57.80 | 59.50 |
| เริ่มชาร์จได้ถึง | 56.40 | 58.40 |

16S LFP: 58.00 V ≈ 3.625 V/cell — ต่ำกว่า HXYP OVP 3.65 V/cell (58.40 V) เล็กน้อย
