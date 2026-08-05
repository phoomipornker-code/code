# Changelog v76 — cv58-boost-v14-forward-v76

## Duty ช้าแปลก — แก้ 2 จุด

### สาเหตุ
1. **v34/v42** ตั้ง SoftStart/CC ขั้นเล็กมาก (กัน Cin) → เปิดช้า
2. คอลัมน์ **Duty โชว์เป็น %** (raw/1023) → เห็นแค่ 0→1→2…45 ทั้งที่ raw จริงสูงถึง 460

### แก้
| รายการ | เดิม (v75) | ตอนนี้ |
|--------|------------|--------|
| SoftStart step | 0.8 | **1.5** |
| CC step / far | 0.6 / 1.2 | **1.2 / 2.0** |
| SoftStart เวลา | 5 s | **3.5 s** |
| SoftStart seed | 45% ของเป้า | **55%** |
| Serial Duty | % | **raw 0–460** (ตรง `DmaxF`) |

CV ยังขั้นละเอียด (const-V) · freeze climb ถ้า AC&lt;115 V ยังอยู่

Flash: `[BOOT] cv58-boost-v14-forward-v76 | …`
Duty ในตารางควรปีนเป็น raw เช่น `60 → 150 → 280` ไม่ใช่ `%` เล็ก ๆ
