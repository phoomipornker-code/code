# cv58-boost-v14-forward-v87

Forward = cascade PI ตามไดอะแกรม + **เทaper กระแสก่อน 57 V** ให้ BMS บาลานซ์ทัน  
Boost ไม่ได้แก้

สาเหตุ BMS ตัด: ที่ 57 V ลูปแรงดัน Kp=15 ยังสั่ง Iref = 3 A จนใกล้ 58.2 V  
เซลล์ที่เต็มก่อน (3.65 V) โดน OVP เพราะบาลานซ์ passive ตามไม่ทัน

## Iref vs แรงดันแพ็ก

| Vpack | Iref สูงสุด |
|-------|-------------|
| ≤ 55.2 V | 3.0 A (CC) |
| 56.0 V | ~1.7 A |
| ≥ 56.8 V | **0.35 A** (หน้าต่างบาลานซ์) |
| 58.4 V | CV ลดต่อจน FULL |

## แฟลช

Arduino IDE → เปิด `cv58-boost-v14-forward-v84.ino`  
ต้องเห็น `cv58-boost-v14-forward-v87` และบรรทัด `FWD Iref taper 55.2V@3A → 56.8V@0.35A`
