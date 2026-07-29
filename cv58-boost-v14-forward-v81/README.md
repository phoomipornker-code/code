# cv58-boost-v14-forward-v81

โฟลเดอร์ sketch สำหรับ Arduino IDE — เปิดไฟล์นี้โดยตรง

## แฟลช

1. ดาวน์โหลด ZIP branch แล้วเปิดโฟลเดอร์นี้  
   หรือบน GitHub: `cv58-boost-v14-forward-v81/`
2. Arduino IDE → เปิด `cv58-boost-v14-forward-v81.ino`
3. Board: ESP32 Dev Module · Upload
4. Serial 115200 — ต้องเห็น:

```text
[BOOT] cv58-boost-v14-forward-v81 | …
```

## ค่าสำคัญ

- ควบคุม = ชุด v75 ที่ชาร์จได้
- ตาราง Serial ทุก **20 วินาที**
- `CAL_SCALE_V_BAT = 42.3`
- Boost แช่แข็ง v14 · Forward SoftStart→CC→CV→DONE (no PID)

## ปุ่ม

| ปุ่ม | STANDBY | ชาร์จ |
|------|---------|--------|
| STOP | สลับ BOOST ↔ FORWARD | กดค้าง ~350 ms หยุด |
| START | เริ่มโหมดที่เลือก | — |
