# Changelog v78 — cv58-boost-v14-forward-v78

## คืนโค้ดที่ชาร์จได้ = v63

ผู้ใช้ยืนยันว่า `[BOOT] cv58-boost-v14-forward-v63` ชาร์จได้

### สิ่งที่ทำ
- คืน **`firmware.ino` ทั้งไฟล์** จากแท็ก `cv58-boost-v14-forward-v63`
- เปลี่ยนแท็กบูตเป็น **`cv58-boost-v14-forward-v78`** (เนื้อหาควบคุม = v63)
- **ไม่** นำการแก้ v64–v77 (Serial table / TaskSerialLog / duty % / SoftStart เร็วขึ้น ฯลฯ) กลับเข้ามา

### ยังเหมือน v63
- Boost แช่แข็ง v14
- Forward SoftStart→CC→CV→DONE (no PID), CC 3 A, CV 55.90 V, Dmax 460
- LCD ปิดตอนชาร์จจน FULL
- `[STAT]` เปิดตาม v63

Flash แล้วต้องเห็น:

```text
[BOOT] cv58-boost-v14-forward-v78 | …
```
