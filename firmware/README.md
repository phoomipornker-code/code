# เฟิร์มแวร์ ESP32 — ชาร์จเจอร์ Boost PV + Forward AC

ไฟล์หลัก: [`firmware.ino`](firmware.ino)  
แท็ก: `cv58-boost-v14-forward-v31`

## นโยบายเวอร์ชันนี้

- **Boost (PV):** **แช่แข็ง** ตามโค้ดที่ชาร์จ PV สำเร็จ `cv58-stability-v14-cv-stable` — PWM **50 kHz**, CC **6 A**, SoftStart→CC_MPPT→CV→DONE
- **Forward (AC):** SoftStart→CC→CV→DONE — **CC 3 A** (HW 5 A@D≈45% feedforward) @ **67 kHz**
- **เลือกโหมดก่อน START:** กด **STOP** ตอน STANDBY สลับ `BOOST` ↔ `FORWARD` แล้วค่อยกด **START** (ไม่สลับอัตโนมัติตามอินพุต)

## วิธีแฟลชลง ESP32 (Arduino IDE)

1. ติดตั้ง **Arduino IDE** + บอร์ด **ESP32** (Espressif)
2. เลือกบอร์ด: **ESP32 Dev Module** (หรือบอร์ดที่ใช้)
3. ติดตั้งไลบรารี:
   - `Adafruit ADS1X15`
   - `LiquidCrystal I2C` (Frank de Brabander หรือเทียบเท่า)
4. เปิดโฟลเดอร์ `firmware` (ไฟล์ `firmware.ino` ต้องอยู่ในโฟลเดอร์ชื่อเดียวกัน)
5. Upload ลง ESP32
6. เปิด Serial Monitor **115200 baud** — ควรเห็น `[BOOT] Firmware: cv58-boost-v14-forward-v31`

## ปุ่มใช้งาน

| ปุ่ม | ตอน STANDBY | ตอนชาร์จ |
|------|-------------|----------|
| **STOP** (GPIO 26) | สลับโหมด **BOOST ↔ FORWARD** | หยุดชาร์จ / เคลียร์ OVP เมื่อแรงดันปลอดภัย |
| **START** (GPIO 25) | เริ่มเฉพาะโหมดที่เลือก (ถ้าอินพุตพร้อม) | — |

LCD บรรทัดแรกตอน STANDBY แสดงโหมด เช่น `STANDBY  BOOST` / `STANDBY  FORWD`  
แถวล่าง: `STOP=mode START=go`

## พิน (GPIO)

| หน้าที่ | GPIO |
|---------|------|
| PWM Forward | **14** @ 67 kHz |
| PWM Boost | **27** @ 50 kHz |
| Relay PV | 32 |
| Relay AC | 33 |
| ปุ่ม START | 25 |
| ปุ่ม STOP | 26 |
| I2C SDA / SCL | 21 / 22 |
| ADS1115 แรงดัน / กระแส | 0x48 / 0x49 |

## ค่าควบคุม

| รายการ | Boost | Forward |
|--------|-------|---------|
| PWM | **50 kHz** | **67 kHz** |
| CC | **6.0 A** | **3.0 A** |
| CV | 56.0 V | 56.0 V |
| Duty max | 760 | 460 (~45%, Nr=Np) |
| Input min | PV ≥ 42 V | บริดจ์ DC ≥ **95 V** (AC 110 V) |
| โหมด | SoftStart→CC_MPPT→CV→DONE | SoftStart→CC→CV→DONE |

แบตต้องอยู่ในช่วง **40.0 … 56.4 V** ก่อน START  
OVP latch เคลียร์ด้วย **STOP** เมื่อแรงดันลดลง

สรุปลอจิก Forward: [`FORWARD_CONTROL_NOTES.md`](FORWARD_CONTROL_NOTES.md)  
Changelog: [`CHANGELOG_v31.md`](CHANGELOG_v31.md) · [`CHANGELOG_v30.md`](CHANGELOG_v30.md) · [`CHANGELOG_v29.md`](CHANGELOG_v29.md) · [`CHANGELOG_v28.md`](CHANGELOG_v28.md) · [`CHANGELOG_v27.md`](CHANGELOG_v27.md) · [`CHANGELOG_v26.md`](CHANGELOG_v26.md) · [`CHANGELOG_v25.md`](CHANGELOG_v25.md) · [`CHANGELOG_v24.md`](CHANGELOG_v24.md) · [`CHANGELOG_v23.md`](CHANGELOG_v23.md) · [`CHANGELOG_v22.md`](CHANGELOG_v22.md) · [`CHANGELOG_v21.md`](CHANGELOG_v21.md) · [`CHANGELOG_v20.md`](CHANGELOG_v20.md) · [`CHANGELOG_v19.md`](CHANGELOG_v19.md) · [`CHANGELOG_v18.md`](CHANGELOG_v18.md) · [`CHANGELOG_v17.md`](CHANGELOG_v17.md)
