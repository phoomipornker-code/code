# เฟิร์มแวร์ ESP32 — ชาร์จเจอร์ Boost PV + Forward AC

ไฟล์หลัก: [`firmware.ino`](firmware.ino)  
แท็ก: `cv58-boost-v14-forward-v17`

## นโยบายเวอร์ชันนี้

- **Boost (PV):** ยึดตามโค้ดที่ทดสอบจริง `cv58-stability-v14` — PWM **50 kHz**, CC **6 A**, SoftStart→CC_MPPT→CV→DONE
- **Forward (AC):** SoftStart→CC→CV→DONE ที่ **67 kHz / 5 A**, พร้อม safety สำหรับใช้งานจริง

## วิธีแฟลชลง ESP32 (Arduino IDE)

1. ติดตั้ง **Arduino IDE** + บอร์ด **ESP32** (Espressif)
2. เลือกบอร์ด: **ESP32 Dev Module** (หรือบอร์ดที่ใช้)
3. ติดตั้งไลบรารี:
   - `Adafruit ADS1X15`
   - `LiquidCrystal I2C` (Frank de Brabander หรือเทียบเท่า)
4. เปิดโฟลเดอร์ `firmware` (ไฟล์ `firmware.ino` ต้องอยู่ในโฟลเดอร์ชื่อเดียวกัน)
5. Upload ลง ESP32
6. เปิด Serial Monitor **115200 baud** — ควรเห็น `[BOOT] Firmware: cv58-boost-v14-forward-v17`

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
| CC | **6.0 A** | **5.0 A** |
| CV | 56.0 V | 56.0 V |
| Duty max | 760 | 460 (~45%, Nr=Np) |
| Input min | PV ≥ 42 V | AC ≥ **200 V** |
| โหมด | SoftStart→CC_MPPT→CV→DONE | SoftStart→CC→CV→DONE |

แบตต้องอยู่ในช่วง **40.0 … 56.4 V** ก่อน START  
OVP latch เคลียร์ด้วย **STOP** เมื่อแรงดันลดลง

สรุปลอจิก Forward: [`FORWARD_CONTROL_NOTES.md`](FORWARD_CONTROL_NOTES.md)  
Changelog: [`CHANGELOG_v17.md`](CHANGELOG_v17.md)
