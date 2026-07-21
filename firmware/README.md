# เฟิร์มแวร์ ESP32 — ชาร์จเจอร์ Boost PV + Forward AC

ไฟล์หลัก: [`firmware.ino`](firmware.ino)  
แท็ก: `cv58-forward-67khz-5a-v16`

## วิธีแฟลชลง ESP32 (Arduino IDE)

1. ติดตั้ง **Arduino IDE** + บอร์ด **ESP32** (Espressif)
2. เลือกบอร์ด: **ESP32 Dev Module** (หรือบอร์ดที่ใช้)
3. ติดตั้งไลบรารี:
   - `Adafruit ADS1X15`
   - `LiquidCrystal I2C` (Frank de Brabander หรือเทียบเท่า)
4. เปิดโฟลเดอร์ `firmware` (ไฟล์ `firmware.ino` ต้องอยู่ในโฟลเดอร์ชื่อเดียวกัน)
5. Upload ลง ESP32
6. เปิด Serial Monitor **115200 baud** — ควรเห็น `[BOOT] Firmware: cv58-forward-67khz-5a-v16`

ดาวน์โหลดไฟล์ดิบ:
https://raw.githubusercontent.com/phoomipornker-code/code/cursor/forward-converter-circuit-12b8/firmware/firmware.ino

## พิน (GPIO)

| หน้าที่ | GPIO |
|---------|------|
| PWM Forward | **14** |
| PWM Boost | **27** |
| Relay PV | 32 |
| Relay AC | 33 |
| ปุ่ม START | 25 |
| ปุ่ม STOP | 26 |
| I2C SDA / SCL | 21 / 22 |
| ADS1115 แรงดัน / กระแส | 0x48 / 0x49 |

## ค่าควบคุม Forward

| รายการ | ค่า |
|--------|-----|
| ความถี่ PWM | **67 kHz**, 10-bit |
| Duty สูงสุด Forward | **460** (~45%, Nr=Np) |
| CC / CV | **5.0 A** / **56.0 V** |
| โหมด | SoftStart → CC → CV → DONE |
| รีชาร์จ | ≤ 54.0 V |
| หยุดแรงดันสูง | ≥ 56.8 V นาน 300 ms |

สรุปลอจิก: [`FORWARD_CONTROL_NOTES.md`](FORWARD_CONTROL_NOTES.md)  
Changelog: [`CHANGELOG_v16.md`](CHANGELOG_v16.md)  
โฟลว์ชาร์ต: [`../docs/flowchart-forward-only.md`](../docs/flowchart-forward-only.md)
