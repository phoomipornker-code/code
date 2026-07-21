# เฟิร์มแวร์ชาร์จเจอร์ (Boost PV + Forward AC)

แท็กที่อ้างอิง: `cv58-stability-v14-cv-stable`

วางสเก็ตช์เต็มเป็น `charger_main.ino` ในโฟลเดอร์นี้ (เปิดด้วย Arduino IDE เป็น sketch book ของโฟลเดอร์ `firmware`)

สรุปลอจิก Forward จากโค้ดหลัก: [`FORWARD_CONTROL_NOTES.md`](FORWARD_CONTROL_NOTES.md)

โฟลว์ชาร์ต Forward (หมายเลข ①–⑰ + อธิบายไทย):  
[`../docs/flowchart-forward-firmware.md`](../docs/flowchart-forward-firmware.md)

## ค่าจากโค้ดหลัก (Forward)

| รายการ | ค่า |
|--------|-----|
| บอร์ด | ESP32 |
| `PWM_FREQ` | **50 kHz**, 10-bit |
| Forward PWM | GPIO **14**, `MAX_DUTY_FORWARD = 490` |
| Boost PWM | GPIO **27**, `MAX_DUTY_BOOST = 760` |
| CC / CV | **6.0 A** / **56.0 V** |
| ควบคุม Forward | PID คู่ แล้ว `min(pid_cc, pid_cv)` |

## จุดที่ควรจัดให้ตรงฮาร์ดแวร์ Forward + Nr

| รายการ | ในโค้ดตอนนี้ | แนะนำกับหม้อแปลง Nr=Np |
|--------|--------------|-------------------------|
| `PWM_FREQ` | 50000 | 50000–67000 ได้ |
| `MAX_DUTY_FORWARD` | 490 (~47.9%) | **≤ 460 (~45%)** |
| MOSFET | — | STW20N95K5 |
| หม้อแปลง | — | ETD49 ตาม fs (เช่น 50:22:50 @ 65 kHz) |
