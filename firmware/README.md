ไฟล์หลัก: [`charger_main.ino`](charger_main.ino) — แท็ก `cv58-forward-67khz-5a-v16`

# เฟิร์มแวร์ชาร์จเจอร์ (Boost PV + Forward AC)

วางสเก็ตช์เต็มเป็น `charger_main.ino` ในโฟลเดอร์นี้ (เปิดด้วย Arduino IDE เป็น sketch book ของโฟลเดอร์ `firmware`)

สรุปลอจิก Forward: [`FORWARD_CONTROL_NOTES.md`](FORWARD_CONTROL_NOTES.md)  
Changelog: [`CHANGELOG_v16.md`](CHANGELOG_v16.md)

โฟลว์ชาร์ต Forward:  
[`../docs/flowchart-forward-only.md`](../docs/flowchart-forward-only.md)

## ค่าจากโค้ดหลัก (Forward)

| รายการ | ค่า |
|--------|-----|
| บอร์ด | ESP32 |
| `PWM_FREQ` | **67 kHz**, 10-bit |
| Forward PWM | GPIO **14**, `MAX_DUTY_FORWARD = 460` |
| Boost PWM | GPIO **27**, `MAX_DUTY_BOOST = 760` |
| CC / CV | **5.0 A** / **56.0 V** |
| ควบคุม Forward | **SoftStart → CC → CV → DONE** (PI ซ้อน) |

## จุดที่ควรจัดให้ตรงฮาร์ดแวร์ Forward + Nr

| รายการ | ในโค้ด | ฮาร์ดแวร์ |
|--------|--------|-----------|
| `PWM_FREQ` | **67000** | Forward @ 67 kHz |
| `MAX_DUTY_FORWARD` | **460** (~45%) | Nr=Np, D≤0.45 |
| MOSFET | — | STW20N95K5 |
| หม้อแปลง | — | ETD49 **48:21:48 @ 67 kHz** |
