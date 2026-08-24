# วงจรฟอร์เวิร์ดคอนเวอร์เตอร์

Single-switch forward + ขดรีเซ็ต \(N_r\) สำหรับชาร์จ **16S LiFePO4** จากไฟบ้าน

| | |
|--|--|
| อินพุต | AC **240 V** (216–264 V) |
| เอาต์พุต | DC **58 V / 5 A** (290 W) |
| โทโพโลยี | Isolated forward, สวิตช์ตัวเดียว + tertiary reset |
| หม้อแปลง | **ETD49/25/16 N87** · \(N_p:N_s:N_r = 48:21:48\) |
| \(f_s\) | **67 kHz** · \(D \le 0.45\) |
| Q1 | **STW20N95K5** (950 V) + RCD snubber |

ภาพวงจร: [artifacts/forward_schematic.svg](artifacts/forward_schematic.svg)

ช่วง ON / OFF: [artifacts/forward_on_off.svg](artifacts/forward_on_off.svg)

คลื่นอุดมคติ CCM: [artifacts/forward_waveforms.svg](artifacts/forward_waveforms.svg)

```bash
python3 -m sim.generate
python3 -m unittest discover -s sim -p 'test_*.py'
python3 -m sim.design
```

## เอกสาร

- [docs/forward-converter.md](docs/forward-converter.md) — หลักการและวิธีอ่านวงจร
- [docs/design-240vac-58v-5a.md](docs/design-240vac-58v-5a.md) — คำนวณรอบขด, L, ความเครียด
- [docs/bom-240vac-58v-5a.md](docs/bom-240vac-58v-5a.md) — รายการชิ้นส่วน

## ความปลอดภัย

วงจรนี้ต่อ **ไฟบ้าน 240 VAC** — แรงดันปฐมภูมิเป็นอันตรายถึงชีวิต มีฟิวส์, แยกกราวนด์ทุติยภูมิ, และทดสอบด้วย isolation transformer / variac ก่อนต่อแบตเตอรี่ เอกสารนี้เป็นแนวออกแบบ ไม่ใช่ใบรับรองผลิต
