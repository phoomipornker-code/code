# วงจรฟอร์เวิร์ดคอนเวอร์เตอร์ (Forward Converter)

เอกสารและตัวอย่างจำลอง **Single-Switch Forward + ขดรีเซ็ต Nr**

## โครงสร้างไฟล์

| ไฟล์ | คำอธิบาย |
|------|----------|
| [docs/forward-converter.md](docs/forward-converter.md) | หลักการทำงานทั่วไป |
| [docs/design-240vac-58v-5a.md](docs/design-240vac-58v-5a.md) | ออกแบบ **AC 240 V → DC 58 V / 5 A** |
| [docs/bom-240vac-58v-5a.md](docs/bom-240vac-58v-5a.md) | สรุปอุปกรณ์ (BOM) |
| [docs/flowchart-forward-only.md](docs/flowchart-forward-only.md) | **โฟลว์ชาร์ตเฉพาะ Forward** (หมายเลข + อธิบายไทย) |
| [docs/flowchart-forward-only.mmd](docs/flowchart-forward-only.mmd) | Mermaid ล้วนเฉพาะ Forward |
| [firmware/](firmware/) | โน้ตเฟิร์มแวร์ ESP32 Forward |
| [sim/design_240vac_58v_5a.py](sim/design_240vac_58v_5a.py) | เครื่องคิดเลขพารามิเตอร์ |
| [sim/forward_converter.py](sim/forward_converter.py) | จำลอง waveform |

## ออกแบบสเปก — Forward @ **67 kHz**

```bash
pip install -r requirements.txt
python sim/design_240vac_58v_5a.py --fs 67000
python sim/forward_converter.py --design 240vac-nr
```

จุดออกแบบหลัก (\(N_r = N_p\), แกน **ETD49**, **\(f_s=67\,\mathrm{kHz}\)**):

- ขด \(N_p:N_s:N_r = \mathbf{48:21:48}\) — ลวด **SWG 24**: 2× / 3× / 1× ขนาน
- \(B_{\max}\approx 0.20\,\mathrm{T}\)
- MOSFET **STW20N95K5** + RCD (**1 nF**, **100 Ω**, UF4007)
- Dr **UF4007**
- \(L \approx 520\,\mu\mathrm{H}\), \(C_o = 470\text{–}1000\,\mu\mathrm{F}\)
- เฟิร์มแวร์: `PWM_FREQ = 67000`, `MAX_DUTY_FORWARD ≤ 460` (~45%)

## สรุปโทโพโลยี

- \(V_o = V_{in}\cdot(N_s/N_p)\cdot D\)
- รีเซ็ตด้วย Nr: \(D_{\max}\le 0.45\) เมื่อ \(N_r=N_p\)
- \(V_{DS}\approx 2V_{in}\) → MOSFET 950 V + RCD
