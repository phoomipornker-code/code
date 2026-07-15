# วงจรฟอร์เวิร์ดคอนเวอร์เตอร์ (Forward Converter)

เอกสารและตัวอย่างจำลอง **Single-Switch Forward + ขดรีเซ็ต Nr**

## โครงสร้างไฟล์

| ไฟล์ | คำอธิบาย |
|------|----------|
| [docs/forward-converter.md](docs/forward-converter.md) | หลักการทำงานทั่วไป |
| [docs/design-240vac-58v-5a.md](docs/design-240vac-58v-5a.md) | ออกแบบ **AC 240 V → DC 58 V / 5 A** แบบสวิตช์เดียว + Nr |
| [docs/bom-240vac-58v-5a.md](docs/bom-240vac-58v-5a.md) | **สรุปอุปกรณ์ (BOM)** |
| [sim/design_240vac_58v_5a.py](sim/design_240vac_58v_5a.py) | เครื่องคิดเลขพารามิเตอร์ |
| [sim/forward_converter.py](sim/forward_converter.py) | จำลอง waveform |
| [requirements.txt](requirements.txt) | dependencies |

## ออกแบบสเปก AC 240 V → 58 V / 5 A

**ความถี่ 50–67 kHz ใช้ได้** — แนะนำ **65 kHz**

```bash
pip install -r requirements.txt
python sim/design_240vac_58v_5a.py --fs 65000
python sim/design_240vac_58v_5a.py --fs 50000   # ขอบล่าง
python sim/forward_converter.py --design 240vac-nr
```

จุดออกแบบหลัก (\(N_r = N_p\), แกน **ETD49**, **65 kHz**):

- ขด \(N_p:N_s:N_r = \mathbf{50:22:50}\) — ลวด **SWG 24**: 2× / 3× / 1× ขนาน
- \(B_{\max}\approx 0.20\,\mathrm{T}\)
- MOSFET **STW20N95K5** + RCD (**1 nF**, **100 Ω / 5–10 W**, UF4007)
- Dr **UF4007**
- \(L \approx 540\,\mu\mathrm{H}\), \(C_o = 470\text{–}1000\,\mu\mathrm{F}\)
- ห้ามใช้ขด 32:14:32 กับ fs 50–67 kHz (B จะสูงเกิน)

## สรุปโทโพโลยี

- \(V_o = V_{in}\cdot(N_s/N_p)\cdot D\)
- รีเซ็ตด้วย Nr: \(D_{\max}\le N_p/(N_p+N_r)\)
- \(V_{DS}\approx V_{in}(1+N_p/N_r)\)
