# วงจรฟอร์เวิร์ดคอนเวอร์เตอร์ (Forward Converter)

เอกสารและตัวอย่างจำลอง **Single-Switch Forward + ขดรีเซ็ต Nr**

## โครงสร้างไฟล์

| ไฟล์ | คำอธิบาย |
|------|----------|
| [docs/forward-converter.md](docs/forward-converter.md) | หลักการทำงานทั่วไป |
| [docs/design-240vac-58v-5a.md](docs/design-240vac-58v-5a.md) | ออกแบบ **AC 240 V → DC 58 V / 5 A** แบบสวิตช์เดียว + Nr |
| [sim/design_240vac_58v_5a.py](sim/design_240vac_58v_5a.py) | เครื่องคิดเลขพารามิเตอร์ |
| [sim/forward_converter.py](sim/forward_converter.py) | จำลอง waveform (รวมโหมด 240vac-nr) |
| [requirements.txt](requirements.txt) | dependencies |

## ออกแบบสเปก AC 240 V → 58 V / 5 A (สวิตช์เดียว + Nr)

```bash
pip install -r requirements.txt
python sim/design_240vac_58v_5a.py
python sim/forward_converter.py --design 240vac-nr
```

จุดออกแบบหลักเมื่อ \(N_r = N_p\) บนแกน **ETD49/25/16 (N87)**:

- ขด \(N_p:N_s:N_r = \mathbf{32:14:32}\) (\(n=0.4375\), \(B_{\max}\approx 0.20\,\mathrm{T}\))
- \(D_{\max} \approx 0.45\), ที่ \(V_{in}=340\,\mathrm{V}\) ได้ \(D\approx 0.39\)
- \(V_{DS,max}\approx 2V_{in}\) → MOSFET **STW20N95K5** (950 V, TO-247) + RCD snubber
- ไดโอดรีเซ็ต **Dr = UF4007** (1 A / 1000 V) หรือ STTH112A
- \(L \approx 350\,\mu\mathrm{H}\), \(C_o = 470\text{–}1000\,\mu\mathrm{F}\)

## สรุปโทโพโลยี

- ส่งพลังงานตอนสวิตช์เปิด: \(V_o = V_{in}\cdot(N_s/N_p)\cdot D\)
- รีเซ็ตฟลักซ์ด้วยขด Nr: \(D_{\max}\le N_p/(N_p+N_r)\)
- ความเครียดสวิตช์: \(V_{DS}\approx V_{in}(1+N_p/N_r)\)
