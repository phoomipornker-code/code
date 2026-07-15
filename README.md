# วงจรฟอร์เวิร์ดคอนเวอร์เตอร์ (Forward Converter)

เอกสารและตัวอย่างจำลองวงจร **Forward Converter** — สวิตชิ่งเพาเวอร์ซัพพลายชนิดแยกกราวนด์ (isolated) ที่ส่งพลังงานไปยังโหลดขณะสวิตช์เปิด

## โครงสร้างไฟล์

| ไฟล์ | คำอธิบาย |
|------|----------|
| [docs/forward-converter.md](docs/forward-converter.md) | หลักการทำงาน สมการออกแบบ และจุดสำคัญ |
| [sim/forward_converter.py](sim/forward_converter.py) | สคริปต์ Python จำลอง waveform แบบอุดมคติ |
| [requirements.txt](requirements.txt) | dependencies สำหรับการจำลอง |

## รันการจำลอง

```bash
pip install -r requirements.txt
python sim/forward_converter.py
```

ผลลัพธ์จะบันทึกภาพคลื่นที่ `artifacts/forward_converter_waveforms.png`

## สรุปสั้น ๆ

- **โทโพโลยี:** Isolated buck-derived (ส่งพลังงานตอน ON)
- **องค์ประกอบหลัก:** หม้อแปลง (มี reset winding), MOSFET, ไดโอดเรกติไฟเออร์ + ฟรีวีล, ตัวกรอง LC
- **Duty cycle สูงสุด:** มักจำกัดที่ \(D < 0.5\) เพื่อรีเซ็ตฟลักซ์ในหม้อแปลง
- **อัตราส่วนแรงดัน:** \(V_o = V_{in} \cdot (N_s/N_p) \cdot D\)
