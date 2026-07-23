# Changelog v64 — cv58-boost-v14-forward-v64

## Serial CSV สำหรับพล็อตกราฟใน Excel

ส่งค่าดีบักเป็นบรรทัด **TSV** (คั่นด้วย Tab) ขึ้นต้นด้วย `CSV` ทุก 1 วินาที

### คอลัมน์
`t_ms, t_s, run, Dpct, Draw, Vbat, Vf, Ibat, If, Vin, sel`

### วิธีใส่ Excel
1. เปิด Serial Monitor 115200 — คัดลอกเฉพาะบรรทัดที่ขึ้นต้นด้วย `CSV`
2. วางใน Excel (Notepad ก่อนก็ได้ แล้ว Save เป็น `.txt`)
3. Data → Text to Columns → Delimited → **Tab**
4. Insert → Chart: แกน X = `t_s`, แกน Y = `Vbat` / `Vf` / `Ibat` / `Dpct`

ตัวอย่าง:

```text
CSV	t_ms	t_s	run	Dpct	Draw	Vbat	Vf	Ibat	If	Vin	sel
CSV	12345	12.3	F_CC	36	368	53.97	53.97	0.50	0.50	147.7	FORW
```

`[STAT]` ยังมีตามเดิม (ช้ากว่า) สำหรับอ่านเร็ว ๆ

Flash: `[BOOT] cv58-boost-v14-forward-v64 | …`
