# พล็อตกราฟจาก Serial ลง Excel

บรรทัด **CSV** ทุก ~1 วินาที (คั่นด้วย Tab):

```text
CSV	t_s	Vin	Vout	Iout	Duty
CSV	12.3	147.7	53.97	0.50	36
```

| คอลัมน์ | ความหมาย |
|---------|----------|
| t_s | เวลา (วินาที) — แกน X |
| Vin | แรงดันเข้า (PV / AC) |
| Vout | แรงดันออกแบต |
| Iout | กระแสออกแบต |
| Duty | Duty % |

## ขั้นตอน

1. Serial Monitor **115200** — คัดลอกเฉพาะบรรทัดขึ้นต้น `CSV`
2. วาง Excel → Data → Text to Columns → **Tab**
3. Chart: X = `t_s`, Y = `Vin` / `Vout` / `Iout` / `Duty`
