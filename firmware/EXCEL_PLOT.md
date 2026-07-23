# พล็อตกราฟจาก Serial ลง Excel

จัดวางตามตัวอย่าง:

```text
Tim      Iin     Vin      Iout     Vout     Duty
00:12:34 0.15    147.7    0.50     53.97    36
```

Serial จะขึ้นต้นด้วย `CSV` แล้วคั่นด้วย Tab:

```text
CSV	Tim	Iin	Vin	Iout	Vout	Duty
CSV	00:12:34	0.15	147.7	0.50	53.97	36
```

## ขั้นตอน

1. คัดลอกบรรทัดขึ้นต้น `CSV`
2. Excel → Text to Columns → **Tab**
3. พล็อตกราฟแยกต่อคอลัมน์: X = `Tim`, Y = `Iin` / `Vin` / `Iout` / `Vout` / `Duty`
