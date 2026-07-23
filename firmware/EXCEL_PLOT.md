# พล็อตกราฟจาก Serial ลง Excel

รูปแบบ Serial:

```text
           Tim     Iin     Vin     Iout    Vout    Duty
          00:00:13   0.00    183.8   0.00   53.31    1
```

## ขั้นตอน

1. คัดลอกเฉพาะแถวเวลา (`00:…`) — ไม่ต้องเอา `[STAT]` / `[INFO]` / `[BOOT]`
2. วาง Excel → Text to Columns → **Space** (หรือ Fixed width)
3. ใส่หัวคอลัมน์: Tim Iin Vin Iout Vout Duty
4. พล็อตกราฟแยกต่อค่า
