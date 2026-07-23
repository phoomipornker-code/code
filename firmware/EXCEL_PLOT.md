# พล็อตกราฟจาก Serial ลง Excel

```text
Tim           Iin        Vin     Iout    Vout    Duty
00:00:02   0.00    181.1   0.00   53.32    0
```

แสดงทันทีหลังบูตทุก ~1 วินาที — ไม่มี `[WARN]`/`[INFO]` แทรก (v73)

## ขั้นตอน

1. คัดลอกแถวเวลา (`00:…`) — ไม่เอา `[STAT]` / `[INFO]` / `[BOOT]`
2. Excel → Text to Columns → **Space**
3. พล็อตกราฟแยกต่อคอลัมน์
