# พล็อตกราฟจาก Serial ลง Excel

```text
Tim           Iin        Vin     Iout    Vout    Duty
00:00:02   0.00    181.1   0.00   53.32    0
```

`Duty` = **raw PWM** (0–460 Forward / 0–1023 Boost scale) — ไม่ใช่ %

ตารางพิมพ์โดย **TaskSerialLog** (แยกจากลูปควบคุม) — เปิด Monitor ได้โดยไม่ตัดชาร์จ (v77)

แสดงทันทีหลังบูตทุก ~1 วินาที — ไม่มี `[WARN]`/`[INFO]` แทรก (v73)

เหตุการณ์สำคัญยังขึ้นแยกบรรทัด (v75): `[START]` `[STOP]` `[FULL]` `[OVP]` `[OC]` `[MODE]` — อย่าคัดลอกไปพล็อต

## ขั้นตอน

1. คัดลอกแถวเวลา (`00:…`) — ไม่เอา `[STAT]` / `[INFO]` / `[BOOT]`
2. Excel → Text to Columns → **Space**
3. พล็อตกราฟแยกต่อคอลัมน์
