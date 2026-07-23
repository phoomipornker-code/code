# พล็อตกราฟจาก Serial (v78 = โค้ด v63)

v78 คืนเฟิร์มแวร์ที่ชาร์จได้จาก `cv58-boost-v14-forward-v63`  
Serial ใช้บรรทัด `[STAT]` (ทุก ~3 s ตอนชาร์จ / ~5 s ตอน STANDBY) — **ไม่มี**ตาราง Tim/Iin แบบ v70+

ตัวอย่าง:

```text
[BOOT] cv58-boost-v14-forward-v78 | B_CC=6A F_CC=3A CV=55.90V DmaxF=460
[STAT] … START …
[STAT] … FWD Soft … D=…% BAT … I=…A …
```

## ขั้นตอนคร่าว ๆ

1. คัดลอกค่าจาก `[STAT]` ที่สนใจ (V / I / Duty)
2. วางใน Excel แล้วแยกคอลัมน์
3. พล็อตกราฟ

อย่าคัดลอก `[WARN]` / `[INFO]` / `[CRITICAL]` / `[BOOT]` ปนกับตัวเลข
