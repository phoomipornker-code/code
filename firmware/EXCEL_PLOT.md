# พล็อตกราฟจาก Serial (v79 = โค้ด v63)

ควบคุมเหมือน `cv58-boost-v14-forward-v63`  
Serial ใช้บรรทัด `[STAT]` ทุก **1 นาที** (v79)

ตัวอย่าง:

```text
[BOOT] cv58-boost-v14-forward-v79 | B_CC=6A F_CC=3A CV=55.90V DmaxF=460
[STAT] … START …
[STAT] … FWD Soft … D=…% BAT … I=…A …
```

## ขั้นตอนคร่าว ๆ

1. คัดลอกค่าจาก `[STAT]` ที่สนใจ (V / I / Duty)
2. วางใน Excel แล้วแยกคอลัมน์
3. พล็อตกราฟ

อย่าคัดลอก `[WARN]` / `[INFO]` / `[CRITICAL]` / `[BOOT]` ปนกับตัวเลข
