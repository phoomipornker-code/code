# cv58-boost-v14-forward-v85

ควบคุมชุดเดียวกับ v81 — เป้า **CV 58.00 V** และเพดานกันพุ่งถึง 60 V

## แฟลช

Arduino IDE → เปิด `cv58-boost-v14-forward-v84.ino`  
Serial 115200 ต้องเห็น:

```text
[BOOT] cv58-boost-v14-forward-v85 | B_CC=6A F_CC=3A CV=58.00V DmaxF=460
```

## v85 — กัน overshoot ไป 60 V

v84 ขยับเป้าไป 58 V แต่เพดานช้าเกินไป:

- high-V stop รอ 300 ms ที่ 58.4 V → ที่ ~5 V/s พุ่งถึง ~60 V
- HARD OVP 59.5 V ต้องรอ filt ตามไม่ทัน
- เกต +2 V บัง 58→60 แล้ว PWM ยังเปิด

v85:

| ชั้น | ค่า | ทำอะไร |
|------|-----|--------|
| โฮลด์ CV | 58.00 | เป้าเดิม |
| ห้าม duty-up (peak) | 57.90 | ไม่ปีนต่อเมื่อใกล้ 58 |
| ทุบ duty | 58.10 | bleed ทันที |
| PWM off / FULL HOLD | **58.35** | ตัดรอบนี้ ไม่รอ confirm |
| high-V stop | 58.20 / 80 ms | เดิม 58.40 / 300 ms |
| HARD OVP | 58.80 | เดิม 59.50 |

ลูป SoftStart→CC→CV / PWM / ADC ไม่ได้รื้อใหม่
