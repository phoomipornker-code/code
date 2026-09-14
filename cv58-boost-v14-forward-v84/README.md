# cv58-boost-v14-forward-v86

Forward ใช้ **cascade PI ตามไดอะแกรม Simulink** — Boost ไม่ได้แก้

```text
58.4 − V_OUT → PI(Kp=15, Ki=1) → sat Iref [CV]
Iref − I_OUT → PI(Kp=0.5, Ki=23) + duty[CC] → sat Duty → PWM
```

เมื่อ V ต่ำ Iref ชนเพดาน 3 A = โหมด CC  
เมื่อ V ใกล้ 58.4 V Iref ลดลงเอง = โหมด CV  
เพดาน PWM-off ที่ **target + 0.45 V** (Forward ≈ 58.85 V) กันพุ่งถึง 60 V

## แฟลช

Arduino IDE → เปิด `cv58-boost-v14-forward-v84.ino`  
Serial 115200 ต้องเห็น:

```text
[BOOT] cv58-boost-v14-forward-v86 | B_CC=6A F_CC=3A CV=58.40V DmaxF=460
[BOOT] FWD cascade PI V(Kp=15 Ki=1) I(Kp=0.50 Ki=23) Ts=20ms
```
