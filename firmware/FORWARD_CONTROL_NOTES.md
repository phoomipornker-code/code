# โน้ตส่วน STATE_FORWARD จากโค้ดหลัก

แท็กเฟิร์มแวร์: `cv58-forward-67khz-5a-v16`

## ค่าคงที่สำคัญ

```cpp
PWM_FREQ              = 67000;      // 67 kHz  (Forward)
PWM_FORWARD_PIN       = 14;
MAX_DUTY_FORWARD      = 460;        // ~45% ของ 1023 (Nr=Np)
TARGET_CC_CURRENT     = 5.0;        // A
TARGET_CV_VOLTAGE     = 56.00;      // V
CV_DEADBAND_V         = 0.12;
RESTART_CHARGE_VOLTAGE = 54.0;
HIGH_VOLTAGE_STOP_VOLTAGE = 56.80;
HIGH_VOLTAGE_STOP_CONFIRM_MS = 300;
MIN_AC_VOLTAGE        = 140.0;
```

Forward modes:

```cpp
enum ForwardMode { FWD_SOFTSTART, FWD_CC, FWD_CV, FWD_DONE };
```

## แกนควบคุม (ตรงโค้ด)

```text
เข้า STATE_FORWARD
  → duty=0, forwardNewResetOnEntry()
  → FWD_SOFTSTART : ramp duty ไป seed (~80), พร้อมเมื่อ Ibat≥0.35A หรือครบ 2s
  → FWD_CC        : current PI → Iref=5A (taper ใกล้ 56V)
                    เข้า CV เมื่อ Vbat≥55.5 (confirm) หรือ ≥55.7 (force)
  → FWD_CV        : voltage PI → Iref, current PI → duty
                    deadband แช่ duty; over-V ลด duty เบาๆ
                    FULL เมื่อ V≥55.9 และ I≤0.5A นาน 60s
  → FWD_DONE / FULL HOLD → รีชาร์จเมื่อ Vbat≤54V
```

Duty จำกัดเสมอที่ `MAX_DUTY_FORWARD` (460) พร้อม slew up/down  
PWM ออก GPIO 14 @ 67 kHz, Boost PWM = 0

## จูนกับฮาร์ดแวร์ Forward + Nr

```cpp
const int PWM_FREQ = 67000;
const int MAX_DUTY_FORWARD = 460;  // ~45%
```

วางไฟล์สเก็ตช์เต็มชื่อ `charger_main.ino` ในโฟลเดอร์นี้ (Arduino IDE: เปิดโฟลเดอร์ `firmware` เป็น sketch)

โฟลว์ชาร์ต: [`../docs/flowchart-forward-only.md`](../docs/flowchart-forward-only.md)
