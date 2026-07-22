# โน้ตส่วน STATE_FORWARD

แท็กเฟิร์มแวร์: `cv58-boost-v14-forward-v41`

## ค่าคงที่สำคัญ

```cpp
PWM_FREQ_FORWARD      = 67000;
MAX_DUTY_FORWARD      = 460;        // ~45% — HW ~5 A ที่ D นี้
FWD_DESIGN_I_AT_D45   = 5.0;
FWD_TARGET_CC_CURRENT = 3.0;        // A setpoint
TARGET_CV_VOLTAGE     = 56.00;
MIN_AC_VOLTAGE        = 95.0;
// ไม่ใช้ PID — step/hysteresis
FWD_STEP_UP_CC        = 1.5;        // raw/tick
FWD_STEP_DOWN_CC      = 3.0;
FWD_CC_HOLD_BAND_A    = 0.15;       // hold เมื่อ |I−Iref| ในแบนด์
FWD_AC_HOLD_CLIMB_V   = 115.0;      // freeze เพิ่ม duty ถ้าบัสดิป
```

Forward modes:

```cpp
enum ForwardMode { FWD_SOFTSTART, FWD_CC, FWD_CV, FWD_DONE };
```

## แกนควบคุม (ไม่มี PID)

```text
เข้า STATE_FORWARD
  → FWD_SOFTSTART : เพิ่ม duty เป็นขั้นเล็กๆ ไปหา seed (~45% ของ duty เป้า CC)
  → FWD_CC        : I < Iref−band → +duty จนถึงแบนด์หรือ Dmax (ไม่มีเพดาน FF)
                    เข้า CV เมื่อ Vbat≥**55.10** (confirm) / force **55.35**
  → FWD_CV        : **รักษาระดับแรงดันคงที่ ~56 V**
                    V ต่ำ → +duty; ในแบนด์ → hold; V สูง → −duty (กระแสถดเองเมื่อแบตเต็ม)
                    FULL เมื่อ V≥55.9 และ I≤0.5A นาน 60s
  → FWD_DONE
```

## Safety (Forward)

- AC sag/blip: **freeze duty-up เท่านั้น** (ไม่ dump duty); AC=0 ขณะ Ibat ยังไหล = glitch (ไม่เข้า sag); shutdown ถ้าหายจริง ≥ 15 s
- Duty open ช้า (Cin); freeze climb ถ้า AC&lt;115 V
- ADC mutex / bus-glitch / ACblip hold
- Soft over-current / OVP ตามเดิม

Duty จำกัดที่ `MAX_DUTY_FORWARD` (460)  
PWM GPIO 14 @ 67 kHz

## จุดวัด AC

`v_ac_in` = DC ที่ขาออกไดโอดบริดจ์ จาก AC 110 V
