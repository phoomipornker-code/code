# โน้ตส่วน STATE_FORWARD

แท็กเฟิร์มแวร์: `cv58-boost-v14-forward-v21`

## ค่าคงที่สำคัญ

```cpp
PWM_FREQ_FORWARD      = 67000;      // แยกจาก Boost 50 kHz
PWM_FORWARD_PIN       = 14;
MAX_DUTY_FORWARD      = 460;        // ~45% ของ 1023 (Nr=Np)
FWD_TARGET_CC_CURRENT = 5.0;        // A (Boost ใช้ TARGET_CC_CURRENT=6.0)
TARGET_CV_VOLTAGE     = 56.00;      // V
// v_ac_in = DC หลังไดโอดบริดจ์ จาก AC 110 V (~155 Vpeak)
MIN_AC_VOLTAGE        = 95.0;
BAT_PRESENT_MIN_V     = 40.0;
BAT_START_MAX_V       = 56.40;
RESTART_CHARGE_VOLTAGE = 54.0;
HIGH_VOLTAGE_STOP_VOLTAGE = 56.80;
```

Forward modes:

```cpp
enum ForwardMode { FWD_SOFTSTART, FWD_CC, FWD_CV, FWD_DONE };
```

## แกนควบคุม

```text
เข้า STATE_FORWARD (หลังเช็กแบต 40..56.4 V และ บริดจ์ DC ≥ 95 V)
  → duty=0, forwardNewResetOnEntry()
  → FWD_SOFTSTART : ramp duty → seed ~80
                    Ibat≥0.35A → CC
                    timeout + I≈0 → FAULT latch
  → FWD_CC        : current PI → Iref=5A (taper ใกล้ 56V)
                    เข้า CV เมื่อ Vbat≥55.5 (confirm) หรือ ≥55.7 (force)
  → FWD_CV        : voltage PI → Iref, current PI → duty
                    FULL เมื่อ V≥55.9 และ I≤0.5A นาน 60s
  → FWD_DONE / FULL HOLD → รีชาร์จเมื่อ Vbat≤54V
```

## Safety (Forward)

- BMS-open / spike preempt ใช้ร่วมกับ Boost ใกล้โซน 56 V
- Duty preempt cap ใกล้ `BMS_PREEMPT_ZONE_V`
- Primary `Iac` หรือ `Ibat` เกิน hard limit → latched shutdown
- OVP latch เคลียร์ด้วยปุ่ม **STOP** เมื่อแรงดันกลับสู่โซนปลด

Duty จำกัดที่ `MAX_DUTY_FORWARD` (460)  
PWM ออก GPIO 14 @ 67 kHz, Boost PWM = 0

## จุดวัด AC

`v_ac_in` = **แรงดัน DC ที่ขาออกไดโอดบริดจ์** จากไฟ AC 110 V  
ไม่ใช่ค่า VAC RMS โดยตรง — คาลิเบรต `CAL_SCALE_V_AC` ให้ตรงกับมัลติมิเตอร์ที่จุดเดียวกัน

หมายเหตุ: เอกสารออกแบบ 240 V + Ns/Np≈21/48 ที่ Dmax≈45% จะดันไม่ถึง 56 V จากบัส ~155 V  
ฮาร์ดแวร์ 110 V ต้องใช้เรโชหม้อแปลงที่เหมาะกับแรงดันนี้
