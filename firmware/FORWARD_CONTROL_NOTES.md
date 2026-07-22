# โน้ตส่วน STATE_FORWARD

แท็กเฟิร์มแวร์: `cv58-boost-v14-forward-v33`

## ค่าคงที่สำคัญ

```cpp
PWM_FREQ_FORWARD      = 67000;      // แยกจาก Boost 50 kHz
PWM_FORWARD_PIN       = 14;
MAX_DUTY_FORWARD      = 460;        // ~45% ของ 1023 (Nr=Np) — HW ~5 A ที่ D นี้
FWD_DESIGN_I_AT_D45   = 5.0;        // ความสามารถฮาร์ดแวร์สำหรับ feedforward
FWD_DESIGN_DUTY_FRAC  = 0.45;       // 5 A @ D=45%
FWD_TARGET_CC_CURRENT = 3.0;        // A setpoint (Boost ใช้ TARGET_CC_CURRENT=6.0)
TARGET_CV_VOLTAGE     = 56.00;      // V
// v_ac_in = DC หลังไดโอดบริดจ์ จาก AC 110 V (~155 Vpeak)
MIN_AC_VOLTAGE        = 95.0;
// PI / CV / slew = ชุดเดียวกับ Boost ที่พิสูจน์แล้ว
FWD_CURR_KP/KI        = 14.0 / 55.0;
FWD_VOLT_KP/KI        = 0.85 / 0.45;
FWD_SOFTSTART_MS      = 2500;
```

Forward modes:

```cpp
enum ForwardMode { FWD_SOFTSTART, FWD_CC, FWD_CV, FWD_DONE };
```

## แกนควบคุม (คล้าย Boost)

```text
เข้า STATE_FORWARD (หลังเช็กแบต 40..56.4 V และ บริดจ์ DC ≥ 95 V)
  → duty=0, forwardNewResetOnEntry()
  → FWD_SOFTSTART : ramp ไปใกล้ duty สำหรับ CC 3 A (~27% จาก HW 5A@45%)
                    พร้อมเมื่อใกล้ seed และ Ibat≥0.4A หรือครบ 2.5s  → CC
  → FWD_CC        : current PI (14/55) + feedforward จาก 5A↔D45% จำกัดที่ 3 A
                    เข้า CV เมื่อ Vbat≥55.5 (confirm) หรือ ≥55.7 (force)
  → FWD_CV        : voltage PI → Iref → current PI → duty (สูตรเดียวกับ Boost)
                    FULL เมื่อ V≥55.9 และ I≤0.5A นาน 60s
  → FWD_DONE / FULL HOLD → รีชาร์จเมื่อ Vbat≤54V
```

ไม่มี MPPT (ต่างจาก Boost ที่มี CC_MPPT)

## Safety (Forward)

- AC sag ชั่วคราว: **ไม่พัก PWM** — แค่ **freeze duty-up** (ห้ามเพิ่ม duty); ลดได้ตาม safety; shutdown ถ้าหาย ≥ **15 s**
- ADC: ปล่อย I2C mutex ทันทีหลังอ่าน ADS; hold AC เฉพาะ multi-ch bus glitch (ไม่บัง sag จริง); stale trip 2.5 s
- ถ้า AC+BAT ยุบพร้อมกัน ~ค่าเดียวกัน = bus glitch — hold ค่าเดิม
- BMS-open / spike preempt ใช้ร่วมกับ Boost
- Duty preempt cap ใกล้ `BMS_PREEMPT_ZONE_V`
- Over-current: **soft cut duty** แบบ Boost (ไม่ latch จากกระแส); hard soft-cut ~3.75 A
- Hard OVP latch ยังมี — เคลียร์ด้วย **STOP** เมื่อแรงดันลด

Duty จำกัดที่ `MAX_DUTY_FORWARD` (460)  
PWM GPIO 14 @ 67 kHz

## จุดวัด AC

`v_ac_in` = DC ที่ขาออกไดโอดบริดจ์ จาก AC 110 V
