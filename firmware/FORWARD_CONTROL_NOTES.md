# โน้ตส่วน STATE_FORWARD จากโค้ดหลัก

แท็กเฟิร์มแวร์: `cv58-stability-v14-cv-stable`

## ค่าคงที่สำคัญ

```cpp
PWM_FREQ              = 67000;      // 67 kHz  (Forward)
PWM_FORWARD_PIN       = 14;
MAX_DUTY_FORWARD      = 460;        // ~45% ของ 1023 (Nr=Np)
TARGET_CC_CURRENT     = 6.0;        // A
TARGET_CV_VOLTAGE     = 56.00;      // V
CV_DEADBAND_V         = 0.12;
RESTART_CHARGE_VOLTAGE = 54.0;
HIGH_VOLTAGE_STOP_VOLTAGE = 56.80;
HIGH_VOLTAGE_STOP_CONFIRM_MS = 300;
MIN_AC_VOLTAGE        = 140.0;
```

PID Forward:

```cpp
Kp_cc=0.15, Ki_cc=0.01, Kd_cc=0.005;
Kp_cv=0.5,  Ki_cv=0.015, Kd_cv=0.005;
```

## แกนควบคุม (ตรงโค้ด)

```cpp
// เมื่อ currentState == STATE_FORWARD
pid_error_cc = TARGET_CC_CURRENT - i_bat_charge_filt;
// ... PID CC → pid_out_cc

pid_error_cv = TARGET_CV_VOLTAGE - v_bat_filt;
if (fabs(pid_error_cv) <= CV_DEADBAND_V) { pid_error_cv = 0; pid_integral_cv *= 0.90; }
// ... PID CV → pid_out_cv

float final_battery_pid = min(pid_out_cc, pid_out_cv);
final_battery_pid = constrain-ish to [-4.0, +1.5];
duty_accumulator += final_battery_pid;
duty_accumulator = constrain(duty_accumulator, 0, MAX_DUTY_FORWARD);

raw_duty = quantizeDutyWithDither(duty_accumulator, &forward_dither_phase, MAX_DUTY_FORWARD);
ledcWrite(PWM_FORWARD_PIN, raw_duty);
ledcWrite(PWM_BOOST_PIN, 0);
```

## แนะนำจูนกับฮาร์ดแวร์ Forward + Nr

```cpp
// แนะนำเปลี่ยนเพื่อรีเซ็ตฟลักซ์ทัน (Nr = Np)
const int PWM_FREQ = 67000;
const int MAX_DUTY_FORWARD = 460;  // ~45%
```

วางไฟล์สเก็ตช์เต็มชื่อ `charger_main.ino` ในโฟลเดอร์นี้ (Arduino IDE: เปิดโฟลเดอร์ `firmware` เป็น sketch)

โฟลว์ชาร์ตหมายเลข + อธิบายไทย: [`../docs/flowchart-forward-firmware.md`](../docs/flowchart-forward-firmware.md)
