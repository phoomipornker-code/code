# Battery Charger

ตัวอย่างโค้ดเครื่องชาร์จแบตเตอรี่แบบ constant-current/constant-voltage
(CC/CV) มีทั้ง ESP32 Arduino sketch สำหรับควบคุมวงจรชาร์จ PV/AC และ Python
simulator สำหรับจำลองการชาร์จจาก command line

> คำเตือน: อย่าต่อ Arduino เข้ากับไฟบ้านหรือแบตเตอรี่โดยตรง โค้ดนี้ต้องใช้ร่วมกับ
> power stage, MOSFET/driver, fuse, sensor และระบบป้องกันที่เหมาะกับชนิดแบตเตอรี่
> ของคุณ ตรวจสอบวงจรจริงด้วยเครื่องมือวัดก่อนใช้งานกับแบตเตอรี่จริง

## Arduino usage

เปิดไฟล์นี้ด้วย Arduino IDE หรือ PlatformIO:

```text
arduino/charger_controller/charger_controller.ino
```

sketch นี้ออกแบบสำหรับ ESP32 และใช้:

- `Adafruit_ADS1X15` สำหรับ ADS1115 จำนวน 2 ตัว
- `LiquidCrystal_I2C` สำหรับ LCD 20x4
- Arduino-ESP32 core ที่รองรับ `ledcAttach(pin, freq, resolution)`
- I2C pins: SDA `GPIO21`, SCL `GPIO22`

ค่าเริ่มต้นใน sketch:

- battery CV target: `58.0 V`
- battery CC target: `5.0 A`
- PV start threshold: `41.0 V`
- PV critical undervoltage cutoff: `38.0 V`
- AC input threshold: `140.0 V`
- PWM frequency: `50 kHz`, resolution: `10-bit`

ปรับค่าด้านบนของไฟล์ `.ino` ให้ตรงกับวงจรจริงก่อน upload โดยเฉพาะ:

- `TARGET_CV_VOLTAGE`, `TARGET_CC_CURRENT`
- `MIN_PV_VOLTAGE`, `UNDER_PV_VOLTAGE_CRIT`, `MIN_AC_VOLTAGE`
- `MAX_DUTY_FORWARD`, `MAX_DUTY_BOOST`
- ค่า PID `Kp`, `Ki`, `Kd`, `Kp_cc`, `Ki_cc`, `Kd_cc`, `Kp_cv`, `Ki_cv`, `Kd_cv`
- ค่า calibration `OFFSET_*` และ `CAL_SCALE_*`

### Arduino pin map

| ESP32 pin | ใช้สำหรับ |
| --- | --- |
| GPIO32 | relay PV |
| GPIO33 | relay AC |
| GPIO25 | ปุ่ม start, `INPUT_PULLUP` |
| GPIO26 | ปุ่ม stop, `INPUT_PULLUP` |
| GPIO14 | PWM forward/AC stage |
| GPIO27 | PWM boost/PV stage |
| GPIO21 | I2C SDA |
| GPIO22 | I2C SCL |

### ADS1115 channel map

| Device | Address | Channel | ใช้สำหรับ |
| --- | --- | --- | --- |
| `ads_volt` | `0x48` | A0 | PV voltage |
| `ads_volt` | `0x48` | A2 | AC voltage |
| `ads_volt` | `0x48` | A1 | Battery voltage |
| `ads_curr` | `0x49` | A0 | PV current |
| `ads_curr` | `0x49` | A1 | AC current |
| `ads_curr` | `0x49` | A2 | Battery current |

เปิด Serial Monitor ที่ `115200 baud` เพื่อดูสถานะ:

```text
[DEBUG INTERFACE] System: ON  | State: BOOST | Active Duty: 8%
```

## Python simulator

### Usage

```bash
python3 -m charger --capacity 5000 --soc 40 --voltage 3.7 --temperature 25 --minutes 30
```

ตัวอย่างผลลัพธ์:

```text
status: charging
current_ma: 2000.00
delivered_mah: 920.00
state_of_charge: 58.40%
voltage: 3.701V
message: Charging at bulk current.
```

### Run tests

```bash
python3 -m unittest
```
