# Battery Charger

ตัวอย่างโค้ดเครื่องชาร์จแบตเตอรี่แบบ constant-current/constant-voltage
(CC/CV) มีทั้ง Arduino sketch สำหรับควบคุมวงจรชาร์จ DC ต่ำ และ Python simulator
สำหรับจำลองการชาร์จจาก command line

> คำเตือน: อย่าต่อ Arduino เข้ากับไฟบ้านหรือแบตเตอรี่โดยตรง โค้ดนี้ต้องใช้ร่วมกับ
> power stage, MOSFET/driver, fuse, sensor และระบบป้องกันที่เหมาะกับชนิดแบตเตอรี่
> ของคุณ ตรวจสอบวงจรจริงด้วยเครื่องมือวัดก่อนใช้งานกับแบตเตอรี่จริง

## Arduino usage

เปิดไฟล์นี้ด้วย Arduino IDE:

```text
arduino/charger_controller/charger_controller.ino
```

ค่าเริ่มต้นเหมาะกับแบตเตอรี่ Li-ion 1 cell:

- bulk current: `1000 mA`
- target voltage: `4.20 V`
- max voltage cutoff: `4.25 V`
- temperature range: `0 C` ถึง `45 C`

ปรับค่าด้านบนของไฟล์ `.ino` ให้ตรงกับวงจรและแบตเตอรี่ของคุณก่อน upload
โดยเฉพาะ:

- `TARGET_VOLTAGE`, `MAX_VOLTAGE`, `BULK_CURRENT_MA`, `TAPER_CURRENT_MA`
- `VOLTAGE_DIVIDER_R1`, `VOLTAGE_DIVIDER_R2`
- `CURRENT_SENSOR_ZERO_V`, `CURRENT_SENSOR_MV_PER_AMP`
- ค่า thermistor เช่น `THERMISTOR_BETA`

### Arduino pin map

| Arduino pin | ใช้สำหรับ |
| --- | --- |
| D9 | PWM ไปยัง MOSFET/charger driver |
| D8 | enable ขา driver หรือ relay |
| D13 | fault LED |
| A0 | อ่านแรงดันแบตเตอรี่ผ่าน voltage divider |
| A1 | อ่านกระแสจาก current sensor |
| A2 | อ่านอุณหภูมิจาก 10k NTC thermistor |

เปิด Serial Monitor ที่ `9600 baud` เพื่อดูสถานะ:

```text
state=bulk voltage=3.912V current=984.2mA temperature=29.5C pwm=87
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
