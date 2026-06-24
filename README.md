# Battery Charger

ตัวอย่างโค้ดเครื่องชาร์จแบตเตอรี่แบบ constant-current/constant-voltage
(CC/CV) เขียนด้วย Python โดยมี safety checks สำหรับอุณหภูมิ และมี CLI สำหรับ
จำลองการชาร์จจาก command line

## Usage

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

## Run tests

```bash
python3 -m unittest
```
