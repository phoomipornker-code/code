# Changelog v70 — cv58-boost-v14-forward-v70

## Serial ตรงตัวอย่างที่ต้องการ

```text
[BOOT] cv58-boost-v14-forward-v70 | B_CC=6A F_CC=3A CV=55.90V DmaxF=460
           Tim     Iin     Vin     Iout    Vout    Duty
[STAT] 00:00:07 STANDBY FORW BAT 53.32V PV 0.0 AC 181.1
          00:00:12   0.00    183.3   0.00   53.30    0
[STAT] 00:00:12 START D=0% BAT 53.31V/53.30V I=0.00A IN=183.3V
[INFO] Enter FORWARD SoftStart->CC->CV control.
          00:00:13   0.00    183.8   0.00   53.31    1
```

### Changes
- ไม่มีคำนำหน้า `CSV`
- หัวตาราง + แถวข้อมูลจัดคอลัมน์ด้วยช่องว่าง
- `[STAT]` เฉพาะสลับโหมด / กด START — ไม่สแปมระหว่างชาร์จ

Flash: `[BOOT] cv58-boost-v14-forward-v70 | …`
