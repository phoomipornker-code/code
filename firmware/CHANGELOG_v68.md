# Changelog v68 — cv58-boost-v14-forward-v68

## แก้ CSV กลับเป็นค่าจริง + เวลา

เลิกเลื่อนแกนปลอม (Iplot/Dplot/Vinplot) — ผิดสำหรับอ่านค่า/พล็อตแยก

```text
CSV	time	Vin	Vout	Iout	Duty
CSV	00:12:34	147.7	53.97	0.50	36
```

- `time` = HH:MM:SS นับจากบูต
- Vin / Vout / Iout / Duty = **ค่าจริง**
- `[STAT]` ก็มีเวลาเช่นกัน
- STANDBY ยังขึ้นเฉพาะตอนสลับโหมด

ใน Excel: พล็อตแยกกราฟต่อตัวแปร (หรือใช้แกนรอง) — ไม่ต้องชดเชย offset

Flash: `[BOOT] cv58-boost-v14-forward-v68 | …`
