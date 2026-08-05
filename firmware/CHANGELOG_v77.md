# Changelog v77 — cv58-boost-v14-forward-v77

## ชาร์จไม่ได้ + แสดงค่าโดยไม่รบกวนควบคุม

### สาเหตุที่ชาร์จตัด
`Serial.printf` อยู่ใน task เดียวกับ ADC/PWM — เมื่อเปิด Serial Monitor แล้ว USB บล็อก  
ลูปควบคุมช้า → ADC stale → `[STOP] ADC sample timeout` → ตัดชาร์จ

### แก้
- **TaskSerialLog** (priority ต่ำ, core 1): พิมพ์ตารางทุก ~1 s + ดึงคิวเหตุการณ์
- **ADC_PWM_Task** ไม่เรียก `Serial.*` แล้ว — แค่ `logEventf()` ใส่คิว (ไม่บล็อก)
- LCD ตอนชาร์จยังปิดตามเดิม (กัน EMI) — ดูค่าจาก Serial

ยังแสดง:
```text
[START] FORWARD SoftStart->CC->CV
00:00:13     0.50     180.1    0.80   53.40   150
[STOP] hold 350ms end (FORWARD)
[OVP] …
[OC] …
```

Flash: `[BOOT] cv58-boost-v14-forward-v77 | …`
