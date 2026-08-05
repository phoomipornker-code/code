# Changelog v60 — cv58-boost-v14-forward-v60

## LCD: ปิดจอตอนชาร์จ จนกว่าจะเต็ม

User request: เมื่อกดเริ่มชาร์จ ให้ตัดการแสดงผลจอจนกว่าแบตจะเต็ม

### Behavior
- กด **START** → เคลียร์จอ + ปิด backlight ทันที แล้ว**ไม่**อัปเดต LCD ระหว่างชาร์จ (ลด EMI/I2C)
- เมื่อเข้า **FULL HOLD** → เปิดจอกลับ แสดงรูปแบต + SOC + V/I
- กด **STOP** ระหว่างชาร์จ → กลับ STANDBY ตามปกติ
- **OVP / ERROR** ยังแสดงได้ทันที

สถานะระหว่างชาร์จดูจาก Serial `[STAT]` ได้ตามเดิม

Flash until boot shows:

`[BOOT] cv58-boost-v14-forward-v60 | …`
