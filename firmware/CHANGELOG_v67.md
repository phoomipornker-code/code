# Changelog v67 — cv58-boost-v14-forward-v67

## ไม่สแปม `[STAT] STANDBY`

User: ไม่ต้องแสดง `[STAT] STANDBY …` หากไม่มีการเปลี่ยนโหมด

### Behavior
- Idle STANDBY: **ไม่** พิมพ์ `[STAT]` ซ้ำ
- กด STOP สลับ BOOST↔FORWARD: พิมพ์ `[STAT] STANDBY` ครั้งเดียว (+ `[INFO] Mode select` ตามเดิม)
- ตอนชาร์จ / FULL / OVP: `[STAT]` ตามรอบเดิม
- CSV Excel: ส่งเฉพาะตอนชาร์จ/FULL/OVP (ไม่ส่งตอน idle)

Flash: `[BOOT] cv58-boost-v14-forward-v67 | …`
