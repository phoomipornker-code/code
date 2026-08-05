# Changelog v18 — cv58-boost-v14-forward-v18

## UI: manual mode select before START

- ไม่เลือก BOOST/FORWARD อัตโนมัติจาก PV/AC อีกต่อไป
- ตอน **STANDBY**: กด **STOP** สลับ `BOOST PV` ↔ `FORWARD AC`
- กด **START** เริ่มเฉพาะโหมดที่เลือก (ต้องมีอินพุตของโหมดนั้น + แบตอยู่ในหน้าต่างสตาร์ท)
- LCD แสดงโหมดที่เลือก และข้อความ `STOP=mode START=go`

## Unchanged

- Boost control = proven v14 (50 kHz / 6 A)
- Forward SoftStart→CC→CV + v17 safety
