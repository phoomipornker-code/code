# Changelog v19 — cv58-boost-v14-forward-v19

## Forward AC sense = diode-bridge DC @ 110 VAC

- ผู้ใช้วัด `v_ac_in` ที่ **ขาออกไดโอดบริดจ์** (แรงดัน DC หลังเรกติฟาย) จาก **AC 110 V**
- คาดหวังบัสประมาณ **~155 Vpeak** (ไม่มีโหลด) / ต่ำลงเมื่อมีโหลด
- `MIN_AC_VOLTAGE`: **200 → 120 V** (เกณฑ์บัส DC ไม่ใช่ VAC RMS)

## หมายเหตุฮาร์ดแวร์

เอกสารออกแบบเดิมเป็น AC 240 V + `Ns/Np≈21/48` + `Dmax≈45%` — ที่บัส ~155 V จะดันไม่ถึง 56 V  
ถ้าใช้งานจริงที่ 110 V ต้องใช้เรโชหม้อแปลงที่เหมาะกับ 110 V (หรือยืนยันเรโชที่ใช้จริง) แยกจากเกณฑ์ซอฟต์แวร์นี้

## Unchanged

- Boost = proven v14
- Mode select ด้วย STOP ก่อน START (v18)
- Forward SoftStart→CC→CV + safety latch
