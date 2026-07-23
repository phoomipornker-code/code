# Changelog v62 — cv58-boost-v14-forward-v62

## Field: ไม่ตัดตอนชาร์จเต็ม (ค้าง FWD_CV)

```text
[FWD ] phase=CV hold Vcv:55.90V Vf:55.21V Ibat:2.12A duty_acc:137.8
[BAT] V: 56.38V  I: 0.00A  Vf: 55.21V  If: 2.12A  Iabs: 0.00A
```

**Cause:** ใกล้เต็มแล้ว V raw ≈56.4 V แต่ `Vf` ค้าง ~55.2 (หลัง BATspike) และ `If` ค้าง ~2 A ทั้งที่ `Iabs≈0`  
เงื่อนไขเก่าต้องการ `Vf≥55.8` และ `If≤0.5` นาน **60 s** → ไม่เคยครบ → ไม่เข้า FULL

### v62 fixes
- Heal `Vf` เมื่อ V raw สูงกว่า filt แบบสมเหตุสมผลติดกัน
- Heal `If` เมื่อ I raw ≈0 แต่ filt ยังสูง
- Forward FULL ใช้ **max(V,Vf)** และ **min(If,Iabs)**
- Confirm ปกติ **15 s**; fast path เมื่อ V≥55.9 และ Iabs≤0.35 A → **5 s**
- High-V stop ใช้ peak V เช่นกัน

หมายเหตุ: ล็อกที่มี `[DEBUG]` + RAW เป็นเฟิร์มแวร์เก่า — แฟลช v62 แล้วจะเห็น `[STAT]` / `[INFO] Battery FULL`

Flash until boot shows:

`[BOOT] cv58-boost-v14-forward-v62 | …`
