# cv58-boost-v14-forward-v100

ฟิลด์ v99: F-CC Iref=3 A แต่ Ibat=0 ตลอด, Vbat สลับ 91 / 1.7 / 53, duty ถูก AdsX ตัดแล้วไต่ใหม่ — ไม่ชาร์จ

**v100**
- SoftStart ต้องมีกระแสจริงถึงจะเข้า CC — หมดเวลาแล้วยัง I=0 จะ `[STOP] SoftStart no current`
- F-CC ที่ยังไม่เคยมีกระแส แล้วเจอ AdsX 91 V → `[STOP] AdsX no-current` (ไม่ไล่ 3 A ซ้ำ)
- ADS นอก 35–62 V ห้ามไต่ duty (รวม Vbat=1.7 V)

Boost ไม่แตะ
