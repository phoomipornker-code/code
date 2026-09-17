# cv58-boost-v14-forward-v98

SoftStart ที่ ~53 V / I=0 ห้ามตัด FULL HOLD จาก ADS ~90 V
(ฟิลด์ v97: `[STOP] CV ceiling FULL HOLD V=90.70 filt=53.25` แล้ว auto-restart เพราะ filt < 54 V → จอ LCD ติดๆ ดับๆ)

- ไม่ใช้ ADS >62 V ตอน SoftStart
- ใช้ ADS >62 V เมื่อ pack filt ≥ 55.2 V และเคยมีกระแสชาร์จ แล้วกระแสยุบจริง
- ไม่ auto-restart FULL HOLD ถ้าตอนตัด pack ยังไม่ใกล้เต็ม (กันลูป 53 V)

Iref taper: 3.0 A @ 55.2 V → 0.35 A @ 58.2 V

Forward FULL: V ≥ 58.20 V และ I ≤ 0.10 A

Boost ไม่แตะ
