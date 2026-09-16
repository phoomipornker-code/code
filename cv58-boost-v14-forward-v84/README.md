# cv58-boost-v14-forward-v97

F-CV ต้องไต่ไป **58.4 V** — ไม่หนีบ Iref ที่ 0.35 A ตั้งแต่ 56 V (ฟิลด์ v95 ค้าง ~56 V แล้ว FULL HOLD จาก ADS 83 V)

Iref taper: **3.0 A @ 55.2 V → 0.35 A @ 58.2 V** (ที่ 56 V ≈ 2.3 A)

ADS >62 V ใช้เป็น CV sense เฉพาะเมื่อกระแสยุบจริง (Iabs < 0.15 A และ Ifilt < 0.20 A)

Forward FULL: V ≥ 58.20 V และ I ≤ **0.10 A**

Boost ไม่แตะ
