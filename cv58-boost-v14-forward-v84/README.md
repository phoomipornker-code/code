# cv58-boost-v14-forward-v91

Duty-up ของ Forward ไม่สอดคล้องกับ ADS1115:

- ADS 860 SPS × 6 ch ≈ 8–12 ms แล้วหน่วง 20 ms ⇒ ลูปจริง ~30–40 ms
- กรอง Ibat 8 จุด ≈ 0.25–0.3 s
- PI เดิม slew +8 / รอบ ⇒ 0→45% ใน ~2 s และทับ SoftStart

ปรับแล้ว (Kp/Ki ตาม Simulink คงเดิม):

- slew-up 1.2 raw / รอบ (รอรอบ ADS + ฟิลเตอร์)
- SoftStart แค่เปิด seed — ไม่ให้ current PI ทับ
- `dt` จากเวลาลูปจริง ไม่ใช้ 20 ms ปลอม
- ไม่เพิ่มดิวตี้ถ้า ADS อ่านไม่สำเร็จ

เซ็นเซอร์ `CAL_SCALE_V_BAT = 41.5` คงเดิม
