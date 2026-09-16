# cv58-boost-v14-forward-v92

Serial โชว์ค่าเดียวกับที่ ESP ใช้คุม — ทุก 200 ms ทั้ง STANDBY และตอนชาร์จ

คอลัมน์:
`Tim Ph Sel PVv PVi ACv ACi Vbat Vf Ibat If Dr D% Iref note`

- PVv/PVi, ACv/ACi, Vbat/Vf, Ibat/If = ค่าจริงในลูป (ดิบกับกรอง)
- Dr = PWM raw, D% = percent, Iref = เป้ากระแสของ PI
- note: `Ipv` กระแส PV ทั้งที่แรงดัน PV≈0, `Iac` แบบเดียวกัน, `Vspk` Vbat กระโดดจากฟิลเตอร์, `ADC` อ่านไม่ทัน, `Ilag` If ค้าง

ไม่ได้ยุบเป็น Iin/Vin ตามโหมดแล้ว (ตารางเก่าทำให้ Vin=0 ทั้งที่มี AC)
