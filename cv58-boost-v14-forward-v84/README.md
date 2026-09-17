# cv58-boost-v14-forward-v101

**90 V คือบั๊ก ADS** ไม่ใช่แรงดันแพ็กจริง — ห้ามตัด PWM / FULL HOLD / OVP จากค่านั้น

คอนโทรลใช้เฉพาะ Vf ในช่วง 35–62 V
ตัวอย่าง Vbat=91 / 1.7 ถูกทิ้ง (Serial ยังโชว์ AdsX / Vspk ให้ดู)

SoftStart ยังต้องมีกระแสจริงก่อนเข้า CC

Boost ไม่แตะ
