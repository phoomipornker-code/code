# cv58-boost-v14-forward-v99

ทำไมสโคปเห็น ~90 V แต่ Serial Vf ยัง ~53 V: ฟิลเตอร์ pack รับแค่ 35–62 V จึงทิ้ง ADS 90 V — OVP/FULL มองไม่เห็น

**v99** ตัด PWM จาก `v_bat_ads > 62 V` ทันที (รวม SoftStart) ค้าง 80 ms แล้ว `[STOP] ADS fly-up PWM-off` — **ไม่** FULL HOLD (กันจอกระพริบ)

Vf / SOC / FULL ยังใช้ค่า pack 35–62 V

Boost ไม่แตะ PI
