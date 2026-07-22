# Changelog v25 — cv58-boost-v14-forward-v25

## Guideline: proven PV firmware

ผู้ใช้ยืนยันว่าชาร์จ **PV สำเร็จ** ด้วย `cv58-stability-v14-cv-stable`  
→ **แช่แข็งลูป Boost** ให้ตรงโค้ดนั้น

### Boost (ไม่แตะพฤติกรรมที่พิสูจน์แล้ว)
- SoftStart → CC_MPPT → CV → DONE
- PI / taper / BMS-open / spike / runaway / OVP auto-clear
- BMS / spike / runaway ทำงานเฉพาะ `STATE_BOOST`
- หลัง BMS-open: `forceSafeShutdown()` แล้วตั้ง `charge_full_hold = true` ตามต้นฉบับ

### คงชั้นของเราไว้ (ไม่กระทบ Boost)
- เลือกโหมดด้วย STOP ก่อน START
- Forward SoftStart→CC→CV (PI แบบ Boost) @ 67 kHz / 5 A / AC 110 V bridge
- Debug โหมด + เซ็นเซอร์ครบช่อง
- `PWM_FREQ_BOOST=50000` / `PWM_FREQ_FORWARD=67000`
