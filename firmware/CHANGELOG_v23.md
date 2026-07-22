# Changelog v23 — cv58-boost-v14-forward-v23

## Revert v20 current-related patches

ยกเลิกการแก้กระแสที่เพิ่มใน v20:

1. **ไม่** บังคับ `I=0` เมื่อ `V=0` (คืนค่ากระแสตามเซ็นเซอร์จริง)
2. คืนเงื่อนไข ADC glitch แบบเดิมที่ใช้เกตกระแส (`ADC_GLITCH_CURRENT_GATE_A`)

## คงไว้จาก v20+

- `MIN_AC_VOLTAGE = 95 V`
- เคลียร์ stuck `FULL_HOLD` เมื่อ system OFF
- Debug โหมดทำงาน + เซ็นเซอร์ครบช่อง (v21/v22)
