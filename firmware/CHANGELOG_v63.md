# Changelog v63 — cv58-boost-v14-forward-v63

## Field: false SPIKE-PRECUT → OVP หลังเข้า CV

```text
[INFO] FORWARD CC -> CV at Vbat=55.04 / filt=54.95
[CRITICAL] SPIKE-PRECUT at 57.92V (step=2.89V, filt=55.33V, duty=0).
[STAT] OVP …
```

**Cause:** จัมพ์ +2.89 V ≈ **70 mV** raw ผ่านเกต BATspike เก่า (80 mV ≈ 3.3 V) แล้ว `SPIKE-PRECUT` latch OVP จาก raw≥57.80 ทั้งที่ **filt=55.33** และกระแสยังไหล (~2 A)

### v63 fixes
- รัด `ADC_BAT_HIGH_SPIKE_MV` เป็น **50 mV** (~2.1 V)
- เกต spike ฝั่งแรงดัน: ปฏิเสธจัมพ์ **>+2.0 V** เทียบ last-good
- `SPIKE-PRECUT` / `RUNAWAY-CUT` **ไม่ latch OVP** ถ้า filt ยังห่างจาก HARD OVP — แค่ soft-cut duty
- Latch OVP จาก spike เฉพาะเมื่อ filt ใกล้ trip และกระแสยุบจริง

Flash until boot shows:

`[BOOT] cv58-boost-v14-forward-v63 | …`

Expect `[WARN] Spike soft-cut…` instead of OVP latch on a lone 57–58 V mux ghost.
