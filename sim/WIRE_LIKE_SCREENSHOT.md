# ต่อ CC/CV แบบบล็อกในภาพ — ไม่ใช้ MATLAB Function

ได้ ใช้กล่องแบบเดิม (Constant, Sum, Gain, Integrator, Sat, Switch, คลื่น PWM)  
แต่ **ต้องต่อสายใหม่** — ห้ามให้ทั้งสองลูปออก Duty แล้วมา Switch

```text
ภาพเดิม (ห้ามใช้)          ต่อใหม่ (ตามเฟิร์มแวร์)
───────────────            ─────────────────────
CV PI ──┐                  CV PI ──► Iref (แอมป์ 0–3)
        ├─ Switch ─ Duty           │
CC PI ──┘                  Switch เลือก Iref=5 หรือ Iref จาก CV
                           CC PI(Iref−I) ──► Δduty
                           Memory+Add ──► duty 0–460
                           ÷1023 ──► Duty 0–0.45 ──► PWM เดิม
```

---

## 1) เก็บครึ่งขวาของภาพ

เหลือ Saturation ของ Duty, คลื่น `Duty_Cy`, ตัวเทียบ `>=`, `[PWM]`  
ลบ Switch `> 55.8` และเส้นที่ `[CV]` / `[CC]` ต่อเข้า Switch

---

## 2) ลูปบน = แรงดัน → **Iref** (ของเดิมที่เขียน [CV])

ใช้ของเดิมได้ แค่เปลี่ยนค่าและเปลี่ยนป้ายว่าเป็นแอมป์

| บล็อก | ค่าใหม่ |
|---|---|
| Constant | **57.60** (เดิม 56.8) |
| Sum | `57.60 − [V_OUT]` |
| Gain P | **0.70** (เดิม 25) |
| Gain I | **0.35** (เดิม 4) |
| Integrator | `K·Ts/(z−1)` **Ts = 0.02** K = 1 |
| ZOH ของ P | Ts = **0.02** |
| Saturation หลังรวม P+I | **0 ถึง 3** (หน่วย A) |

แท็กเส้นนี้ว่า `[Iref_CV]` — **ห้ามต่อเข้า Duty**

เปิด Limit ของ integrator ถ้ามี: 0 ถึง 3 (กัน windup)

---

## 3) แทน Switch เดิมด้วย Relay + Switch ที่เลือก **Iref**

1. **Relay** (ไม่ใช่ Switch เทียบ 55.8)
   - เข้า: `[V_OUT]`
   - Switch on: **57.10**
   - Switch off: **56.40**
   - Output when on = 1 (โหมด CV)
   - Output when off = 0 (โหมด CC)
2. Constant `[I_CC]` = **5**
3. **Switch**
   - ขาบน (u1): `[Iref_CV]` จากข้อ 2
   - ขากลาง (u2): ออกจาก Relay
   - ขาล่าง (u3): `5`
   - Threshold = **0.5**
4. ออกจาก Switch = `[Iref]`

เมื่อ V สูงเกิน 57.10 ใช้ Iref จากลูปแรงดัน  
เมื่อ V ต่ำกว่า 56.40 ใช้ Iref = 5 A

---

## 4) ลูปล่าง = กระแส → **Δduty** (ของเดิมที่เขียน [CC])

อย่าลบลูปนี้ แค่เปลี่ยนอินพุตและเกน

| บล็อก | ค่าใหม่ |
|---|---|
| Sum | **`[Iref] − [I_OUT]`** (เดิมเป็น `5 − I_OUT` ตรงๆ) |
| Gain P | **8** (เดิม 0.5) |
| Gain I | **35** (เดิม 17) |
| Integrator | Ts = **0.02** |
| Saturation หลังรวม P+I | **−20 ถึง +25** |

แท็กว่า `[dDuty]` — ค่านี้ยัง **ไม่ใช่** Duty ที่เข้าคลื่น

---

## 5) สะสม duty (กล่องใหม่ 3 ตัว ต่อหลังลูปกระแส)

ภาพเดิมไม่มีจุดนี้ ต้องเพิ่ม:

```text
[dDuty] ──► Add ──► Sat 0..460 ──► [DutyRaw] ──► Gain 1/1023 ──► Sat 0..0.45 ──► คลื่นเดิม
              ▲
              └── Memory (เก็บ DutyRaw รอบที่แล้ว)
```

1. **Memory** (Discrete → Memory) sample 0.02, initial = 0
2. **Add**: `dDuty + Memory`
3. **Saturation**: **0 ถึง 460**
4. ออก Sat วนกลับเข้า Memory
5. **Gain** `1/1023`
6. ต่อเข้า Saturation สุดท้ายในภาพ ตั้ง **0 ถึง 0.45**
7. เส้นเดิมเข้า `Duty_Cy` กับ `>=` ใช้ต่อได้

อย่าใส่ Integrator แบบ `Ts/(z−1)` ตรงนี้ — จะคูณ 0.02 ซ้ำ  
ใช้ Memory+Add เท่านั้น (บวกทีละติ๊ก 20 ms)

---

## 6) Solver

Fixed-step ของลูปควบคุม = **0.02**  
คลื่น 67 kHz ถ้าจะดูพัลส์ต้อง sample เร็วกว่า

---

## สิ่งที่ยังไม่ครบเมื่อต่อแบบภาพ (ยอมได้ในซิม)

แบบบล็อกนี้ **ไม่มี**: SoftStart 2 s, ยืนยันเข้า CV 200 ms, taper ใกล้ 56.4 V, ลดเกนใกล้เป้า  
บนบอร์ดมีในโค้ด — ในซิมบล็อกนี้ยังเห็น CC → CV ถูกทาง

ถ้าต้องการครบทุกอย่างของเฟิร์มแวร์ ต้องใช้ `fwd_cccv_step` (MATLAB Function)

---

## สร้างโมเดลบล็อกให้ใน MATLAB

```matlab
cd sim
build_forward_cccv_blocks
```

ได้ `forward_cccv_blocks.slx` — ไม่มี MATLAB Function มีแต่กล่องมาตรฐาน
