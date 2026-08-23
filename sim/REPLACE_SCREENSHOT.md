# แก้ระบบ CC/CV ในภาพ (แทนที่ครึ่งซ้าย)

อย่าไปแก้ตัวเลขในลูปเดิม — ลูป CV/CC ในภาพออก **Duty** ทั้งคู่ ซึ่งไม่ตรงเฟิร์มแวร์

เก็บไว้แค่ครึ่งขวา: Saturation สุดท้าย + คลื่น `Duty_Cy` + ตัวเทียบ `>=` + `[PWM]`

---

## ขั้นตอนใน Simulink (ภาพเดิม)

### 1) ลบครึ่งซ้าย

ลบกล่องเหล่านี้ทั้งชุด:

- Constant `56.8` และลูป CV ทั้งก้อน (Gain 25, Gain 4, integrator, sat, tag `[CV]`)
- Constant `5` และลูป CC ทั้งก้อน (Gain 0.5, Gain 17, integrator, sat, tag `[CC]`)
- กล่อง **Switch** ที่เงื่อนไข `> 55.8`

เหลือ: เส้น `[V_OUT]`, `[I_OUT]`, Saturation ก่อน Duty, คลื่น, PWM

### 2) วางไฟล์ควบคุม

คัดลอก `fwd_cccv_step.m` ไว้ในโฟลเดอร์เดียวกับไฟล์ `.slx`  
ใน MATLAB: `cd` ไปโฟลเดอร์นั้น

### 3) ใส่บล็อกใหม่

1. Library → *User-Defined Functions* → **Interpreted MATLAB Function**
2. ดับเบิลคลิก ใส่

```text
fwd_cccv_step(u(1),u(2),u(3))
```

3. คลิกขวาบล็อก → **Block Parameters → Sample time** = `0.02`

### 4) รวมอินพุต

1. วาง **Mux** 3 ช่อง
2. ต่อ
   - `u(1)` ← `[V_OUT]`
   - `u(2)` ← `[I_OUT]`
   - `u(3)` ← Constant `220` (แรงดัน AC; ถ้าโมเดลมี Vac อยู่แล้วให้ต่อค่านั้น)
3. ออก Mux เข้า Interpreted MATLAB Function

### 5) ต่อออกไป PWM เดิม

1. ออกจากบล็อกใหม่ = **D ทศนิยม 0–0.45** (โค้ดใช้ 460/1023)
2. ต่อเข้า Saturation สุดท้ายในภาพ — ตั้งเพดาน **0 ถึง 0.45** (อย่าปล่อย 0–1)
3. เส้นเดิมเข้าคลื่น `Duty_Cy` และ `>=` ใช้ต่อได้

### 6) Solver

*Model Settings → Solver*: Fixed-step, step ของลูปควบคุม `0.02`  
คลื่น 67 kHz ต้องมี sample เร็วกว่า (Rate Transition) ถ้าต้องการดูพัลส์  
ถ้าแค่ดู CC/CV ให้ดูสัญญาณ **Duty** ก็พอ ไม่ต้องซูม PWM

### 7) กด Run

ควรเห็น:

- V ต่ำกว่า 57.1 → โหมด CC, กระแสเข้าหา 5 A
- V ≥ 57.1 → เข้า CV, กระแสค่อยลด, V เข้าหา **57.6** (ไม่ใช่ 56.8)
- Duty ไม่เกิน 0.45

---

## สร้างโมเดลใหม่ทั้งไฟล์ (ทางลัด)

```matlab
cd sim
build_forward_cccv_model
```

จะได้ `forward_cccv_v4.slx` แล้วต่อ `V_OUT` / `I_OUT` จากแพลนต์ Forward ของคุณเข้า Inport
