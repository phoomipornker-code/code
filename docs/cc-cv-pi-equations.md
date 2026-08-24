# สมการ CC / CV PI

ชุดนี้ตรงกับ `learn/pi.py` และลูป Forward ในเฟิร์มแวร์ (PI แบบขนาน + แช่อินทิกรัลตอนอิ่มตัว)  
คาบสุ่มตัวควบคุม \(\Delta t = 0.02\,\mathrm{s}\) ไม่ใช่คาบ PWM \(67\,\mathrm{kHz}\)

กลับไปบทเรียน: [learn-cc-cv-pi.md](learn-cc-cv-pi.md)

---

## 1. นิยาม error

\[
e[k] = r[k] - y[k]
\]

| ลูป | เป้า \(r\) | วัด \(y\) | เอาต์พุต |
|---|---|---|---|
| กระแส (ใน) | \(I_{\mathrm{ref}}\) | \(\lvert I_{\mathrm{bat}}\rvert\) | \(\Delta d\) (นับ LEDC / ติ๊ก) |
| แรงดัน (นอก, โหมด CV) | \(V_{\mathrm{CV}}=57.60\,\mathrm{V}\) | \(V_{\mathrm{bat}}\) | \(I_{\mathrm{req}}\) (แอมป์) |

---

## 2. PI แบบต่อเนื่อง (parallel form)

\[
u(t) = K_p\, e(t) + K_i \int_0^t e(\tau)\,d\tau
\qquad
G_{\mathrm{PI}}(s) = K_p + \frac{K_i}{s}
\]

เทียบกับแบบอนุกรม \(K_p\bigl(1 + 1/(T_i s)\bigr)\):

\[
T_i = \frac{K_p}{K_i}
\]

| ลูป | \(K_p\) | \(K_i\) | \(T_i\) | ความหมาย |
|---|---|---|---|---|
| กระแส | \(8.0\) | \(35.0\) | \(0.229\,\mathrm{s}\) | ลูปใน — เร็ว |
| แรงดัน | \(0.70\) | \(0.35\) | \(2.0\,\mathrm{s}\) | ลูปนอก — ช้า |

ไม่มีเทอม D: \(K_d = 0\)

---

## 3. PI แบบดิสครีต + anti-windup (ตรงโค้ด)

อินทิกรัลแบบสี่เหลี่ยมไปข้างหน้า (forward Euler) ตาม `run_pi` / `boostRunPI`:

\[
\begin{aligned}
P[k] &= K_p\, e[k] \\
I^\star[k] &= I[k-1] + K_i\, e[k]\,\Delta t \\
u^\star[k] &= P[k] + I^\star[k] \\
u[k] &= \mathrm{sat}\bigl(u^\star[k],\, u_{\min},\, u_{\max}\bigr)
\end{aligned}
\]

แช่อินทิกรัลเมื่อจะชนเพดาน:

\[
I[k] =
\begin{cases}
I^\star[k] & \text{ถ้า } u_{\min} \le u^\star[k] \le u_{\max} \\
I[k-1] & \text{นอกนั้น}
\end{cases}
\]

ฟังก์ชันอิ่มตัว:

\[
\mathrm{sat}(x, a, b) = \max\bigl(a,\, \min(x, b)\bigr)
\]

เพดานที่ใช้:

\[
\begin{aligned}
u_i &\in [-20,\, +25]
&& \text{ลูปกระแส, หน่วย \(\Delta d\)} \\
u_v &\in [0,\, 3]
&& \text{ลูปแรงดัน, หน่วยแอมป์}
\end{aligned}
\]

---

## 4. จำกัดความชัน (slew)

\[
\mathrm{slew}(x^\star, x, \delta_\uparrow, \delta_\downarrow) =
\begin{cases}
x + \delta_\uparrow & x^\star - x > \delta_\uparrow \\
x - \delta_\downarrow & x^\star - x < -\delta_\downarrow \\
x^\star & \text{อื่น ๆ}
\end{cases}
\]

---

## 5. โหมด CC — ลูปกระแสอย่างเดียว

เป้ากระแส (มี taper ใกล้ CV):

\[
I_{\mathrm{ref}}^{\mathrm{CC}}[k] =
I_{\mathrm{CC}} \cdot
\mathrm{sat}\!\left(
  \frac{V_{\mathrm{CV}} - V[k]}{V_{\mathrm{CV}} - V_{\mathrm{taper}}},
  \; 0.10,\; 1
\right)
\quad\text{เมื่อ } V[k] \ge V_{\mathrm{taper}}
\]

\[
I_{\mathrm{CC}} = 5.0\,\mathrm{A},\quad
V_{\mathrm{taper}} = 56.40\,\mathrm{V},\quad
V_{\mathrm{CV}} = 57.60\,\mathrm{V}
\]

ถ้า \(V < 56.40\) แล้ว \(I_{\mathrm{ref}}^{\mathrm{CC}} = 5.0\,\mathrm{A}\)

ลูป:

\[
\begin{aligned}
e_i[k] &= I_{\mathrm{ref}}^{\mathrm{CC}}[k] - \lvert I_{\mathrm{bat}}[k]\rvert \\
\Delta d[k] &= \mathrm{PI}_i\bigl(e_i[k]\bigr) \\
d'[k] &= \mathrm{sat}\bigl(d[k-1] + \Delta d[k],\; 0,\; 460\bigr) \\
d[k] &= \mathrm{slew}\bigl(d'[k],\, d[k-1],\, 3,\, 5\bigr)
\end{aligned}
\]

ช่วง CC แรงดัน **ไม่ได้** ถูกควบคุม — \(V\) ขึ้นตามประจุที่ไหลเข้าแพ็ก

---

## 6. โหมด CV — ลูปซ้อน (cascade)

ลูปนอก (ช้า) กำหนดกระแสอ้างอิง:

\[
\begin{aligned}
e_v[k] &= V_{\mathrm{CV}} - V_{\mathrm{bat}}[k] \\
I_{\mathrm{req}}[k] &= \mathrm{PI}_v\bigl(e_v[k]\bigr) \in [0, 3]\,\mathrm{A} \\
I_{\mathrm{ref}}[k] &= \mathrm{slew}\bigl(I_{\mathrm{req}}[k],\, I_{\mathrm{ref}}[k-1],\, 0.06,\, 0.06\bigr)
\end{aligned}
\]

ลูปใน (เร็ว) ตาม \(I_{\mathrm{ref}}\) เหมือน CC:

\[
\begin{aligned}
e_i[k] &= I_{\mathrm{ref}}[k] - \lvert I_{\mathrm{bat}}[k]\rvert \\
\Delta d[k] &= \mathrm{PI}_i\bigl(e_i[k]\bigr) \\
d[k] &= \mathrm{slew}\!\left(
  \mathrm{sat}\bigl(d[k-1] + \Delta d[k],\; 0,\; 460\bigr),
  \; d[k-1],\; 2,\; 3.5
\right)
\end{aligned}
\]

เขียนรวมเป็นชั้นเดียว:

\[
I_{\mathrm{ref}} = \mathrm{PI}_v(V_{\mathrm{CV}} - V),\qquad
\Delta d = \mathrm{PI}_i(I_{\mathrm{ref}} - I)
\]

---

## 7. Duty → PWM

นับ LEDC 10 บิต:

\[
D = \frac{d}{1023},\qquad
0 \le d \le d_{\max} = 460
\quad\Rightarrow\quad
D_{\max} \approx 0.450
\]

ความถี่สวิตช์ \(f_{\mathrm{sw}} = 67\,\mathrm{kHz}\) ไม่เข้าสมการ PI

---

## 8. สลับโหมด (ไม่ใช่สมการอนุพันธ์ แต่เป็นเงื่อนไข)

\[
\begin{aligned}
\text{SOFT}\to\text{CC}:&\quad t \ge 2\,\mathrm{s}
  \quad\text{(เฟิร์มแวร์ยังออกได้เมื่อ } I \ge 0.35\,\mathrm{A}\text{)} \\
\text{CC}\to\text{CV}:&\quad V \ge 57.10\,\mathrm{V}\ \text{นาน } 200\,\mathrm{ms}
  \ \text{หรือ}\ V \ge 57.30\,\mathrm{V} \\
\text{CV}\to\text{DONE}:&\quad V \ge 57.40\,\mathrm{V}\ \text{และ}\ I \le 0.50\,\mathrm{A}\ \text{นานพอ} \\
\text{ตอนเข้า CV}:&\quad I_i \leftarrow 0,\; I_v \leftarrow 0,\;
  I_{\mathrm{ref}} \leftarrow \mathrm{sat}(\lvert I_{\mathrm{bat}}\rvert,\, 0.3,\, 2)
\end{aligned}
\]

บรรทัดสุดท้ายคือการส่งต่อแบบไม่สะดุด (bumpless) — รีเซ็ตอินทิกรัลแล้วตั้ง \(I_{\mathrm{ref}}\) เท่ากระแสที่มีอยู่

---

## 9. สิ่งที่ห้ามสับสน

โครงเก่า (ไม่ใช้แล้ว):

\[
d = \min\bigl(\mathrm{PI}_{\mathrm{cc}}(I_{\mathrm{CC}}-I),\; \mathrm{PI}_{\mathrm{cv}}(V_{\mathrm{CV}}-V)\bigr)
\]

โครงปัจจุบัน: \(\mathrm{PI}_v\) **ห้าม** ออก duty — ออกได้แค่แอมป์

อย่าเอา \(K_p=8\) ไปคูณ error แล้วถือว่าได้ duty ทศนิยม \(0\ldots 1\)  
หน่วยของลูปกระแสคือ **นับ PWM ต่อ 20 ms**
