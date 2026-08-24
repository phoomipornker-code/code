# หนังสือและนิพนธ์สำหรับ CC / CV PI

รายการนี้จัดตามโครงที่เครื่องนี้ใช้: **ลูปกระแส PI ใน / ลูปแรงดัน PI นอก**, SoftStart → CC → CV, anti-windup, Forward/Boost ชาร์จแพ็ก LiFePO4

บทเรียนในรีโป: [learn-cc-cv-pi.md](learn-cc-cv-pi.md) · [cc-cv-pi-equations.md](cc-cv-pi-equations.md)

อ่านจากบนลงล่าง — ชุดแรกดาวน์โหลดฟรี

---

## 1. เริ่มที่นี่ (เปิดอ่านได้ โครงตรงโค้ด)

งานเหล่านี้ใช้ **cascade** แบบเดียวกับเฟิร์มแวร์: ลูปนอกออก `Iref` ลูปในออก duty ไม่ใช่ `min(duty_cc, duty_cv)`

| # | งาน | ทำไมถึงตรง | ลิงก์ |
|---|---|---|---|
| 1 | H. Suryoatmojo et al., “Design and Implementation of Battery Charger Using Buck Converter in Constant Current and Voltage Modes…,” *Symmetry*, 18(1), 147, 2026 | ชุดทดลองสอน: cascade PI กระแส/แรงดัน + CC-CV บนไมโครคอนโทรลเลอร์ | [PDF เปิด](https://www.mdpi.com/2073-8994/18/1/147) |
| 2 | A. Alzahrani et al., “Battery Charger Based on High Step-Down DC–DC Converter…,” *Arab. J. Sci. Eng.*, 2025 | **เทียบ cascade กับสวิตช์เลือกโหมด** — สรุปว่า cascade สลับ CC→CV นุ่มกว่า | [DOI](https://doi.org/10.1007/s13369-025-10312-9) |
| 3 | J. C. Mayo-Maldonado et al., “High-Gain Step-Down DC–DC Converter Employed in a Battery Charging Application,” *IEEE Access*, 2023 | PI กระแสช่วง CC, PI แรงดันช่วง CV, มีฮาร์ดแวร์ทดลอง | [OA](https://doi.org/10.1109/access.2023.3327728) |
| 4 | N. Zanatta, *Design and implementation of a buck converter for low-voltage battery charging…*, M.Sc. thesis, Univ. of Padova | ใกล้บอร์ดที่สุด: **STM32 + cascade PI + anti-windup + CC/CV** | [PDF](https://thesis.unipd.it/retrieve/e44f6b46-a5a6-4db9-8781-5fd902efa862/Thesis-Zanatta.pdf) |
| 5 | A. El Aroudi et al., “Modeling and Control of a Three-Phase Interleaved Buck Converter as a Battery Charger,” | หลายลูปซ้อน CC-CV, จูน PI ด้วย root locus, แยกแบนด์วิดท์ลูปใน/นอก | [PDF](https://repositori.urv.cat/repositori/getDocument/imarina%3A9443137?ds=DocumentPrincipal&mime=application%2Fpdf) |
| 6 | A. N. Afandi et al., “Modelling and Simulation of Battery Charger Li-Ion using CC-CV PI Method,” ICEEIE 2021 | แพ็กอนุกรม (10S) + เป้า CC/CV ชัด — อ่านง่าย | [PDF](https://www.scitepress.org/Papers/2021/109680/109680.pdf) |
| 7 | R. D. Puriyanto et al., “Solar charging controller using DC-DC buck converter with cascaded PI…,” *Sustinere JES*, 9(1), 2025 | ใกล้เส้น Boost/PV ของเครื่องนี้: โซลาร์ + cascade PI กันชาร์จเกิน | [DOI](https://doi.org/10.22515/sustinere.jes.v9i1.444) |

อ่านข้อ 2 ให้จบก่อนข้ออื่น — มันอธิบายว่าทำไมโปรเจกต์นี้ทิ้ง `min(duty_cc, duty_cv)` แล้วไป cascade

---

## 2. หนังสือ (รากฐานที่ควรมีบนโต๊ะ)

เรียงตามประโยชน์ต่อเครื่องนี้ ไม่ใช่ตามชื่อเสียงอย่างเดียว

### อิเล็กทรอนิกส์กำลัง — ลูปกระแส/แรงดันของคอนเวอร์เตอร์

1. **R. W. Erickson and D. Maksimović**, *Fundamentals of Power Electronics*, 3rd ed., Springer, 2020.  
   ISBN 978-3-030-43862-3  
   บท averaging, current-programmed control, และ digital control ของสวิตช์คอนเวอร์เตอร์  
   → ใช้เมื่อจะหา \(G_{id}(s)\), \(G_{vi}(s)\) ของ Boost/Forward ก่อนจูน \(K_p, K_i\)

2. **C. P. Basso**, *Designing Control Loops for Linear and Switching Power Supplies: A Tutorial Guide*, Artech House, 2012.  
   ISBN 978-1-60807-557-7  
   **หมวด 5.5.10 “A Dual-Loop Approach in CC-CV Applications”** (หน้า ~288)  
   → หนังสือเล่มเดียวที่มีหัวข้อ CC-CV สองลูปชัด ๆ เหมาะกับชาร์จเจอร์

3. **A. I. Pressman, K. Billings, and T. Morey**, *Switching Power Supply Design*, 3rd ed., McGraw-Hill, 2009.  
   บท Forward converter (หม้อแปลง, reset, duty สูงสุด ~45% เมื่อ \(N_r = N_p\))  
   → คู่กับเส้น Forward 67 kHz / \(d_{\max}=460\) ของเครื่องนี้

### PI ดิจิทัล และ anti-windup

4. **K. J. Åström and T. Hägglund**, *PID Controllers: Theory, Design, and Tuning*, 2nd ed., ISA, 1995.  
   ISBN 978-1-55617-516-9  
   แบบขนาน vs อนุกรม, \(T_i = K_p/K_i\), **reset windup**, การสลับโหมด  
   ฉบับลึกขึ้น: *Advanced PID Control*, ISA, 2006

5. **K. J. Åström and L. Rundqwist**, “Integrator Windup and How to Avoid It,” *Proc. ACC*, 1989.  
   กระดาษสั้นเรื่องแช่อินทิกรัลตอนอิ่มตัว — ตรง `run_pi` / `boostRunPI`

6. **G. F. Franklin, J. D. Powell, and M. L. Workman**, *Digital Control of Dynamic Systems*, 3rd ed.  
   คาบสุ่ม \(\Delta t\), Euler, ความต่างระหว่าง \(f_s\) ของตัวควบคุม (50 Hz) กับ PWM (67 kHz)

### โปรไฟล์ชาร์จแบต (ทำไม 3.60 V/เซลล์ ไม่ใช่ 3.65)

7. **T. B. Reddy (ed.)**, *Linden’s Handbook of Batteries*, 4th ed., McGraw-Hill, 2011.  
   บทลิเธียม / LiFePO4: CC แล้ว CV, กระแสตัดท้าย, อย่าชน OVP ของ BMS

---

## 3. นิพนธ์ / วิทยานิพนธ์

| งาน | ระดับ | ตรงเรื่องไหน |
|---|---|---|
| N. Zanatta, Univ. of Padova (ลิงก์ในข้อ 4 ตารางบน) | ปริญญาโท | ฮาร์ดแวร์จริง, cascade PI, anti-windup, CC/CV |
| W. A. W. M. Zain, *PI controller for battery charger system*, UMP, 2008 | ปริญญาตรี/โปรเจกต์ | PI + buck ชาร์จเจอร์ — พื้นฐาน อ่านง่าย | [บันทึก UMP](https://umpir.ump.edu.my/) ค้นชื่อเรื่อง |
| จิรายุส สุวรรณกวิน (ที่ปรึกษา), *วิธีการควบคุมคอนเวอร์เตอร์ของระบบกักเก็บพลังงานด้วยแบตเตอรี่…*, จุฬาฯ, 2016 | ปริญญาโท | ควบคุมคอนเวอร์เตอร์ของ BESS (ไมโครกริด) ไม่ใช่ CC-CV โดยตรง แต่เป็นภาษาไทยเรื่องลูปคอนเวอร์เตอร์ | [CUIR DOI](https://doi.org/10.58837/chula.the.2016.938) |

ค้นเพิ่มภาษาไทย (ดาวน์โหลดฟรี ไม่ต้องสมัคร):

- [CUIR จุฬาฯ](https://cuir.car.chula.ac.th/)
- [TCI-ThaiJO](https://www.tci-thaijo.org/)
- [ThaiLIS / TDC](https://tdc.thailis.or.th/)

คำค้นที่เจองานใกล้เครื่องนี้:

```text
CC-CV PI charger
constant current constant voltage PI converter
เครื่องชาร์จแบตเตอรี่ กระแสคงที่ แรงดันคงที่
ตัวควบคุมพีไอ บักคอนเวอร์เตอร์
MPPT CC CV boost LiFePO4
forward converter PI
```

---

## 4. งานที่ควรอ่านเพื่อ **ไม่** ทำตาม

โครง `duty = min(PI_cc, PI_cv)` หรือสวิตช์เลือกโหมดที่จุดโวลต์จุดเดียว — ตรงกับของเก่าในโปรเจกต์นี้ที่ทิ้งไป

- M. O. Mahmoud et al., “High efficiency multi power source control constant current/constant voltage charger…,” *IJECE*, 13(1), 2023 — ใช้ MUX สลับ PWM ของ CC กับ CV  
  [OA](https://doi.org/10.11591/ijece.v13i1.pp207-217)

อ่านคู่กับข้อ 2 ในตารางบน จะเห็นว่าทำไม cascade ชนะตอนสลับโหมด

---

## 5. โน้ตปฏิบัติ (ไม่ใช่วิทยานิพนธ์ แต่สั้นและใช้ได้)

- Siemens EDA, “Seamless Transition Control in CC-CV Battery Charging Using PSIM”  
  [บทความ](https://community.sw.siemens.com/s/article/63648-seamless-transition-control-in-cc-cv-battery-charging-using-psim)  
  ลูปนอก = โวลต์ → `Iref`, ลูปใน = กระแส → duty, วิธีไม่ให้ `Iref` กระตุกตอนเข้า CV (bumpless)

- Imperix, *TN108 Cascaded voltage control* — จูนลูปนอกด้วย symmetrical optimum, ลูปในต้องเร็วกว่าลูปนอก  
  [PDF](https://imperix.com/doc/wp-content/uploads/post_to_pdf/TN108.pdf)

---

## 6. ลำดับอ่านที่แนะนำสำหรับเครื่องนี้

1. บทเรียนในรีโป + [สมการ](cc-cv-pi-equations.md)  
2. Zanatta (วิทยานิพนธ์ Padova) — ดูบท control และ anti-windup  
3. Basso หมวด 5.5.10 — dual-loop CC-CV  
4. เปเปอร์ข้อ 2 (cascade vs mode selector)  
5. Erickson บท current-mode / averaging เมื่อจะจูนเกนจากโมเดล Forward/Boost  
6. Åström เมื่อจะทำ anti-windup / สลับโหมดให้ไม่กระตุกบน ESP32
