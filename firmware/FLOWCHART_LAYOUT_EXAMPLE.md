# Flowchart จัดวางแบบตัวอย่าง + หมายเลข

โครงเดียวกับตัวอย่าง:  
**เริ่ม → SoftStart → อ่านเซนเซอร์ → Fault → คำนวณ → แยก CC|CV → จำกัด D → อัปเดต PWM → วนกลับ → DONE/Float**

แท็ก: `cv58-boost-v14-forward-v83`

---

## BOOST (PV) — ตรงตัวอย่าง · ใช้ PI

```mermaid
flowchart TB
    %% ===== หัวบน: เริ่ม + SoftStart =====
    A(["① เริ่มต้น"]) --> B["② ตั้งค่าเริ่มต้น<br/>D, ΔD, Kp, Ki, Ts<br/>Vcv_ref = 56.00 V<br/>Icc_ref = 6.0 A<br/>I_cutoff = 0.50 A<br/>Dmin / Dmax = 760<br/>พารามิเตอร์ SoftStart"]
    B --> C["③ Soft-start<br/>Enable PWM"]

    %% ===== จุดวนหลัก: อ่านเซนเซอร์ =====
    C --> D["④ อ่านค่าเซนเซอร์<br/>Vpv, Ipv, Vbat, Ibat"]

    D --> E{"⑤ มี Fault?<br/>OVP / OCP / UVLO"}
    E -- ใช่ --> F["⑥ Disable PWM<br/>รอ / เคลียร์"]
    F --> D
    E -- ไม่ --> G["⑦ คำนวณ Ppv = Vpv × Ipv<br/>รัน MPPT → P_avail"]

    %% ===== เพชรกลาง: แยก CC / CV =====
    G --> H{"⑧ Vbat ≥ Vcv_ref ?"}

    %% ----- ซ้าย: CC -----
    H -- ไม่ --> I["⑨ โหมด CC<br/>I_ref = min(Icc_ref,<br/>ปรับเมื่อ PV ทรุด / taper)"]
    I --> J["⑩ E = I_ref − Ibat<br/>อัลกอริทึม PI → ปรับ D"]
    J --> K{"⑪ Vbat ≥ Vcv_entry<br/>ต่อเนื่องครบ T_cv_delay?"}
    K -- ใช่ --> L
    K -- ไม่ --> M

    %% ----- ขวา: CV -----
    H -- ใช่ --> L["⑫ โหมด CV<br/>V_ref = Vcv_ref"]
    L --> O["⑬ E = V_ref − Vbat<br/>PI แรงดัน → iReq<br/>แคปตาม Ppv / ใกล้เป้า<br/>PI กระแส → ปรับ D"]
    O --> M

    %% ===== รวมทาง: จำกัด D + PWM =====
    M["⑭ จำกัด Dmin ≤ D ≤ Dmax<br/>Anti-windup"]
    M --> N["⑮ อัปเดต PWM"]
    N --> D

    %% ===== ล่าง: จบจาก CV =====
    O --> R{"⑯ Ibat ≤ I_cutoff<br/>ต่อเนื่องครบ T_end?"}
    R -- ไม่ --> N
    R -- ใช่ --> S(["⑰ Float / DONE<br/>หยุดจ่าย PWM"])
    S --> T{"⑱ Vbat &lt; V_recharge ?"}
    T -- ไม่ --> S
    T -- ใช่ --> D
```

### ค่าในโค้ด BOOST

| สัญลักษณ์ | ค่า | เลขที่เกี่ยวข้อง |
|-----------|-----|------------------|
| Vcv_ref | 56.00 V | ② ⑧ ⑫ |
| Icc_ref | 6.0 A | ② ⑨ |
| Vcv_entry / force | 55.50 / 55.70 | ⑪ |
| T_cv_delay | 200 ms | ⑪ |
| I_cutoff | 0.50 A | ⑯ |
| T_end | 60 s | ⑯ |
| V_recharge | 54.0 V | ⑱ |
| Dmax | 760 | ⑭ |
| Ts | ≈ 20 ms | ② |
| CV exit → CC | Vf ≤ 54.20 นาน 5 s | (ใน ⑫–⑯) |

---

## FORWARD (AC) — จัดวางเหมือนตัวอย่าง · ไม่ใช้ PID

```mermaid
flowchart TB
    %% ===== หัวบน =====
    A(["① เริ่มต้น"]) --> B["② ตั้งค่าเริ่มต้น<br/>D, ΔD, Ts<br/>ไม่มี Kp/Ki (ไม่ใช้ PID)<br/>Vcv_ref = 55.90 V<br/>Icc_ref = 3.0 A<br/>I_cutoff = 0.50 A<br/>Dmin / Dmax = 460<br/>พารามิเตอร์ SoftStart"]
    B --> C["③ Soft-start<br/>Enable PWM · duty += 0.8"]

    %% ===== จุดวนหลัก =====
    C --> D["④ อ่านค่าเซนเซอร์<br/>Vac, Iac, Vbat, Ibat"]

    D --> E{"⑤ มี Fault?<br/>OVP / OCP / AC หาย"}
    E -- ใช่ --> F["⑥ Disable PWM<br/>ตัด duty / Shutdown"]
    F --> D
    E -- ไม่ --> G{"⑦ freezeDutyUp?<br/>AC อ่อน / บัสตก"}
    G -- ใช่ --> G1["⑧ ห้ามเพิ่ม D"]
    G -- ไม่ --> G2["⑨ อนุญาตเพิ่ม D"]
    G1 --> H
    G2 --> H{"⑩ โหมดควบคุม?"}

    %% ----- ซ้าย: SoftStart / CC -----
    H -- SoftStart/CC --> I["⑪ โหมด CC<br/>I_ref = 3.0 A<br/>ลดเมื่อ AC&lt;100<br/>taper เมื่อ Vf≥54.70"]
    I --> J["⑫ E = I_ref − Ibat<br/>step ±0.6 / ±1.2 / ±1.5<br/>hold ใน ±0.10 A"]
    J --> K{"⑬ Vbat ≥ Vcv_entry<br/>ต่อเนื่องครบ T_cv_delay?<br/>หรือ force ≥55.30"}
    K -- ใช่ --> L
    K -- ไม่ --> M

    %% ----- ขวา: CV -----
    H -- CV --> L["⑭ โหมด CV<br/>V_ref = Vcv_ref"]
    L --> O["⑮ E = V_ref − Vbat<br/>step ±0.20…1.50<br/>ใกล้เต็ม I≤0.40 → ห้ามปีน D"]
    O --> M

    %% ===== รวมทาง =====
    M["⑯ จำกัด Dmin ≤ D ≤ Dmax"]
    M --> N["⑰ อัปเดต PWM<br/>GPIO14 · 67 kHz"]
    N --> D

    %% ===== ล่าง: DONE =====
    O --> R{"⑱ Ibat ≤ I_cutoff<br/>และ V สูงพอ<br/>ครบ T_end?<br/>slow 15s / fast 5s"}
    R -- ไม่ --> R2{"⑲ Vbat ≤ 54.50<br/>นาน 5s → กลับ CC"}
    R2 -- ใช่ --> I
    R2 -- ไม่ --> N
    R -- ใช่ --> S(["⑳ Float / DONE<br/>หยุดจ่าย PWM"])
    S --> T{"㉑ Vbat &lt; V_recharge ?<br/>และมี AC"}
    T -- ไม่ --> S
    T -- ใช่ --> D
```

### ค่าในโค้ด FORWARD

| สัญลักษณ์ | ค่า | เลขที่เกี่ยวข้อง |
|-----------|-----|------------------|
| Vcv_ref | 55.90 V | ② ⑭ |
| Icc_ref | 3.0 A | ② ⑪ |
| Vcv_entry / force | 55.00 / 55.30 | ⑬ |
| T_cv_delay | 200 ms | ⑬ |
| I_cutoff | 0.50 A (slow) / 0.35 A (fast) | ⑱ |
| T_end | 15 s / 5 s | ⑱ |
| V_recharge | 54.0 V | ㉑ |
| Dmax | 460 | ⑯ |
| SoftStart | seed≈45% · +0.8 · 5 s | ③ |
| freezeDutyUp | AC&lt;95 หรือ AC&lt;115 &amp; D&gt;40 | ⑦ ⑧ |

---

## เทียบกับตัวอย่าง

| จุดจัดวางในตัวอย่าง | BOOST | FORWARD |
|---------------------|-------|---------|
| แถวบน: เริ่ม + SoftStart | ①–③ | ①–③ |
| กลางบน: อ่านเซนเซอร์ (จุดวน) | ④ | ④ |
| เพชร Fault แล้วกลับ | ⑤–⑥ | ⑤–⑥ |
| คำนวณก่อนแยกโหมด | ⑦ MPPT | ⑦–⑨ freeze |
| เพชรกลาง แยก CC \| CV | ⑧ | ⑩ |
| สาขาซ้าย CC | ⑨–⑪ | ⑪–⑬ |
| สาขาขวา CV | ⑫–⑬ | ⑭–⑮ |
| รวมทาง จำกัด D + PWM | ⑭–⑮ | ⑯–⑰ |
| ลูกศรกลับไปอ่านเซนเซอร์ | ⑮→④ | ⑰→④ |
| ล่าง Float / ชาร์จต่อ | ⑯–⑱ | ⑱–㉑ |
