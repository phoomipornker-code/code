# Flowchart จัดวางบน→ล่าง แบบตัวอย่าง

ลำดับแนวตั้งเหมือนตัวอย่างทุกขั้น  
แท็ก: `cv58-boost-v14-forward-v83`

---

## BOOST (PV)

```mermaid
flowchart TD
    A(["① เริ่มต้น"])
    A --> B["② ตั้งค่าเริ่มต้น<br/>D, ΔD, Kp, Ki, Ts<br/>Vcv_ref, Icc_ref, I_cutoff<br/>Dmin, Dmax, SoftStart"]
    B --> C["③ Soft-start<br/>Enable PWM"]
    C --> D["④ อ่านค่าเซนเซอร์<br/>Vpv, Ipv, Vbat, Ibat"]
    D --> E{"⑤ มี Fault?<br/>OVP / OCP / UVLO"}
    E -->|ใช่| F["⑥ Disable PWM<br/>รอ / เคลียร์"]
    F --> D
    E -->|ไม่| G["⑦ คำนวณ Ppv = Vpv × Ipv<br/>รันอัลกอริทึม MPPT → P_avail"]
    G --> H{"⑧ Vbat ≥ Vcv_ref ?"}

    H -->|ไม่| I["⑨ โหมด CC<br/>I_ref = min(Icc_ref, …)"]
    I --> J["⑩ E = I_ref − Ibat<br/>อัลกอริทึม PI → ปรับ D"]
    J --> K{"⑪ Vbat ≥ Vcv_entry<br/>ต่อเนื่องครบ T_cv_delay?"}
    K -->|ใช่| L
    K -->|ไม่| M

    H -->|ใช่| L["⑫ โหมด CV<br/>V_ref = Vcv_ref"]
    L --> O["⑬ E = V_ref − Vbat<br/>อัลกอริทึม PI → ปรับ D"]
    O --> M

    M["⑭ จำกัด Dmin ≤ D ≤ Dmax<br/>Anti-windup"]
    M --> N["⑮ อัปเดต PWM"]
    N --> D

    O --> R{"⑯ Ibat ≤ I_cutoff<br/>ต่อเนื่องครบ T_end?"}
    R -->|ไม่| N
    R -->|ใช่| S(["⑰ หยุดจ่าย PWM / DONE"])
    S --> T{"⑱ Vbat &lt; V_recharge ?"}
    T -->|ไม่| S
    T -->|ใช่| D
```

**ลำดับบน→ล่าง**
1. เริ่มต้น  
2. ตั้งค่า  
3. Soft-start  
4. อ่านเซนเซอร์ ← *จุดวนกลับ*  
5. Fault?  
6. (ถ้าใช่) ปิด PWM แล้วกลับ ④  
7. คำนวณ Ppv / MPPT  
8. แยก CC / CV  
9–⑪ สาขา CC  
12–⑬ สาขา CV  
14. จำกัด D  
15. อัปเดต PWM → กลับ ④  
16–⑱ เต็ม / ชาร์จต่อ  

---

## FORWARD (AC) — วางแนวเดียวกัน

```mermaid
flowchart TD
    A(["① เริ่มต้น"])
    A --> B["② ตั้งค่าเริ่มต้น<br/>D, ΔD, Ts · ไม่มี Kp/Ki<br/>Vcv_ref=55.90 · Icc_ref=3.0<br/>I_cutoff · Dmax=460 · SoftStart"]
    B --> C["③ Soft-start<br/>Enable PWM"]
    C --> D["④ อ่านค่าเซนเซอร์<br/>Vac, Iac, Vbat, Ibat"]
    D --> E{"⑤ มี Fault?<br/>OVP / OCP / AC หาย"}
    E -->|ใช่| F["⑥ Disable PWM<br/>รอ / เคลียร์"]
    F --> D
    E -->|ไม่| G["⑦ ตรวจ freezeDutyUp<br/>AC อ่อน → ห้ามเพิ่ม D"]
    G --> H{"⑧ โหมด SoftStart / CC / CV ?"}

    H -->|SoftStart หรือ CC| I["⑨ โหมด CC<br/>I_ref = 3.0 A"]
    I --> J["⑩ E = I_ref − Ibat<br/>step ปรับ D"]
    J --> K{"⑪ Vbat ≥ Vcv_entry<br/>ต่อเนื่องครบ T_cv_delay?"}
    K -->|ใช่| L
    K -->|ไม่| M

    H -->|CV| L["⑫ โหมด CV<br/>V_ref = 55.90 V"]
    L --> O["⑬ E = V_ref − Vbat<br/>step ปรับ D"]
    O --> M

    M["⑭ จำกัด Dmin ≤ D ≤ Dmax"]
    M --> N["⑮ อัปเดต PWM"]
    N --> D

    O --> R{"⑯ แบตเต็ม?<br/>ครบ T_end?"}
    R -->|ไม่| N
    R -->|ใช่| S(["⑰ หยุดจ่าย PWM / DONE"])
    S --> T{"⑱ Vbat &lt; V_recharge ?"}
    T -->|ไม่| S
    T -->|ใช่| D
```

**ลำดับบน→ล่าง** เหมือน BOOST  
ต่างแค่ ⑦ ไม่มี MPPT / ใช้ step แทน PI
