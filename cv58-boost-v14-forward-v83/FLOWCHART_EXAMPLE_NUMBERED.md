# Flowchart แบบตัวอย่าง — มีหมายเลขลำดับ

โครงสร้างเหมือนตัวอย่าง: SoftStart → อ่านเซนเซอร์ → Fault → CC/CV → PWM → DONE/Float  
แท็ก: `cv58-boost-v14-forward-v83`

---

## 1) BOOST (PV) — ใช้ PI ตามตัวอย่าง

ค่าหลัก: `Icc_ref = 6 A` · `Vcv_ref = 56.00 V` · `Dmax = 760` · `Ts ≈ 20 ms`

```mermaid
flowchart TD
    A(["① เริ่มต้น"]) --> B["② ตั้งค่าเริ่มต้น<br/>D=0 · ΔD · Kp/Ki<br/>Ts≈20ms · Vcv_ref=56.00<br/>Icc_ref=6.0 · I_cutoff=0.50<br/>Dmin/Dmax · SoftStart"]
    B --> C["③ SoftStart<br/>เปิด PWM ช้า ๆ"]
    C --> D["④ อ่านเซนเซอร์<br/>Vpv Ipv Vbat Ibat"]
    D --> E{"⑤ มี Fault?<br/>OVP / OC / UVLO"}
    E -- ใช่ --> F["⑥ ปิด PWM<br/>รอ / เคลียร์"]
    F --> D
    E -- ไม่ --> G["⑦ คำนวณ Ppv = Vpv×Ipv<br/>รัน MPPT → P_avail"]
    G --> H{"⑧ Vbat ≥ Vcv_ref ?"}

    H -- ไม่ --> I["⑨ โหมด CC<br/>I_ref = min(Icc_ref, P_avail/Vbat*)"]
    I --> J["⑩ E = I_ref − Ibat<br/>PI → ปรับ D"]
    J --> K{"⑪ Vbat ≥ Vcv_entry<br/>นานพอ? (→ CV)"}
    K -- ใช่ --> L
    K -- ไม่ --> M["⑫ จำกัด Dmin..Dmax<br/>Anti-windup"]
    M --> N["⑬ อัปเดต PWM"]
    N --> D

    H -- ใช่ --> L["⑭ โหมด CV<br/>V_ref = 56.00 V"]
    L --> O["⑮ E = V_ref − Vbat<br/>PI → ปรับ D<br/>จำกัด Ibat"]
    O --> P["⑯ จำกัด Dmin..Dmax<br/>Anti-windup"]
    P --> Q["⑰ อัปเดต PWM"]
    Q --> R{"⑱ Ibat ≤ I_cutoff<br/>นานครบ T_end?"}
    R -- ไม่ --> D
    R -- ใช่ --> S(["⑲ DONE / Float<br/>หยุด PWM"])
    S --> T{"⑳ Vbat &lt; V_recharge?"}
    T -- ไม่ --> S
    T -- ใช่ --> D
```

\*ในโค้ดจริง CC **ไม่แคป** Iref จาก Ppv (กันติดกระแสต่ำ) — ลด Iref เมื่อ PV ทรุดเท่านั้น  
ใน **CV** จะแคป iReq ตามกำลัง PV (v83) เพื่อไม่ให้แกว่ง

| เลข | ขั้นตอน |
|----:|---------|
| ①–③ | เริ่ม + SoftStart |
| ④–⑦ | อ่านค่า + Fault + MPPT |
| ⑧–⑬ | สาขา CC |
| ⑭–⑱ | สาขา CV |
| ⑲–⑳ | เต็มแล้ว / ชาร์จต่อ |

---

## 2) FORWARD (AC) — โครงเดียวกับตัวอย่าง แต่**ไม่ใช้ PID**

ค่าหลัก: `Icc_ref = 3 A` · `Vcv_ref = 55.90 V` · `Dmax = 460` · step/hysteresis

```mermaid
flowchart TD
    A(["① เริ่มต้น"]) --> B["② ตั้งค่าเริ่มต้น<br/>D=0 · ไม่มี Kp/Ki<br/>Ts≈20ms · Vcv_ref=55.90<br/>Icc_ref=3.0 · I_cutoff=0.50<br/>Dmax=460 · SoftStart"]
    B --> C["③ SoftStart<br/>duty += 0.8 หา seed"]
    C --> D["④ อ่านเซนเซอร์<br/>Vac Iac Vbat Ibat"]
    D --> E{"⑤ มี Fault?<br/>OVP / OC / AC หาย"}
    E -- ใช่ --> F["⑥ ปิด PWM / ลด duty"]
    F --> D
    E -- ไม่ --> G{"⑦ freezeDutyUp?<br/>AC อ่อน / บัสตก"}
    G -- ใช่ --> G1["⑧ ห้ามเพิ่ม D"]
    G -- ไม่ --> H
    G1 --> H{"⑨ เฟสตอนนี้?"}

    H -- SoftStart --> SS["⑩ เพิ่ม D ช้า ๆ<br/>จนพร้อม → ไป CC"]
    SS --> N
    H -- CC --> I["⑪ โหมด CC<br/>I_ref ≈ 3.0 A"]
    I --> J["⑫ E = I_ref − Ibat<br/>step ±0.6 / ±1.2"]
    J --> K{"⑬ Vbat ใกล้เต็ม?<br/>≥55.00 / force 55.30"}
    K -- ใช่ --> L
    K -- ไม่ --> M["⑭ จำกัด D 0..460"]
    M --> N["⑮ อัปเดต PWM"]
    N --> D

    H -- CV --> L["⑯ โหมด CV<br/>V_ref = 55.90 V"]
    L --> O["⑰ E = V_ref − Vbat<br/>step ±0.2..1.5"]
    O --> P["⑱ จำกัด D 0..460"]
    P --> Q["⑲ อัปเดต PWM"]
    Q --> R{"⑳ แบตเต็ม?<br/>slow 15s / fast 5s"}
    R -- ไม่ --> R2{"㉑ V ตกกลับ CC?"}
    R2 -- ใช่ --> I
    R2 -- ไม่ --> D
    R -- ใช่ --> S(["㉒ DONE / Float<br/>หยุด PWM"])
    S --> T{"㉓ Vbat ≤ 54.0<br/>และมี AC?"}
    T -- ไม่ --> S
    T -- ใช่ --> I
```

| เลข | ขั้นตอน |
|----:|---------|
| ①–③ | เริ่ม + SoftStart |
| ④–⑧ | อ่านค่า + Fault + กัน AC อ่อน |
| ⑨–⑮ | SoftStart / CC |
| ⑯–㉑ | CV |
| ㉒–㉓ | เต็มแล้ว / ชาร์จต่อ |

---

## สรุปเทียบตัวอย่าง

| รายการ | ตัวอย่างในรูป | BOOST ในโค้ด | FORWARD ในโค้ด |
|--------|---------------|--------------|----------------|
| SoftStart | มี | มี | มี |
| อ่านเซนเซอร์ → Fault | มี | มี | มี |
| MPPT / P_avail | มี | มี | ไม่มี (ใช้ AC) |
| CC แล้วเข้า CV | มี | มี (PI) | มี (step) |
| เต็ม → Float/DONE | มี | มี | มี |
| ชาร์จต่อเมื่อ V ตก | มี | มี | มี |
