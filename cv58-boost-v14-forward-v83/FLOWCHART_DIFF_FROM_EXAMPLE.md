# จุดที่ต่างจากตัวอย่าง vs โค้ดจริง

แท็ก: `cv58-boost-v14-forward-v83`  
ตัวอย่าง = รูป SoftStart → CC/CV/Float แบบทั่วไป  
โค้ด = พฤติกรรมจริงในเฟิร์มแวร์

---

## BOOST — ต่างจากตัวอย่างกี่จุด

```mermaid
flowchart TB
    A(["① เริ่มต้น"]) --> B["② ตั้งค่า<br/>Vcv_ref=56.00 · Icc_ref=6.0<br/>Dmax=760 · Kp/Ki · Ts≈20ms"]
    B --> C["③ SoftStart เป็นโหมดใน state machine<br/>ไม่ใช่ทำครั้งเดียวแล้วจบ"]
    C --> D["④ อ่านเซนเซอร์ทุก 20ms"]
    D --> E{"⑤ Fault?<br/>BMS-OPEN / HARD OVP / PV&lt;39<br/>ไม่มี OTP ในโค้ด"}
    E -- ใช่ --> F["⑥ ปิด PWM"] --> D
    E -- ไม่ --> G["⑦ MPPT ทุก 100ms<br/>ได้ Iref_mppt · P_avail"]
    G --> H{"⑧ เฟส state?<br/>ไม่ใช่เพชร V≥Vcv อย่างเดียว"}

    H -- SoftStart --> SS["⑨ ปีน duty หา seed<br/>พร้อม: I≥0.4 หรือ 2.5s → CC"]
    SS --> M

    H -- CC --> I["⑩ โหมด CC<br/>I_ref = min(6.0, Iref_mppt)<br/>❌ ไม่ใช้ min(Icc, P_avail/Vbat)<br/>✔ ลดเมื่อ PV&lt;41 · taper V≥54.80"]
    I --> J["⑪ PI กระแส → ปรับ D"]
    J --> K{"⑫ เข้า CV?<br/>V≥55.50 ×200ms<br/>หรือ force V≥55.70<br/>ยังไม่รอถึง 56.00"}
    K -- ใช่ --> L
    K -- ไม่ --> M

    H -- CV --> L["⑬ โหมด CV V_ref=56.00<br/>PI แรงดัน → iReq<br/>แล้ว PI กระแส → D<br/>แคป iReq: near/mid/Ppv"]
    L --> M

    M["⑭ จำกัด D 0..760 + slew"] --> N["⑮ อัปเดต PWM"] --> D

    L --> R{"⑯ เต็ม? Vf≥55.90 และ If≤0.50 ×60s"}
    R -- ไม่ --> R2{"⑰ Vf≤54.20 ×5s → กลับ CC"}
    R2 -- ใช่ --> I
    R2 -- ไม่ --> N
    R -- ใช่ --> S(["⑱ DONE หยุด PWM<br/>ไม่ทำ Float แรงดันค้าง"])
    S --> T{"⑲ Vf≤54.0 และ PV≥42?"}
    T -- ไม่ --> S
    T -- ใช่ --> I
```

### ตารางจุดต่าง BOOST

| # | เรื่อง | ตัวอย่าง | โค้ดจริง |
|---|--------|---------|----------|
| 1 | แยก CC/CV | เพชร `Vbat ≥ Vcv_ref` ทุกรอบ | state: SoftStart→CC→CV→DONE |
| 2 | เข้า CV | เมื่อถึง **Vcv_ref (56.0)** | เข้าเร็วที่ **55.50 / 55.70** |
| 3 | I_ref ใน CC | `min(Icc, P_avail/Vbat)` | **ไม่แคปจาก Ppv** (กันติดกระแสต่ำ) |
| 4 | ควบคุม CV | PI แรงดัน → D โดยตรง | **PI แรงดัน → iReq → PI กระแส → D** |
| 5 | SoftStart | ทำครั้งเดียวตอนเริ่ม | เป็นโหมด วนจนพร้อม |
| 6 | เต็มแล้ว | Float หรือหยุด | **DONE หยุด PWM** (ไม่ float) |
| 7 | OTP | มีใน Fault | **ไม่มี** |
| 8 | ออก CV→CC | ไม่เน้นในรูป | มี: Vf≤**54.20** นาน 5s |
| 9 | MPPT | ทุกวงรอบ | ทุก **100 ms** |

---

## FORWARD — ต่างจากตัวอย่างมากขึ้น

```mermaid
flowchart TB
    A(["① เริ่มต้น"]) --> B["② ตั้งค่า<br/>Vcv=55.90 · Icc=3.0 · Dmax=460<br/>❌ ไม่มี Kp/Ki"]
    B --> C["③ SoftStart โหมด"] --> D["④ อ่านเซนเซอร์"]
    D --> E{"⑤ Fault?"}
    E -- ใช่ --> F["⑥ ตัด PWM"] --> D
    E -- ไม่ --> G{"⑦ freezeDutyUp?<br/>มีในโค้ด ไม่มีในตัวอย่าง"}
    G --> H{"⑧ เฟส SoftStart/CC/CV/DONE"}

    H -- CC --> I["⑨ CC step/hysteresis<br/>❌ ไม่ใช่ PI<br/>❌ ไม่มี MPPT/P_avail"]
    I --> J["⑩ ปรับ D เป็นขั้น ±0.6..1.5"]
    J --> K{"⑪ เข้า CV ที่ 55.00/55.30"}
    K -- ใช่ --> L
    K -- ไม่ --> M

    H -- CV --> L["⑫ CV step/hysteresis<br/>❌ ไม่ใช่ PI"]
    L --> O["⑬ ปรับ D เป็นขั้น"]
    O --> M["⑭ จำกัด D"] --> N["⑮ PWM"] --> D

    O --> R{"⑯ เต็ม? slow15s / fast5s"}
    R -- ไม่ --> R2{"⑰ V≤54.50 ×5s → CC"}
    R2 -- ใช่ --> I
    R2 -- ไม่ --> N
    R -- ใช่ --> S(["⑱ DONE"]) --> T{"⑲ ชาร์จต่อ?"}
    T -- ใช่ --> I
    T -- ไม่ --> S
```

### ตารางจุดต่าง FORWARD

| # | เรื่อง | ตัวอย่าง | โค้ดจริง |
|---|--------|---------|----------|
| 1 | ตัวควบคุม | PI (Kp/Ki) | **step / hysteresis** |
| 2 | MPPT / P_avail | มี | **ไม่มี** (ชาร์จจาก AC) |
| 3 | แยกโหมด | V ≥ Vcv ทุกรอบ | state SoftStart→CC→CV→DONE |
| 4 | freezeDutyUp | ไม่มี | **มี** กัน AC อ่อน / Cin |
| 5 | Vcv_ref | มัก 56 V | **55.90 V** |
| 6 | Icc_ref | 6 A ในรูป Boost | **3.0 A** |
| 7 | FULL | I ≤ I_cutoff × T_end | slow **15s** + fast **5s** |
| 8 | Float | มีในตัวอย่าง | **DONE หยุด** |

---

## สรุปสั้น

- **ตัวอย่าง** = แผนภาพสอนโครง CC/CV ทั่วไป  
- **BOOST ในโค้ด** ≈ ตัวอย่าง แต่ต่างที่การเข้า CV, ไม่แคป CC จาก Ppv, CV ใช้ PI สองชั้น, ไม่มี Float/OTP  
- **FORWARD ในโค้ด** = โครงเฟสเดียวกัน แต่ **ไม่มี PID/MPPT** ใช้ step แทน
