# Flowchart มีหมายเลข + รายละเอียด

แท็ก: `cv58-boost-v14-forward-v83`  
โครงแบบตัวอย่าง: SoftStart → อ่านเซนเซอร์ → Fault → CC/CV → PWM → DONE  
อ่านตามเลข **① → ⑳**

---

## 1) BOOST (PV) — ใช้ PI · CC 6 A · CV 56.00 V

```mermaid
flowchart TD
    A(["① เริ่มต้น BOOST"]) --> B["② ตั้งค่าเริ่มต้น<br/>D=0 · ΔD · Kp/Ki<br/>Ts≈20 ms<br/>Vcv_ref = 56.00 V<br/>Icc_ref = 6.0 A<br/>I_cutoff = 0.50 A<br/>Dmax = 760<br/>SoftStart 2.5 s"]

    B --> C["③ SoftStart<br/>duty เลื่อนหา seedDuty<br/>slew +2 / −5"]
    C --> D["④ อ่านเซนเซอร์<br/>Vpv · Ipv · Vbat · Ibat<br/>กรอง Vf / If"]
    D --> E{"⑤ มี Fault?<br/>BMS-OPEN 56.30<br/>HARD OVP 57.80<br/>PV &lt; 39 V นาน 2s<br/>OC Ipv &gt; 16.3 A"}
    E -- ใช่ --> F["⑥ ปิด PWM<br/>OVP latch / Shutdown"]
    F --> D
    E -- ไม่ --> G["⑦ คำนวณ Ppv = Vpv×Ipv<br/>MPPT ทุก 100 ms<br/>→ P_avail · Vref · Iref_mppt"]
    G --> H{"⑧ เฟสตอนนี้?"}

    H -- SoftStart --> H1{"⑨ พร้อม?<br/>Ibat≥0.4 หรือครบ 2.5s"}
    H1 -- ไม่ --> N
    H1 -- ใช่ --> H2["⑩ ไป CC_MPPT"]
    H2 --> N

    H -- CC_MPPT --> I["⑪ โหมด CC<br/>I_ref = min(6.0, Iref_mppt)<br/>ลดเมื่อ PV&lt;41 หรือ V≥54.80 taper"]
    I --> J["⑫ E = I_ref − Ibat<br/>PI (Kp=14, Ki=55)<br/>ช่วยปีนถ้า I ต่ำ + duty&lt;280"]
    J --> K{"⑬ เข้า CV?<br/>V≥55.70 ทันที<br/>หรือ V≥55.50 นาน 200ms"}
    K -- ใช่ --> L
    K -- ไม่ --> M["⑭ จำกัด D 0..760<br/>Anti-windup · slew +4/−6"]
    M --> N["⑮ อัปเดต PWM<br/>GPIO27 · 50 kHz · dither"]
    N --> D

    H -- CV --> L["⑯ โหมด CV<br/>V_ref = 56.00 V"]
    L --> O["⑰ E = V_ref − Vbat<br/>PI แรงดัน → iReq<br/>แคป near / mid / Ppv<br/>แล้ว PI กระแส → D"]
    O --> P["⑱ จำกัด D 0..760<br/>deadband hold ±0.12 V<br/>เหนือเป้า bleed duty"]
    P --> Q["⑲ อัปเดต PWM"]
    Q --> R{"⑳ แบตเต็ม?<br/>Vf≥55.90 และ If≤0.50<br/>นาน 60 s"}
    R -- ไม่ --> R2{"㉑ Vf ≤ 54.20<br/>นาน 5 s? → กลับ CC"}
    R2 -- ใช่ --> I
    R2 -- ไม่ --> D
    R -- ใช่ --> S(["㉒ DONE / FULL<br/>หยุด PWM"])
    S --> T{"㉓ Vf ≤ 54.0<br/>และ PV ≥ 42?"}
    T -- ไม่ --> S
    T -- ใช่ --> I
```

### รายละเอียด Boost ตามเลข

| เลข | รายละเอียดจากโค้ด |
|----:|---------------------|
| ② | `boostNewResetOnEntry` · Iref_mppt เริ่ม = 6 A · PvRef ≈ Vpv |
| ③–⑩ | SoftStart: `boostEstimateDutyRaw` · พร้อมเมื่อ I≥0.4 หรือ 2500 ms |
| ⑦ | MPPT: ก้าว Vref ±0.10 V ช่วง 40–45 · Iref_mppt += 0.08×(Vpv−Vref) |
| ⑪ | CC: ไม่แคป Iref จาก Ppv · ลดเมื่อ PV ทรุด / taper ใกล้เต็ม |
| ⑫ | PI กระแส · dDuty จำกัดด้วย slew |
| ⑬ | เข้า CV ที่ 55.50 (200 ms) หรือ force 55.70 |
| ⑰ | CV: แคป iReq near/mid + **Ppv** (v83) · Iref slew 0.04 A/tick |
| ⑳ | FULL: Vf≥55.90 และ If≤0.50 นาน 60 s |
| ㉑ | ออก CV→CC เมื่อ Vf≤54.20 นาน 5 s |
| ㉓ | ชาร์จต่อเมื่อ Vf≤54.0 และมี PV |

---

## 2) FORWARD (AC) — โครงเดียวกับตัวอย่าง · **ไม่ใช้ PID**

เป้า: SoftStart → CC 3 A → CV 55.90 V · step/hysteresis · Dmax 460

```mermaid
flowchart TD
    A(["① เริ่มต้น FORWARD"]) --> B["② ตั้งค่าเริ่มต้น<br/>D=0 · ไม่มี Kp/Ki<br/>Ts≈20 ms<br/>Vcv_ref = 55.90 V<br/>Icc_ref = 3.0 A<br/>I_cutoff = 0.50 A<br/>Dmax = 460<br/>SoftStart 5 s"]

    B --> C["③ SoftStart<br/>seed ≈ 45% ของเป้า CC<br/>min 60 · duty += 0.8"]
    C --> D["④ อ่านเซนเซอร์<br/>Vac · Iac · Vbat · Ibat<br/>กรอง Vf / If · กัน ADC glitch"]
    D --> E{"⑤ มี Fault?<br/>OVP 57.80 · OC Iac&gt;2.5<br/>OC Ibat&gt;3.75<br/>AC หาย นาน 15s"}
    E -- ใช่ --> F["⑥ ปิด PWM / ตัด duty<br/>OVP latch หรือ Shutdown"]
    F --> D
    E -- ไม่ --> G{"⑦ freezeDutyUp?<br/>AC&lt;95 จริง / collapsing<br/>หรือ AC&lt;115 และ duty&gt;40"}
    G -- ใช่ --> G1["⑧ ห้ามเพิ่ม D"]
    G -- ไม่ --> H
    G1 --> H{"⑨ เฟสตอนนี้?"}

    H -- SoftStart --> SS["⑩ เพิ่ม D ช้า ๆ<br/>ตัดที่ seed"]
    SS --> SS1{"⑪ พร้อม?<br/>duty≥seed×0.55 และ I≥0.25<br/>หรือครบ 5 s"}
    SS1 -- ไม่ --> N
    SS1 -- ใช่ --> SS2["⑫ ไป CC"]
    SS2 --> N

    H -- CC --> I["⑬ โหมด CC<br/>I_ref = 3.0 A<br/>ลดเมื่อ AC&lt;100<br/>taper เมื่อ Vf≥54.70"]
    I --> J["⑭ E = I_ref − Ibat<br/>|&gt;0.10| → ±0.6<br/>|&gt;0.60| → +1.2 / −1.5<br/>ใน ±0.10 → hold"]
    J --> K{"⑮ เข้า CV?<br/>maxV ≥ 55.30 ทันที<br/>หรือ ≥ 55.00 นาน 200 ms"}
    K -- ใช่ --> L
    K -- ไม่ --> M["⑯ จำกัด D 0..460"]
    M --> N["⑰ อัปเดต PWM<br/>GPIO14 · 67 kHz · ไม่ dither"]
    N --> D

    H -- CV --> L["⑱ โหมด CV<br/>V_ref = 55.90 V"]
    L --> O["⑲ E = V_ref − Vf<br/>ต่ำ: +0.40 / +0.20<br/>สูง: −0.35 / −0.80 / −1.50<br/>ใกล้เต็ม I≤0.40 → ห้ามปีน D"]
    O --> P["⑳ จำกัด D 0..460<br/>outer OC / OVP เบา"]
    P --> Q["㉑ อัปเดต PWM"]
    Q --> R{"㉒ แบตเต็ม?<br/>slow: maxV≥55.80 · minI≤0.50 · 15s<br/>fast: maxV≥55.90 · Iabs≤0.35 · 5s"}
    R -- ไม่ --> R2{"㉓ Vf ≤ 54.50<br/>นาน 5 s? → กลับ CC"}
    R2 -- ใช่ --> I
    R2 -- ไม่ --> D
    R -- ใช่ --> S(["㉔ DONE / FULL<br/>หยุด PWM · ปิด relay"])
    S --> T{"㉕ Vf ≤ 54.0<br/>และ AC ≥ 95?"}
    T -- ไม่ --> S
    T -- ใช่ --> I
```

### รายละเอียด Forward ตามเลข

| เลข | รายละเอียดจากโค้ด |
|----:|---------------------|
| ② | `forwardNewResetOnEntry` · forwardMode = SoftStart |
| ③–⑫ | SoftStart: seed จาก `forwardEstimateDutyRaw` · step +0.8 · พร้อมตาม ⑪ |
| ⑦–⑧ | `freezeDutyUp` กัน Cin / AC อ่อน (ไม่ถือ AC=0 ปลอมตอน I ยังไหล) |
| ⑬–⑭ | CC step/hysteresis · hold ±0.10 A |
| ⑮ | เข้า CV: force 55.30 หรือ entry 55.00 × 200 ms |
| ⑱–㉑ | CV step · hold ±0.05 V · freeze ปีนเมื่อ I≤0.40 ใกล้เป้า |
| ㉒ | FULL slow 15 s / fast 5 s |
| ㉓ | ออก CV→CC เมื่อ Vf≤54.50 นาน 5 s |
| ㉕ | ชาร์จต่อจาก DONE เมื่อ Vf≤54.0 และมี AC |

---

## ค่าสำคัญเทียบกัน

| รายการ | BOOST | FORWARD |
|--------|------:|--------:|
| ควบคุม | PI | step / hysteresis |
| CC | 6.0 A | 3.0 A |
| CV | 56.00 V | 55.90 V |
| PWM | 50 kHz · GPIO27 | 67 kHz · GPIO14 |
| Dmax | 760 | 460 |
| SoftStart | 2.5 s | 5 s |
| FULL | 60 s | 15 s / 5 s |
| ออก CV→CC | 54.20 V × 5 s | 54.50 V × 5 s |
