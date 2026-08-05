# Forward control flowchart (รูปแบบเดียวกับตัวอย่าง)

ตามโค้ด `cv58-boost-v14-forward-v81` — **ไม่ใช้ PID** · SoftStart→CC→CV→DONE

```mermaid
flowchart TD
    A([เริ่มต้น FORWARD]) --> B[ตั้งค่าเริ่มต้น<br/>forwardMode = SoftStart<br/>duty = 0 · Dmax = 460<br/>Icc_ref = 3.0 A<br/>Vcv_ref = 55.90 V<br/>Ts ≈ 20 ms · ไม่มี Kp/Ki]
    B --> C[เปิด Relay AC · Soft-start<br/>PWM Forward GPIO14]
    C --> D[/อ่านค่าเซนเซอร์<br/>Vac, Vbat, Vf, Ibat, If, Iac/]

    D --> E{มี Fault หรือไม่?<br/>OVP / OC / AC หาย ≥15s<br/>STOP ค้าง 350ms}
    E -- ใช่ --> F[ปิด PWM + Relay<br/>[STOP] / [OVP] / [OC]]
    F --> Z([สิ้นสุด / STANDBY])
    E -- ไม่ใช่ --> FR{freezeDutyUp?<br/>AC&lt;95 จริง หรือ<br/>AC&lt;115 และ duty&gt;40}

    FR --> G{forwardMode?}

    G -- SoftStart --> H[เป้า seedDuty ≈ 45% ของ CC<br/>min 60 · ≤ 460]
    H --> H1{!freeze และ duty &lt; seed?}
    H1 -- ใช่ --> H2[duty += 0.8]
    H1 -- ไม่ใช่ --> H3[ไม่เพิ่ม / ตัดที่ seed]
    H2 --> H3
    H3 --> H4{duty ≥ seed×0.55 และ I≥0.25<br/>หรือ ครบ 5000 ms?}
    H4 -- ไม่ใช่ --> M
    H4 -- ใช่ --> H5[forwardMode = CC]
    H5 --> M

    G -- CC --> I[I_ref = 3.0 A<br/>backoff ถ้า AC&lt;100<br/>taper ถ้า Vf≥54.70]
    I --> I1[e = I_ref − Ibat_filt]
    I1 --> I2{e เทียบแบนด์ ±0.10 A}
    I2 -- e &gt; +0.10 และ !freeze --> I3{e &gt; 0.60?}
    I3 -- ใช่ --> I4[duty += 1.2]
    I3 -- ไม่ใช่ --> I5[duty += 0.6]
    I2 -- e &lt; −0.10 --> I6{e &lt; −0.60?}
    I6 -- ใช่ --> I7[duty −= 1.5]
    I6 -- ไม่ใช่ --> I8[duty −= 0.6]
    I2 -- ในแบนด์ --> I9[hold duty]
    I4 --> I10
    I5 --> I10
    I7 --> I10
    I8 --> I10
    I9 --> I10{max(V,Vf) ≥ 55.30?}
    I10 -- ใช่ --> I11[forwardMode = CV]
    I10 -- ไม่ใช่ --> I12{max(V,Vf) ≥ 55.00<br/>ต่อเนื่อง ≥ 200 ms?}
    I12 -- ใช่ --> I11
    I12 -- ไม่ใช่ --> M
    I11 --> M

    G -- CV --> J[V_ref = 55.90 V<br/>vErr = V_ref − Vf]
    J --> J1{vPeak &gt; V_ref+0.25?}
    J1 -- ใช่ --> J2[duty −= 1.50 + over×1.5]
    J1 -- ไม่ใช่ --> J3{vErr &gt; +0.05?}
    J3 -- ใช่ --> J4{!freeze และ Iabs &lt; 3?}
    J4 -- I≤0.40 และ vErr&lt;0.60 --> J5[freeze ปีน duty]
    J4 -- ใช่ ปีน --> J6{vErr &gt; 0.30?}
    J6 -- ใช่ --> J7[duty += 0.40]
    J6 -- ไม่ใช่ --> J8[duty += 0.20]
    J4 -- I สูงเกินใกล้เป้า --> J9[duty −= 0.35]
    J3 -- vErr &lt; −0.05 --> J10[duty −= ตาม over]
    J3 -- |vErr| ≤ 0.05 --> J11[hold = คงแรงดัน]
    J2 --> J12
    J5 --> J12
    J7 --> J12
    J8 --> J12
    J9 --> J12
    J10 --> J12
    J11 --> J12{Vf ≤ 54.50<br/>ต่อเนื่อง ≥ 5 s?}
    J12 -- ใช่ --> J13[forwardMode = CC]
    J12 -- ไม่ใช่ --> J14{FULL?<br/>V≥55.8 และ I≤0.50 ×15s<br/>หรือ V≥55.9 และ Iabs≤0.35 ×5s}
    J14 -- ไม่ใช่ --> M
    J14 -- ใช่ --> J15[forwardMode = DONE]
    J13 --> M
    J15 --> M

    G -- DONE --> T[ชาร์จครบ · duty = 0<br/>ปิด PWM · [FULL]]
    T --> U{Vf &lt; 54.0 และ AC≥95?}
    U -- ใช่ --> U1[forwardMode = CC]
    U -- ไม่ใช่ --> M
    U1 --> M

    M[จำกัดค่า<br/>0 ≤ duty ≤ 460<br/>ตัด OC/OVP นอกเฟส]
    M --> N[อัปเดต PWM Forward]
    N --> O[รอ tick ถัดไป ~20 ms]
    O --> D
```
