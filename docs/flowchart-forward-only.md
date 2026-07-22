# โฟลว์ชาร์ต Forward — cv58-boost-v14-forward-v23

โหมด **STATE_FORWARD** (AC 110 V → ชาร์จแบต 56 V)  
วัดอินพุตที่ **ขาออกไดโอดบริดจ์ (DC)**

ไฟล์ Mermaid: [`flowchart-forward-only.mmd`](flowchart-forward-only.mmd)

---

## ค่าในโค้ด

| รายการ | ค่า |
|--------|-----|
| PWM | **67 kHz**, GPIO 14 |
| CC / CV | **5.0 A** / **56.0 V** |
| Duty max | **460** (~45%, Nr=Np) |
| บริดจ์ DC ขั้นต่ำ | **95 V** (AC 110 V) |
| หน้าต่างแบตก่อน START | **40.0 … 56.4 V** |
| SoftStart | seed duty ~80, 2 s / I≥0.35 A |
| SoftStart fault | timeout + I&lt;0.15 A → latch |
| เข้า CV | ≥55.5 V confirm / ≥55.7 V force |
| FULL | V≥55.9 และ I≤0.5 A นาน 60 s |
| รีชาร์จ | ≤ 54.0 V |
| หยุดฉุกเฉิน | ≥ 56.8 V นาน 300 ms |

---

## แผนภาพรวม

```mermaid
flowchart TD
    A(["STANDBY"]) --> M{"กด STOP<br/>สลับโหมด"}
    M --> A
    A --> S{"โหมดที่เลือก<br/>= FORWARD?<br/>แล้วกด START"}
    S -- ไม่ใช่ / ไม่กด --> A
    S -- ใช่ --> B{"แบต 40..56.4 V<br/>และบริดจ์ DC ≥ 95 V?"}
    B -- ไม่ผ่าน --> ERR["ERROR<br/>ไม่มีอินพุต/แบต"] --> A
    B -- ผ่าน --> C["เปิดรีเลย์ AC<br/>duty=0<br/>FWD_SOFTSTART"]
    C --> LOOP["อ่านเซนเซอร์<br/>Vbridge / Vbat / Ibat / Iac"]
    LOOP --> F{"ฟอลต์?<br/>OVP / BMS-open / OC / ADC stale / AC ตก"}
    F -- ใช่ --> KILL["PWM=0 ตัดรีเลย์<br/>OVP latch"] --> A
    F -- ไม่ --> H{"FULL HOLD?"}
    H -- ใช่ --> R{"Vbat ≤ 54 V<br/>และ DC ≥ 95 V?"}
    R -- ใช่ --> RESUME["ปลด FULL HOLD"] --> LOOP
    R -- ไม่ --> HOLD["คงหยุด PWM"] --> LOOP
    H -- ไม่ --> PH{"เฟส Forward"}

    PH -- SoftStart --> SS{"Ibat ≥ 0.35 A?"}
    SS -- ใช่ --> CC
    SS -- ไม่ --> TO{"ครบ 2 s?"}
    TO -- ยัง --> RAMP["ramp duty → seed ~80"] --> LOOP
    TO -- ครบ + I&lt;0.15 A --> FAULT["SoftStart no-load<br/>FAULT latch"] --> A
    TO -- ครบ + มีกระแส --> CC

    PH -- CC --> CC["CC: Iref=5 A<br/>taper ใกล้ 56 V<br/>PI → duty"]
    CC --> ENT{"Vbat ≥ 55.7 force<br/>หรือ ≥ 55.5 confirm?"}
    ENT -- ยัง --> LOOP
    ENT -- ใช่ --> CV

    PH -- CV --> CV["CV: V→Iref→duty<br/>deadband / slew"]
    CV --> FULL{"V≥55.9 และ I≤0.5 A<br/>นาน 60 s?"}
    FULL -- ใช่ --> Q["FULL HOLD"] --> LOOP
    FULL -- ไม่ --> HV{"V≥56.8 นาน 300 ms?"}
    HV -- ใช่ --> Q
    HV -- ไม่ --> EXIT{"Vbat ≤ 54.8<br/>นาน 5 s?"}
    EXIT -- ใช่ --> CC
    EXIT -- ไม่ --> LOOP

    PH -- DONE --> Q
```

---

## เฟสควบคุมสั้นๆ

```text
STOP (STANDBY) → เลือก FORWARD
START → เช็กแบต + บริดจ์ DC ≥ 95 V
  → SoftStart (ramp duty)
      → CC (5 A, taper ใกล้ CV)
          → CV (56 V)
              → DONE / FULL HOLD
                  → รีชาร์จเมื่อ Vbat ≤ 54 V
```

### Safety ระหว่างทำงาน
- Hard OVP 57.8 V → latch (เคลียร์ด้วย STOP เมื่อแรงดันลด)
- BMS-open / spike ใกล้ 56 V → ตัด PWM
- Ibat &gt; 5.75 A หรือ |Iac| &gt; 2.5 A → latch
- Duty cap ใกล้โซน BMS

---

## ไฟล์ที่เกี่ยวข้อง

| ไฟล์ | เนื้อหา |
|------|---------|
| [`flowchart-forward-only.mmd`](flowchart-forward-only.mmd) | Mermaid ล้วน |
| [`../firmware/FORWARD_CONTROL_NOTES.md`](../firmware/FORWARD_CONTROL_NOTES.md) | โน้ตค่าคงที่/ลูป |
| [`../firmware/firmware.ino`](../firmware/firmware.ino) | โค้ดจริง |
