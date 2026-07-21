# โฟลว์ชาร์ต Forward (จากโค้ดหลัก) — มีหมายเลข + อธิบายไทย

อ้างอิงเฟิร์มแวร์: `firmware/charger_main.ino`  
แท็ก: `cv58-forward-67khz-5a-v16`  
โหมด **STATE_FORWARD** (AC → ชาร์จแบต) ใช้ **SoftStart → CC → CV → DONE**

ค่าจริงในโค้ดตอนนี้:

| รายการ | ค่าในโค้ด |
|--------|-----------|
| `PWM_FREQ` | **67 kHz** |
| `TARGET_CC_CURRENT` | **5.0 A** |
| `TARGET_CV_VOLTAGE` | **56.00 V** |
| `MAX_DUTY_FORWARD` | **460 / 1023 ≈ 45%** |
| `RESTART_CHARGE_VOLTAGE` | **54.0 V** |
| `HIGH_VOLTAGE_STOP_VOLTAGE` | **56.80 V** |
| `FULL_END_CURRENT` | **0.50 A** |

เอกสารไทยฉบับเต็ม: [`flowchart-forward-only.md`](flowchart-forward-only.md)  
ไฟล์ Mermaid: [`flowchart-forward-only.mmd`](flowchart-forward-only.mmd)

---

## แผนภาพสรุป

```mermaid
flowchart TD
    A(["① START"]) --> B{"② มี AC ≥ 140V?"}
    B -- ไม่มี --> Z["Standby / NO INPUT"]
    B -- มี --> C["③ STATE_FORWARD SoftStart<br/>duty=0, รีเลย์ AC ON"]
    C --> D["④ อ่านเซนเซอร์ + กรอง"]
    D --> E{"⑤ Fault?"}
    E -- ใช่ --> F["⑥ ปิด PWM/รีเลย์"] --> D
    E -- ไม่ --> G{"⑦ FULL HOLD?"}
    G -- ใช่ --> H{"⑧ Vbat ≤ 54V?"}
    H -- ใช่ --> I["⑨ ปลด HOLD"] --> D
    H -- ไม่ --> D
    G -- ไม่ --> S{"⑩ Soft / CC / CV / DONE"}
    S -- Soft --> SS["⑪ ramp duty"] --> D
    S -- CC --> CC["⑫ Iref=5A + taper"] --> CVIN{"⑬ ≥55.5/55.7 → CV"}
    CVIN -- ยัง --> D
    CVIN -- ใช่ --> CV
    S -- CV --> CV["⑭ V→Iref→duty"] --> FULL{"⑮ เต็ม? / ≥56.8V?"}
    FULL -- ใช่ --> Q["⑯ FULL HOLD"] --> D
    FULL -- ไม่ --> D
```

---

## สูตรสำคัญ

```text
SoftStart → CC (Iref=5A, taper ใกล้ 56V)
         → CV (voltage PI → Iref, current PI → duty)
         → DONE / FULL HOLD
duty ≤ 460, PWM @ 67 kHz on GPIO 14
```
