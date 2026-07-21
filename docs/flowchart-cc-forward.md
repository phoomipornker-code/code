# โฟลว์ชาร์ต Current Mode Control (CC) — Forward Converter

Peak **Current Mode Control** สำหรับ Single-Switch Forward + Nr  
สเปก: AC 240 V → DC 58 V / 5 A | \(f_s \approx 65\,\mathrm{kHz}\) | Q1 = STW20N95K5

---

## 0. โฟลว์ชาร์ตหลักแบบ CC → CV (ดิจิทัล / MCU)

สไตล์เดียวกับลูป PI + Soft-start + Fault + สลับโหมด CV (เช่น ชาร์จแบตถึง 58 V)

```mermaid
flowchart TD
    A([เริ่มต้น]) --> B["ตั้งค่าเริ่มต้น:<br/>I_ref=5A, V_CV=58V<br/>Kp, Ki, Ts<br/>D_min, D_max=0.45"]
    B --> C[เริ่ม PWM และ Soft-start]
    C --> D[วัดค่า I_out, V_out, V_in]
    D --> E{"มี Fault ไหม?<br/>(OV / OC / OTP / UV)"}
    E -- มี --> F[ปิด PWM + แจ้ง Fault]
    F --> D
    E -- ไม่มี --> G{"โหมดปัจจุบัน?"}
    G -- CC --> H["error: e = I_ref - I_out"]
    G -- CV --> I["error: e = V_CV - V_out"]
    H --> J["PI Controller:<br/>u = u_prev + Kp(e-e_prev) + Ki·e·Ts"]
    I --> J
    J --> K["ปรับ Duty: D = D + u<br/>(หรือปรับ Ipeak_ref ถ้าวงจร Peak-CC)"]
    K --> L["จำกัด Duty:<br/>D_min ≤ D ≤ D_max=0.45"]
    L --> M{"I_out > I_limit_hard<br/>หรือ ip_peak > I_peak_max ?"}
    M -- ใช่ --> N[ลด D แบบฉุกเฉิน / ปิด PWM ชั่วคราว]
    M -- ไม่ --> O[อัปเดต PWM ด้วยค่า D ใหม่]
    N --> O
    O --> P[บันทึกค่าเดิม: e_prev, u_prev]
    P --> Q{"ถึงเงื่อนไข CV หรือยัง?<br/>(V_out ≥ V_CV และโหมด CC)"}
    Q -- ยัง --> D
    Q -- ใช่ --> R[สลับไปโหมด CV]
    R --> D
    P --> S{"ถึงเงื่อนไขกลับ CC?<br/>(I_out < I_ref·0.9 และโหมด CV)"}
    S -- ใช่ --> T[สลับกลับโหมด CC]
    S -- ไม่ --> D
    T --> D
```

### พารามิเตอร์แนะนำสำหรับดีไซน์นี้

| ตัวแปร | ค่า |
|--------|-----|
| `I_ref` | 5 A (โหมด CC) |
| `V_CV` | 58 V |
| `D_max` | **0.45** (รีเซ็ต Nr = Np) |
| `D_min` | ≈ 0.05–0.10 |
| `Ts` | คาบลูปนอก เช่น 50–200 µs (ช้ากว่า \(f_s\)) |
| `I_limit_hard` | ≈ 5.5–6 A |
| `I_peak_max` (ปฐมภูมิ) | ≈ 3.2–3.5 A |

> ถ้ายังใช้ **analog Peak-CC (UC384x)** แทนการปรับ Duty โดยตรง: ให้ PI ออกเป็น **`Ipeak_ref` / Vcomp** แล้วให้ latch ตัดเกตเมื่อ `Vs ≥ Vcomp`

---

## 1. บล็อกไดอะแกรมระบบ

```text
                    ┌─────────────────────────────────────────┐
   AC 240V → Bridge → Cin → [Forward + Nr] → LC → Vo 58V/5A  │
                    │         Q1 STW20N95K5                    │
                    │              ▲ gate                      │
                    │         Driver 10–12V                    │
                    │              ▲                           │
                    │         PWM latch (UC384x ฯลฯ)           │
                    │         ▲           ▲                    │
                    │    Vsense      Vcomp (จากออปโต)         │
                    │    (Rsense)         ▲                    │
                    │                     │                    │
                    │              Opto ← TL431 ← Vo           │
                    └─────────────────────────────────────────┘
```

| ลูป | หน้าที่ |
|-----|---------|
| **Current loop (เร็ว)** | เปรียบเทียบ \(v_{sense}=i_p R_s\) กับ \(v_c\) → ตัดเกตเมื่อถึงพีค |
| **Voltage loop (ช้า)** | TL431 + ออปโต ปรับ \(v_c\) ให้ \(V_o=58\,\mathrm{V}\) |

---

## 2. โฟลว์ชาร์ตต่อรอบสวิตช์ (Peak CC)

```mermaid
flowchart TD
    A([เริ่มรอบ: Clock / Oscillator]) --> B[Set PWM latch<br/>เปิดเกต Q1]
    B --> C[กระแสปฐมภูมิ ip เพิ่ม<br/>Vs = ip × Rsense]
    C --> D{Vs ≥ Vcomp<br/>หรือ hit current limit?}
    D -->|ยังไม่| C
    D -->|ใช่| E[Reset PWM latch<br/>ปิดเกต Q1]
    E --> F[รีเซ็ตฟลักซ์ผ่าน Nr + Dr<br/>D2 ฟรีวีลด้านทุติยภูมิ]
    F --> G{หมดคาบ Ts?<br/>รอ clock ถัดไป}
    G -->|ยัง| F
    G -->|ใช่| A

    H[Voltage loop<br/>TL431 เทียบ Vo กับ 2.5V ref] -.->|ปรับ Vcomp ผ่านออปโต| D
```

### คำอธิบายสั้น

1. **Clock** เปิด MOSFET ทุกต้นรอบ  
2. กระแส \(i_p\) ไหลผ่าน \(R_{sense}\) → \(V_s\)  
3. เมื่อ \(V_s \ge V_{comp}\) → **ปิดเกตทันที** (cycle-by-cycle)  
4. ช่วง OFF: Nr รีเซ็ตแกน, D2 ฟรีวีล  
5. TL431 ดู \(V_o\) แล้วดึงออปโต → เปลี่ยน \(V_{comp}\) (ตั้งพีคกระแส)

---

## 3. โฟลว์ชาร์ตลำดับทำงานทั้งระบบ

```mermaid
flowchart TD
    S([จ่าย AC / Soft-start]) --> S1[ชาร์จ Cin ผ่าน NTC/ฟิวส์]
    S1 --> S2[VCC คอนโทรลเลอร์ขึ้น<br/>UVLO ปล่อย]
    S2 --> S3[Soft-start: Vcomp เพิ่มช้า ๆ]
    S3 --> RUN{โหมดปกติ CC}

    RUN --> CLK[Clock เปิด Q1]
    CLK --> RAMP[ip เพิ่มขึ้น]
    RAMP --> CMP{Vs ≥ Vcomp?}
    CMP -->|No| RAMP
    CMP -->|Yes| OFF[ปิด Q1]
    OFF --> RST[Reset winding + freewheel]
    RST --> FB[TL431 ปรับออปโต → Vcomp]
    FB --> PROT{Fault?}

    PROT -->|OVP / OCP / OTP / UVLO| FLT[Latch off / hiccup]
    PROT -->|ปกติ| CLK
    FLT --> WAIT[รอรีเซ็ต / soft-start ใหม่]
    WAIT --> S2
```

---

## 4. โฟลว์ชาร์ตป้องกัน (Protection)

```mermaid
flowchart LR
    subgraph sense [ตรวจจับ]
        A1[Vsense สูงเกิน<br/>OCP]
        A2[Vo สูงเกิน<br/>OVP via TL431]
        A3[VCC ต่ำ<br/>UVLO]
        A4[อุณหภูมิ<br/>OTP ถ้ามี]
    end

    subgraph action [การตอบสนอง]
        B1[ตัดเกตทันที<br/>cycle-by-cycle]
        B2[ดึง COMP ลง / ปิด PWM]
        B3[หยุดสวิตช์<br/>จน VCC กลับ]
    end

    A1 --> B1
    A2 --> B2
    A3 --> B3
    A4 --> B2
```

---

## 5. สัญญาณสำคัญในหนึ่งคาบ

```text
Clock   : ─┐                 ┌──────────────
           └─────────────────┘
Gate    : ─┐  ┌──┐           ┌──┐
           └──┘  └───────────┘  └────  (ตันเมื่อ Vs=Vcomp)
ip / Vs :    ／|                ／|
           ／  |              ／  |
          ／   └────────────／    └──
               ↑
            Vs = Vcomp  → ปิดเกต
Vcomp   : ────────────────  (ช้า จากลูปแรงดัน)
```

ภาพรวม: `artifacts/flowchart_cc_forward.png`

ข้อจำกัด Forward + Nr: **\(D < 0.5\)** (เมื่อ \(N_r=N_p\)) — ตั้ง max duty ที่คอนโทรลเลอร์ / ramp

---

## 6. จุดเชื่อมกับฮาร์ดแวร์ดีไซน์นี้

| จุด | ค่าแนวทาง |
|-----|-----------|
| \(R_{sense}\) | ให้ \(V_s\) พีค ≈ 0.8–1.0 V ที่ \(I_{p}\approx 3\,\mathrm{A}\) → \(R_s \approx 0.27\text{–}0.33\,\Omega\) / ≥ 2 W |
| Soft-start | คาปาซิเตอร์ที่ขา SS ของ UC3842/3/4/5 |
| Slope compensation | แนะนำใส่เมื่อ \(D>0.4\) (ที่ Vin ต่ำ D≈0.44) |
| Max duty | จำกัด < 0.45 |
| Gate | 10–12 V ไป STW20N95K5 |

---

## 7. สรุปหนึ่งประโยค

**Clock เปิด Q1 → กระแสขึ้นจน Vs ชน Vcomp แล้วปิด → Nr รีเซ็ต → TL431 ปรับ Vcomp ให้ Vo = 58 V**
