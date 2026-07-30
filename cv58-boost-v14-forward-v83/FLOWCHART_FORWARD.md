# Flowchart Forward — ตามโค้ด `cv58-boost-v14-forward-v81.ino`

ไม่ใช้ PID · SoftStart → CC → CV → DONE · step/hysteresis · ลูป ~20 ms

## 1) ปุ่ม / เข้าโหมด FORWARD

```mermaid
flowchart TD
    START([เริ่มต้น STANDBY]) --> STOPQ{กด STOP?}
    STOPQ -->|ใช่ edge + ไม่ OVP| MODE["สลับโหมด BOOST ↔ FORWARD<br/>[MODE] FORWARD AC"]
    STOPQ -->|ใช่ + OVP และ V ≤ 55.8| CLROVP["เคลียร์ ovp_latched<br/>[OVP] cleared by STOP"]
    MODE --> STANDBY2[STANDBY]
    CLROVP --> STANDBY2
    STOPQ -->|ไม่| STARTQ{กด START edge?}
    STANDBY2 --> STARTQ
    STARTQ -->|ไม่| START
    STARTQ -->|ใช่| OVPQ{ovp_latched?}
    OVPQ -->|ใช่| ERR_OVP[LCD: OVP TRIPPED<br/>system_ON=false]
    OVPQ -->|ไม่| BATQ{"Vbat_filt ใน 40.0…56.40?"}
    BATQ -->|ไม่| ERR_NP[LCD: ERROR / NO POWER]
    BATQ -->|ใช่| ACQ{"v_ac_in ≥ 95 V?"}
    ACQ -->|ไม่| ERR_NP
    ACQ -->|ใช่| ON["system_ON = true"]
    ON --> ENTRY["STATE_OFF → เข้า FORWARD:<br/>PWM=0, Relay PV=OFF, delay 500ms<br/>Relay AC=ON<br/>forwardMode = FWD_SOFTSTART<br/>[START] FORWARD SoftStart->CC->CV"]
    ENTRY --> CTRL([เข้าลูปควบคุม Forward])
    ERR_OVP --> START
    ERR_NP --> START
```

## 2) ระหว่างชาร์จ — หยุด / AC หาย

```mermaid
flowchart TD
    RUN([กำลังชาร์จ FORWARD]) --> HOLDSTOP{"กด STOP ค้าง ≥ 350 ms?"}
    HOLDSTOP -->|ใช่| STOPH["system_ON=false<br/>[STOP] hold 350ms end (FORWARD)"]
    HOLDSTOP -->|ไม่| ACSAG{"v_ac_in &lt; 95 และ<br/>Ibat ไม่ไหล (&lt;0.35A)?"}
    ACSAG -->|ไม่ / I ยังไหล| FAKE["ถือว่า AC glitch<br/>ไม่เข้า sag"]
    ACSAG -->|ใช่ ค้าง ≥ 15 s| STOPAC["system_ON=false<br/>[STOP] AC bridge lost"]
    ACSAG -->|ใช่ ยังไม่ครบ 15s| FREEZE1[ac_is_collapsing → freeze duty-up]
    FAKE --> PHASE
    FREEZE1 --> PHASE
    STOPH --> END([กลับ STANDBY])
    STOPAC --> END
    PHASE([ไปเฟส SoftStart/CC/CV])
```

## 3) `freezeDutyUp` (ใช้ทุกเฟสตอนเพิ่ม duty)

```mermaid
flowchart TD
    FQ{freezeDutyUp?}
    FQ -->|"AC ต่ำจริง หรือ collapsing"| YES[ห้ามเพิ่ม duty]
    FQ -->|"v_ac_in &lt; 115 และ raw_duty &gt; 40<br/>(Cin stress)"| YES
    FQ -->|ไม่เข้าเงื่อนไข| NO[อนุญาตเพิ่ม duty]
```

## 4) SoftStart → CC

```mermaid
flowchart TD
    SS([FWD_SOFTSTART]) --> SEED["คำนวณ seedDuty<br/>~45% ของเป้า CC · min 60 · max 460"]
    SEED --> UPSS{"!freezeDutyUp และ<br/>duty &lt; seedDuty?"}
    UPSS -->|ใช่| STEPSS["duty += 0.8 / tick"]
    UPSS -->|ไม่| CAP
    STEPSS --> CAP["duty ≤ seedDuty"]
    CAP --> RDY{"duty ≥ 55%·seed และ I≥0.25A<br/>หรือ ครบ 5000 ms?"}
    RDY -->|ไม่| SS
    RDY -->|ใช่| CC([FWD_CC])
```

## 5) CC → CV

```mermaid
flowchart TD
    CC([FWD_CC]) --> IREF["iRef = 3.0 A"]
    IREF --> SAG{"v_ac_in &lt; 100?"}
    SAG -->|ใช่| IREF2["ลด iRef ตาม sag ×0.15…1"]
    SAG -->|ไม่| TAP
    IREF2 --> TAP{"Vbat_filt ≥ 54.70?"}
    TAP -->|ใช่| IREF3["taper iRef ลงหา 55.90"]
    TAP -->|ไม่| ERR
    IREF3 --> ERR["iErr = iRef − Ibat_filt"]
    ERR --> BAND{"|iErr| เทียบแบนด์ ±0.10 A"}
    BAND -->|iErr &gt; 0.10 และ !freeze| UPCC["duty += 0.6<br/>หรือ +1.2 ถ้า iErr &gt; 0.60"]
    BAND -->|iErr &lt; −0.10| DNCC["duty −= 1.5 หรือ 0.6 fine"]
    BAND -->|ในแบนด์| HOLDCC[hold duty]
    UPCC --> CVENT
    DNCC --> CVENT
    HOLDCC --> CVENT
    CVENT{"max(V,Vf) ≥ 55.30?"}
    CVENT -->|ใช่| CV([FWD_CV] force)
    CVENT -->|ไม่| CVENT2{"max(V,Vf) ≥ 55.00 นาน ≥ 200 ms?"}
    CVENT2 -->|ใช่| CV
    CVENT2 -->|ไม่| CC
```

## 6) CV → FULL / กลับ CC

```mermaid
flowchart TD
    CV([FWD_CV] เป้า 55.90 V) --> VERR["vErr = 55.90 − Vf<br/>vPeak = max(V,Vf)"]
    VERR --> OVER{"vPeak &gt; 55.90+0.25?"}
    OVER -->|ใช่| DNOVER["duty −= 1.50 + over×1.5"]
    OVER -->|ไม่| LOW{"vErr &gt; +0.05?"}
    LOW -->|ใช่| CLIMB{"!freeze และ Iabs &lt; 3A?"}
    CLIMB -->|I≤0.40 และ vErr&lt;0.60| FREEZECV[freeze duty-up ใกล้เต็ม]
    CLIMB -->|ใช่ ปีน| UPCV["duty += 0.40 หรือ 0.20 near"]
    CLIMB -->|I สูงเกิน| DNFINE["duty −= 0.35"]
    LOW -->|vErr &lt; -0.05| DNCV["duty -= over/fine/near"]
    LOW -->|ใน hold ±0.05| HOLDCV[hold = const-V]
    DNOVER --> EXITQ
    FREEZECV --> EXITQ
    UPCV --> EXITQ
    DNFINE --> EXITQ
    DNCV --> EXITQ
    HOLDCV --> EXITQ
    EXITQ{"Vf ≤ 54.50 นาน ≥ 5 s?"}
    EXITQ -->|ใช่| BACKCC([กลับ FWD_CC])
    EXITQ -->|ไม่| FULLQ
    FULLQ{"FULL slow: max(V,Vf)≥55.80 และ min(I)≤0.50<br/>นาน 15 s<br/>หรือ FULL fast: max≥55.90 และ Iabs≤0.35 นาน 5 s?"}
    FULLQ -->|ใช่| DONE["FWD_DONE + charge_full_hold<br/>PWM/relay off<br/>[FULL] FORWARD CV"]
    FULLQ -->|ไม่| CV
    DONE --> HOLD([FULL HOLD])
    HOLD --> RESUME{"Vf ≤ 54.0 และ AC≥95?"}
    RESUME -->|ใช่| BACKCC2([กลับ FWD_CC ชาร์จต่อ])
    RESUME -->|ไม่| HOLD
```

## 7) Outer safety ทุก tick (หลัง SoftStart/CC/CV)

```mermaid
flowchart TD
    SAFE([หลังเฟสควบคุม]) --> OC1{"Ibat &gt; 3.25 และไม่ใช่ CV?"}
    OC1 -->|ใช่| CUT1[duty ตัดตาม I]
    OC1 --> OC2
    CUT1 --> OC2
    OC2{"|Iac| &gt; 2.5 A?"}
    OC2 -->|ใช่| CUT2["duty −= 2 · [OC] FORWARD AC"]
    OC2 --> OC3
    CUT2 --> OC3
    OC3{"Ibat &gt; 3.75 A?"}
    OC3 -->|ใช่| CUT3["duty −= 5 หรือ 1 ใน CV · [OC] BAT"]
    OC3 --> OVPO
    CUT3 --> OVPO
    OVPO[ตัด duty ถ้า V สูงเกินเป้า<br/>แรงนอก CV / ละเอียดใน CV]
    OVPO --> BMS{"V ≥ 55.95?"}
    BMS -->|CV และ V≥56.80| CAP["cap duty ≤ 140 / อาจ ≤80"]
    BMS -->|ไม่ใช่ CV| CAP2[cap duty ≤ 140]
    BMS --> CLAMP
    CAP --> CLAMP
    CAP2 --> CLAMP
    CLAMP["duty = clamp 0…460<br/>ledcWrite FORWARD"]
```

## ค่าคงที่อ้างอิงจากโค้ด

| รายการ | ค่า |
|--------|-----|
| CC | 3.0 A |
| CV | 55.90 V |
| Dmax | 460 |
| SoftStart | seed ≈45%, step +0.8, สูงสุด 5 s |
| CC→CV | entry 55.00 (200 ms) / force 55.30 |
| FULL | 55.80+I≤0.5 @15 s หรือ 55.90+Iabs≤0.35 @5 s |
| STOP ค้าง | 350 ms |
| AC shutdown | &lt;95 V ค้าง 15 s |
| freeze climb | AC&lt;115 และ duty&gt;40 |
