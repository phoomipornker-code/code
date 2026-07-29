# Forward flowchart รวม — `cv58-boost-v14-forward-v81`

ตามโค้ดจริง · ไม่ใช้ PID · SoftStart → CC → CV → DONE  
เขียนแบบ **Sequence / Selection / Iteration** · Tick ≈ 20 ms · Dmax = 460

```mermaid
flowchart TD
    START([เริ่ม FORWARD]) --> SS_SEED[SoftStart<br/>คำนวณ seedDuty ≈ 45% ของ CC<br/>min 60 · ≤ 460]

    SS_SEED --> SS_FR{freezeDutyUp?}
    SS_FR -- Y --> SS_HOLD[ไม่เพิ่ม duty]
    SS_FR -- N --> SS_LT{duty &lt; seed?}
    SS_LT -- Y --> SS_UP[duty += 0.8]
    SS_LT -- N --> SS_HOLD
    SS_UP --> SS_HOLD
    SS_HOLD --> SS_RDY{duty ≥ seed×0.55<br/>และ Ibat ≥ 0.25<br/>หรือ ครบ 5 s?}
    SS_RDY -- N --> SS_SEED
    SS_RDY -- Y --> CC_IREF

    CC_IREF[CC<br/>iRef = 3.0 A] --> CC_AC{AC &lt; 100?}
    CC_AC -- Y --> CC_SAG[ลด iRef ตาม sag]
    CC_AC -- N --> CC_TAP
    CC_SAG --> CC_TAP{Vf ≥ 54.70?}
    CC_TAP -- Y --> CC_TP[taper iRef ลงหา 55.90]
    CC_TAP -- N --> CC_ERR
    CC_TP --> CC_ERR[iErr = iRef − Ibat]
    CC_ERR --> CC_ADJ{iErr?}
    CC_ADJ --|&gt; +0.10| CC_UP{freezeDutyUp?}
    CC_UP -- N --> CC_ADD[duty += 0.6 หรือ 1.2]
    CC_UP -- Y --> CC_CVQ
    CC_ADD --> CC_CVQ
    CC_ADJ --|&lt; −0.10| CC_DN[duty −= 0.6 หรือ 1.5]
    CC_DN --> CC_CVQ
    CC_ADJ -->|ใน ±0.10| CC_HLD[hold duty]
    CC_HLD --> CC_CVQ
    CC_CVQ{max V ≥ 55.30<br/>หรือ ≥ 55.00 นาน 200 ms?}
    CC_CVQ -- N --> CC_IREF
    CC_CVQ -- Y --> CV_ERR

    CV_ERR[CV เป้า 55.90 V<br/>vErr = 55.90 − Vf] --> CV_ADJ{vErr?}
    CV_ADJ --|&gt; +0.05| CV_CLIMB{freeze หรือ<br/>I ต่ำใกล้เต็ม?}
    CV_CLIMB -- Y --> CV_HOLD[hold ไม่ปีน duty]
    CV_CLIMB -- N --> CV_UP[duty += 0.40 หรือ 0.20]
    CV_ADJ -->|ใน ±0.05| CV_HOLD2[hold = คงแรงดัน]
    CV_ADJ --|&lt; −0.05| CV_DN[ลด duty]
    CV_HOLD --> CV_EXIT
    CV_UP --> CV_EXIT
    CV_HOLD2 --> CV_EXIT
    CV_DN --> CV_EXIT
    CV_EXIT{Vf ≤ 54.50 นาน ≥ 5 s?}
    CV_EXIT -- Y --> CC_IREF
    CV_EXIT -- N --> CV_FULL{แบตเต็ม?}
    CV_FULL -- N --> CV_ERR
    CV_FULL -- Y slow<br/>maxV≥55.80 · minI≤0.50 · 15s --> DONE
    CV_FULL -- Y fast<br/>maxV≥55.90 · Iabs≤0.35 · 5s --> DONE

    DONE([DONE / FULL<br/>duty = 0 · ปิด PWM]) --> DONE_Q{Vf ≤ 54.0<br/>และ AC ≥ 95?}
    DONE_Q -- N --> DONE
    DONE_Q -- Y --> CC_IREF
```

## freezeDutyUp (ห้ามเพิ่ม duty)

- AC จริงอ่อน (&lt; 95 หรือ collapsing) และไม่ใช่ glitch ตอน I ยังไหล  
- **หรือ** AC &lt; 115 และ `raw_duty` &gt; 40

## ค่าสำคัญ

| รายการ | ค่า |
|--------|-----|
| CC | 3.0 A |
| CV | 55.90 V |
| SoftStart | +0.8 / timeout 5 s |
| CC hold | ±0.10 A |
| CV hold | ±0.05 V |
| เข้า CV | 55.00 (200 ms) / force 55.30 |
| ออก CV → CC | 54.50 นาน 5 s |
| Dmax | 460 |
