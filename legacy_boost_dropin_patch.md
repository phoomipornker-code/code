# Legacy code migration: replace BOOST section with new MPPT->CC->CV control

ไฟล์นี้คือชุดโค้ดสำหรับย้ายคอนโทรลใหม่เข้าโค้ดเดิมของคุณ **เฉพาะส่วน BOOST** โดยไม่ต้องรื้อส่วน AC/LCD

> เป้าหมาย: 16S LiFePO4, CC 6A, CV 58.0V, PWM 50kHz

---

## 1) แก้ค่าคงที่หลักในโค้ดเดิม

```cpp
// เดิม 3.0A
const float TARGET_CC_CURRENT = 6.0;
const float TARGET_CV_VOLTAGE = 58.0;

// ปรับให้เข้ากับโหมด CV ใหม่
const float BOOST_CV_ENTRY_VOLTAGE = 57.8;
const float BOOST_CV_EXIT_VOLTAGE  = 57.4;
const float BOOST_CV_TARGET_VOLTAGE = 58.0;

// ตัดเมื่อใกล้เต็ม (CV taper)
const float FULL_DETECT_VOLTAGE = 57.95;
const float FULL_END_CURRENT = 0.45;
const unsigned long FULL_CONFIRM_MS = 120000;
```

---

## 2) เพิ่ม enum + ตัวแปรใหม่ (วางใน global section)

```cpp
enum BoostNewMode { BOOST_NEW_SOFTSTART, BOOST_NEW_CC_MPPT, BOOST_NEW_CV, BOOST_NEW_DONE };
volatile BoostNewMode boostNewMode = BOOST_NEW_SOFTSTART;

float boostNewCurrIntegrator = 0.0f;
float boostNewVoltIntegrator = 0.0f;
float boostNewPvRef = 42.3f;
float boostNewLastPower = 0.0f;
float boostNewLastVpv = 0.0f;
int   boostNewMpptDir = 1;
float boostNewIrefMppt = 1.0f;
float boostNewPAvailFilt = 0.0f;

unsigned long boostNewLastMpptMs = 0;
unsigned long boostNewCvEnterMs = 0;
unsigned long boostNewCvExitMs = 0;
unsigned long boostNewDoneMs = 0;

const float BOOST_NEW_MPPT_STEP_V = 0.10f;
const float BOOST_NEW_ETA_EST = 0.90f;
const float BOOST_NEW_DUTY_SLEW_UP = 2.2f;
const float BOOST_NEW_DUTY_SLEW_DOWN = 5.0f;

const float BOOST_NEW_CURR_KP = 12.0f;
const float BOOST_NEW_CURR_KI = 40.0f;
const float BOOST_NEW_VOLT_KP = 1.2f;
const float BOOST_NEW_VOLT_KI = 0.9f;

const float BOOST_NEW_CURR_OUT_MIN = -30.0f;
const float BOOST_NEW_CURR_OUT_MAX =  30.0f;
const float BOOST_NEW_VOLT_OUT_MIN =   0.0f;
const float BOOST_NEW_VOLT_OUT_MAX = TARGET_CC_CURRENT;
```

---

## 3) เพิ่ม helper functions (วางก่อน `TaskSampleData`)

```cpp
static inline float boostClampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline int boostEstimateDutyRaw(float vin, float vout, int maxDuty) {
    float vinUse = (vin > 38.0f) ? vin : 38.0f;
    float voutUse = (vout > vinUse + 2.0f) ? vout : (vinUse + 2.0f);
    float d = 1.0f - (vinUse / voutUse);
    d += 0.03f;  // margin for losses
    int raw = (int)roundf(d * 1023.0f);
    return constrain(raw, 20, maxDuty);
}

static inline float boostRunPI(float err, float kp, float ki, float dt, float *integ, float outMin, float outMax) {
    float p = kp * err;
    float iCandidate = *integ + (ki * err * dt);
    float out = p + iCandidate;
    if (out > outMax) out = outMax;
    else if (out < outMin) out = outMin;
    else *integ = iCandidate;  // anti-windup
    return out;
}

static inline float boostApplySlew(float target, float current, float upStep, float downStep) {
    float d = target - current;
    if (d > upStep) return current + upStep;
    if (d < -downStep) return current - downStep;
    return target;
}

static void boostNewResetOnEntry(float vpvNow) {
    boostNewMode = BOOST_NEW_SOFTSTART;
    boostNewCurrIntegrator = 0.0f;
    boostNewVoltIntegrator = 0.0f;
    boostNewPvRef = (vpvNow > 40.0f) ? vpvNow : 42.3f;
    boostNewLastPower = 0.0f;
    boostNewLastVpv = vpvNow;
    boostNewMpptDir = 1;
    boostNewIrefMppt = 1.0f;
    boostNewPAvailFilt = 0.0f;
    boostNewLastMpptMs = 0;
    boostNewCvEnterMs = 0;
    boostNewCvExitMs = 0;
    boostNewDoneMs = 0;
}
```

---

## 4) ตอนเข้า `STATE_BOOST` ให้รีเซ็ตคอนโทรลใหม่

ในบล็อกที่สลับเข้า `STATE_BOOST` (จุดที่คุณตั้ง `currentState = STATE_BOOST;`)  
เพิ่มโค้ดนี้ต่อท้าย:

```cpp
boostNewResetOnEntry(v_solar);
```

---

## 5) แทนที่บล็อกควบคุม BOOST เดิม

ใน `TaskSampleData` ให้แทนที่บล็อกเดิม:

```cpp
else if (currentState == STATE_BOOST) {
    // ...โค้ดเดิม RAMP/MPPT/CV_HOLD...
}
```

ด้วยบล็อกนี้:

```cpp
else if (currentState == STATE_BOOST) {
    const float dt = 0.02f; // loop 20ms

    // ---------- state machine ----------
    if (boostNewMode == BOOST_NEW_SOFTSTART) {
        int seed = boostEstimateDutyRaw(v_solar, BOOST_CV_TARGET_VOLTAGE, allowed_max_duty);
        float target = (float)seed;
        duty_accumulator = boostApplySlew(target, duty_accumulator, 2.0f, 5.0f);

        bool rampDone = (i_bat_charge_filt > 0.4f) || (duty_accumulator >= (target - 2.0f));
        if (rampDone) {
            boostNewMode = BOOST_NEW_CC_MPPT;
            boostNewCurrIntegrator = 0.0f;
        }
    }
    else if (boostNewMode == BOOST_NEW_CC_MPPT) {
        // MPPT task @100ms
        if (now - boostNewLastMpptMs >= 100) {
            boostNewLastMpptMs = now;
            float pPv = v_solar * i_solar_mag;
            float dP = pPv - boostNewLastPower;
            float dV = v_solar - boostNewLastVpv;

            if (fabs(dP) > 0.2f) {
                if (dP > 0.0f) boostNewMpptDir = (dV >= 0.0f) ? 1 : -1;
                else           boostNewMpptDir = (dV >= 0.0f) ? -1 : 1;
            }
            boostNewPvRef += (float)boostNewMpptDir * BOOST_NEW_MPPT_STEP_V;
            boostNewPvRef = boostClampf(boostNewPvRef, 40.0f, 45.0f);

            float pvErr = v_solar - boostNewPvRef;
            boostNewIrefMppt += 0.08f * pvErr;
            boostNewIrefMppt = boostClampf(boostNewIrefMppt, 0.0f, TARGET_CC_CURRENT);

            boostNewPAvailFilt = (boostNewPAvailFilt <= 0.01f) ? pPv : (0.22f * pPv + 0.78f * boostNewPAvailFilt);
            boostNewLastPower = pPv;
            boostNewLastVpv = v_solar;
        }

        float pAvail = min(boostNewPAvailFilt, 650.0f);
        float iRefPower = (v_bat_filt > 5.0f) ? ((pAvail * BOOST_NEW_ETA_EST) / v_bat_filt) : 0.0f;
        float iRef = min(TARGET_CC_CURRENT, min(boostNewIrefMppt, iRefPower));
        iRef = boostClampf(iRef, 0.0f, TARGET_CC_CURRENT);

        float iErr = iRef - i_bat_charge_filt;
        float dDuty = boostRunPI(iErr, BOOST_NEW_CURR_KP, BOOST_NEW_CURR_KI, dt,
                                 &boostNewCurrIntegrator, BOOST_NEW_CURR_OUT_MIN, BOOST_NEW_CURR_OUT_MAX);
        float targetDuty = duty_accumulator + dDuty;
        targetDuty = boostClampf(targetDuty, 0.0f, (float)allowed_max_duty);
        duty_accumulator = boostApplySlew(targetDuty, duty_accumulator, BOOST_NEW_DUTY_SLEW_UP, BOOST_NEW_DUTY_SLEW_DOWN);

        // CC -> CV
        if (v_bat_filt >= BOOST_CV_ENTRY_VOLTAGE) {
            if (boostNewCvEnterMs == 0) boostNewCvEnterMs = now;
            if (now - boostNewCvEnterMs >= 8000) {
                boostNewMode = BOOST_NEW_CV;
                boostNewCurrIntegrator = 0.0f;
                boostNewVoltIntegrator = 0.0f;
            }
        } else {
            boostNewCvEnterMs = 0;
        }
    }
    else if (boostNewMode == BOOST_NEW_CV) {
        // keep MPPT cap update
        if (now - boostNewLastMpptMs >= 100) {
            boostNewLastMpptMs = now;
            float pPv = v_solar * i_solar_mag;
            boostNewPAvailFilt = (boostNewPAvailFilt <= 0.01f) ? pPv : (0.22f * pPv + 0.78f * boostNewPAvailFilt);
            float pvErr = v_solar - boostNewPvRef;
            boostNewIrefMppt += 0.06f * pvErr;
            boostNewIrefMppt = boostClampf(boostNewIrefMppt, 0.0f, TARGET_CC_CURRENT);
        }

        // voltage PI -> current ref
        float vErr = BOOST_CV_TARGET_VOLTAGE - v_bat_filt;
        float iReq = boostRunPI(vErr, BOOST_NEW_VOLT_KP, BOOST_NEW_VOLT_KI, dt,
                                &boostNewVoltIntegrator, BOOST_NEW_VOLT_OUT_MIN, BOOST_NEW_VOLT_OUT_MAX);

        float pAvail = min(boostNewPAvailFilt, 650.0f);
        float iRefPower = (v_bat_filt > 5.0f) ? ((pAvail * BOOST_NEW_ETA_EST) / v_bat_filt) : 0.0f;
        float iRef = min(iReq, min(boostNewIrefMppt, iRefPower));
        iRef = boostClampf(iRef, 0.0f, TARGET_CC_CURRENT);

        // current PI -> duty
        float iErr = iRef - i_bat_charge_filt;
        float dDuty = boostRunPI(iErr, BOOST_NEW_CURR_KP, BOOST_NEW_CURR_KI, dt,
                                 &boostNewCurrIntegrator, BOOST_NEW_CURR_OUT_MIN, BOOST_NEW_CURR_OUT_MAX);
        float targetDuty = duty_accumulator + dDuty;

        // minimum duty from boost physics when still below CV
        if (v_bat_filt < (BOOST_CV_TARGET_VOLTAGE - 0.2f)) {
            int cvMinDuty = boostEstimateDutyRaw(v_solar, BOOST_CV_TARGET_VOLTAGE, allowed_max_duty);
            if (targetDuty < (float)cvMinDuty) targetDuty = (float)cvMinDuty;
        }

        targetDuty = boostClampf(targetDuty, 0.0f, (float)allowed_max_duty);
        duty_accumulator = boostApplySlew(targetDuty, duty_accumulator, BOOST_NEW_DUTY_SLEW_UP, BOOST_NEW_DUTY_SLEW_DOWN);

        // CV -> CC fallback
        if (v_bat_filt <= BOOST_CV_EXIT_VOLTAGE && i_bat_charge_filt < (TARGET_CC_CURRENT - 0.6f)) {
            if (boostNewCvExitMs == 0) boostNewCvExitMs = now;
            if (now - boostNewCvExitMs >= 3000) {
                boostNewMode = BOOST_NEW_CC_MPPT;
                boostNewCurrIntegrator = 0.0f;
                boostNewVoltIntegrator = 0.0f;
            }
        } else {
            boostNewCvExitMs = 0;
        }

        // charge done
        bool doneCond = (v_bat_filt >= 57.95f) && (i_bat_charge_filt <= 0.45f);
        if (doneCond) {
            if (boostNewDoneMs == 0) boostNewDoneMs = now;
            if (now - boostNewDoneMs >= 120000) {
                boostNewMode = BOOST_NEW_DONE;
            }
        } else {
            boostNewDoneMs = 0;
        }
    }
    else { // BOOST_NEW_DONE
        duty_accumulator = 0.0f;
        if (v_bat_filt <= RESTART_CHARGE_VOLTAGE) {
            boostNewMode = BOOST_NEW_CC_MPPT;
            boostNewCurrIntegrator = 0.0f;
            boostNewVoltIntegrator = 0.0f;
        }
    }

    // ---------- shared guardrails ----------
    if (i_bat_charge_filt > (TARGET_CC_CURRENT + 0.25f)) {
        duty_accumulator -= (2.0f + (i_bat_charge_filt - TARGET_CC_CURRENT) * 3.0f);
        boostNewCurrIntegrator *= 0.8f;
    }
    if (i_solar_mag > 16.0f) {
        duty_accumulator -= 5.0f;
    }
    if (v_solar < 40.5f) {
        duty_accumulator -= (2.0f + (40.5f - v_solar) * 2.5f);
    }
    if (v_bat_filt > 58.15f) {
        duty_accumulator -= (2.5f + (v_bat_filt - 58.15f) * 7.5f);
        boostNewCurrIntegrator = 0.0f;
    }

    duty_accumulator = boostClampf(duty_accumulator, 0.0f, (float)allowed_max_duty);
}
```

---

## 6) ลบ/ปิด guardrail เก่าทั้งก้อน

ให้ลบหรือคอมเมนต์บล็อกเดิมนี้ออกทั้งก้อน (เพราะจะชนกับคอนโทรลใหม่):

```cpp
// Guardrail ตอนเริ่มและขณะบูสต์: รักษา I<=3A, Vpv>=42V ...
if (currentState == STATE_BOOST) {
   ...
}
```

ถ้าไม่ลบ โค้ดเดิมจะดึง duty ลงจน CV ไม่ขึ้นเหมือนเดิม

---

## 7) แนะนำ log สำหรับเช็คว่า CV ทำงานจริง

พิมพ์ตัวแปรเพิ่มใน debug:

- `boostNewMode`
- `duty_accumulator`
- `v_bat_filt`
- `i_bat_charge_filt`
- `boostNewIrefMppt`

พฤติกรรมที่ถูกต้อง:

- ช่วง CC กระแสวิ่งใกล้ 6A
- เข้า CV แถว 57.8V แล้วแรงดันนิ่งใกล้ 58.0V
- กระแสค่อยๆ ลดลง
- ครบเงื่อนไขแล้วค่อยเข้า DONE

