# Exact replacement guide for your `cv58-stability-v5` code

เอกสารนี้ผูกกับโค้ดที่คุณส่งล่าสุดโดยตรง (โครงเดิมครบ)  
เป้าหมายคือเปลี่ยนเฉพาะ **ฝั่ง BOOST** ให้เป็นคอนโทรลใหม่ `SOFTSTART -> CC_MPPT -> CV -> DONE`

---

## 1) แก้ค่าคงที่หลัก

ค้นหาบรรทัดเดิม:

```cpp
const float TARGET_CC_CURRENT = 3.0;
```

แทนเป็น:

```cpp
const float TARGET_CC_CURRENT = 6.0;
```

ค้นหาค่าชุดนี้แล้วแทน:

```cpp
const float FULL_END_CURRENT = 0.12;
const unsigned long FULL_CONFIRM_MS = 90000;
const float RESTART_CHARGE_VOLTAGE = 55.6;
```

เป็น:

```cpp
const float FULL_END_CURRENT = 0.45;
const unsigned long FULL_CONFIRM_MS = 120000;
const float RESTART_CHARGE_VOLTAGE = 54.0;
```

ค้นหาค่าชุด CV เดิม:

```cpp
const int BOOST_START_DUTY_NEAR_FULL_CAP_RAW = 70;
const float BOOST_CV_ENTRY_VOLTAGE = 57.2;
const float BOOST_CV_EXIT_VOLTAGE = 56.8;
```

แทนเป็น:

```cpp
const int BOOST_START_DUTY_NEAR_FULL_CAP_RAW = 280;
const float BOOST_CV_ENTRY_VOLTAGE = 57.8;
const float BOOST_CV_EXIT_VOLTAGE = 57.4;
```

---

## 2) เพิ่ม enum + ตัวแปรใหม่ (วางใต้ `boostMode`)

```cpp
enum BoostNewMode { BOOST_NEW_SOFTSTART, BOOST_NEW_CC_MPPT, BOOST_NEW_CV, BOOST_NEW_DONE };
volatile BoostNewMode boostNewMode = BOOST_NEW_SOFTSTART;

float boostNewCurrIntegrator = 0.0f;
float boostNewVoltIntegrator = 0.0f;
float boostNewPvRef = 42.3f;
float boostNewLastPower = 0.0f;
float boostNewLastVpv = 0.0f;
int boostNewMpptDir = 1;
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
const float BOOST_NEW_CURR_OUT_MAX = 30.0f;
const float BOOST_NEW_VOLT_OUT_MIN = 0.0f;
const float BOOST_NEW_VOLT_OUT_MAX = TARGET_CC_CURRENT;
```

---

## 3) เพิ่ม helper functions (วางก่อน `setup()`)

```cpp
static inline float boostClampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline float boostApplySlew(float target, float current, float upStep, float downStep) {
    float d = target - current;
    if (d > upStep) return current + upStep;
    if (d < -downStep) return current - downStep;
    return target;
}

static inline float boostRunPI(float err, float kp, float ki, float dt,
                               float *integ, float outMin, float outMax) {
    float p = kp * err;
    float iCandidate = *integ + (ki * err * dt);
    float out = p + iCandidate;
    if (out > outMax) out = outMax;
    else if (out < outMin) out = outMin;
    else *integ = iCandidate;  // anti-windup
    return out;
}

static inline int boostEstimateDutyRaw(float vin, float vout, int maxDuty) {
    float vinUse = (vin > 38.0f) ? vin : 38.0f;
    float voutUse = (vout > (vinUse + 2.0f)) ? vout : (vinUse + 2.0f);
    float d = 1.0f - (vinUse / voutUse);
    d += 0.03f;  // margin loss
    int raw = (int)roundf(d * 1023.0f);
    return constrain(raw, 20, maxDuty);
}

static inline void boostNewResetOnEntry(float vpvNow) {
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

## 4) ตอนเข้า STATE_BOOST ให้รีเซ็ตคอนโทรลใหม่

ในบล็อก:

```cpp
if (v_solar >= MIN_PV_VOLTAGE) {
   ...
   currentState = STATE_BOOST;
   ...
}
```

หลังจากตั้งค่าเริ่มต้นเดิม ให้เพิ่ม:

```cpp
boostNewResetOnEntry(v_solar);
```

---

## 5) แทนที่โค้ดส่วน BOOST คอนโทรลใหม่

ค้นหาใน `TaskSampleData` บล็อกนี้:

```cpp
else if (currentState == STATE_BOOST) {
    // =================================================================
    // ☀️ โหมด PV: แยก 3 ช่วง RAMP / MPPT / CV_HOLD
    // =================================================================
    ...
}
```

ให้แทนทั้งบล็อกด้วยโค้ดนี้:

```cpp
else if (currentState == STATE_BOOST) {
    const float dt = 0.02f;  // 20ms loop

    if (v_solar <= 0.0f) {
        duty_accumulator = 0.0f;
    } else if (boostNewMode == BOOST_NEW_SOFTSTART) {
        int seed = boostEstimateDutyRaw(v_solar, BOOST_CV_TARGET_VOLTAGE, allowed_max_duty);
        duty_accumulator = boostApplySlew((float)seed, duty_accumulator, 2.0f, 5.0f);
        if (i_bat_charge_filt > 0.4f || fabsf((float)seed - duty_accumulator) < 2.5f) {
            boostNewMode = BOOST_NEW_CC_MPPT;
            boostNewCurrIntegrator = 0.0f;
        }
    } else if (boostNewMode == BOOST_NEW_CC_MPPT) {
        if (now - boostNewLastMpptMs >= 100) {
            boostNewLastMpptMs = now;
            float pPv = v_solar * i_solar_mag;
            float dP = pPv - boostNewLastPower;
            float dV = v_solar - boostNewLastVpv;

            if (fabsf(dP) > 0.2f) {
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
    } else if (boostNewMode == BOOST_NEW_CV) {
        if (now - boostNewLastMpptMs >= 100) {
            boostNewLastMpptMs = now;
            float pPv = v_solar * i_solar_mag;
            boostNewPAvailFilt = (boostNewPAvailFilt <= 0.01f) ? pPv : (0.22f * pPv + 0.78f * boostNewPAvailFilt);
            float pvErr = v_solar - boostNewPvRef;
            boostNewIrefMppt += 0.06f * pvErr;
            boostNewIrefMppt = boostClampf(boostNewIrefMppt, 0.0f, TARGET_CC_CURRENT);
        }

        float vErr = BOOST_CV_TARGET_VOLTAGE - v_bat_filt;
        float iReq = boostRunPI(vErr, BOOST_NEW_VOLT_KP, BOOST_NEW_VOLT_KI, dt,
                                &boostNewVoltIntegrator, BOOST_NEW_VOLT_OUT_MIN, BOOST_NEW_VOLT_OUT_MAX);

        float pAvail = min(boostNewPAvailFilt, 650.0f);
        float iRefPower = (v_bat_filt > 5.0f) ? ((pAvail * BOOST_NEW_ETA_EST) / v_bat_filt) : 0.0f;
        float iRef = min(iReq, min(boostNewIrefMppt, iRefPower));
        iRef = boostClampf(iRef, 0.0f, TARGET_CC_CURRENT);

        float iErr = iRef - i_bat_charge_filt;
        float dDuty = boostRunPI(iErr, BOOST_NEW_CURR_KP, BOOST_NEW_CURR_KI, dt,
                                 &boostNewCurrIntegrator, BOOST_NEW_CURR_OUT_MIN, BOOST_NEW_CURR_OUT_MAX);
        float targetDuty = duty_accumulator + dDuty;

        if (v_bat_filt < (BOOST_CV_TARGET_VOLTAGE - 0.2f)) {
            int cvMinDuty = boostEstimateDutyRaw(v_solar, BOOST_CV_TARGET_VOLTAGE, allowed_max_duty);
            if (targetDuty < (float)cvMinDuty) targetDuty = (float)cvMinDuty;
        }

        targetDuty = boostClampf(targetDuty, 0.0f, (float)allowed_max_duty);
        duty_accumulator = boostApplySlew(targetDuty, duty_accumulator, BOOST_NEW_DUTY_SLEW_UP, BOOST_NEW_DUTY_SLEW_DOWN);

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

        bool doneCond = (v_bat_filt >= FULL_DETECT_VOLTAGE) && (i_bat_charge_filt <= FULL_END_CURRENT);
        if (doneCond) {
            if (boostNewDoneMs == 0) boostNewDoneMs = now;
            if (now - boostNewDoneMs >= FULL_CONFIRM_MS) {
                boostNewMode = BOOST_NEW_DONE;
            }
        } else {
            boostNewDoneMs = 0;
        }
    } else {  // BOOST_NEW_DONE
        duty_accumulator = 0.0f;
        if (v_bat_filt <= RESTART_CHARGE_VOLTAGE) {
            boostNewMode = BOOST_NEW_CC_MPPT;
            boostNewCurrIntegrator = 0.0f;
            boostNewVoltIntegrator = 0.0f;
        }
    }

    // guardrails แบบง่ายและไม่ขัด CV
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

## 6) ลบบล็อก guardrail เดิมทั้งก้อน

ลบก้อนนี้ออกทั้งหมด (เพราะชนกับคอนโทรลใหม่):

```cpp
if (currentState == STATE_BOOST) {
    // Guardrail ตอนเริ่มและขณะบูสต์: รักษา I<=3A ...
    ...
}
```

---

## 7) ปรับ debug label

ในส่วน debug เดิม:

```cpp
if (boostMode == BOOST_RAMP) state_label = "BOOST_RAMP";
else if (boostMode == BOOST_MPPT) state_label = "BOOST_MPPT";
else state_label = "BOOST_CV";
```

แทนเป็น:

```cpp
if (boostNewMode == BOOST_NEW_SOFTSTART) state_label = "BOOST_SOFT";
else if (boostNewMode == BOOST_NEW_CC_MPPT) state_label = "BOOST_CCMP";
else if (boostNewMode == BOOST_NEW_CV) state_label = "BOOST_CV";
else state_label = "BOOST_DONE";
```

---

## 8) ค่าที่ต้องเห็นหลังแก้สำเร็จ

- CC phase: `i_bat_charge_filt` เกาะใกล้ `6A`
- เข้า CV: เมื่อ `v_bat_filt >= 57.8V` ต่อเนื่อง 8s
- CV phase: `v_bat_filt` เกาะ `58.0V`, กระแสค่อยๆ ลด
- DONE: เมื่อ `Vbat>=57.95` และ `I<=0.45A` ต่อเนื่อง 120s

