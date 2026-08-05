/**
 * =============================================================================
 *  MPPT Cascade Logic (ศึกษาจาก Simulink)
 *  โครง:  ModINC → ลูปแรงดันแผง → จำกัดกระแส(CC/CV) → ลูปกระแส → Duty
 *
 *  เป้าหมายเครื่องคุณ (16S LiFePO4):
 *    CC = 6.0 A
 *    CV = 56.0 V
 *    แผง ~ Vmp 42.3 V
 *
 *  หมายเหตุ:
 *    - นี่คือโครงลอจิกให้อ่าน/ทดลอง ไม่ใช่ดรอปอินแทนสเก็ตช์ LCD เต็ม
 *    - เซ็นเซอร์ Ipv พัง: ตั้ง PV_CURRENT_SENSOR_OK = false จะประมาณ Ipv จาก Pbat
 * =============================================================================
 */

#include <Arduino.h>
#include <math.h>

// -----------------------------
// ฮาร์ดแวร์ (ปรับตามบอร์ดจริง)
// -----------------------------
static const int PIN_PWM_BOOST = 27;
static const uint32_t PWM_FREQ_HZ = 50000;
static const uint8_t  PWM_RES_BITS = 10;
static const int PWM_RAW_MAX = (1 << PWM_RES_BITS) - 1;  // 1023
static const int DUTY_RAW_MAX = 760;

// -----------------------------
// เป้าชาร์จ
// -----------------------------
static const float CC_A = 6.0f;
static const float CV_V = 56.0f;
static const float CV_ENTER_V = 55.50f;
static const float CV_EXIT_V  = 54.80f;

// -----------------------------
// ขอบเขต MPPT (Vr)
// -----------------------------
static const float VR_INIT = 42.3f;
static const float VR_MIN  = 40.0f;
static const float VR_MAX  = 45.0f;
static const float DELTA_V = 0.08f;     // ก้าวฐาน (ของเปเปอร์เล็กเกินสำหรับฮาร์ดแวร์จริง)
static const float DP_DEADBAND_W = 0.5f;

// เซ็นเซอร์กระแส PV พัง → ใช้ค่าประมาณ
static const bool PV_CURRENT_SENSOR_OK = false;
static const float EFF_EST = 0.90f;

// -----------------------------
// คาบเวลา (ms)
// -----------------------------
static const uint32_t CTRL_MS = 20;    // ลูปแรงดัน+กระแส  50 Hz
static const uint32_t MPPT_MS = 100;   // ModINC ช้ากว่าชั้นใน
static const uint32_t LOG_MS  = 500;

// -----------------------------
// เกนคอนโทรลเลอร์ (รูปแบบเดียวกับ Cvi/Cid: u+= a*e - b*e_prev)
// ค่าเริ่มต้นต้องจูนบนฮาร์ดแวร์จริง — เริ่มอ่อนก่อน
// -----------------------------
// ลูปแรงดันแผง: error = Vr - Vpv  → ออกเป็น I_star (แอมป์)
static const float CVI_A = 0.40f;
static const float CVI_B = 0.38f;
static const float I_STAR_MIN = 0.0f;
static const float I_STAR_MAX = CC_A;

// ลูปกระแส: error = I_max - Ibat → ออกเป็น delta-duty (raw)
static const float CID_A = 8.0f;
static const float CID_B = 7.6f;
static const float DUTY_SLEW_UP = 2.0f;
static const float DUTY_SLEW_DOWN = 3.0f;

// =============================================================================
// สถานะ
// =============================================================================
struct Sensors {
  float vPv;
  float iPv;      // วัดหรือประมาณ
  float iPvRaw;   // ค่าดิบ ADC (ถ้ามี)
  float vBat;
  float iBat;     // |Ibat|
};

static float g_vr = VR_INIT;
static float g_iStar = 0.0f;
static float g_iMax = 0.0f;
static float g_duty = 0.0f;
static bool  g_inCv = false;

// Cvi state
static float g_cviU = 0.0f;
static float g_cviEprev = 0.0f;

// Cid state
static float g_cidU = 0.0f;
static float g_cidEprev = 0.0f;

// ModINC persistent
static float g_vOld = VR_INIT;
static float g_iOld = 0.0f;
static float g_pOld = 0.0f;
static float g_vrOld = VR_INIT;
static bool  g_mpptInit = false;

static uint32_t g_lastCtrl = 0;
static uint32_t g_lastMppt = 0;
static uint32_t g_lastLog = 0;

// =============================================================================
// ยูทิล
// =============================================================================
static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static float maxf(float a, float b) { return (a > b) ? a : b; }
static float minf(float a, float b) { return (a < b) ? a : b; }

static float slew(float target, float cur, float upStep, float downStep) {
  float d = target - cur;
  if (d > upStep) return cur + upStep;
  if (d < -downStep) return cur - downStep;
  return target;
}

static void setDutyRaw(int raw) {
  raw = (int)clampf((float)raw, 0.0f, (float)DUTY_RAW_MAX);
  ledcWrite(PIN_PWM_BOOST, raw);
}

// =============================================================================
// (1) ModINC — แก้บั๊ก Pold ไม่ได้เซฟ + สเกลเข้าแผงคุณ
//     เอาต์พุต: Vr
// =============================================================================
static float modInc(float V, float I, float Vb) {
  if (!g_mpptInit) {
    g_vOld = V;
    g_iOld = I;
    g_pOld = V * I;
    g_vrOld = clampf(V, VR_MIN, VR_MAX);
    g_vr = g_vrOld;
    g_mpptInit = true;
    return g_vr;
  }

  // ใกล้/เกิน CV: หยุดไล่ MPP แรงๆ (แทน Vb>14.4 ในเปเปอร์)
  if (Vb >= (CV_V - 0.3f)) {
    g_vr = g_vrOld;
    g_vOld = V;
    g_iOld = I;
    g_pOld = V * I;
    return g_vr;
  }

  float P = V * I;
  float dP = P - g_pOld;
  float dV = V - g_vOld;
  float dI = I - g_iOld;
  float M = fabsf(dP);

  float Vr = g_vrOld;

  if (M < DP_DEADBAND_W) {
    Vr = g_vrOld;  // กำลังนิ่ง — ค้าง Vr
  } else if (fabsf(dV) < 1e-4f) {
    // dV ≈ 0
    if (fabsf(dI) < 1e-4f) {
      Vr = g_vrOld;
    } else if (dI > 0.0f) {
      Vr = g_vrOld + (M * DELTA_V * 0.02f);
    } else {
      Vr = g_vrOld - (M * DELTA_V * 0.02f);
    }
  } else {
    // Incremental conductance: sign(V*dI + I*dV)
    float inc = V * dI + I * dV;
    if (fabsf(inc) < 1e-4f) {
      Vr = g_vrOld;  // ที่ MPP
    } else if (inc > 0.0f) {
      // ยังอยู่ฝั่งกำลังขึ้นตามทิศ dV
      if (dV > 0.0f) Vr = g_vrOld + (M * DELTA_V * 0.02f);
      else           Vr = g_vrOld - (M * DELTA_V * 0.02f);
    } else {
      // เลย MPP — กลับทิศ
      if (dV > 0.0f) Vr = g_vrOld - DELTA_V;
      else           Vr = g_vrOld + (M * DELTA_V * 0.02f);
    }
  }

  // ถ้าหลุดกรอบ ให้ค้างค่าเดิม (ตามลอจิกเปเปอร์)
  if (Vr >= VR_MAX || Vr <= VR_MIN) {
    Vr = g_vrOld;
  }
  Vr = clampf(Vr, VR_MIN, VR_MAX);

  // ★ สำคัญ: เซฟประวัติ (ใน MATLAB ต้นฉบับลืม Pold=P)
  g_vrOld = Vr;
  g_vOld = V;
  g_iOld = I;
  g_pOld = P;
  g_vr = Vr;
  return Vr;
}

// =============================================================================
// (2) ลูปแรงดันแผง Cvi — รูปแบบเดียวกับ (35z-34.77)/(z-1)
//     u[k] = u[k-1] + A*e[k] - B*e[k-1]
//     หมายเหตุ: ใน Simulink มีคูณ -1 เพราะ duty↑ มักดึง Vpv ลง
//     ที่นี่นิยาม e = Vr - Vpv แล้วให้ I_star เพิ่มเมื่อ Vpv ต่ำกว่า Vr
// =============================================================================
static float runCvi(float Vr, float Vpv) {
  float e = Vr - Vpv;   // ต้องการให้ Vpv ตาม Vr
  g_cviU = g_cviU + CVI_A * e - CVI_B * g_cviEprev;
  g_cviEprev = e;
  g_cviU = clampf(g_cviU, I_STAR_MIN, I_STAR_MAX);
  g_iStar = g_cviU;
  return g_iStar;
}

// =============================================================================
// (3) Battery Charging Current Control (+ ขยายเป็น CV)
//     ต้นฉบับ: IL_max = min(IL_star, IL_ref)
//     ของเรา:  IL_max = min(IL_star, CC, I_cv)
// =============================================================================
static float batteryCurrentLimit(float iStar, float vBat, float iBat) {
  // เข้า/ออก CV
  if (!g_inCv && vBat >= CV_ENTER_V) g_inCv = true;
  if (g_inCv && vBat <= CV_EXIT_V)  g_inCv = false;

  float iCc = CC_A;
  float iCv = CC_A;

  if (g_inCv) {
    // ยิ่งใกล้/เกิน CV ยิ่งลดเพดานกระแส
    float vErr = CV_V - vBat;
    if (vErr <= 0.0f) {
      iCv = 0.2f;  // เกินเป้า — เหลือกระแสจิ๋ว
    } else {
      // เชิงเส้นหยาบ: ไกล 1V → ได้เกือบ CC, ใกล้ 0V → กระแสต่ำ
      iCv = clampf(vErr * 4.0f, 0.3f, CC_A);
    }
  }

  // ลอจิกเดียวกับ MATLAB + ชั้น CV
  float iLim = iCc;
  if (iStar < iLim) iLim = iStar;
  if (iCv < iLim) iLim = iCv;

  // กันกระแทก: ถ้าวัดได้เกิน CC มาก ให้ตัดลง
  if (iBat > (CC_A + 0.4f)) {
    iLim = minf(iLim, CC_A * 0.7f);
  }

  g_iMax = maxf(iLim, 0.0f);
  return g_iMax;
}

// =============================================================================
// (4) ลูปกระแส Cid → duty
//     u[k] = u[k-1] + A*e[k] - B*e[k-1]
// =============================================================================
static float runCid(float iMax, float iBat) {
  float e = iMax - iBat;
  g_cidU = g_cidU + CID_A * e - CID_B * g_cidEprev;
  g_cidEprev = e;
  g_cidU = clampf(g_cidU, 0.0f, (float)DUTY_RAW_MAX);

  g_duty = slew(g_cidU, g_duty, DUTY_SLEW_UP, DUTY_SLEW_DOWN);
  return g_duty;
}

// =============================================================================
// อ่านเซ็นเซอร์ (โครง — ใส่ ADC จริงของคุณตรงนี้)
// =============================================================================
static Sensors readSensorsStub() {
  Sensors s;
  // TODO: แทนที่ด้วย ADS1115 / ตัวกรองจริง
  // ค่าด้านล่างเป็น stub ให้คอมไพล์ดูลอจิกได้
  s.vPv = 43.5f;
  s.iPvRaw = 5.1f;     // ค่าเพี้ยนจากเซ็นเซอร์พัง (ตัวอย่าง)
  s.vBat = 55.7f;
  s.iBat = 1.6f;

  if (PV_CURRENT_SENSOR_OK) {
    s.iPv = s.iPvRaw;
  } else {
    // ประมาณจากสมดุลกำลังบูสต์: Pin ≈ Pout/eff
    if (s.vPv > 38.0f && s.iBat > 0.05f) {
      s.iPv = (s.vBat * s.iBat) / maxf(s.vPv * EFF_EST, 1.0f);
    } else {
      s.iPv = 0.0f;
    }
  }
  return s;
}

// =============================================================================
// setup / loop
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("[BOOT] mppt_cascade_logic_16s");
  Serial.println("[BOOT] Flow: ModINC -> Cvi(Vpv) -> min(CC,CV) -> Cid(Ibat) -> Duty");
  Serial.printf("[BOOT] CC=%.1fA CV=%.1fV PV_SENSOR_OK=%d\n",
                CC_A, CV_V, PV_CURRENT_SENSOR_OK ? 1 : 0);

  ledcAttach(PIN_PWM_BOOST, PWM_FREQ_HZ, PWM_RES_BITS);
  setDutyRaw(0);

  g_lastCtrl = millis();
  g_lastMppt = millis();
  g_lastLog = millis();
}

void loop() {
  uint32_t now = millis();

  // ---------- ชั้นควบคุมเร็ว (Cvi + limit + Cid) ----------
  if (now - g_lastCtrl >= CTRL_MS) {
    g_lastCtrl = now;
    Sensors s = readSensorsStub();

    // ---------- ชั้น MPPT ช้า ----------
    if (now - g_lastMppt >= MPPT_MS) {
      g_lastMppt = now;
      modInc(s.vPv, s.iPv, s.vBat);
    }

    float iStar = runCvi(g_vr, s.vPv);
    float iMax  = batteryCurrentLimit(iStar, s.vBat, s.iBat);
    float duty  = runCid(iMax, s.iBat);
    setDutyRaw((int)lroundf(duty));

    if (now - g_lastLog >= LOG_MS) {
      g_lastLog = now;
      Serial.println("=========================================================================================");
      Serial.printf("Vr=%.2f Vpv=%.2f | Istar=%.2f Imax=%.2f Ibat=%.2f | CV=%d Duty=%.0f (%.0f%%)\n",
                    g_vr, s.vPv, iStar, iMax, s.iBat,
                    g_inCv ? 1 : 0, duty, 100.0f * duty / 1023.0f);
      Serial.printf("Ipv=%.2fA (%s raw=%.2f) Ppv≈%.1fW Pbat≈%.1fW\n",
                    s.iPv,
                    PV_CURRENT_SENSOR_OK ? "SENSOR" : "EST",
                    s.iPvRaw,
                    s.vPv * s.iPv,
                    s.vBat * s.iBat);
      Serial.println("=========================================================================================");
    }
  }
}
