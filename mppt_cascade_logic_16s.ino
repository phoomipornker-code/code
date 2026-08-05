/**
 * =============================================================================
 *  MPPT Cascade — โครงสร้างตรงแผนภาพ Simulink
 *
 *  Vpv,Ipv ──┐
 *  Ipv ───────┼──► ModINC ──► Vr (= Vpv,ref)
 *  Vb ────────┘
 *                   │
 *                   ▼
 *         Hv=1/5    Vr
 *         Hv=1/5    Vpv
 *              e_v = Hv*(Vr - Vpv)
 *                   │
 *                   ▼
 *              Cvi(z) = (35z - 34.77)/(z - 1)
 *                   │
 *                   ▼
 *                 ×(-1) ──► IL_star
 *                   │
 *                   ▼
 *   Battery_Charging_Current_Control(IL_ref, IL_star) ──► IL_max
 *                   │
 *                   ▼
 *         Hi=1/25   IL_max
 *         Hi=1/25   iL
 *              e_i = Hi*(IL_max - iL)
 *                   │
 *                   ▼
 *              Cid(z) = (5.5z - 5.413)/(z - 1)
 *                   │
 *                   ▼
 *                   d  (duty / PWM)
 *
 *  หมายเหตุสเกล:
 *    - ค่า Hv/Hi/Cvi/Cid มาจากโมเดลในรูป (แพ็กเล็ก ~12V, IL_ref=12)
 *    - เครื่องคุณ 16S: ใช้ USE_FIELD_16S=true จะเปลี่ยนเฉพาะขอบเขต Vr / IL_ref / CV
 *      แต่เกน Cvi/Cid ยังต้องจูนบนฮาร์ดจริง (อย่าคาดหวังเลขเปเปอร์เสถียรทันที)
 * =============================================================================
 */

#include <Arduino.h>
#include <math.h>

// =============================================================================
// โปรไฟล์
// =============================================================================
// false = ตัวเลขตามแผนภาพเปเปอร์, true = ขอบเขตเข้าเครื่อง 16S ของคุณ
static const bool USE_FIELD_16S = true;

// เซ็นเซอร์ Ipv พัง → ประมาณจากกำลังแบต
static const bool PV_CURRENT_SENSOR_OK = false;
static const float EFF_EST = 0.90f;

// -----------------------------
// PWM
// -----------------------------
static const int PIN_PWM_BOOST = 27;
static const uint32_t PWM_FREQ_HZ = 50000;
static const uint8_t  PWM_RES_BITS = 10;
static const int PWM_RAW_MAX = 1023;
static const int DUTY_RAW_MAX = 760;   // เผื่อฮาร์ดแวร์; ในซิมพอร์ต d เป็น 0..1

// -----------------------------
// สเกลตามแผนภาพ
// -----------------------------
static const float Hv = 1.0f / 5.0f;     // บล็อก Hv
static const float Hi = 1.0f / 25.0f;    // บล็อก Hi

// Cvi(z) = (35z - 34.77)/(z - 1)
// u[k] = u[k-1] + 35*e[k] - 34.77*e[k-1]
static const float CVI_B0 = 35.0f;
static const float CVI_B1 = 34.77f;

// Cid(z) = (5.5z - 5.413)/(z - 1)
// u[k] = u[k-1] + 5.5*e[k] - 5.413*e[k-1]
static const float CID_B0 = 5.5f;
static const float CID_B1 = 5.413f;

// -----------------------------
// ขอบเขตตามโปรไฟล์
// -----------------------------
struct Profile {
  float vrInit, vrMin, vrMax, deltaV, dpDeadW;
  float ilRef;          // เพดาน CC (= ค่าคงที่ 12 ในรูป)
  float vbMaxReset;     // เกณฑ์รีเซ็ตใน ModINC (เปเปอร์ = 14.4)
  float cvV, cvEnter, cvExit;
  bool  enableCvExtra;  // เพิ่มชั้น CV นอกเหนือจากแผนภาพ
};

static Profile PRO_PAPER = {
  18.1f, 16.4f, 18.122f, 0.0001f, 0.007f,
  12.0f,
  14.4f,
  14.4f, 14.2f, 13.8f,
  false
};

static Profile PRO_16S = {
  42.3f, 40.0f, 45.0f, 0.08f, 0.5f,
  6.0f,             // CC ของคุณ
  56.0f,            // แทน Vbmax 14.4
  56.0f, 55.50f, 54.80f,
  true              // เพิ่ม CV limit
};

static Profile& P() { return USE_FIELD_16S ? PRO_16S : PRO_PAPER; }

// -----------------------------
// จังหวะ
// -----------------------------
static const uint32_t CTRL_MS = 20;
static const uint32_t MPPT_MS = 100;
static const uint32_t LOG_MS  = 500;

// =============================================================================
// สถานะบล็อก
// =============================================================================
struct Sensors {
  float Vpv;
  float Ipv;
  float IpvRaw;
  float Vb;
  float iL;     // กระแสเหนี่ยวนำ/ชาร์จ (ในเครื่องคุณใช้ |Ibat|)
};

// ModINC persistent
static float Vold = 0, Iold = 0, Pold = 0, Vrold = 0;
static bool  mpptInit = false;

// Cvi state
static float cvi_u = 0.0f;
static float cvi_e_prev = 0.0f;

// Cid state
static float cid_u = 0.0f;
static float cid_e_prev = 0.0f;

// outputs (สำหรับ log)
static float g_Vr = 0.0f;
static float g_IL_star = 0.0f;
static float g_IL_max = 0.0f;
static float g_d = 0.0f;          // 0..1 ตามพอร์ต d ในรูป
static bool  g_inCv = false;

static uint32_t tCtrl = 0, tMppt = 0, tLog = 0;

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

static void setDutyFromUnit(float d_unit) {
  // ในซิมพอร์ต d เป็นสัดส่วน; แปลงเป็น raw PWM
  d_unit = clampf(d_unit, 0.0f, 1.0f);
  int raw = (int)lroundf(d_unit * (float)PWM_RAW_MAX);
  if (raw > DUTY_RAW_MAX) raw = DUTY_RAW_MAX;
  ledcWrite(PIN_PWM_BOOST, raw);
}

// =============================================================================
// บล็อก 1: ModINC  (ตรงฟังก์ชัน MATLAB + แก้ Pold=P)
// =============================================================================
static float Mod_INC(float V, float I, float Vb) {
  const float Vrinit = P().vrInit;
  const float Vrmax  = P().vrMax;
  const float Vrmin  = P().vrMin;
  const float deltaV = P().deltaV;

  if (!mpptInit) {
    Vold = V;
    Iold = 0.0f;
    Pold = 0.0f;
    Vrold = Vrinit;
    mpptInit = true;
  }

  // ของเปเปอร์: if Vb > 14.4 → รีเซ็ต
  if (Vb > P().vbMaxReset) {
    Iold = 0.0f;
    Vrold = USE_FIELD_16S ? clampf(V, Vrmin, Vrmax) : 22.1f;
  }

  float Pnow = V * I;
  float dP = Pnow - Pold;
  float dV = V - Vold;
  float dI = I - Iold;
  float M = fabsf(dP);

  float Vr;

  if (M < P().dpDeadW) {
    Vr = Vrold;
  } else {
    if (dV == 0.0f) {
      if (dI == 0.0f) {
        Vr = Vrold;
      } else if (dI > 0.0f) {
        Vr = Vrold + (M * deltaV);
      } else {
        Vr = Vrold - (M * deltaV);
      }
    } else {
      float cond = V * dI + I * dV;   // IncCond
      if (cond == 0.0f) {
        Vr = Vrold;
      } else if (cond > 0.0f) {
        if (dV > 0.0f) Vr = Vrold + (M * deltaV);
        else           Vr = Vrold - (M * deltaV);
      } else {
        if (dV > 0.0f) Vr = Vrold - deltaV;
        else           Vr = Vrold + (M * deltaV);
      }
    }
  }

  if (Vr >= Vrmax || Vr <= Vrmin) {
    Vr = Vrold;
  }

  // ★ แก้บั๊กต้นฉบับ: ต้องเซฟ Pold
  Vrold = Vr;
  Vold = V;
  Iold = I;
  Pold = Pnow;

  g_Vr = Vr;
  return Vr;
}

// =============================================================================
// บล็อก 2: ลูปแรงดันแผง
//   e = Hv*Vr - Hv*Vpv
//   y = Cvi(e)
//   IL_star = -y
// =============================================================================
static float Cvi_block(float Vr, float Vpv) {
  float e = Hv * Vr - Hv * Vpv;   // เหมือนแผนภาพ: ทั้งคู่ผ่าน Hv แล้วลบกัน
  // Cvi(z)=(35z-34.77)/(z-1)
  cvi_u = cvi_u + CVI_B0 * e - CVI_B1 * cvi_e_prev;
  cvi_e_prev = e;

  float IL_star = -1.0f * cvi_u;  // บล็อก Gain = -1
  g_IL_star = IL_star;
  return IL_star;
}

// =============================================================================
// บล็อก 3: Battery_Charging_Current_Control  (ตรง MATLAB)
//   + ชั้น CV เสริมเมื่อ USE_FIELD_16S
// =============================================================================
static float Battery_Charging_Current_Control(float IL_ref, float IL_star, float Vb) {
  float IL;
  if (IL_star >= IL_ref) IL = IL_ref;
  else                   IL = IL_star;

  // แผนภาพเดิมไม่มี CV — เพิ่มเฉพาะโปรไฟล์สนาม
  if (P().enableCvExtra) {
    if (!g_inCv && Vb >= P().cvEnter) g_inCv = true;
    if (g_inCv && Vb <= P().cvExit)  g_inCv = false;
    if (g_inCv) {
      float vErr = P().cvV - Vb;
      float iCv;
      if (vErr <= 0.0f) iCv = 0.2f;
      else              iCv = clampf(vErr * 4.0f, 0.3f, IL_ref);
      if (iCv < IL) IL = iCv;
    }
  }

  g_IL_max = IL;
  return IL;
}

// =============================================================================
// บล็อก 4: ลูปกระแส → d
//   e = Hi*IL_max - Hi*iL
//   d = Cid(e)
// =============================================================================
static float Cid_block(float IL_max, float iL) {
  float e = Hi * IL_max - Hi * iL;
  // Cid(z)=(5.5z-5.413)/(z-1)
  cid_u = cid_u + CID_B0 * e - CID_B1 * cid_e_prev;
  cid_e_prev = e;

  // ในซิมพอร์ต d มักอิ่มตัว 0..1
  float d = clampf(cid_u, 0.0f, 1.0f);
  g_d = d;
  return d;
}

// =============================================================================
// อ่านเซ็นเซอร์ — ใส่ ADS จริงตรงนี้
// =============================================================================
static Sensors readSensors() {
  Sensors s;
  // TODO: แทนที่ด้วย ADS1115 จริง
  s.Vpv = 43.5f;
  s.IpvRaw = 5.1f;
  s.Vb = 55.7f;
  s.iL = 1.6f;   // ใช้ |Ibat| เป็น iL

  if (PV_CURRENT_SENSOR_OK) {
    s.Ipv = s.IpvRaw;
  } else {
    if (s.Vpv > 38.0f && s.iL > 0.05f)
      s.Ipv = (s.Vb * s.iL) / maxf(s.Vpv * EFF_EST, 1.0f);
    else
      s.Ipv = 0.0f;
  }
  return s;
}

// =============================================================================
// setup / loop — ลำดับเรียกตรงแผนภาพ
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("[BOOT] Cascade = Simulink diagram (ModINC/Cvi/Limit/Cid)");
  Serial.printf("[BOOT] PROFILE=%s  IL_ref=%.1f  PV_SENSOR_OK=%d\n",
                USE_FIELD_16S ? "16S_FIELD" : "PAPER",
                P().ilRef, PV_CURRENT_SENSOR_OK ? 1 : 0);
  Serial.println("[BOOT] Hv=1/5  Cvi=(35z-34.77)/(z-1)  x(-1)");
  Serial.println("[BOOT] Limit=min(IL_star,IL_ref)  Hi=1/25  Cid=(5.5z-5.413)/(z-1)");

  ledcAttach(PIN_PWM_BOOST, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcWrite(PIN_PWM_BOOST, 0);

  Vrold = P().vrInit;
  g_Vr = Vrold;
  tCtrl = tMppt = tLog = millis();
}

void loop() {
  uint32_t now = millis();
  if (now - tCtrl < CTRL_MS) return;
  tCtrl = now;

  Sensors s = readSensors();

  // ---- (1) ModINC @ อัตราส่วนช้ากว่า ----
  if (now - tMppt >= MPPT_MS) {
    tMppt = now;
    Mod_INC(s.Vpv, s.Ipv, s.Vb);          // → g_Vr
  }

  // ---- (2) ลูปแรงดันแผง ----
  float IL_star = Cvi_block(g_Vr, s.Vpv); // Hv → Cvi → ×(-1)

  // ---- (3) จำกัดกระแสแบต ----
  float IL_max = Battery_Charging_Current_Control(P().ilRef, IL_star, s.Vb);

  // ---- (4) ลูปกระแส → duty ----
  float d = Cid_block(IL_max, s.iL);      // Hi → Cid → d
  setDutyFromUnit(d);

  if (now - tLog >= LOG_MS) {
    tLog = now;
    Serial.println("=========================================================================================");
    Serial.printf("ModINC: Vr=%.3f | Vpv=%.2f Ipv=%.2f (%s)\n",
                  g_Vr, s.Vpv, s.Ipv, PV_CURRENT_SENSOR_OK ? "sensor" : "EST");
    Serial.printf("Cvi:    IL_star=%.3f\n", g_IL_star);
    Serial.printf("Limit:  IL_ref=%.2f  IL_max=%.3f  CV=%d  Vb=%.2f\n",
                  P().ilRef, g_IL_max, g_inCv ? 1 : 0, s.Vb);
    Serial.printf("Cid:    iL=%.2f  d=%.4f  dutyPWM=%d (%.1f%%)\n",
                  s.iL, g_d, (int)lroundf(g_d * PWM_RAW_MAX),
                  100.0f * g_d);
    Serial.println("=========================================================================================");
  }
}
