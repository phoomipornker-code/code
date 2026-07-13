#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// ============================================================
// MPPT -> CC -> CV controller for 16S LiFePO4 boost charger
// Target: CC 6A, CV 58.0V, PWM 50kHz (ESP32)
// ============================================================

// --------------------------
// Hardware pins
// --------------------------
static const int RELAY_PV_PIN = 32;
static const int BUTTON_START_PIN = 25;
static const int BUTTON_STOP_PIN = 26;
static const int PWM_BOOST_PIN = 27;

// --------------------------
// PWM config
// --------------------------
static const uint32_t PWM_FREQ_HZ = 50000;
static const uint8_t PWM_RES_BITS = 10;
static const int PWM_CHANNEL_BOOST = 0;
static const int PWM_RAW_MAX = (1 << PWM_RES_BITS) - 1;  // 1023
static const int BOOST_DUTY_RAW_MAX = 760;               // keep margin for hardware
static const int BOOST_DUTY_RAW_MIN = 0;

// --------------------------
// Charger target (16S LFP)
// --------------------------
static const float CC_CURRENT_A = 6.0f;
static const float CV_VOLTAGE_V = 58.0f;
static const float CV_ENTER_V = 57.8f;
static const float CV_EXIT_V = 57.4f;
static const float RECHARGE_V = 54.0f;
static const float CUTOFF_CURRENT_A = 0.45f;

// --------------------------
// PV limits (your panel)
// --------------------------
static const float PV_VMP_V = 42.3f;
static const float PV_MIN_START_V = 42.0f;
static const float PV_CRITICAL_LOW_V = 39.0f;
static const float PV_VREF_MIN_V = 40.0f;
static const float PV_VREF_MAX_V = 45.0f;
static const float PV_POWER_LIMIT_W = 650.0f;
static const float POWER_CAP_ENABLE_W = 80.0f;       // avoid startup deadlock at tiny sampled power
static const float PV_CURRENT_SOFT_A = 15.5f;
static const float PV_CURRENT_HARD_A = 16.3f;
static const float ETA_EST = 0.90f;

// --------------------------
// Protection
// --------------------------
static const float HARD_OVP_V = 58.4f;
static const float HARD_OVP_RELEASE_V = 57.2f;
static const float OTP_C = 65.0f;
static const float UVLO_BAT_V = 40.0f;

// --------------------------
// Timing (ms)
// --------------------------
static const uint32_t CONTROL_PERIOD_MS = 20;    // 50 Hz
static const uint32_t MPPT_PERIOD_MS = 100;      // 10 Hz
static const uint32_t SERIAL_PERIOD_MS = 500;
static const uint32_t CV_ENTER_CONFIRM_MS = 8000;
static const uint32_t CV_EXIT_CONFIRM_MS = 3000;
static const uint32_t DONE_CONFIRM_MS = 120000;
static const uint32_t SOFTSTART_MS = 2500;
static const uint32_t PV_LOW_SHUTDOWN_MS = 2000;

// --------------------------
// Control tuning
// --------------------------
static const float MPPT_STEP_V = 0.10f;
static const float MPPT_TRACK_K = 0.08f;           // converts PV-V error to Iref adaptation
static const float DUTY_SLEW_UP_RAW = 2.2f;
static const float DUTY_SLEW_DOWN_RAW = 5.0f;
static const float CV_MIN_DUTY_MARGIN = 0.03f;     // +3%

// Current PI (controls Ibat via duty)
static const float CURR_KP = 12.0f;
static const float CURR_KI = 40.0f;
static const float CURR_OUT_MIN = -30.0f;
static const float CURR_OUT_MAX = 30.0f;

// Voltage PI (in CV, outputs current reference)
static const float VOLT_KP = 1.2f;
static const float VOLT_KI = 0.9f;
static const float VOLT_OUT_MIN = 0.0f;
static const float VOLT_OUT_MAX = CC_CURRENT_A;

// --------------------------
// ADC conversion/calibration
// --------------------------
static const float ADC_LSB_MV = 0.1875f;
static const float CAL_SCALE_V_SOLAR = 41.9f;
static const float FIELD_TRIM_V_SOLAR = 0.9589f;   // one-point trim: 44.3 / 46.2
static const float CAL_SCALE_V_BAT = 41.85f;
static const float CAL_SCALE_I_SOLAR = 42.46f;
static const float CAL_SCALE_I_BAT = 42.46f;

static const float OFFSET_V_SOLAR_MV = 0.0f;
static const float OFFSET_V_BAT_MV = 0.0f;
static const float OFFSET_I_SOLAR_MV = 1659.7f;
static const float OFFSET_I_BAT_MV = 1646.9f;

static const float ADC_NOISE_V = 0.4f;
static const float ADC_NOISE_I = 0.05f;

// ADS1115 map:
// ads_volt: CH0=PV V, CH1=BAT V
// ads_curr: CH0=PV I, CH2=BAT I
Adafruit_ADS1115 ads_volt;
Adafruit_ADS1115 ads_curr;

enum ChargerState {
  STANDBY = 0,
  SOFTSTART,
  CC_MPPT,
  CV_HOLD,
  CHARGE_DONE,
  FAULT
};

struct SensorSample {
  float vPv;
  float iPvAbs;
  float vBat;
  float iBatAbs;
  float tempC;
  float pPv;
};

struct PIController {
  float kp;
  float ki;
  float outMin;
  float outMax;
  float integrator;
};

static ChargerState g_state = STANDBY;
static uint32_t g_stateEnterMs = 0;
static uint32_t g_lastControlMs = 0;
static uint32_t g_lastMpptMs = 0;
static uint32_t g_lastSerialMs = 0;
static uint32_t g_pvLowStartMs = 0;
static uint32_t g_cvEnterStartMs = 0;
static uint32_t g_cvExitStartMs = 0;
static uint32_t g_doneStartMs = 0;

static bool g_startLatch = false;
static bool g_lastStartRaw = HIGH;
static bool g_lastStopRaw = HIGH;
static const char* g_faultReason = "";

static float g_vpvRef = PV_VMP_V;
static float g_mpptLastPower = 0.0f;
static float g_mpptLastVpv = 0.0f;
static int g_mpptDir = 1;
static float g_iRefMppt = 1.0f;
static float g_pAvailFilt = 0.0f;

static float g_dutyCmd = 0.0f;
static int g_dutyRaw = 0;
static SensorSample g_lastSample = {0};

static PIController g_currPi = {CURR_KP, CURR_KI, CURR_OUT_MIN, CURR_OUT_MAX, 0.0f};
static PIController g_voltPi = {VOLT_KP, VOLT_KI, VOLT_OUT_MIN, VOLT_OUT_MAX, 0.0f};

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static int clampi(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static void resetPI(PIController& pi) {
  pi.integrator = 0.0f;
}

static float runPI(PIController& pi, float err, float dt) {
  float p = pi.kp * err;
  float iCandidate = pi.integrator + (pi.ki * err * dt);
  float out = p + iCandidate;

  if (out > pi.outMax) {
    out = pi.outMax;
  } else if (out < pi.outMin) {
    out = pi.outMin;
  } else {
    pi.integrator = iCandidate;
  }
  return out;
}

static float applySlew(float target, float current, float upStep, float downStep) {
  float delta = target - current;
  if (delta > upStep) return current + upStep;
  if (delta < -downStep) return current - downStep;
  return target;
}

static void setBoostDutyRaw(int raw) {
  g_dutyRaw = clampi(raw, BOOST_DUTY_RAW_MIN, BOOST_DUTY_RAW_MAX);
  ledcWrite(PWM_CHANNEL_BOOST, g_dutyRaw);
}

static void enablePowerPath(bool en) {
  digitalWrite(RELAY_PV_PIN, en ? HIGH : LOW);
  if (!en) {
    g_dutyCmd = 0.0f;
    setBoostDutyRaw(0);
  }
}

static int16_t readAdcSingleEndedSafe(Adafruit_ADS1115& adc, uint8_t ch) {
  int16_t s = adc.readADC_SingleEnded(ch);
  if (s < 0) {
    s = adc.readADC_SingleEnded(ch);
    if (s < 0) s = 0;
  }
  return s;
}

static float ema(float x, float yPrev, float alpha) {
  return (alpha * x) + ((1.0f - alpha) * yPrev);
}

static bool readSensors(SensorSample& s) {
  const float alphaV = 0.25f;
  const float alphaI = 0.25f;

  static float vPvF = 0.0f;
  static float vBatF = 0.0f;
  static float iPvF = 0.0f;
  static float iBatF = 0.0f;
  static bool initialized = false;

  float mv_v_pv = (float)readAdcSingleEndedSafe(ads_volt, 0) * ADC_LSB_MV;
  float mv_v_bat = (float)readAdcSingleEndedSafe(ads_volt, 1) * ADC_LSB_MV;
  float mv_i_pv = (float)readAdcSingleEndedSafe(ads_curr, 0) * ADC_LSB_MV;
  float mv_i_bat = (float)readAdcSingleEndedSafe(ads_curr, 2) * ADC_LSB_MV;

  float vPv = ((mv_v_pv - OFFSET_V_SOLAR_MV) / 1000.0f) * CAL_SCALE_V_SOLAR * FIELD_TRIM_V_SOLAR;
  float vBat = ((mv_v_bat - OFFSET_V_BAT_MV) / 1000.0f) * CAL_SCALE_V_BAT;
  float iPv = ((mv_i_pv - OFFSET_I_SOLAR_MV) / 1000.0f) * CAL_SCALE_I_SOLAR;
  float iBat = ((mv_i_bat - OFFSET_I_BAT_MV) / 1000.0f) * CAL_SCALE_I_BAT;

  if (fabsf(vPv) < ADC_NOISE_V) vPv = 0.0f;
  if (fabsf(vBat) < ADC_NOISE_V) vBat = 0.0f;
  if (fabsf(iPv) < ADC_NOISE_I) iPv = 0.0f;
  if (fabsf(iBat) < ADC_NOISE_I) iBat = 0.0f;

  if (!initialized) {
    vPvF = vPv;
    vBatF = vBat;
    iPvF = fabsf(iPv);
    iBatF = fabsf(iBat);
    initialized = true;
  } else {
    vPvF = ema(vPv, vPvF, alphaV);
    vBatF = ema(vBat, vBatF, alphaV);
    iPvF = ema(fabsf(iPv), iPvF, alphaI);
    iBatF = ema(fabsf(iBat), iBatF, alphaI);
  }

  s.vPv = vPvF;
  s.iPvAbs = iPvF;
  s.vBat = vBatF;
  s.iBatAbs = iBatF;
  s.tempC = 30.0f;  // TODO: replace with real temperature input
  s.pPv = s.vPv * s.iPvAbs;
  return true;
}

static bool pvReady(const SensorSample& s) {
  return s.vPv >= PV_MIN_START_V;
}

static int estimateBoostDutyRaw(float vin, float vout) {
  float vinUse = max(vin, 38.0f);
  float voutUse = max(vout, vinUse + 2.0f);
  float d = 1.0f - (vinUse / voutUse);
  d += CV_MIN_DUTY_MARGIN;
  int raw = (int)roundf(d * (float)PWM_RAW_MAX);
  return clampi(raw, 20, BOOST_DUTY_RAW_MAX);
}

static float mpptCurrentCapFromPower(const SensorSample& s) {
  if (g_pAvailFilt < POWER_CAP_ENABLE_W || s.vBat < 5.0f) {
    return CC_CURRENT_A;
  }
  float pAvail = min(g_pAvailFilt, PV_POWER_LIMIT_W);
  float capFromPower = (pAvail * ETA_EST) / s.vBat;
  return clampf(capFromPower, 0.0f, CC_CURRENT_A);
}

static void enterState(ChargerState next, uint32_t nowMs) {
  g_state = next;
  g_stateEnterMs = nowMs;
  g_cvEnterStartMs = 0;
  g_cvExitStartMs = 0;
  g_doneStartMs = 0;

  if (next == SOFTSTART) {
    resetPI(g_currPi);
    resetPI(g_voltPi);
    g_iRefMppt = 1.0f;
  } else if (next == CC_MPPT) {
    resetPI(g_currPi);
  } else if (next == CV_HOLD) {
    resetPI(g_currPi);
    resetPI(g_voltPi);
  } else if (next == CHARGE_DONE || next == STANDBY || next == FAULT) {
    enablePowerPath(false);
  }
}

static void enterFault(const char* reason, uint32_t nowMs) {
  g_faultReason = reason;
  enterState(FAULT, nowMs);
}

static void updateButtons() {
  bool startRaw = digitalRead(BUTTON_START_PIN);
  bool stopRaw = digitalRead(BUTTON_STOP_PIN);

  bool startEdge = (startRaw == LOW && g_lastStartRaw == HIGH);
  bool stopEdge = (stopRaw == LOW && g_lastStopRaw == HIGH);

  if (startEdge) g_startLatch = true;
  if (stopEdge) g_startLatch = false;

  g_lastStartRaw = startRaw;
  g_lastStopRaw = stopRaw;
}

static void runMpptTask(const SensorSample& s) {
  // P&O updates PV voltage reference
  float dP = s.pPv - g_mpptLastPower;
  float dV = s.vPv - g_mpptLastVpv;

  if (fabsf(dP) > 0.2f) {
    if (dP > 0.0f) {
      g_mpptDir = (dV >= 0.0f) ? 1 : -1;
    } else {
      g_mpptDir = (dV >= 0.0f) ? -1 : 1;
    }
  }

  g_vpvRef += (float)g_mpptDir * MPPT_STEP_V;
  g_vpvRef = clampf(g_vpvRef, PV_VREF_MIN_V, PV_VREF_MAX_V);

  // Track current cap from PV voltage error
  float pvErr = s.vPv - g_vpvRef;
  g_iRefMppt += MPPT_TRACK_K * pvErr;
  g_iRefMppt = clampf(g_iRefMppt, 0.0f, CC_CURRENT_A);

  g_pAvailFilt = (g_pAvailFilt <= 0.01f) ? s.pPv : ema(s.pPv, g_pAvailFilt, 0.22f);
  g_mpptLastPower = s.pPv;
  g_mpptLastVpv = s.vPv;
}

static void applyCurrentControl(const SensorSample& s, float iRef, float dtSec, bool cvMode) {
  float currErr = iRef - s.iBatAbs;
  float deltaDuty = runPI(g_currPi, currErr, dtSec);

  float targetDuty = g_dutyCmd + deltaDuty;

  // Keep duty above physics-based minimum when CV still below target.
  if (cvMode && s.vBat < (CV_VOLTAGE_V - 0.2f)) {
    int cvMinRaw = estimateBoostDutyRaw(s.vPv, CV_VOLTAGE_V);
    if (targetDuty < cvMinRaw) targetDuty = (float)cvMinRaw;
  }

  targetDuty = clampf(targetDuty, (float)BOOST_DUTY_RAW_MIN, (float)BOOST_DUTY_RAW_MAX);
  g_dutyCmd = applySlew(targetDuty, g_dutyCmd, DUTY_SLEW_UP_RAW, DUTY_SLEW_DOWN_RAW);
  setBoostDutyRaw((int)roundf(g_dutyCmd));
}

static void controlLoopStep(uint32_t nowMs) {
  SensorSample s;
  if (!readSensors(s)) {
    enterFault("ADC read fail", nowMs);
    return;
  }
  g_lastSample = s;

  if (s.vBat >= HARD_OVP_V) {
    enterFault("Hard OVP", nowMs);
    return;
  }
  if (s.iPvAbs > PV_CURRENT_HARD_A) {
    enterFault("PV over-current", nowMs);
    return;
  }
  if (s.tempC > OTP_C) {
    enterFault("Over temperature", nowMs);
    return;
  }
  if (s.vBat < UVLO_BAT_V) {
    enterFault("Battery UVLO", nowMs);
    return;
  }

  if (!g_startLatch) {
    enterState(STANDBY, nowMs);
    return;
  }

  if (!pvReady(s) && g_state != CHARGE_DONE) {
    if (g_pvLowStartMs == 0) g_pvLowStartMs = nowMs;
    if (s.vPv < PV_CRITICAL_LOW_V && (nowMs - g_pvLowStartMs >= PV_LOW_SHUTDOWN_MS)) {
      enterState(STANDBY, nowMs);
    }
    return;
  }
  g_pvLowStartMs = 0;

  float dtSec = (float)CONTROL_PERIOD_MS / 1000.0f;

  switch (g_state) {
    case STANDBY: {
      enablePowerPath(true);
      g_vpvRef = PV_VMP_V;
      g_pAvailFilt = 0.0f;
      g_mpptLastPower = 0.0f;
      g_mpptLastVpv = s.vPv;
      g_dutyCmd = 0.0f;
      setBoostDutyRaw(0);
      enterState(SOFTSTART, nowMs);
      break;
    }

    case SOFTSTART: {
      int targetRaw = estimateBoostDutyRaw(s.vPv, CV_VOLTAGE_V);
      float nextDuty = min(g_dutyCmd + 2.0f, (float)targetRaw);
      g_dutyCmd = nextDuty;
      setBoostDutyRaw((int)roundf(g_dutyCmd));

      if ((nowMs - g_stateEnterMs >= SOFTSTART_MS) || (s.iBatAbs > 0.4f)) {
        enterState(CC_MPPT, nowMs);
      }
      break;
    }

    case CC_MPPT: {
      if (nowMs - g_lastMpptMs >= MPPT_PERIOD_MS) {
        g_lastMpptMs = nowMs;
        runMpptTask(s);
      }

      float iRefByPower = mpptCurrentCapFromPower(s);
      float iRef = min(CC_CURRENT_A, min(g_iRefMppt, iRefByPower));
      iRef = min(iRef, PV_CURRENT_SOFT_A);
      iRef = clampf(iRef, 0.0f, CC_CURRENT_A);

      applyCurrentControl(s, iRef, dtSec, false);

      if (s.vBat >= CV_ENTER_V) {
        if (g_cvEnterStartMs == 0) g_cvEnterStartMs = nowMs;
        if (nowMs - g_cvEnterStartMs >= CV_ENTER_CONFIRM_MS) {
          enterState(CV_HOLD, nowMs);
        }
      } else {
        g_cvEnterStartMs = 0;
      }
      break;
    }

    case CV_HOLD: {
      if (nowMs - g_lastMpptMs >= MPPT_PERIOD_MS) {
        g_lastMpptMs = nowMs;
        runMpptTask(s);
      }

      float vErr = CV_VOLTAGE_V - s.vBat;
      float iReqFromVolt = runPI(g_voltPi, vErr, dtSec);
      float iRefByPower = mpptCurrentCapFromPower(s);
      float iRef = min(iReqFromVolt, min(g_iRefMppt, iRefByPower));
      iRef = clampf(iRef, 0.0f, CC_CURRENT_A);

      applyCurrentControl(s, iRef, dtSec, true);

      if (s.vBat <= CV_EXIT_V && s.iBatAbs < (CC_CURRENT_A - 0.6f)) {
        if (g_cvExitStartMs == 0) g_cvExitStartMs = nowMs;
        if (nowMs - g_cvExitStartMs >= CV_EXIT_CONFIRM_MS) {
          enterState(CC_MPPT, nowMs);
          break;
        }
      } else {
        g_cvExitStartMs = 0;
      }

      bool doneCond = (s.vBat >= (CV_VOLTAGE_V - 0.08f)) && (s.iBatAbs <= CUTOFF_CURRENT_A);
      if (doneCond) {
        if (g_doneStartMs == 0) g_doneStartMs = nowMs;
        if (nowMs - g_doneStartMs >= DONE_CONFIRM_MS) {
          enterState(CHARGE_DONE, nowMs);
        }
      } else {
        g_doneStartMs = 0;
      }
      break;
    }

    case CHARGE_DONE: {
      enablePowerPath(false);
      if (s.vBat <= RECHARGE_V && pvReady(s)) {
        enterState(CC_MPPT, nowMs);
        enablePowerPath(true);
      }
      break;
    }

    case FAULT: {
      enablePowerPath(false);
      bool faultRelease = (s.vBat <= HARD_OVP_RELEASE_V) && (s.iPvAbs < PV_CURRENT_SOFT_A) && (s.tempC < (OTP_C - 5.0f));
      if (faultRelease && g_startLatch) {
        enterState(STANDBY, nowMs);
      }
      break;
    }
  }
}

static const char* stateName(ChargerState st) {
  switch (st) {
    case STANDBY: return "STANDBY";
    case SOFTSTART: return "SOFTSTART";
    case CC_MPPT: return "CC_MPPT";
    case CV_HOLD: return "CV_HOLD";
    case CHARGE_DONE: return "DONE";
    case FAULT: return "FAULT";
    default: return "UNKNOWN";
  }
}

void setup() {
  Serial.begin(115200);
  delay(80);
  Serial.println("[BOOT] MPPT-CC-CV controller (16S, 6A, 58V)");

  pinMode(RELAY_PV_PIN, OUTPUT);
  pinMode(BUTTON_START_PIN, INPUT_PULLUP);
  pinMode(BUTTON_STOP_PIN, INPUT_PULLUP);
  digitalWrite(RELAY_PV_PIN, LOW);

  ledcSetup(PWM_CHANNEL_BOOST, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(PWM_BOOST_PIN, PWM_CHANNEL_BOOST);
  setBoostDutyRaw(0);

  Wire.begin(21, 22);
  Wire.setClock(100000);

  bool okVolt = ads_volt.begin(0x48);
  bool okCurr = ads_curr.begin(0x49);
  if (!okVolt || !okCurr) {
    g_faultReason = "ADS1115 not found";
    g_state = FAULT;
    Serial.println("[FATAL] ADS1115 init failed.");
    return;
  }

  ads_volt.setDataRate(RATE_ADS1115_860SPS);
  ads_curr.setDataRate(RATE_ADS1115_860SPS);

  g_state = STANDBY;
  g_stateEnterMs = millis();
}

void loop() {
  uint32_t nowMs = millis();
  updateButtons();

  if (nowMs - g_lastControlMs >= CONTROL_PERIOD_MS) {
    g_lastControlMs = nowMs;
    controlLoopStep(nowMs);
  }

  if (nowMs - g_lastSerialMs >= SERIAL_PERIOD_MS) {
    g_lastSerialMs = nowMs;
    Serial.printf(
      "[%s] Duty=%d Vpv=%.2f Ipv=%.2f Ppv=%.1f Vbat=%.2f Ibat=%.2f IrefMppt=%.2f VrefPv=%.2f",
      stateName(g_state), g_dutyRaw, g_lastSample.vPv, g_lastSample.iPvAbs, g_lastSample.pPv,
      g_lastSample.vBat, g_lastSample.iBatAbs, g_iRefMppt, g_vpvRef
    );
    if (g_state == FAULT) {
      Serial.printf(" FAULT=%s", g_faultReason);
    }
    Serial.println();
  }

  delay(2);
}
