#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

/*
 * ESP32 PV boost charger
 * - 50 kHz PWM
 * - Incremental Conductance MPPT
 * - 6 A constant-current limit
 * - 56 V constant-voltage limit
 * - ADS1115 voltage/current measurements
 *
 * IMPORTANT:
 * 1. Verify all voltage-divider and current-sensor calibration constants.
 * 2. Use an isolated gate driver with hardware cycle-by-cycle overcurrent
 *    protection. Software protection is not fast enough for MOSFET safety.
 * 3. Test first with a current-limited laboratory supply and dummy load.
 */

const char *FW_VERSION_TAG = "pv-inccond-cccv-v1";

// --------------------------------------------------------------------------
// Hardware
// --------------------------------------------------------------------------
const int RELAY_PV_PIN     = 32;
const int RELAY_AC_PIN     = 33;
const int BUTTON_START_PIN = 25;
const int BUTTON_STOP_PIN  = 26;
const int PWM_FORWARD_PIN  = 14;
const int PWM_BOOST_PIN    = 27;

const int PWM_FREQ = 50000;
const int PWM_RES  = 10;
const int PWM_FULL_SCALE = (1 << PWM_RES) - 1;

Adafruit_ADS1115 ads_volt;
Adafruit_ADS1115 ads_curr;
LiquidCrystal_I2C lcd(0x27, 20, 4);
SemaphoreHandle_t i2cMutex;

// --------------------------------------------------------------------------
// Charge targets and operating limits
// --------------------------------------------------------------------------
const float TARGET_CV_VOLTAGE = 56.0f;
const float TARGET_CC_CURRENT = 6.0f;

const float MIN_PV_START_VOLTAGE = 42.0f;
const float PV_SHUTDOWN_VOLTAGE  = 39.0f;
const float PV_COLLAPSE_VOLTAGE  = 41.0f;

const float HARD_OVP_VOLTAGE = 57.8f;
const float HARD_OCP_CURRENT = 6.5f;
const float FULL_VOLTAGE      = 55.9f;
const float FULL_CURRENT      = 0.5f;
const float RESTART_VOLTAGE   = 54.0f;

const uint32_t FULL_CONFIRM_MS = 60000;
const uint32_t SENSOR_TIMEOUT_MS = 700;

// 45% is sufficient for approximately 42 V -> 56 V boost conversion.
// Raise this only after checking the inductor, MOSFET, diode and gate driver.
const float DUTY_MIN = 0.0f;
const float DUTY_MAX = 0.45f;

// --------------------------------------------------------------------------
// Sensor calibration copied from the supplied firmware
// --------------------------------------------------------------------------
const float OFFSET_V_SOLAR = 0.0f;
const float OFFSET_V_BAT   = 0.0f;
const float OFFSET_I_SOLAR = 1659.7f;
const float OFFSET_I_BAT   = 1646.9f;

const float CAL_SCALE_V_SOLAR = 41.9f;
const float FIELD_TRIM_V_SOLAR = 0.9589f;
const float CAL_SCALE_V_BAT = 41.85f;
const float CAL_SCALE_I_SOLAR = 42.46f;
const float CAL_SCALE_I_BAT = 42.46f;

const float NOISE_V_THRESHOLD = 0.5f;
const float NOISE_I_THRESHOLD = 0.08f;
const uint32_t I2C_CLOCK_HZ = 100000;

float currentOffsetSolar = OFFSET_I_SOLAR;
float currentOffsetBat   = OFFSET_I_BAT;

// --------------------------------------------------------------------------
// Timing
// ADS1115 sampling and control run every 20 ms. PWM remains at 50 kHz.
// --------------------------------------------------------------------------
const float CONTROL_DT = 0.020f;
const uint32_t CONTROL_PERIOD_MS = 20;
const uint32_t MPPT_PERIOD_MS = 100;
const uint32_t LCD_PERIOD_MS = 200;
const uint32_t DEBUG_PERIOD_MS = 500;
const uint32_t SOFTSTART_MS = 2500;

// --------------------------------------------------------------------------
// Incremental Conductance settings
// --------------------------------------------------------------------------
const float MPPT_INITIAL_VREF = 42.3f;
const float MPPT_VREF_MIN = 40.0f;
const float MPPT_VREF_MAX = 45.0f;
const float MPPT_STEP_V = 0.10f;
const float INCCOND_DV_EPS = 0.02f;
const float INCCOND_DI_EPS = 0.02f;
const float INCCOND_G_EPS  = 0.002f;

// Converts PV-voltage error to a battery-current reference.
const float MPPT_VOLT_TO_IREF_GAIN = 0.08f;

// --------------------------------------------------------------------------
// PI settings
// Current loop is the inner loop. CV and MPPT generate current requests.
// --------------------------------------------------------------------------
const float CURRENT_KP = 14.0f;
const float CURRENT_KI = 55.0f;
const float CURRENT_DELTA_MIN = -35.0f; // raw PWM counts per control tick
const float CURRENT_DELTA_MAX = 45.0f;

const float VOLTAGE_KP = 0.85f;
const float VOLTAGE_KI = 0.45f;
const float CV_IREF_MIN = 0.0f;
const float CV_IREF_MAX = 3.5f;

const float DUTY_SLEW_UP_COUNTS   = 4.0f;
const float DUTY_SLEW_DOWN_COUNTS = 6.0f;

// --------------------------------------------------------------------------
// Runtime state
// --------------------------------------------------------------------------
enum ChargerState {
  CHARGER_OFF,
  CHARGER_SOFTSTART,
  CHARGER_MPPT_CC,
  CHARGER_CV,
  CHARGER_FULL,
  CHARGER_FAULT
};

volatile ChargerState chargerState = CHARGER_OFF;
volatile bool systemOn = false;
volatile bool sensorOk = false;
volatile bool faultLatched = false;

volatile float vSolar = 0.0f;
volatile float iSolar = 0.0f;
volatile float vBat = 0.0f;
volatile float iBatCharge = 0.0f;

volatile float dutyCommand = 0.0f;
volatile int rawDuty = 0;

float currentIntegrator = 0.0f;
float voltageIntegrator = 0.0f;

float mpptVref = MPPT_INITIAL_VREF;
float mpptIref = 1.0f;
float previousVpv = 0.0f;
float previousIpv = 0.0f;
bool incCondReady = false;

uint32_t stateEntryMs = 0;
uint32_t lastSampleMs = 0;
uint32_t lastMpptMs = 0;
uint32_t fullConditionMs = 0;
float totalWh = 0.0f;

// --------------------------------------------------------------------------
// Utility functions
// --------------------------------------------------------------------------
static inline float clampf(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

static inline float applySlew(
    float target,
    float current,
    float upStep,
    float downStep) {
  float difference = target - current;
  if (difference > upStep) return current + upStep;
  if (difference < -downStep) return current - downStep;
  return target;
}

static inline float runPI(
    float error,
    float kp,
    float ki,
    float dt,
    float *integrator,
    float outputMin,
    float outputMax) {
  float candidateIntegrator = *integrator + ki * error * dt;
  float unsaturated = kp * error + candidateIntegrator;
  float saturated = clampf(unsaturated, outputMin, outputMax);

  // Conditional integration anti-windup.
  if (saturated == unsaturated ||
      (saturated >= outputMax && error < 0.0f) ||
      (saturated <= outputMin && error > 0.0f)) {
    *integrator = candidateIntegrator;
  }

  return saturated;
}

static inline int16_t readAdcStable(
    Adafruit_ADS1115 &adc,
    uint8_t channel,
    bool discardFirst = false) {
  if (discardFirst) {
    (void)adc.readADC_SingleEnded(channel);
  }

  int16_t sample = adc.readADC_SingleEnded(channel);
  if (sample < 0) {
    sample = adc.readADC_SingleEnded(channel);
    if (sample < 0) sample = 0;
  }
  return sample;
}

void setBoostDuty(float normalizedDuty) {
  normalizedDuty = clampf(normalizedDuty, DUTY_MIN, DUTY_MAX);
  rawDuty = (int)lroundf(normalizedDuty * PWM_FULL_SCALE);
  ledcWrite(PWM_BOOST_PIN, rawDuty);
  ledcWrite(PWM_FORWARD_PIN, 0);
  dutyCommand = normalizedDuty;
}

void disablePowerStage() {
  setBoostDuty(0.0f);
  digitalWrite(RELAY_PV_PIN, LOW);
  digitalWrite(RELAY_AC_PIN, LOW);
}

void resetControllers() {
  currentIntegrator = 0.0f;
  voltageIntegrator = 0.0f;
  dutyCommand = 0.0f;

  mpptVref = MPPT_INITIAL_VREF;
  mpptIref = 1.0f;
  previousVpv = 0.0f;
  previousIpv = 0.0f;
  incCondReady = false;
  lastMpptMs = 0;
  fullConditionMs = 0;
}

void enterState(ChargerState newState) {
  chargerState = newState;
  stateEntryMs = millis();

  if (newState == CHARGER_OFF ||
      newState == CHARGER_FULL ||
      newState == CHARGER_FAULT) {
    disablePowerStage();
  }
}

// --------------------------------------------------------------------------
// Incremental Conductance MPPT
// --------------------------------------------------------------------------
void syncIncCond(float vpv, float ipv) {
  previousVpv = vpv;
  previousIpv = ipv;
  incCondReady = true;
}

void updateIncCond(float vpv, float ipv) {
  if (!isfinite(vpv) || !isfinite(ipv) ||
      vpv <= 0.1f || ipv < 0.0f) {
    return;
  }

  if (!incCondReady) {
    syncIncCond(vpv, ipv);
    return;
  }

  float dV = vpv - previousVpv;
  float dI = ipv - previousIpv;
  float referenceChange = 0.0f;

  if (fabsf(dV) <= INCCOND_DV_EPS) {
    if (dI > INCCOND_DI_EPS) {
      referenceChange = MPPT_STEP_V;
    } else if (dI < -INCCOND_DI_EPS) {
      referenceChange = -MPPT_STEP_V;
    }
  } else {
    // At MPP: dI/dV + I/V = 0.
    float mppCondition = dI / dV + ipv / fmaxf(vpv, 0.1f);

    if (mppCondition > INCCOND_G_EPS) {
      referenceChange = MPPT_STEP_V;
    } else if (mppCondition < -INCCOND_G_EPS) {
      referenceChange = -MPPT_STEP_V;
    }
  }

  mpptVref = clampf(
      mpptVref + referenceChange,
      MPPT_VREF_MIN,
      MPPT_VREF_MAX);

  syncIncCond(vpv, ipv);
}

// --------------------------------------------------------------------------
// Sensor acquisition
// --------------------------------------------------------------------------
bool sampleSensors() {
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    return false;
  }

  float mvVpv = readAdcStable(ads_volt, 0, true) * 0.1875f;
  float mvVbat = readAdcStable(ads_volt, 1, true) * 0.1875f;
  float mvIpv = readAdcStable(ads_curr, 0) * 0.1875f;
  float mvIbat = readAdcStable(ads_curr, 2) * 0.1875f;

  xSemaphoreGive(i2cMutex);

  float newVpv =
      fmaxf(0.0f, mvVpv - OFFSET_V_SOLAR) * 0.001f *
      CAL_SCALE_V_SOLAR * FIELD_TRIM_V_SOLAR;
  float newVbat =
      fmaxf(0.0f, mvVbat - OFFSET_V_BAT) * 0.001f *
      CAL_SCALE_V_BAT;

  float newIpv =
      (mvIpv - currentOffsetSolar) * 0.001f * CAL_SCALE_I_SOLAR;
  float newIbat =
      (mvIbat - currentOffsetBat) * 0.001f * CAL_SCALE_I_BAT;

  // The supplied hardware reports negative battery current while charging.
  newIpv = fabsf(newIpv);
  newIbat = fabsf(newIbat);

  if (newVpv < NOISE_V_THRESHOLD) newVpv = 0.0f;
  if (newVbat < NOISE_V_THRESHOLD) newVbat = 0.0f;
  if (newIpv < NOISE_I_THRESHOLD) newIpv = 0.0f;
  if (newIbat < NOISE_I_THRESHOLD) newIbat = 0.0f;

  // First-order filtering. Alpha 0.25 at a nominal 20 ms sample interval.
  const float alpha = 0.25f;
  if (lastSampleMs == 0) {
    vSolar = newVpv;
    iSolar = newIpv;
    vBat = newVbat;
    iBatCharge = newIbat;
  } else {
    vSolar += alpha * (newVpv - vSolar);
    iSolar += alpha * (newIpv - iSolar);
    vBat += alpha * (newVbat - vBat);
    iBatCharge += alpha * (newIbat - iBatCharge);
  }

  lastSampleMs = millis();
  return true;
}

void calibrateCurrentOffsets() {
  const int samples = 80;
  float solarSum = 0.0f;
  float batSum = 0.0f;

  disablePowerStage();
  delay(100);

  for (int i = 0; i < samples; ++i) {
    solarSum += readAdcStable(ads_curr, 0, true) * 0.1875f;
    batSum += readAdcStable(ads_curr, 2, true) * 0.1875f;
    delay(2);
  }

  currentOffsetSolar = solarSum / samples;
  currentOffsetBat = batSum / samples;

  Serial.printf(
      "[CAL] Ipv offset %.2f mV, Ibat offset %.2f mV\n",
      currentOffsetSolar,
      currentOffsetBat);
}

// --------------------------------------------------------------------------
// Charger control
// --------------------------------------------------------------------------
void runSoftStart() {
  float outputVoltage = fmaxf(vBat, vSolar + 2.0f);
  float estimatedDuty = 1.0f - vSolar / outputVoltage + 0.03f;
  estimatedDuty = clampf(estimatedDuty, 0.02f, DUTY_MAX);

  float rawStep = 2.0f / PWM_FULL_SCALE;
  float nextDuty = applySlew(
      estimatedDuty,
      dutyCommand,
      rawStep,
      5.0f / PWM_FULL_SCALE);

  setBoostDuty(nextDuty);

  if (iBatCharge >= 0.4f ||
      millis() - stateEntryMs >= SOFTSTART_MS) {
    currentIntegrator = 0.0f;
    enterState(CHARGER_MPPT_CC);
  }
}

void runMpptCc() {
  uint32_t now = millis();

  if (now - lastMpptMs >= MPPT_PERIOD_MS) {
    lastMpptMs = now;

    bool ccIsLimiting =
        mpptIref >= TARGET_CC_CURRENT - 0.05f &&
        iBatCharge >= TARGET_CC_CURRENT - 0.15f;

    // MPPT is held while the battery CC limit intentionally curtails power.
    if (ccIsLimiting) {
      syncIncCond(vSolar, iSolar);
    } else {
      updateIncCond(vSolar, iSolar);
    }

    float pvVoltageError = vSolar - mpptVref;
    mpptIref += MPPT_VOLT_TO_IREF_GAIN * pvVoltageError;
    mpptIref = clampf(mpptIref, 0.0f, TARGET_CC_CURRENT);
  }

  float currentReference = fminf(mpptIref, TARGET_CC_CURRENT);

  if (vSolar < PV_COLLAPSE_VOLTAGE) {
    float sag = PV_COLLAPSE_VOLTAGE - vSolar;
    float scale = clampf(1.0f - 0.35f * sag, 0.15f, 1.0f);
    currentReference *= scale;
  }

  // Gentle taper before CV entry.
  if (vBat >= 54.8f) {
    float taper = clampf(
        (TARGET_CV_VOLTAGE - vBat) /
        (TARGET_CV_VOLTAGE - 54.8f),
        0.10f,
        1.0f);
    currentReference *= taper;
  }

  float currentError = currentReference - iBatCharge;
  float dutyDeltaCounts = runPI(
      currentError,
      CURRENT_KP,
      CURRENT_KI,
      CONTROL_DT,
      &currentIntegrator,
      CURRENT_DELTA_MIN,
      CURRENT_DELTA_MAX);

  float targetDuty =
      dutyCommand + dutyDeltaCounts / PWM_FULL_SCALE;
  targetDuty = clampf(targetDuty, DUTY_MIN, DUTY_MAX);

  setBoostDuty(applySlew(
      targetDuty,
      dutyCommand,
      DUTY_SLEW_UP_COUNTS / PWM_FULL_SCALE,
      DUTY_SLEW_DOWN_COUNTS / PWM_FULL_SCALE));

  if (vBat >= 55.5f) {
    voltageIntegrator = 0.0f;
    enterState(CHARGER_CV);
  }
}

void runCv() {
  float voltageError = TARGET_CV_VOLTAGE - vBat;

  float currentReference = runPI(
      voltageError,
      VOLTAGE_KP,
      VOLTAGE_KI,
      CONTROL_DT,
      &voltageIntegrator,
      CV_IREF_MIN,
      CV_IREF_MAX);

  if (fabsf(voltageError) <= 0.12f) {
    voltageIntegrator *= 0.95f;
  }

  float currentError = currentReference - iBatCharge;
  float dutyDeltaCounts = runPI(
      currentError,
      CURRENT_KP * 0.55f,
      CURRENT_KI * 0.45f,
      CONTROL_DT,
      &currentIntegrator,
      -8.0f,
      8.0f);

  float targetDuty =
      dutyCommand + dutyDeltaCounts / PWM_FULL_SCALE;

  if (vBat > TARGET_CV_VOLTAGE) {
    targetDuty -=
        (0.8f + 4.0f * (vBat - TARGET_CV_VOLTAGE)) /
        PWM_FULL_SCALE;
  }

  targetDuty = clampf(targetDuty, DUTY_MIN, DUTY_MAX);
  setBoostDuty(applySlew(
      targetDuty,
      dutyCommand,
      1.2f / PWM_FULL_SCALE,
      2.0f / PWM_FULL_SCALE));

  if (vBat < 54.8f) {
    currentIntegrator = 0.0f;
    voltageIntegrator = 0.0f;
    enterState(CHARGER_MPPT_CC);
  }

  bool fullCondition =
      vBat >= FULL_VOLTAGE &&
      iBatCharge <= FULL_CURRENT;

  if (fullCondition) {
    if (fullConditionMs == 0) fullConditionMs = millis();
    if (millis() - fullConditionMs >= FULL_CONFIRM_MS) {
      enterState(CHARGER_FULL);
    }
  } else {
    fullConditionMs = 0;
  }
}

void checkSafety() {
  if (vBat >= HARD_OVP_VOLTAGE ||
      iBatCharge >= HARD_OCP_CURRENT) {
    faultLatched = true;
    systemOn = false;
    enterState(CHARGER_FAULT);

    Serial.printf(
        "[FAULT] Vbat %.2f V, Ibat %.2f A\n",
        vBat,
        iBatCharge);
  }

  if (systemOn &&
      chargerState != CHARGER_OFF &&
      vSolar < PV_SHUTDOWN_VOLTAGE) {
    systemOn = false;
    enterState(CHARGER_OFF);
    Serial.println("[STOP] PV voltage is below shutdown limit");
  }
}

void runChargerControl() {
  checkSafety();
  if (!systemOn || faultLatched) return;

  switch (chargerState) {
    case CHARGER_SOFTSTART:
      runSoftStart();
      break;

    case CHARGER_MPPT_CC:
      runMpptCc();
      break;

    case CHARGER_CV:
      runCv();
      break;

    case CHARGER_FULL:
      if (vBat <= RESTART_VOLTAGE &&
          vSolar >= MIN_PV_START_VOLTAGE) {
        resetControllers();
        digitalWrite(RELAY_PV_PIN, HIGH);
        enterState(CHARGER_SOFTSTART);
      }
      break;

    default:
      break;
  }
}

// --------------------------------------------------------------------------
// User interface
// --------------------------------------------------------------------------
const char *stateName(ChargerState state) {
  switch (state) {
    case CHARGER_SOFTSTART: return "SOFT";
    case CHARGER_MPPT_CC:   return "MPPT/CC";
    case CHARGER_CV:        return "CV";
    case CHARGER_FULL:      return "FULL";
    case CHARGER_FAULT:     return "FAULT";
    default:                return "OFF";
  }
}

void updateLcd() {
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    return;
  }

  char line[21];

  lcd.setCursor(0, 0);
  snprintf(line, sizeof(line), "%-8s D:%3d%%",
           stateName(chargerState),
           (int)lroundf(dutyCommand * 100.0f));
  lcd.printf("%-20s", line);

  lcd.setCursor(0, 1);
  snprintf(line, sizeof(line), "PV %5.1fV %4.1fA", vSolar, iSolar);
  lcd.printf("%-20s", line);

  lcd.setCursor(0, 2);
  snprintf(line, sizeof(line), "BT %5.1fV %4.1fA", vBat, iBatCharge);
  lcd.printf("%-20s", line);

  lcd.setCursor(0, 3);
  snprintf(line, sizeof(line), "REF:%4.1f P:%4.0fW",
           mpptVref,
           vSolar * iSolar);
  lcd.printf("%-20s", line);

  xSemaphoreGive(i2cMutex);
}

// --------------------------------------------------------------------------
// FreeRTOS tasks
// --------------------------------------------------------------------------
void controlTask(void *parameter) {
  uint32_t lastDebugMs = 0;

  for (;;) {
    uint32_t now = millis();

    bool sampled = sampleSensors();
    if (!sampled &&
        systemOn &&
        now - lastSampleMs > SENSOR_TIMEOUT_MS) {
      faultLatched = true;
      systemOn = false;
      enterState(CHARGER_FAULT);
      Serial.println("[FAULT] ADC timeout");
    }

    if (sampled) {
      runChargerControl();
      totalWh +=
          vBat * iBatCharge * CONTROL_DT / 3600.0f;
    }

    if (now - lastDebugMs >= DEBUG_PERIOD_MS) {
      lastDebugMs = now;
      Serial.printf(
          "[%s] PV %.2fV %.2fA %.1fW Vref %.2fV | "
          "BAT %.2fV %.2fA | Duty %.3f\n",
          stateName(chargerState),
          vSolar,
          iSolar,
          vSolar * iSolar,
          mpptVref,
          vBat,
          iBatCharge,
          dutyCommand);
    }

    vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
  }
}

void uiTask(void *parameter) {
  bool previousStart = HIGH;
  bool previousStop = HIGH;
  uint32_t lastLcdMs = 0;

  for (;;) {
    bool start = digitalRead(BUTTON_START_PIN);
    bool stop = digitalRead(BUTTON_STOP_PIN);

    if (stop == LOW && previousStop == HIGH) {
      systemOn = false;
      faultLatched = false;
      resetControllers();
      enterState(CHARGER_OFF);
    }

    if (start == LOW && previousStart == HIGH) {
      if (sensorOk &&
          !faultLatched &&
          vSolar >= MIN_PV_START_VOLTAGE) {
        resetControllers();
        systemOn = true;
        digitalWrite(RELAY_AC_PIN, LOW);
        digitalWrite(RELAY_PV_PIN, HIGH);
        enterState(CHARGER_SOFTSTART);
      }
    }

    previousStart = start;
    previousStop = stop;

    if (millis() - lastLcdMs >= LCD_PERIOD_MS) {
      lastLcdMs = millis();
      updateLcd();
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// --------------------------------------------------------------------------
// Arduino setup/loop
// --------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.printf("[BOOT] %s\n", FW_VERSION_TAG);

  pinMode(RELAY_PV_PIN, OUTPUT);
  pinMode(RELAY_AC_PIN, OUTPUT);
  pinMode(BUTTON_START_PIN, INPUT_PULLUP);
  pinMode(BUTTON_STOP_PIN, INPUT_PULLUP);

  ledcAttach(PWM_FORWARD_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(PWM_BOOST_PIN, PWM_FREQ, PWM_RES);
  disablePowerStage();

  Wire.begin(21, 22);
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setTimeOut(25);

  i2cMutex = xSemaphoreCreateMutex();

  bool voltageAdcOk = ads_volt.begin(0x48);
  bool currentAdcOk = ads_curr.begin(0x49);
  sensorOk = voltageAdcOk && currentAdcOk;

  if (sensorOk) {
    ads_volt.setDataRate(RATE_ADS1115_860SPS);
    ads_curr.setDataRate(RATE_ADS1115_860SPS);
    calibrateCurrentOffsets();
  } else {
    faultLatched = true;
    chargerState = CHARGER_FAULT;
    Serial.println("[FAULT] ADS1115 initialization failed");
  }

  lcd.init();
  lcd.backlight();
  lcd.clear();

  xTaskCreatePinnedToCore(
      controlTask, "PV_Control", 6144, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(
      uiTask, "PV_UI", 4096, nullptr, 1, nullptr, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
