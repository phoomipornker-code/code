#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <stdarg.h>

/*
 * Dual-source regulated power supply for ESP32
 *
 * BOOST:
 *   Input       : 650 W PV panel
 *   PV control  : constant-voltage MPPT at 42 V
 *   Output      : 58.0 V, 0..3.0 A
 *   PWM         : GPIO 27, 50 kHz
 *
 * FORWARD:
 *   Input       : 150 VDC
 *   Output      : 58.4 V, 0..3.0 A
 *   PWM         : GPIO 14, 67 kHz, maximum duty 45%
 *
 * This is power-supply firmware. Battery-presence, SOC, FULL/DONE,
 * BMS-open and charge-restart logic from the old charger are removed.
 */

const char *FW_VERSION_TAG = "v81-control-psu-quiet-3a-v3";

// -------------------------------------------------------------------------
// Hardware
// -------------------------------------------------------------------------
const int RELAY_PV_PIN     = 32;
const int RELAY_DC_PIN     = 33;
const int BUTTON_START_PIN = 25;
const int BUTTON_STOP_PIN  = 26;
const int PWM_FORWARD_PIN  = 14;
const int PWM_BOOST_PIN    = 27;

const int PWM_FREQ_BOOST   = 50000;
const int PWM_FREQ_FORWARD = 67000;
const int PWM_RES          = 10;
const int PWM_FULL_SCALE   = (1 << PWM_RES) - 1; // 1023

// Channels 0 and 2 use separate LEDC timers on ESP32.
const int PWM_FORWARD_CHANNEL = 0;
const int PWM_BOOST_CHANNEL   = 2;

const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;
const uint32_t I2C_CLOCK_HZ = 100000;

Adafruit_ADS1115 ads_volt;
Adafruit_ADS1115 ads_curr;
LiquidCrystal_I2C lcd(0x27, 20, 4);
SemaphoreHandle_t i2cMutex;

// ADS map retained from the working hardware:
// ads_volt: ch0=PV, ch2=150 VDC input, ch1=output
// ads_curr: ch0=PV input, ch1=150 VDC input, ch2=output

// -------------------------------------------------------------------------
// User setpoints
// -------------------------------------------------------------------------
const float BOOST_OUTPUT_VOLTAGE_V   = 58.0f;
const float FORWARD_OUTPUT_VOLTAGE_V = 58.4f;
const float OUTPUT_CURRENT_LIMIT_A   = 3.0f;
const float PV_MPPT_VOLTAGE_V        = 42.0f;

// -------------------------------------------------------------------------
// PWM and input limits
// -------------------------------------------------------------------------
const float BOOST_DUTY_MAX_FRAC   = 0.74f;
const float FORWARD_DUTY_MAX_FRAC = 0.45f;
const int MAX_DUTY_BOOST =
    (int)(BOOST_DUTY_MAX_FRAC * PWM_FULL_SCALE + 0.5f);     // 757
const int MAX_DUTY_FORWARD =
    (int)(FORWARD_DUTY_MAX_FRAC * PWM_FULL_SCALE + 0.5f);   // 460

const float PV_START_MIN_V       = 44.0f;
const float PV_RUNNING_MIN_V     = 36.0f;
const float PV_INPUT_CURRENT_MAX_A = 16.3f;

const float DC_START_MIN_V       = 120.0f;
const float DC_RUNNING_MIN_V     = 105.0f;
const float DC_INPUT_MAX_V       = 180.0f;
const float DC_INPUT_CURRENT_MAX_A = 2.5f;
const unsigned long FORWARD_INPUT_GLITCH_HOLD_MS = 1500;

// Output protection. This must remain above the 58.4 V regulation target.
const float OUTPUT_OVP_TRIP_V    = 60.0f;
const float OUTPUT_OVP_RELEASE_V = 58.0f;
const float OUTPUT_OC_WARN_A     = OUTPUT_CURRENT_LIMIT_A;
const float OUTPUT_OC_TRIP_A     = OUTPUT_CURRENT_LIMIT_A + 0.60f;
const unsigned long OUTPUT_OC_CONFIRM_MS = 100;
const unsigned long INPUT_BAD_CONFIRM_MS = 1000;
const unsigned long ADC_STALE_TIMEOUT_MS = 1000;

// -------------------------------------------------------------------------
// Field-proven v81 control values, adapted from charger to continuous PSU
// -------------------------------------------------------------------------
// Boost: SoftStart -> CC/MPPT -> CV (FULL/DONE and battery taper removed)
const float BOOST_CURR_KP = 14.0f;
const float BOOST_CURR_KI = 55.0f;
const float BOOST_CURR_DELTA_MIN = -35.0f;
const float BOOST_CURR_DELTA_MAX = 45.0f;
const float BOOST_VOLT_KP = 0.85f;
const float BOOST_VOLT_KI = 0.45f;
const float BOOST_CV_IREF_SLEW_A = 0.08f;
const float BOOST_CV_NEAR_BAND_V = 0.35f;
const float BOOST_CV_ENTRY_V = BOOST_OUTPUT_VOLTAGE_V - 0.50f;
const float BOOST_CV_EXIT_V = BOOST_OUTPUT_VOLTAGE_V - 1.20f;
const unsigned long BOOST_CV_EXIT_CONFIRM_MS = 5000;
const unsigned long BOOST_MPPT_UPDATE_MS = 100;
const float BOOST_MPPT_DEADBAND_V = 0.20f;
const float BOOST_MPPT_IREF_STEP_UP_A = 0.03f;
const float BOOST_MPPT_IREF_STEP_DOWN_A = 0.08f;
const float BOOST_DUTY_SLEW_UP   = 2.0f;
const float BOOST_DUTY_SLEW_DOWN = 4.0f;

// Forward: original v81 step/hysteresis controller (no PI)
const float FWD_CV_ENTRY_V = FORWARD_OUTPUT_VOLTAGE_V - 1.00f;
const float FWD_CV_EXIT_V = FORWARD_OUTPUT_VOLTAGE_V - 1.80f;
const float FWD_CV_NEAR_BAND_V = 0.30f;
const float FWD_CV_HOLD_BAND_V = 0.10f;
const float FWD_CC_HOLD_BAND_A = 0.15f;
const float FWD_CC_FAR_BAND_A = 0.60f;
const float FWD_STEP_UP_SOFT = 0.4f;
const float FWD_STEP_UP_CC = 0.3f;
const float FWD_STEP_UP_CC_FAR = 0.6f;
const float FWD_STEP_DOWN_CC = 0.8f;
const float FWD_STEP_DOWN_CC_FINE = 0.3f;
const float FWD_STEP_UP_CV = 0.20f;
const float FWD_STEP_UP_CV_NEAR = 0.10f;
const float FWD_STEP_DOWN_CV = 0.40f;
const float FWD_STEP_DOWN_CV_FINE = 0.15f;
const float FWD_STEP_DOWN_CV_OVER = 0.80f;
const float FWD_DC_HOLD_CLIMB_V = 115.0f;
const float FWD_SOFTSTART_SEED_DUTY = 60.0f;
const float FWD_NS_NP_EST = 0.70f;
const float FWD_SOFTSTART_SEED_FRAC = 0.45f;

const unsigned long BOOST_SOFTSTART_MS   = 2500;
const unsigned long FORWARD_SOFTSTART_MS = 5000;
const unsigned long DEBUG_SERIAL_INTERVAL_MS = 5000;

// -------------------------------------------------------------------------
// Calibration retained from the old working firmware
// -------------------------------------------------------------------------
const float OFFSET_V_SOLAR = 0.0f;
const float OFFSET_V_DC    = 0.0f;
const float OFFSET_V_OUT   = 0.0f;

const float CAL_SCALE_V_SOLAR = 41.9f;
const float FIELD_TRIM_V_SOLAR = 0.9589f;
const float CAL_SCALE_V_DC  = 71.43f;
const float CAL_SCALE_V_OUT = 41.5f;

const float DEFAULT_OFFSET_I_SOLAR = 1659.7f;
const float DEFAULT_OFFSET_I_DC    = 1646.9f;
const float DEFAULT_OFFSET_I_OUT   = 1646.9f;
const float CAL_SCALE_I_SOLAR = 42.46f;
const float CAL_SCALE_I_DC    = 42.46f;
const float CAL_SCALE_I_OUT   = 42.46f;

const float NOISE_V_THRESHOLD = 0.5f;
const float NOISE_I_THRESHOLD = 0.08f;

// -------------------------------------------------------------------------
// Runtime state
// -------------------------------------------------------------------------
enum SupplyMode {
    MODE_BOOST,
    MODE_FORWARD
};

enum FaultCode {
    FAULT_NONE,
    FAULT_ADC,
    FAULT_OUTPUT_OVP,
    FAULT_OUTPUT_OC,
    FAULT_INPUT
};

enum BoostControlMode {
    BOOST_SOFTSTART,
    BOOST_CC_MPPT,
    BOOST_CV
};

enum ForwardControlMode {
    FORWARD_SOFTSTART,
    FORWARD_CC,
    FORWARD_CV
};

volatile SupplyMode selectedMode = MODE_BOOST;
volatile BoostControlMode boostControlMode = BOOST_SOFTSTART;
volatile ForwardControlMode forwardControlMode = FORWARD_SOFTSTART;
volatile bool outputEnabled = false;
volatile bool powerStageActive = false;
volatile FaultCode faultCode = FAULT_NONE;

volatile float vPv = 0.0f;
volatile float vDc = 0.0f;
volatile float vOut = 0.0f;
volatile float iPv = 0.0f;
volatile float iDc = 0.0f;
volatile float iOut = 0.0f;
volatile float vOutFiltered = 0.0f;
volatile float iOutFiltered = 0.0f;
volatile float activeCurrentReference = 0.0f;
volatile int activeDutyRaw = 0;
volatile int activeDutyPercent = 0;
volatile bool softStartActive = false;
volatile int softStartPercent = 0;
volatile bool lcdBlankedForRun = false;
volatile unsigned long lastAdcSampleMs = 0;

float currentOffsetPv  = DEFAULT_OFFSET_I_SOLAR;
float currentOffsetDc  = DEFAULT_OFFSET_I_DC;
float currentOffsetOut = DEFAULT_OFFSET_I_OUT;

float boostCurrIntegrator = 0.0f;
float boostVoltIntegrator = 0.0f;
float dutyAccumulator = 0.0f;
float boostPvReference = PV_MPPT_VOLTAGE_V;
float boostLastPower = 0.0f;
float boostLastPvVoltage = 0.0f;
float boostMpptCurrentReference = 1.0f;
float boostCvCurrentReference = 0.0f;
int boostMpptDirection = 1;
float lastGoodDcVoltage = NAN;

unsigned long lastControlUs = 0;
unsigned long softStartBeginMs = 0;
unsigned long boostLastMpptUpdateMs = 0;
unsigned long boostCvExitSinceMs = 0;
unsigned long forwardInputGlitchSinceMs = 0;
unsigned long lastInputGlitchLogMs = 0;
unsigned long inputBadSinceMs = 0;
unsigned long outputOcSinceMs = 0;

float vOutFilterBuffer[8] = {0};
float iOutFilterBuffer[8] = {0};
float vOutFilterSum = 0.0f;
float iOutFilterSum = 0.0f;
int filterIndex = 0;
int filterCount = 0;

// -------------------------------------------------------------------------
// Utility
// -------------------------------------------------------------------------
static inline float clampFloat(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static inline float applySlew(float target, float current,
                              float upStep, float downStep) {
    float change = target - current;
    if (change > upStep) return current + upStep;
    if (change < -downStep) return current - downStep;
    return target;
}

/*
 * Discrete PI with conditional anti-windup.
 * The current integrator state is used for this sample, then K*Ts*error is
 * stored for the next sample, matching KTs/(z-1).
 */
static inline float runPI(float error, float kp, float ki, float dt,
                          float *integrator, float outputMin,
                          float outputMax) {
    float unclamped = kp * error + *integrator;
    float output = clampFloat(unclamped, outputMin, outputMax);

    bool canIntegrate =
        (unclamped > outputMin && unclamped < outputMax) ||
        (unclamped >= outputMax && error < 0.0f) ||
        (unclamped <= outputMin && error > 0.0f);

    if (canIntegrate) {
        *integrator += ki * error * dt;
    }
    return output;
}

static inline int16_t readADCStable(Adafruit_ADS1115 &adc,
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

static void lcdPrintLineRaw(uint8_t row, const char *text) {
    char line[21];
    snprintf(line, sizeof(line), "%-20.20s", text);
    lcd.setCursor(0, row);
    lcd.print(line);
}

static void lcdPrintLineFmt(uint8_t row, const char *format, ...) {
    char text[48];
    va_list args;
    va_start(args, format);
    vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    lcdPrintLineRaw(row, text);
}

static const char *faultName(FaultCode fault) {
    switch (fault) {
        case FAULT_ADC:        return "ADC";
        case FAULT_OUTPUT_OVP: return "OUTPUT OVP";
        case FAULT_OUTPUT_OC:  return "OUTPUT OC";
        case FAULT_INPUT:      return "INPUT";
        default:               return "NONE";
    }
}

// -------------------------------------------------------------------------
// Power-stage management
// -------------------------------------------------------------------------
static void resetControllers() {
    boostCurrIntegrator = 0.0f;
    boostVoltIntegrator = 0.0f;
    boostControlMode = BOOST_SOFTSTART;
    forwardControlMode = FORWARD_SOFTSTART;
    activeCurrentReference = 0.0f;
    boostPvReference = PV_MPPT_VOLTAGE_V;
    boostLastPower = 0.0f;
    boostLastPvVoltage = vPv;
    boostMpptCurrentReference = 1.0f;
    boostCvCurrentReference = 0.0f;
    boostMpptDirection = 1;
    dutyAccumulator = 0.0f;
    lastControlUs = 0;
    softStartBeginMs = 0;
    boostLastMpptUpdateMs = 0;
    boostCvExitSinceMs = 0;
    softStartActive = false;
    softStartPercent = 0;
    inputBadSinceMs = 0;
    forwardInputGlitchSinceMs = 0;
    outputOcSinceMs = 0;
}

static void disablePowerStage() {
    powerStageActive = false;
    activeDutyRaw = 0;
    activeDutyPercent = 0;
    digitalWrite(RELAY_PV_PIN, LOW);
    digitalWrite(RELAY_DC_PIN, LOW);
    ledcWrite(PWM_BOOST_PIN, 0);
    ledcWrite(PWM_FORWARD_PIN, 0);
    resetControllers();
}

static void tripFault(FaultCode fault, const char *reason) {
    if (faultCode == FAULT_NONE) {
        faultCode = fault;
        Serial.printf("[FAULT] %s: %s\n", faultName(fault), reason);
    }
    outputEnabled = false;
    disablePowerStage();
}

static bool selectedInputReady() {
    if (selectedMode == MODE_BOOST) {
        return vPv >= PV_START_MIN_V;
    }
    return vDc >= DC_START_MIN_V && vDc <= DC_INPUT_MAX_V;
}

static bool selectedInputRunningOkay() {
    if (selectedMode == MODE_BOOST) {
        return vPv >= PV_RUNNING_MIN_V &&
               fabsf(iPv) <= PV_INPUT_CURRENT_MAX_A;
    }
    return vDc >= DC_RUNNING_MIN_V &&
           vDc <= DC_INPUT_MAX_V &&
           fabsf(iDc) <= DC_INPUT_CURRENT_MAX_A;
}

static void startSelectedPowerStage() {
    disablePowerStage();

    // Ensure both gate drives are off before changing relays.
    vTaskDelay(pdMS_TO_TICKS(200));

    if (!outputEnabled || faultCode != FAULT_NONE) return;

    if (selectedMode == MODE_BOOST) {
        digitalWrite(RELAY_DC_PIN, LOW);
        digitalWrite(RELAY_PV_PIN, HIGH);
        Serial.println("[START] BOOST 50kHz, PV target 42V");
    } else {
        digitalWrite(RELAY_PV_PIN, LOW);
        digitalWrite(RELAY_DC_PIN, HIGH);
        Serial.println("[START] FORWARD 67kHz, input 150VDC");
    }

    vTaskDelay(pdMS_TO_TICKS(200));
    resetControllers();
    softStartBeginMs = millis();
    softStartActive = true;
    softStartPercent = 0;
    boostControlMode = BOOST_SOFTSTART;
    forwardControlMode = FORWARD_SOFTSTART;
    powerStageActive = true;
}

// -------------------------------------------------------------------------
// Measurement
// -------------------------------------------------------------------------
static void calibrateCurrentOffsetsAtBoot() {
    const int samples = 80;
    float sumPv = 0.0f;
    float sumDc = 0.0f;
    float sumOut = 0.0f;

    disablePowerStage();
    vTaskDelay(pdMS_TO_TICKS(100));

    for (int i = 0; i < samples; ++i) {
        sumPv += readADCStable(ads_curr, 0, true) * 0.1875f;
        sumDc += readADCStable(ads_curr, 1, true) * 0.1875f;
        sumOut += readADCStable(ads_curr, 2, true) * 0.1875f;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    currentOffsetPv = sumPv / samples;
    currentOffsetDc = sumDc / samples;
    currentOffsetOut = sumOut / samples;

    Serial.printf("[CAL] IPv=%.2fmV IDC=%.2fmV Iout=%.2fmV\n",
                  currentOffsetPv, currentOffsetDc, currentOffsetOut);
}

static bool updateMeasurements() {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(80)) != pdTRUE) {
        return false;
    }

    float rawVPv  = readADCStable(ads_volt, 0) * 0.1875f;
    float rawVDc  = readADCStable(ads_volt, 2) * 0.1875f;
    float rawVOut = readADCStable(ads_volt, 1) * 0.1875f;
    float rawIPv  = readADCStable(ads_curr, 0) * 0.1875f;
    float rawIDc  = readADCStable(ads_curr, 1) * 0.1875f;
    float rawIOut = readADCStable(ads_curr, 2) * 0.1875f;

    xSemaphoreGive(i2cMutex);

    float newVPv =
        fmaxf(0.0f, rawVPv - OFFSET_V_SOLAR) /
        1000.0f * CAL_SCALE_V_SOLAR * FIELD_TRIM_V_SOLAR;
    float newVDc =
        fmaxf(0.0f, rawVDc - OFFSET_V_DC) /
        1000.0f * CAL_SCALE_V_DC;
    float newVOut =
        fmaxf(0.0f, rawVOut - OFFSET_V_OUT) /
        1000.0f * CAL_SCALE_V_OUT;

    float newIPv =
        (rawIPv - currentOffsetPv) / 1000.0f * CAL_SCALE_I_SOLAR;
    float newIDc =
        (rawIDc - currentOffsetDc) / 1000.0f * CAL_SCALE_I_DC;
    float newIOut =
        (rawIOut - currentOffsetOut) / 1000.0f * CAL_SCALE_I_OUT;

    if (!isfinite(newVPv) || !isfinite(newVDc) ||
        !isfinite(newVOut) || !isfinite(newIPv) ||
        !isfinite(newIDc) || !isfinite(newIOut)) {
        return false;
    }

    if (newVPv < NOISE_V_THRESHOLD) newVPv = 0.0f;
    if (newVDc < NOISE_V_THRESHOLD) newVDc = 0.0f;
    if (newVOut < NOISE_V_THRESHOLD) newVOut = 0.0f;
    if (fabsf(newIPv) < NOISE_I_THRESHOLD) newIPv = 0.0f;
    if (fabsf(newIDc) < NOISE_I_THRESHOLD) newIDc = 0.0f;
    if (fabsf(newIOut) < NOISE_I_THRESHOLD) newIOut = 0.0f;

    // Reject impossible output samples without requiring a battery-like
    // minimum voltage; a power supply must be allowed to start from 0 V.
    if (newVOut > 70.0f || fabsf(newIOut) > 20.0f) {
        return false;
    }

    /*
     * The working v95 firmware held brief DC-input dropouts while output
     * current was still flowing. A reading such as Vin=3 V with Iin=2 A,
     * followed by 148 V, is an ADS/I2C/EMI glitch rather than a real outage.
     */
    unsigned long sampleNow = millis();
    bool forwardRunning =
        powerStageActive && selectedMode == MODE_FORWARD;
    bool outputStillFlowing =
        fabsf(newIOut) > 0.35f || iOutFiltered > 0.35f;
    bool severeDcDrop =
        newVDc < 20.0f ||
        (!isnan(lastGoodDcVoltage) &&
         newVDc < lastGoodDcVoltage - 60.0f);

    if (forwardRunning && outputStillFlowing && severeDcDrop &&
        !isnan(lastGoodDcVoltage)) {
        if (forwardInputGlitchSinceMs == 0) {
            forwardInputGlitchSinceMs = sampleNow;
        }
        if (sampleNow - forwardInputGlitchSinceMs <
            FORWARD_INPUT_GLITCH_HOLD_MS) {
            if (sampleNow - lastInputGlitchLogMs >= 1000) {
                lastInputGlitchLogMs = sampleNow;
                Serial.printf(
                    "[WARN] DC input ADC glitch %.1fV; holding %.1fV\n",
                    newVDc,
                    lastGoodDcVoltage
                );
            }
            newVDc = lastGoodDcVoltage;
        }
    } else {
        forwardInputGlitchSinceMs = 0;
    }

    if (newVDc >= DC_RUNNING_MIN_V && newVDc <= DC_INPUT_MAX_V) {
        lastGoodDcVoltage = newVDc;
    }

    vPv = newVPv;
    vDc = newVDc;
    vOut = newVOut;
    iPv = newIPv;
    iDc = newIDc;
    iOut = newIOut;

    vOutFilterSum -= vOutFilterBuffer[filterIndex];
    iOutFilterSum -= iOutFilterBuffer[filterIndex];
    vOutFilterBuffer[filterIndex] = newVOut;
    iOutFilterBuffer[filterIndex] = fabsf(newIOut);
    vOutFilterSum += vOutFilterBuffer[filterIndex];
    iOutFilterSum += iOutFilterBuffer[filterIndex];
    filterIndex = (filterIndex + 1) % 8;
    if (filterCount < 8) ++filterCount;

    vOutFiltered = vOutFilterSum / (float)filterCount;
    iOutFiltered = iOutFilterSum / (float)filterCount;
    lastAdcSampleMs = sampleNow;
    return true;
}

// -------------------------------------------------------------------------
// Controllers
// -------------------------------------------------------------------------
static int estimateBoostDutyRaw() {
    float input = fmaxf(vPv, 38.0f);
    float duty = 1.0f - input / BOOST_OUTPUT_VOLTAGE_V + 0.03f;
    duty = clampFloat(duty, 0.05f, BOOST_DUTY_MAX_FRAC);
    return constrain(
        (int)lroundf(duty * PWM_FULL_SCALE),
        20,
        MAX_DUTY_BOOST
    );
}

static int estimateForwardDutyRaw() {
    float input = fmaxf(vDc, 80.0f);
    float dutyFromVoltage =
        FORWARD_OUTPUT_VOLTAGE_V / (input * FWD_NS_NP_EST);
    dutyFromVoltage =
        clampFloat(dutyFromVoltage, 0.08f, FORWARD_DUTY_MAX_FRAC);
    float seedDuty =
        fmaxf(dutyFromVoltage, FORWARD_DUTY_MAX_FRAC) *
        FWD_SOFTSTART_SEED_FRAC;
    seedDuty =
        clampFloat(seedDuty, 0.05f, FORWARD_DUTY_MAX_FRAC);
    return constrain(
        (int)lroundf(seedDuty * PWM_FULL_SCALE),
        (int)FWD_SOFTSTART_SEED_DUTY,
        MAX_DUTY_FORWARD
    );
}

static void runBoostV81Controller(float dt, unsigned long now,
                                  float softStartFraction) {
    float controlCurrent = fmaxf(fabsf(iOut), iOutFiltered);

    if (boostControlMode == BOOST_SOFTSTART) {
        activeCurrentReference =
            OUTPUT_CURRENT_LIMIT_A * softStartFraction;
        float seed = (float)estimateBoostDutyRaw();
        dutyAccumulator = applySlew(
            seed,
            dutyAccumulator,
            2.0f,
            5.0f
        );

        if (controlCurrent >
            activeCurrentReference + FWD_CC_HOLD_BAND_A) {
            dutyAccumulator -= 2.0f;
        }

        if (now - softStartBeginMs >= BOOST_SOFTSTART_MS) {
            boostControlMode = BOOST_CC_MPPT;
            boostCurrIntegrator = 0.0f;
        }
        return;
    }

    if (boostControlMode == BOOST_CC_MPPT) {
        if (boostLastMpptUpdateMs == 0 ||
            now - boostLastMpptUpdateMs >= BOOST_MPPT_UPDATE_MS) {
            boostLastMpptUpdateMs = now;
            float pvError = vPv - PV_MPPT_VOLTAGE_V;
            if (pvError > BOOST_MPPT_DEADBAND_V) {
                boostMpptCurrentReference +=
                    BOOST_MPPT_IREF_STEP_UP_A;
            } else if (pvError < -BOOST_MPPT_DEADBAND_V) {
                boostMpptCurrentReference -=
                    BOOST_MPPT_IREF_STEP_DOWN_A;
            }
            boostMpptCurrentReference = clampFloat(
                boostMpptCurrentReference,
                0.0f,
                OUTPUT_CURRENT_LIMIT_A
            );
        }

        activeCurrentReference =
            fminf(OUTPUT_CURRENT_LIMIT_A,
                  boostMpptCurrentReference);

        if (vPv < PV_MPPT_VOLTAGE_V - 1.0f) {
            float scale = clampFloat(
                1.0f -
                    (PV_MPPT_VOLTAGE_V - 1.0f - vPv) * 0.35f,
                0.15f,
                1.0f
            );
            activeCurrentReference *= scale;
        }

        float currentError =
            activeCurrentReference - controlCurrent;
        float dutyDelta = runPI(
            currentError,
            BOOST_CURR_KP,
            BOOST_CURR_KI,
            dt,
            &boostCurrIntegrator,
            BOOST_CURR_DELTA_MIN,
            BOOST_CURR_DELTA_MAX
        );
        if (currentError > 0.8f &&
            dutyAccumulator < 280.0f) {
            dutyDelta = fmaxf(dutyDelta, 3.0f);
        }

        dutyAccumulator = applySlew(
            dutyAccumulator + dutyDelta,
            dutyAccumulator,
            BOOST_DUTY_SLEW_UP,
            BOOST_DUTY_SLEW_DOWN
        );

        if (fmaxf(vOut, vOutFiltered) >= BOOST_CV_ENTRY_V) {
            boostControlMode = BOOST_CV;
            boostVoltIntegrator = 0.0f;
            boostCurrIntegrator = 0.0f;
            boostCvCurrentReference =
                clampFloat(controlCurrent, 0.0f,
                           OUTPUT_CURRENT_LIMIT_A);
            boostCvExitSinceMs = 0;
        }
        return;
    }

    // Continuous CV mode: unlike the charger, there is no FULL/DONE state.
    float boostControlVoltage =
        fmaxf(vOut, vOutFiltered);
    float voltageError =
        BOOST_OUTPUT_VOLTAGE_V - boostControlVoltage;
    float requestedCurrent = runPI(
        voltageError,
        BOOST_VOLT_KP,
        BOOST_VOLT_KI,
        dt,
        &boostVoltIntegrator,
        0.0f,
        OUTPUT_CURRENT_LIMIT_A
    );
    boostCvCurrentReference = applySlew(
        requestedCurrent,
        boostCvCurrentReference,
        BOOST_CV_IREF_SLEW_A,
        BOOST_CV_IREF_SLEW_A
    );
    activeCurrentReference =
        clampFloat(boostCvCurrentReference,
                   0.0f, OUTPUT_CURRENT_LIMIT_A);

    bool nearVoltage =
        fabsf(voltageError) <= BOOST_CV_NEAR_BAND_V;
    float currentError =
        activeCurrentReference - controlCurrent;
    float dutyDelta = runPI(
        currentError,
        BOOST_CURR_KP * (nearVoltage ? 0.55f : 0.85f),
        BOOST_CURR_KI * (nearVoltage ? 0.45f : 0.70f),
        dt,
        &boostCurrIntegrator,
        nearVoltage ? -8.0f : BOOST_CURR_DELTA_MIN,
        nearVoltage ? 8.0f : 18.0f
    );
    float stepLimit = nearVoltage ? 0.8f : 2.5f;
    dutyDelta = clampFloat(dutyDelta, -stepLimit, stepLimit);

    if (boostControlVoltage >= BOOST_OUTPUT_VOLTAGE_V - 0.10f &&
        dutyDelta > 0.0f) {
        dutyDelta = 0.0f;
    }
    if (boostControlVoltage > BOOST_OUTPUT_VOLTAGE_V) {
        dutyDelta -=
            0.8f +
            (boostControlVoltage - BOOST_OUTPUT_VOLTAGE_V) * 4.0f;
        boostVoltIntegrator *= 0.85f;
    }

    dutyAccumulator = applySlew(
        dutyAccumulator + dutyDelta,
        dutyAccumulator,
        nearVoltage ? 0.8f : 2.5f,
        nearVoltage ? 2.0f : 4.0f
    );

    if (vOutFiltered <= BOOST_CV_EXIT_V) {
        if (boostCvExitSinceMs == 0) {
            boostCvExitSinceMs = now;
        }
        if (now - boostCvExitSinceMs >=
            BOOST_CV_EXIT_CONFIRM_MS) {
            boostControlMode = BOOST_CC_MPPT;
            boostVoltIntegrator = 0.0f;
            boostCurrIntegrator = 0.0f;
            boostCvExitSinceMs = 0;
        }
    } else {
        boostCvExitSinceMs = 0;
    }
}

static void runForwardV81Controller(unsigned long now,
                                    float softStartFraction) {
    // Match the quiet v81 behavior: regulation follows the filtered current.
    // Raw current is still used by applyOutputProtection() for fast safety.
    float controlCurrent = iOutFiltered;
    float fastCurrent = fmaxf(fabsf(iOut), iOutFiltered);
    bool freezeDutyUp =
        vDc < FWD_DC_HOLD_CLIMB_V ||
        inputBadSinceMs != 0;

    // The 8-frame filter is intentionally quiet but delayed. Never wait for
    // it when the raw current has already reached the configured 3 A limit.
    if (fastCurrent >= OUTPUT_CURRENT_LIMIT_A) {
        activeCurrentReference = OUTPUT_CURRENT_LIMIT_A;
        dutyAccumulator -=
            2.0f +
            (fastCurrent - OUTPUT_CURRENT_LIMIT_A) * 6.0f;
        if (dutyAccumulator < 0.0f) dutyAccumulator = 0.0f;
        return;
    }

    if (forwardControlMode == FORWARD_SOFTSTART) {
        activeCurrentReference =
            OUTPUT_CURRENT_LIMIT_A * softStartFraction;
        float seed = (float)estimateForwardDutyRaw();

        if (!freezeDutyUp && dutyAccumulator < seed) {
            dutyAccumulator += FWD_STEP_UP_SOFT;
        }
        if (controlCurrent >
            activeCurrentReference + FWD_CC_HOLD_BAND_A) {
            dutyAccumulator -= FWD_STEP_DOWN_CC;
        }

        if (now - softStartBeginMs >= FORWARD_SOFTSTART_MS) {
            forwardControlMode = FORWARD_CC;
        }
        return;
    }

    if (forwardControlMode == FORWARD_CC) {
        activeCurrentReference = OUTPUT_CURRENT_LIMIT_A;
        float currentError =
            activeCurrentReference - controlCurrent;

        if (currentError > FWD_CC_HOLD_BAND_A) {
            if (!freezeDutyUp) {
                dutyAccumulator +=
                    currentError > FWD_CC_FAR_BAND_A
                        ? FWD_STEP_UP_CC_FAR
                        : FWD_STEP_UP_CC;
            }
        } else if (currentError < -FWD_CC_HOLD_BAND_A) {
            dutyAccumulator -=
                currentError < -FWD_CC_FAR_BAND_A
                    ? FWD_STEP_DOWN_CC
                    : FWD_STEP_DOWN_CC_FINE;
        }

        if (vOutFiltered >= FWD_CV_ENTRY_V) {
            forwardControlMode = FORWARD_CV;
        }
        return;
    }

    // Continuous Forward CV mode; no FULL/DONE or battery taper.
    float voltageError =
        FORWARD_OUTPUT_VOLTAGE_V - vOutFiltered;
    activeCurrentReference = OUTPUT_CURRENT_LIMIT_A;

    if (controlCurrent >
        OUTPUT_CURRENT_LIMIT_A + FWD_CC_HOLD_BAND_A) {
        dutyAccumulator -=
            controlCurrent >
                    OUTPUT_CURRENT_LIMIT_A + FWD_CC_FAR_BAND_A
                ? FWD_STEP_DOWN_CC
                : FWD_STEP_DOWN_CC_FINE;
    } else if (voltageError > FWD_CV_HOLD_BAND_V) {
        if (!freezeDutyUp &&
            controlCurrent < OUTPUT_CURRENT_LIMIT_A) {
            dutyAccumulator +=
                voltageError > FWD_CV_NEAR_BAND_V
                    ? FWD_STEP_UP_CV
                    : FWD_STEP_UP_CV_NEAR;
        }
    } else if (voltageError < -FWD_CV_HOLD_BAND_V) {
        float overVoltage = -voltageError;
        dutyAccumulator -=
            overVoltage > FWD_CV_NEAR_BAND_V
                ? FWD_STEP_DOWN_CV_OVER
                : (overVoltage > 0.12f
                    ? FWD_STEP_DOWN_CV
                    : FWD_STEP_DOWN_CV_FINE);
    }

    if (vOutFiltered <= FWD_CV_EXIT_V) {
        forwardControlMode = FORWARD_CC;
    }
}

static void applyOutputProtection(unsigned long now) {
    float peakVoltage = fmaxf(vOut, vOutFiltered);
    float peakCurrent = fmaxf(fabsf(iOut), iOutFiltered);

    if (peakVoltage >= OUTPUT_OVP_TRIP_V) {
        tripFault(FAULT_OUTPUT_OVP, "output exceeded 60V");
        return;
    }

    if (peakCurrent > OUTPUT_OC_WARN_A) {
        // Fast raw-current guard. At 4 A this removes about 12 raw PWM
        // counts per fresh ADS frame instead of waiting for the boxcar filter.
        dutyAccumulator -= 4.0f +
            (peakCurrent - OUTPUT_OC_WARN_A) * 8.0f;
        if (dutyAccumulator < 0.0f) dutyAccumulator = 0.0f;
    }

    if (peakCurrent >= OUTPUT_OC_TRIP_A) {
        if (outputOcSinceMs == 0) outputOcSinceMs = now;
        if (now - outputOcSinceMs >= OUTPUT_OC_CONFIRM_MS) {
            tripFault(FAULT_OUTPUT_OC, "output exceeded current limit");
        }
    } else {
        outputOcSinceMs = 0;
    }
}

// -------------------------------------------------------------------------
// Control task
// -------------------------------------------------------------------------
static void TaskControl(void *parameter) {
    unsigned long lastDebugMs = 0;

    for (;;) {
        unsigned long now = millis();
        bool sampleOkay = updateMeasurements();

        if (!sampleOkay &&
            now - lastAdcSampleMs >= ADC_STALE_TIMEOUT_MS) {
            tripFault(FAULT_ADC, "measurement timeout");
        }

        if (!outputEnabled) {
            if (powerStageActive) disablePowerStage();
        } else if (faultCode != FAULT_NONE) {
            disablePowerStage();
        } else {
            if (!powerStageActive) {
                if (selectedInputReady()) {
                    startSelectedPowerStage();
                } else {
                    outputEnabled = false;
                    Serial.println("[STOP] selected input is not ready");
                }
            }

            if (powerStageActive) {
                if (!selectedInputRunningOkay()) {
                    if (inputBadSinceMs == 0) inputBadSinceMs = now;
                    if (now - inputBadSinceMs >= INPUT_BAD_CONFIRM_MS) {
                        tripFault(FAULT_INPUT, "input outside safe range");
                    }
                } else {
                    inputBadSinceMs = 0;
                }
            }
        }

        // Never change duty from stale feedback. Hold the previous PWM until
        // both ADS1115 devices produced a complete new measurement frame.
        if (powerStageActive && outputEnabled &&
            faultCode == FAULT_NONE && sampleOkay) {
            unsigned long controlNowMs = millis();
            unsigned long nowUs = micros();
            float dt = 0.020f;
            if (lastControlUs != 0) {
                dt = (float)(nowUs - lastControlUs) * 1.0e-6f;
            }
            lastControlUs = nowUs;
            dt = clampFloat(dt, 0.005f, 0.100f);

            int maximumDuty;
            unsigned long softStartDuration =
                selectedMode == MODE_BOOST
                    ? BOOST_SOFTSTART_MS
                    : FORWARD_SOFTSTART_MS;
            float softStartFraction = clampFloat(
                (float)(controlNowMs - softStartBeginMs) /
                    (float)softStartDuration,
                0.0f,
                1.0f
            );
            softStartPercent =
                (int)lroundf(softStartFraction * 100.0f);
            if (selectedMode == MODE_BOOST) {
                maximumDuty = MAX_DUTY_BOOST;
                runBoostV81Controller(
                    dt,
                    controlNowMs,
                    softStartFraction
                );
            } else {
                maximumDuty = MAX_DUTY_FORWARD;
                runForwardV81Controller(
                    controlNowMs,
                    softStartFraction
                );
            }

            if (softStartActive && softStartFraction >= 1.0f) {
                softStartActive = false;
                softStartPercent = 100;
                Serial.printf(
                    "[SOFTSTART] %s complete\n",
                    selectedMode == MODE_BOOST ? "BOOST" : "FORWARD"
                );
            }

            applyOutputProtection(controlNowMs);

            if (faultCode == FAULT_NONE) {
                dutyAccumulator =
                    clampFloat(dutyAccumulator, 0.0f, maximumDuty);
                activeDutyRaw = constrain(
                    (int)lroundf(dutyAccumulator), 0, maximumDuty
                );

                if (selectedMode == MODE_BOOST) {
                    ledcWrite(PWM_FORWARD_PIN, 0);
                    ledcWrite(PWM_BOOST_PIN, activeDutyRaw);
                } else {
                    ledcWrite(PWM_BOOST_PIN, 0);
                    ledcWrite(PWM_FORWARD_PIN, activeDutyRaw);
                }

                activeDutyPercent =
                    (int)lroundf(activeDutyRaw * 100.0f / PWM_FULL_SCALE);
            }
        }

        if (now - lastDebugMs >= DEBUG_SERIAL_INTERVAL_MS) {
            lastDebugMs = now;
            float inputVoltage =
                selectedMode == MODE_BOOST ? vPv : vDc;
            float inputCurrent =
                selectedMode == MODE_BOOST ? fabsf(iPv) : fabsf(iDc);

            Serial.printf(
                "[%s/%s] Vin=%.1fV Iin=%.2fA "
                "Vout=%.2fV Iout=%.2fA Iref=%.2fA "
                "Duty=%d%% Soft=%d%% Fault=%s\n",
                selectedMode == MODE_BOOST ? "BOOST" : "FORWARD",
                outputEnabled ? "ON" : "OFF",
                inputVoltage,
                inputCurrent,
                vOutFiltered,
                iOutFiltered,
                activeCurrentReference,
                activeDutyPercent,
                softStartPercent,
                faultName(faultCode)
            );
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// -------------------------------------------------------------------------
// Buttons and LCD
// -------------------------------------------------------------------------
static void drawDisplay() {
    float inputVoltage = selectedMode == MODE_BOOST ? vPv : vDc;
    float inputCurrent =
        selectedMode == MODE_BOOST ? fabsf(iPv) : fabsf(iDc);
    float voltageSetpoint =
        selectedMode == MODE_BOOST
            ? BOOST_OUTPUT_VOLTAGE_V
            : FORWARD_OUTPUT_VOLTAGE_V;

    if (faultCode != FAULT_NONE) {
        lcdPrintLineFmt(0, "FAULT: %s", faultName(faultCode));
        lcdPrintLineFmt(1, "OUT:%5.1fV %4.2fA", vOutFiltered, iOutFiltered);
        lcdPrintLineRaw(2, "STOP = clear fault");
        lcdPrintLineRaw(3, "Check power stage");
        return;
    }

    lcdPrintLineFmt(
        0,
        "%s %s D:%2d%%",
        selectedMode == MODE_BOOST ? "BOOST" : "FORWARD",
        outputEnabled
            ? (softStartActive ? "SOFT" : "ON")
            : "OFF",
        activeDutyPercent
    );
    lcdPrintLineFmt(1, "IN :%5.1fV %4.2fA", inputVoltage, inputCurrent);
    lcdPrintLineFmt(2, "OUT:%5.1fV %4.2fA", vOutFiltered, iOutFiltered);
    lcdPrintLineFmt(3, "SET:%4.1fV %3.1fA", voltageSetpoint,
                    OUTPUT_CURRENT_LIMIT_A);
}

static void TaskButtonsDisplay(void *parameter) {
    bool lastStartRaw = HIGH;
    bool lastStopRaw = HIGH;
    bool stableStart = HIGH;
    bool stableStop = HIGH;
    unsigned long startChangedMs = 0;
    unsigned long stopChangedMs = 0;
    unsigned long lastDisplayMs = 0;
    const unsigned long debounceMs = 80;

    for (;;) {
        unsigned long now = millis();
        bool startRaw = digitalRead(BUTTON_START_PIN);
        bool stopRaw = digitalRead(BUTTON_STOP_PIN);

        if (startRaw != lastStartRaw) {
            lastStartRaw = startRaw;
            startChangedMs = now;
        }
        if (stopRaw != lastStopRaw) {
            lastStopRaw = stopRaw;
            stopChangedMs = now;
        }

        bool oldStableStart = stableStart;
        bool oldStableStop = stableStop;
        if (now - startChangedMs >= debounceMs) stableStart = startRaw;
        if (now - stopChangedMs >= debounceMs) stableStop = stopRaw;

        bool startPressed =
            stableStart == LOW && oldStableStart == HIGH;
        bool stopPressed =
            stableStop == LOW && oldStableStop == HIGH;

        if (stopPressed) {
            if (outputEnabled || powerStageActive) {
                outputEnabled = false;
                Serial.println("[STOP] output disabled by user");
            } else if (faultCode != FAULT_NONE) {
                if (vOutFiltered <= OUTPUT_OVP_RELEASE_V) {
                    faultCode = FAULT_NONE;
                    resetControllers();
                    Serial.println("[FAULT] cleared by user");
                } else {
                    Serial.println("[FAULT] output voltage still too high");
                }
            } else {
                selectedMode =
                    selectedMode == MODE_BOOST
                        ? MODE_FORWARD
                        : MODE_BOOST;
                Serial.printf(
                    "[MODE] %s\n",
                    selectedMode == MODE_BOOST ? "BOOST" : "FORWARD"
                );
            }
        }

        if (startPressed && !outputEnabled) {
            if (faultCode != FAULT_NONE) {
                Serial.printf("[START] blocked by %s\n",
                              faultName(faultCode));
            } else if (!selectedInputReady()) {
                Serial.println("[START] selected input is not ready");
            } else {
                outputEnabled = true;
            }
        }

        // The old field-proven firmware stopped all LCD I2C traffic while
        // switching. This avoids periodic ADC latency and audible modulation.
        if (outputEnabled && faultCode == FAULT_NONE) {
            if (!lcdBlankedForRun &&
                xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                lcd.clear();
                lcd.noBacklight();
                lcdBlankedForRun = true;
                xSemaphoreGive(i2cMutex);
            }
        } else if (now - lastDisplayMs >= 500) {
            lastDisplayMs = now;
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                if (lcdBlankedForRun) {
                    lcd.backlight();
                    lcd.clear();
                    lcdBlankedForRun = false;
                }
                drawDisplay();
                xSemaphoreGive(i2cMutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// -------------------------------------------------------------------------
// Arduino entry points
// -------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    Serial.printf("[BOOT] %s\n", FW_VERSION_TAG);
    Serial.println("[BOOT] BOOST=58.0V/3A/42V-MPPT @50kHz");
    Serial.println("[BOOT] FORWARD=58.4V/3A/DC150V @67kHz");

    pinMode(RELAY_PV_PIN, OUTPUT);
    pinMode(RELAY_DC_PIN, OUTPUT);
    pinMode(BUTTON_START_PIN, INPUT_PULLUP);
    pinMode(BUTTON_STOP_PIN, INPUT_PULLUP);

    bool forwardPwmOkay = ledcAttachChannel(
        PWM_FORWARD_PIN,
        PWM_FREQ_FORWARD,
        PWM_RES,
        PWM_FORWARD_CHANNEL
    );
    bool boostPwmOkay = ledcAttachChannel(
        PWM_BOOST_PIN,
        PWM_FREQ_BOOST,
        PWM_RES,
        PWM_BOOST_CHANNEL
    );

    digitalWrite(RELAY_PV_PIN, LOW);
    digitalWrite(RELAY_DC_PIN, LOW);
    ledcWrite(PWM_FORWARD_PIN, 0);
    ledcWrite(PWM_BOOST_PIN, 0);

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_CLOCK_HZ);
    Wire.setTimeOut(40);
    i2cMutex = xSemaphoreCreateMutex();

    bool voltageAdcOkay = ads_volt.begin(0x48);
    bool currentAdcOkay = ads_curr.begin(0x49);
    if (voltageAdcOkay) ads_volt.setDataRate(RATE_ADS1115_860SPS);
    if (currentAdcOkay) ads_curr.setDataRate(RATE_ADS1115_860SPS);

    lcd.init();
    lcd.backlight();
    lcd.clear();

    if (!forwardPwmOkay || !boostPwmOkay ||
        !voltageAdcOkay || !currentAdcOkay ||
        i2cMutex == NULL) {
        faultCode = FAULT_ADC;
        lcdPrintLineRaw(0, "INITIALIZATION FAIL");
        lcdPrintLineRaw(1, "Check ADS/PWM/I2C");
        Serial.println("[FATAL] hardware initialization failed");
    } else {
        calibrateCurrentOffsetsAtBoot();
        updateMeasurements();
    }

    xTaskCreatePinnedToCore(
        TaskControl,
        "Control",
        8192,
        NULL,
        3,
        NULL,
        0
    );
    xTaskCreatePinnedToCore(
        TaskButtonsDisplay,
        "ButtonsDisplay",
        4096,
        NULL,
        1,
        NULL,
        1
    );
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
