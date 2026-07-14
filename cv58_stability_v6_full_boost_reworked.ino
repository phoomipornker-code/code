#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <stdarg.h>

const char* FW_VERSION_TAG = "cv58-stability-v12-cv56-conserve";

// =========================================================================
// Hardware
// =========================================================================
const int RELAY_PV_PIN     = 32;
const int RELAY_AC_PIN     = 33;
const int BUTTON_START_PIN = 25;
const int BUTTON_STOP_PIN  = 26;

const int PWM_FORWARD_PIN  = 14;
const int PWM_BOOST_PIN    = 27;

const int PWM_FREQ         = 50000;
const int PWM_RES          = 10;

Adafruit_ADS1115 ads_volt;
Adafruit_ADS1115 ads_curr;

// =========================================================================
// Targets / safety thresholds
// =========================================================================
// CV=56.0V (~3.50V/cell for 16S LFP) — longevity / BMS-friendly cutoff.
const float TARGET_CV_VOLTAGE = 56.00;
const float TARGET_CC_CURRENT = 6.0;

const float MIN_PV_VOLTAGE = 42.0;
const float UNDER_PV_VOLTAGE_CRIT = 39.0;
const float MIN_AC_VOLTAGE = 140.0;

const int MAX_DUTY_FORWARD = 490;
const int MAX_DUTY_BOOST   = 760;

const unsigned long ADC_STALE_TIMEOUT_MS = 700;
const unsigned long SENSOR_ERROR_LOG_MS = 2000;
const float CV_DEADBAND_V = 0.08;
const float FULL_DETECT_VOLTAGE = 55.90;
const float FULL_END_CURRENT = 0.50;
const unsigned long FULL_CONFIRM_MS = 60000;
const float HIGH_VOLTAGE_STOP_VOLTAGE = 56.80;
const unsigned long HIGH_VOLTAGE_STOP_CONFIRM_MS = 300;
const float RESTART_CHARGE_VOLTAGE = 54.0;

// =========================================================================
// Forward (AC) PID
// =========================================================================
const float Kp_cc = 0.15;
const float Ki_cc = 0.01;
const float Kd_cc = 0.005;

const float Kp_cv = 0.5;
const float Ki_cv = 0.015;
const float Kd_cv = 0.005;

float pid_error_cc = 0.0, pid_last_error_cc = 0.0, pid_integral_cc = 0.0;
float pid_error_cv = 0.0, pid_last_error_cv = 0.0, pid_integral_cv = 0.0;

// =========================================================================
// Calibration
// =========================================================================
const float OFFSET_V_SOLAR = 0.0;
const float CAL_SCALE_V_SOLAR = 41.9;
const float FIELD_TRIM_V_SOLAR = 0.9589;  // one-point trim: 44.3 / 46.2

const float OFFSET_V_AC    = 0.0;
const float OFFSET_V_BAT   = 0.0;

const float OFFSET_I_SOLAR = 1659.7;
const float OFFSET_I_AC    = 1646.9;
const float OFFSET_I_BAT   = 1646.9;

const float CAL_SCALE_V_AC    = 71.43;
const float CAL_SCALE_V_BAT   = 41.85;
const float CAL_SCALE_I_SOLAR = 42.46;
const float CAL_SCALE_I_AC    = 42.46;
const float CAL_SCALE_I_BAT   = 42.46;

const float NOISE_V_THRESHOLD = 0.5;
const float NOISE_I_THRESHOLD = 0.08;
const float ADC_RAW_MIN_VALID_MV = 80.0;
const float ADC_GLITCH_CURRENT_GATE_A = 0.35;
const unsigned long ADC_GLITCH_LOG_MS = 1000;
const float MIN_CURRENT_FOR_ACTIVE_CHARGE = 0.20;

// =========================================================================
// BOOST control (new flow): SOFTSTART -> CC_MPPT -> CV -> DONE
// =========================================================================
const float BOOST_VOLTAGE_FLOOR = 42.0;
const float BOOST_CV_TARGET_VOLTAGE = 56.00;  // conserve pack: stop/hold at 56V
const float BOOST_CV_ENTRY_VOLTAGE = 55.60;
const float BOOST_CV_FORCE_VOLTAGE = 55.80;
const float BOOST_CV_EXIT_VOLTAGE  = 55.20;
const float BOOST_CC_TAPER_START_V = 54.80;
const float BMS_OPEN_DETECT_V = 56.30;          // preempt before hard open spike
const float BMS_OPEN_JUMP_DELTA_V = 1.2;        // faster jump detect
const float BMS_OPEN_CURRENT_MAX_A = 1.20;
const float BMS_PREEMPT_DUTY_CAP_RAW = 120.0;   // hard duty ceiling near BMS zone
const float BMS_PREEMPT_ZONE_V = 55.90;

const float BOOST_PV_POWER_LIMIT_W = 650.0;
const float BOOST_PV_CURRENT_HARD_A = 16.3;
const float BOOST_EFF_EST = 0.90;
const float BOOST_POWER_CAP_ENABLE_W = 80.0;  // avoid startup lock at very low sampled power

const float BOOST_MPPT_STEP_V = 0.10;
const float BOOST_MPPT_VREF_MIN = 40.0;
const float BOOST_MPPT_VREF_MAX = 45.0;
const unsigned long BOOST_MPPT_PERIOD_MS = 100;
const unsigned long BOOST_SOFTSTART_MS = 2500;
const unsigned long BOOST_CV_ENTER_CONFIRM_MS = 200;   // was 8000ms (too late)
const unsigned long BOOST_CV_EXIT_CONFIRM_MS = 3000;

const float BOOST_CURR_KP = 12.0;
const float BOOST_CURR_KI = 40.0;
const float BOOST_CURR_OUT_MIN = -30.0;
const float BOOST_CURR_OUT_MAX = 30.0;

const float BOOST_VOLT_KP = 1.2;
const float BOOST_VOLT_KI = 0.9;
const float BOOST_VOLT_OUT_MIN = 0.0;
const float BOOST_VOLT_OUT_MAX = TARGET_CC_CURRENT;

const float BOOST_DUTY_SLEW_UP = 2.2;
const float BOOST_DUTY_SLEW_DOWN = 5.0;
const float BOOST_EST_DUTY_MARGIN = 0.03;

const float BOOST_VBAT_SPIKE_PRECUT_DELTA_V = 0.7;
const float BOOST_VBAT_SPIKE_PRECUT_RAW_ABOVE_FILT_V = 1.0;
const float HARD_OVP_TRIP_VOLTAGE = 57.80;
const float HARD_OVP_RELEASE_VOLTAGE = 55.80;
const unsigned long HARD_OVP_RELEASE_DELAY_MS = 2500;

const bool ENABLE_DEBUG_VERBOSE = true;
const unsigned long DEBUG_PRINT_INTERVAL_MS = 500;
const unsigned long LCD_REFRESH_INTERVAL_MS = 180;
const uint32_t I2C_CLOCK_HZ = 100000;

// =========================================================================
// Runtime variables
// =========================================================================
volatile float v_solar = 0, v_ac_in = 0, v_bat = 0;
volatile float i_solar = 0, i_ac_in = 0, i_bat = 0;
volatile float v_bat_filt = 0, i_bat_filt = 0;
volatile float i_solar_mag = 0;
volatile float i_bat_charge_filt = 0;
volatile float i_bat_charge_abs = 0;
volatile bool ovp_latched = false;
volatile float ovp_trip_voltage = 0.0;
volatile unsigned long ovp_trip_ms = 0;
volatile bool system_ON = false;
volatile bool charge_full_hold = false;
volatile int active_duty_percent = 0;
int raw_duty = 0;
float duty_accumulator = 0.0;
float boost_dither_phase = 0.0;
float forward_dither_phase = 0.0;
float total_Wh = 0;
unsigned long last_millis = 0;

volatile bool sensor_init_ok = false;
volatile unsigned long last_adc_sample_ms = 0;

enum SystemState { STATE_BOOST, STATE_FORWARD, STATE_OFF };
volatile SystemState currentState = STATE_OFF;
bool last_system_state = false;

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
unsigned long boost_mode_enter_ms = 0;

LiquidCrystal_I2C lcd(0x27, 20, 4);
SemaphoreHandle_t i2c_Mutex;

float raw_mv_v0 = 0, raw_mv_v1 = 0, raw_mv_v2 = 0;
float raw_mv_i0 = 0, raw_mv_i1 = 0, raw_mv_i2 = 0;
float current_offset_i0 = OFFSET_I_SOLAR;
float current_offset_i1 = OFFSET_I_AC;
float current_offset_i2 = OFFSET_I_BAT;
float vbat_filter_buf[8] = {0};
float ibat_filter_buf[8] = {0};
float vbat_filter_sum = 0;
float ibat_filter_sum = 0;
int filter_index = 0;
int filter_count = 0;
float last_vbat_sample = 0.0;
float last_vbat_filt_sample = 0.0;

void TaskSampleData(void * pvParameters);
void TaskLCDLoop(void * pvParameters);
void calibrateCurrentOffsetsAtBoot();
int quantizeDutyWithDither(float duty_cmd, float *phase, int max_duty);
void lcdPrintLineRaw(uint8_t row, const char *text);
void lcdPrintLineFmt(uint8_t row, const char *fmt, ...);
void reinitI2CBusAndLCD();

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
    else *integ = iCandidate;
    return out;
}

static inline int boostEstimateDutyRaw(float vin, float vout, int maxDuty) {
    float vinUse = (vin > 38.0f) ? vin : 38.0f;
    float voutUse = (vout > (vinUse + 2.0f)) ? vout : (vinUse + 2.0f);
    float d = 1.0f - (vinUse / voutUse);
    d += BOOST_EST_DUTY_MARGIN;
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
}

int quantizeDutyWithDither(float duty_cmd, float *phase, int max_duty) {
    duty_cmd = constrain(duty_cmd, 0.0f, (float)max_duty);
    int base = (int)floorf(duty_cmd);
    float frac = duty_cmd - (float)base;
    *phase += frac;
    if (*phase >= 1.0f) {
        base += 1;
        *phase -= 1.0f;
    }
    if (*phase < 0.0f) *phase = 0.0f;
    return constrain(base, 0, max_duty);
}

static inline int16_t readADCStable(Adafruit_ADS1115 &adc, uint8_t channel, bool discard_first = false) {
    if (discard_first) {
        (void)adc.readADC_SingleEnded(channel);
    }
    int16_t sample = adc.readADC_SingleEnded(channel);
    if (sample < 0) {
        sample = adc.readADC_SingleEnded(channel);
        if (sample < 0) sample = 0;
    }
    return sample;
}

static inline void disablePowerStage() {
    currentState = STATE_OFF;
    raw_duty = 0;
    duty_accumulator = 0.0;
    boost_dither_phase = 0.0;
    forward_dither_phase = 0.0;
    digitalWrite(RELAY_PV_PIN, LOW);
    digitalWrite(RELAY_AC_PIN, LOW);
    ledcWrite(PWM_FORWARD_PIN, 0);
    ledcWrite(PWM_BOOST_PIN, 0);
}

static inline void forceSafeShutdown() {
    system_ON = false;
    charge_full_hold = false;
    disablePowerStage();
}

void lcdPrintLineRaw(uint8_t row, const char *text) {
    char line[21];
    snprintf(line, sizeof(line), "%-20.20s", text);
    lcd.setCursor(0, row);
    lcd.print(line);
}

void lcdPrintLineFmt(uint8_t row, const char *fmt, ...) {
    char tmp[48];
    va_list args;
    va_start(args, fmt);
    vsnprintf(tmp, sizeof(tmp), fmt, args);
    va_end(args);
    lcdPrintLineRaw(row, tmp);
}

void reinitI2CBusAndLCD() {
    Wire.end();
    delay(2);
    Wire.begin(21, 22);
    Wire.setClock(I2C_CLOCK_HZ);
    Wire.setTimeOut(25);
    lcd.init();
    lcd.backlight();
    lcd.clear();
}

void calibrateCurrentOffsetsAtBoot() {
    const int CAL_SAMPLES = 80;
    float sum_i0 = 0.0;
    float sum_i1 = 0.0;
    float sum_i2 = 0.0;

    disablePowerStage();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    for (int i = 0; i < CAL_SAMPLES; i++) {
        sum_i0 += readADCStable(ads_curr, 0, true) * 0.1875;
        sum_i1 += readADCStable(ads_curr, 1, true) * 0.1875;
        sum_i2 += readADCStable(ads_curr, 2, true) * 0.1875;
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }

    current_offset_i0 = sum_i0 / CAL_SAMPLES;
    current_offset_i1 = sum_i1 / CAL_SAMPLES;
    current_offset_i2 = sum_i2 / CAL_SAMPLES;

    Serial.printf("[CAL] Current zero offsets (mV): I0=%.2f I1=%.2f I2=%.2f\n",
                  current_offset_i0, current_offset_i1, current_offset_i2);
}

void setup() {
    Serial.begin(115200);
    Serial.printf("[BOOT] Firmware: %s\n", FW_VERSION_TAG);
    Serial.printf("[BOOT] CFG CC=%.2fA CV=%.2fV CVentry=%.2fV CVexit=%.2fV\n",
                  TARGET_CC_CURRENT, BOOST_CV_TARGET_VOLTAGE, BOOST_CV_ENTRY_VOLTAGE, BOOST_CV_EXIT_VOLTAGE);

    Wire.begin(21, 22);
    Wire.setClock(I2C_CLOCK_HZ);
    Wire.setTimeOut(25);

    bool volt_ok = ads_volt.begin(0x48);
    bool curr_ok = ads_curr.begin(0x49);
    sensor_init_ok = (volt_ok && curr_ok);

    if (sensor_init_ok) {
        ads_volt.setDataRate(RATE_ADS1115_860SPS);
        ads_curr.setDataRate(RATE_ADS1115_860SPS);
    }

    pinMode(RELAY_PV_PIN, OUTPUT);
    pinMode(RELAY_AC_PIN, OUTPUT);
    pinMode(BUTTON_START_PIN, INPUT_PULLUP);
    pinMode(BUTTON_STOP_PIN, INPUT_PULLUP);

    ledcAttach(PWM_FORWARD_PIN, PWM_FREQ, PWM_RES);
    ledcWrite(PWM_FORWARD_PIN, 0);

    ledcAttach(PWM_BOOST_PIN, PWM_FREQ, PWM_RES);
    ledcWrite(PWM_BOOST_PIN, 0);

    i2c_Mutex = xSemaphoreCreateMutex();
    reinitI2CBusAndLCD();

    if (!sensor_init_ok) {
        Serial.println("[FATAL] ADS1115 init failed. System is locked in safe standby.");
        forceSafeShutdown();
    } else {
        calibrateCurrentOffsetsAtBoot();
        last_adc_sample_ms = millis();
    }

    xTaskCreatePinnedToCore(TaskSampleData, "ADC_PWM_Task", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(TaskLCDLoop, "LCD_Task", 8192, NULL, 1, NULL, 1);
}

void loop() { vTaskDelay(1000); }

void TaskSampleData(void * pvParameters) {
    unsigned long pv_collapse_start_time = 0;
    bool pv_is_collapsing = false;
    unsigned long last_debug_time = 0;
    unsigned long last_sensor_error_log = 0;
    unsigned long full_condition_start_ms = 0;
    unsigned long high_voltage_stop_start_ms = 0;
    float last_valid_raw_mv_v0 = NAN;
    float last_valid_raw_mv_v2 = NAN;
    unsigned long last_adc_glitch_log = 0;

    for(;;) {
        unsigned long now = millis();

        if (!sensor_init_ok) {
            forceSafeShutdown();
            if (now - last_sensor_error_log >= SENSOR_ERROR_LOG_MS) {
                last_sensor_error_log = now;
                Serial.println("[FATAL] Waiting for manual reset: ADS1115 not detected.");
            }
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        bool sample_ok = false;
        if (xSemaphoreTake(i2c_Mutex, 50)) {
            raw_mv_v0 = readADCStable(ads_volt, 0, true) * 0.1875;
            raw_mv_v1 = readADCStable(ads_volt, 2, true) * 0.1875;
            raw_mv_v2 = readADCStable(ads_volt, 1, true) * 0.1875;

            raw_mv_i0 = readADCStable(ads_curr, 0) * 0.1875;
            raw_mv_i1 = readADCStable(ads_curr, 1) * 0.1875;
            raw_mv_i2 = readADCStable(ads_curr, 2) * 0.1875;

            bool power_stage_active = (raw_duty > 0);
            bool solar_raw_glitch = (raw_mv_v0 < ADC_RAW_MIN_VALID_MV) &&
                                    (power_stage_active ||
                                     i_solar_mag > ADC_GLITCH_CURRENT_GATE_A ||
                                     i_bat_charge_filt > ADC_GLITCH_CURRENT_GATE_A);
            bool bat_raw_glitch = (raw_mv_v2 < ADC_RAW_MIN_VALID_MV) &&
                                  (power_stage_active ||
                                   i_bat_charge_filt > ADC_GLITCH_CURRENT_GATE_A);

            if (solar_raw_glitch && !isnan(last_valid_raw_mv_v0)) {
                raw_mv_v0 = last_valid_raw_mv_v0;
            } else if (!solar_raw_glitch) {
                last_valid_raw_mv_v0 = raw_mv_v0;
            }

            if (bat_raw_glitch && !isnan(last_valid_raw_mv_v2)) {
                raw_mv_v2 = last_valid_raw_mv_v2;
            } else if (!bat_raw_glitch) {
                last_valid_raw_mv_v2 = raw_mv_v2;
            }

            if ((solar_raw_glitch || bat_raw_glitch) &&
                (now - last_adc_glitch_log >= ADC_GLITCH_LOG_MS)) {
                last_adc_glitch_log = now;
                Serial.printf("[WARN] ADC glitch filtered: PVraw=%.1fmV BATraw=%.1fmV duty=%d Is=%.2fA Ib=%.2fA\n",
                              raw_mv_v0, raw_mv_v2, raw_duty, i_solar_mag, i_bat_charge_filt);
            }

            float mv_pure_v0 = raw_mv_v0 - OFFSET_V_SOLAR; if (mv_pure_v0 < 0.0) mv_pure_v0 = 0.0;
            float mv_pure_v1 = raw_mv_v1 - OFFSET_V_AC;    if (mv_pure_v1 < 0.0) mv_pure_v1 = 0.0;
            float mv_pure_v2 = raw_mv_v2 - OFFSET_V_BAT;   if (mv_pure_v2 < 0.0) mv_pure_v2 = 0.0;

            float mv_pure_i0 = raw_mv_i0 - current_offset_i0;
            float mv_pure_i1 = raw_mv_i1 - current_offset_i1;
            float mv_pure_i2 = raw_mv_i2 - current_offset_i2;

            v_solar = (mv_pure_v0 / 1000.0) * CAL_SCALE_V_SOLAR * FIELD_TRIM_V_SOLAR;
            v_ac_in = (mv_pure_v1 / 1000.0) * CAL_SCALE_V_AC;
            v_bat   = (mv_pure_v2 / 1000.0) * CAL_SCALE_V_BAT;

            i_solar = (mv_pure_i0 / 1000.0) * CAL_SCALE_I_SOLAR;
            i_ac_in = (mv_pure_i1 / 1000.0) * CAL_SCALE_I_AC;
            i_bat   = (mv_pure_i2 / 1000.0) * CAL_SCALE_I_BAT;

            if (v_solar < NOISE_V_THRESHOLD) v_solar = 0.0;
            if (v_ac_in < NOISE_V_THRESHOLD) v_ac_in = 0.0;
            if (v_bat   < NOISE_V_THRESHOLD) v_bat   = 0.0;

            if (fabs(i_solar) < NOISE_I_THRESHOLD) i_solar = 0.0;
            if (fabs(i_ac_in) < NOISE_I_THRESHOLD) i_ac_in = 0.0;
            if (fabs(i_bat)   < NOISE_I_THRESHOLD) i_bat   = 0.0;

            vbat_filter_sum -= vbat_filter_buf[filter_index];
            ibat_filter_sum -= ibat_filter_buf[filter_index];
            vbat_filter_buf[filter_index] = v_bat;
            ibat_filter_buf[filter_index] = i_bat;
            vbat_filter_sum += vbat_filter_buf[filter_index];
            ibat_filter_sum += ibat_filter_buf[filter_index];
            filter_index = (filter_index + 1) % 8;
            if (filter_count < 8) filter_count++;

            v_bat_filt = vbat_filter_sum / (float)filter_count;
            i_bat_filt = ibat_filter_sum / (float)filter_count;
            i_solar_mag = fabs(i_solar);
            i_bat_charge_filt = fabs(i_bat_filt);
            i_bat_charge_abs = fabs(i_bat);
            float vbat_step = (last_vbat_sample > 0.0f) ? (v_bat - last_vbat_sample) : 0.0f;
            float vbat_filt_step = (last_vbat_filt_sample > 0.0f) ? (v_bat_filt - last_vbat_filt_sample) : 0.0f;

            // BMS open / near-open: kill PWM ASAP (before waiting for HARD_OVP threshold).
            if (!ovp_latched &&
                system_ON &&
                currentState == STATE_BOOST &&
                raw_duty > 0 &&
                (v_bat_filt >= BMS_PREEMPT_ZONE_V || v_bat >= BMS_PREEMPT_ZONE_V) &&
                ((v_bat >= BMS_OPEN_DETECT_V) ||
                 (vbat_step >= BMS_OPEN_JUMP_DELTA_V) ||
                 (v_bat > (v_bat_filt + 1.8f)) ||
                 (v_bat_filt >= BMS_OPEN_DETECT_V && i_bat_charge_filt <= BMS_OPEN_CURRENT_MAX_A))) {
                ovp_latched = true;
                ovp_trip_voltage = max(v_bat, v_bat_filt);
                ovp_trip_ms = now;
                forceSafeShutdown();
                charge_full_hold = true;
                Serial.printf("[CRITICAL] BMS-OPEN/preempt at raw=%.2f filt=%.2f I=%.2fA step=%.2f. PWM off.\n",
                              v_bat, v_bat_filt, i_bat_charge_filt, vbat_step);
            }

            if (!ovp_latched &&
                (v_bat >= HARD_OVP_TRIP_VOLTAGE || v_bat_filt >= HARD_OVP_TRIP_VOLTAGE)) {
                ovp_latched = true;
                ovp_trip_voltage = max(v_bat, v_bat_filt);
                ovp_trip_ms = now;
                forceSafeShutdown();
                Serial.printf("[CRITICAL] HARD OVP TRIP at %.2fV (trip=%.2fV). Output disabled.\n",
                              ovp_trip_voltage, HARD_OVP_TRIP_VOLTAGE);
            }

            // Near CV, current naturally falls — do soft duty cut, not hard latch
            // (hard latch was killing CV 57V tests with OVP_LOCK).
            if (!ovp_latched &&
                system_ON &&
                currentState == STATE_BOOST &&
                raw_duty > 0 &&
                v_bat_filt >= BOOST_CV_ENTRY_VOLTAGE &&
                i_bat_charge_filt < MIN_CURRENT_FOR_ACTIVE_CHARGE &&
                v_bat > (v_bat_filt + 3.0f)) {
                if (v_bat >= HARD_OVP_TRIP_VOLTAGE) {
                    ovp_latched = true;
                    ovp_trip_voltage = v_bat;
                    ovp_trip_ms = now;
                    forceSafeShutdown();
                    Serial.printf("[CRITICAL] RUNAWAY-CUT at %.2fV (filt=%.2fV, duty=%d).\n",
                                  v_bat, v_bat_filt, raw_duty);
                } else {
                    duty_accumulator = max(0.0f, duty_accumulator - 15.0f);
                    boostNewCurrIntegrator = 0.0f;
                    if (boostNewMode != BOOST_NEW_CV) {
                        boostNewMode = BOOST_NEW_CV;
                        boostNewVoltIntegrator = 0.0f;
                    }
                    Serial.printf("[WARN] Runaway soft-cut duty at %.2fV (filt=%.2fV).\n",
                                  v_bat, v_bat_filt);
                }
            }

            if (!ovp_latched &&
                system_ON &&
                currentState == STATE_BOOST &&
                raw_duty > 0 &&
                v_bat_filt >= (BOOST_CV_ENTRY_VOLTAGE - 0.2f) &&
                (vbat_step > BOOST_VBAT_SPIKE_PRECUT_DELTA_V ||
                 (vbat_step > 0.7f && vbat_filt_step > 0.25f)) &&
                v_bat > (v_bat_filt + BOOST_VBAT_SPIKE_PRECUT_RAW_ABOVE_FILT_V)) {
                if (v_bat >= HARD_OVP_TRIP_VOLTAGE) {
                    ovp_latched = true;
                    ovp_trip_voltage = v_bat;
                    ovp_trip_ms = now;
                    forceSafeShutdown();
                    Serial.printf("[CRITICAL] SPIKE-PRECUT at %.2fV (step=%.2fV, filt=%.2fV, duty=%d).\n",
                                  v_bat, vbat_step, v_bat_filt, raw_duty);
                } else {
                    duty_accumulator = max(0.0f, duty_accumulator - 12.0f);
                    boostNewCurrIntegrator = 0.0f;
                    if (boostNewMode != BOOST_NEW_CV) {
                        boostNewMode = BOOST_NEW_CV;
                        boostNewVoltIntegrator = 0.0f;
                    }
                    Serial.printf("[WARN] Spike soft-cut duty at %.2fV (step=%.2fV).\n",
                                  v_bat, vbat_step);
                }
            }

            if (ovp_latched &&
                (now - ovp_trip_ms >= HARD_OVP_RELEASE_DELAY_MS) &&
                v_bat_filt <= HARD_OVP_RELEASE_VOLTAGE &&
                v_bat <= (HARD_OVP_RELEASE_VOLTAGE + 0.8f)) {
                ovp_latched = false;
                Serial.printf("[INFO] OVP latch cleared at %.2fV (release=%.2fV).\n",
                              max(v_bat, v_bat_filt), HARD_OVP_RELEASE_VOLTAGE);
            }

            last_vbat_sample = v_bat;
            last_vbat_filt_sample = v_bat_filt;

            last_adc_sample_ms = now;
            sample_ok = true;
            xSemaphoreGive(i2c_Mutex);
        }

        if (!sample_ok && system_ON && (now - last_adc_sample_ms > ADC_STALE_TIMEOUT_MS)) {
            forceSafeShutdown();
            Serial.println("[CRITICAL] ADC sample timeout. Auto-shutdown for safety.");
        }

        if (system_ON) {
            if (charge_full_hold) {
                disablePowerStage();
                if ((v_bat_filt <= RESTART_CHARGE_VOLTAGE) &&
                    (v_solar >= MIN_PV_VOLTAGE || v_ac_in >= MIN_AC_VOLTAGE)) {
                    charge_full_hold = false;
                    Serial.println("[INFO] Battery dropped to restart threshold. Charging resumed.");
                }
            }

            if (charge_full_hold) {
                vTaskDelay(20 / portTICK_PERIOD_MS);
                continue;
            }

            if (currentState == STATE_OFF) {
                if (v_solar >= MIN_PV_VOLTAGE) {
                    ledcWrite(PWM_FORWARD_PIN, 0); ledcWrite(PWM_BOOST_PIN, 0);
                    digitalWrite(RELAY_AC_PIN, LOW);
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    digitalWrite(RELAY_PV_PIN, HIGH);
                    currentState = STATE_BOOST;
                    boost_mode_enter_ms = now;

                    raw_duty = 0;
                    duty_accumulator = 0.0f;
                    boost_dither_phase = 0.0f;
                    boostNewResetOnEntry(v_solar);
                    pv_is_collapsing = false;
                    Serial.println("[INFO] Enter BOOST with new MPPT->CC->CV control.");
                }
                else if (v_ac_in >= MIN_AC_VOLTAGE) {
                    ledcWrite(PWM_FORWARD_PIN, 0); ledcWrite(PWM_BOOST_PIN, 0);
                    digitalWrite(RELAY_PV_PIN, LOW);
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    digitalWrite(RELAY_AC_PIN, HIGH);
                    currentState = STATE_FORWARD;
                    pid_integral_cc = 0; pid_last_error_cc = 0;
                    pid_integral_cv = 0; pid_last_error_cv = 0;
                    raw_duty = 10;
                    duty_accumulator = 10.0;
                    forward_dither_phase = 0.0;
                }
                else {
                    system_ON = false;
                }
            }
            else if (currentState == STATE_BOOST) {
                if (v_solar < UNDER_PV_VOLTAGE_CRIT) {
                    if (!pv_is_collapsing) {
                        pv_is_collapsing = true;
                        pv_collapse_start_time = now;
                    }
                    if (now - pv_collapse_start_time >= 2000) {
                        system_ON = false;
                        Serial.println("[CRITICAL] Solar panel fully collapsed below 39V! Auto-Shutdown.");
                    }
                } else {
                    pv_is_collapsing = false;
                }
            }
            else if (currentState == STATE_FORWARD) {
                if (v_ac_in < MIN_AC_VOLTAGE) { system_ON = false; }
            }
        }

        if (!system_ON) {
            digitalWrite(RELAY_PV_PIN, LOW);
            digitalWrite(RELAY_AC_PIN, LOW);
            currentState = STATE_OFF;
            raw_duty = 0;
            duty_accumulator = 0.0;
            boost_dither_phase = 0.0;
            forward_dither_phase = 0.0;
            pv_is_collapsing = false;
            boostNewMode = BOOST_NEW_SOFTSTART;
        }

        if (system_ON && currentState != STATE_OFF) {
            int allowed_max_duty = (currentState == STATE_FORWARD) ? MAX_DUTY_FORWARD : MAX_DUTY_BOOST;

            if (currentState == STATE_FORWARD) {
                pid_error_cc = TARGET_CC_CURRENT - i_bat_charge_filt;
                pid_integral_cc += pid_error_cc;
                pid_integral_cc = constrain(pid_integral_cc, -100, 100);
                float delta_error_cc = pid_error_cc - pid_last_error_cc;
                float pid_out_cc = (Kp_cc * pid_error_cc) + (Ki_cc * pid_integral_cc) + (Kd_cc * delta_error_cc);
                pid_last_error_cc = pid_error_cc;

                pid_error_cv = TARGET_CV_VOLTAGE - v_bat_filt;
                if (fabs(pid_error_cv) <= CV_DEADBAND_V) {
                    pid_error_cv = 0.0;
                    pid_integral_cv *= 0.90;
                }
                pid_integral_cv += pid_error_cv;
                pid_integral_cv = constrain(pid_integral_cv, -100, 100);
                float delta_error_cv = pid_error_cv - pid_last_error_cv;
                float pid_out_cv = (Kp_cv * pid_error_cv) + (Ki_cv * pid_integral_cv) + (Kd_cv * delta_error_cv);
                pid_last_error_cv = pid_error_cv;

                float final_battery_pid = min(pid_out_cc, pid_out_cv);
                if (final_battery_pid > 1.5) final_battery_pid = 1.5;
                if (final_battery_pid < -4.0) final_battery_pid = -4.0;
                duty_accumulator += final_battery_pid;
            }
            else if (currentState == STATE_BOOST) {
                const float dt = 0.02f;

                if (v_solar <= 0.0f) {
                    duty_accumulator = 0.0f;
                    boostNewCurrIntegrator = 0.0f;
                    boostNewVoltIntegrator = 0.0f;
                } else if (boostNewMode == BOOST_NEW_SOFTSTART) {
                    int seedDuty = boostEstimateDutyRaw(v_solar, BOOST_CV_TARGET_VOLTAGE, allowed_max_duty);
                    duty_accumulator = boostApplySlew((float)seedDuty, duty_accumulator, 2.0f, 5.0f);

                    bool ready = (i_bat_charge_filt >= 0.4f) ||
                                 (now - boost_mode_enter_ms >= BOOST_SOFTSTART_MS);
                    if (ready) {
                        boostNewMode = BOOST_NEW_CC_MPPT;
                        boostNewCurrIntegrator = 0.0f;
                    }
                } else if (boostNewMode == BOOST_NEW_CC_MPPT) {
                    if (now - boostNewLastMpptMs >= BOOST_MPPT_PERIOD_MS) {
                        boostNewLastMpptMs = now;
                        float pPv = v_solar * i_solar_mag;
                        float dP = pPv - boostNewLastPower;
                        float dV = v_solar - boostNewLastVpv;
                        if (fabsf(dP) > 0.2f) {
                            if (dP > 0.0f) boostNewMpptDir = (dV >= 0.0f) ? 1 : -1;
                            else           boostNewMpptDir = (dV >= 0.0f) ? -1 : 1;
                        }
                        boostNewPvRef += (float)boostNewMpptDir * BOOST_MPPT_STEP_V;
                        boostNewPvRef = boostClampf(boostNewPvRef, BOOST_MPPT_VREF_MIN, BOOST_MPPT_VREF_MAX);

                        float pvErr = v_solar - boostNewPvRef;
                        boostNewIrefMppt += 0.08f * pvErr;
                        boostNewIrefMppt = boostClampf(boostNewIrefMppt, 0.0f, TARGET_CC_CURRENT);

                        boostNewPAvailFilt = (boostNewPAvailFilt <= 0.01f) ? pPv : (0.22f * pPv + 0.78f * boostNewPAvailFilt);
                        boostNewLastPower = pPv;
                        boostNewLastVpv = v_solar;
                    }

                    float iRefPower = TARGET_CC_CURRENT;
                    if (boostNewPAvailFilt >= BOOST_POWER_CAP_ENABLE_W && v_bat_filt > 5.0f) {
                        float pAvail = min(boostNewPAvailFilt, BOOST_PV_POWER_LIMIT_W);
                        iRefPower = (pAvail * BOOST_EFF_EST) / v_bat_filt;
                    }
                    float iRef = min(TARGET_CC_CURRENT, min(boostNewIrefMppt, iRefPower));

                    // Pre-CV current taper: reduce Iref as battery approaches BMS CV
                    if (v_bat_filt >= BOOST_CC_TAPER_START_V) {
                        float span = max(0.20f, BOOST_CV_TARGET_VOLTAGE - BOOST_CC_TAPER_START_V);
                        float rem = BOOST_CV_TARGET_VOLTAGE - v_bat_filt;
                        float taper = boostClampf(rem / span, 0.10f, 1.0f);
                        iRef *= taper;
                    }
                    iRef = boostClampf(iRef, 0.0f, TARGET_CC_CURRENT);

                    float iErr = iRef - i_bat_charge_filt;
                    float dDuty = boostRunPI(iErr, BOOST_CURR_KP, BOOST_CURR_KI, dt,
                                             &boostNewCurrIntegrator, BOOST_CURR_OUT_MIN, BOOST_CURR_OUT_MAX);
                    float dutyTarget = duty_accumulator + dDuty;
                    dutyTarget = boostClampf(dutyTarget, 0.0f, (float)allowed_max_duty);
                    duty_accumulator = boostApplySlew(dutyTarget, duty_accumulator, BOOST_DUTY_SLEW_UP, BOOST_DUTY_SLEW_DOWN);

                    // Force CV immediately near BMS threshold; short confirm for soft entry
                    if (v_bat_filt >= BOOST_CV_FORCE_VOLTAGE || max(v_bat, v_bat_filt) >= BOOST_CV_FORCE_VOLTAGE) {
                        boostNewMode = BOOST_NEW_CV;
                        boostNewCurrIntegrator = 0.0f;
                        boostNewVoltIntegrator = 0.0f;
                        boostNewCvEnterMs = 0;
                        Serial.printf("[INFO] Force CV at Vbat=%.2f / filt=%.2f\n", v_bat, v_bat_filt);
                    } else if (v_bat_filt >= BOOST_CV_ENTRY_VOLTAGE) {
                        if (boostNewCvEnterMs == 0) boostNewCvEnterMs = now;
                        if (now - boostNewCvEnterMs >= BOOST_CV_ENTER_CONFIRM_MS) {
                            boostNewMode = BOOST_NEW_CV;
                            boostNewCurrIntegrator = 0.0f;
                            boostNewVoltIntegrator = 0.0f;
                        }
                    } else {
                        boostNewCvEnterMs = 0;
                    }
                } else if (boostNewMode == BOOST_NEW_CV) {
                    if (now - boostNewLastMpptMs >= BOOST_MPPT_PERIOD_MS) {
                        boostNewLastMpptMs = now;
                        float pPv = v_solar * i_solar_mag;
                        boostNewPAvailFilt = (boostNewPAvailFilt <= 0.01f) ? pPv : (0.22f * pPv + 0.78f * boostNewPAvailFilt);
                        float pvErr = v_solar - boostNewPvRef;
                        boostNewIrefMppt += 0.06f * pvErr;
                        boostNewIrefMppt = boostClampf(boostNewIrefMppt, 0.0f, TARGET_CC_CURRENT);
                    }

                    float vErr = BOOST_CV_TARGET_VOLTAGE - v_bat_filt;
                    if (fabs(vErr) <= CV_DEADBAND_V) {
                        vErr = 0.0f;
                        boostNewVoltIntegrator *= 0.90f;
                    }
                    float iReq = boostRunPI(vErr, BOOST_VOLT_KP, BOOST_VOLT_KI, dt,
                                            &boostNewVoltIntegrator, BOOST_VOLT_OUT_MIN, BOOST_VOLT_OUT_MAX);

                    float iRefPower = TARGET_CC_CURRENT;
                    if (boostNewPAvailFilt >= BOOST_POWER_CAP_ENABLE_W && v_bat_filt > 5.0f) {
                        float pAvail = min(boostNewPAvailFilt, BOOST_PV_POWER_LIMIT_W);
                        iRefPower = (pAvail * BOOST_EFF_EST) / v_bat_filt;
                    }
                    float iRef = min(iReq, min(boostNewIrefMppt, iRefPower));
                    iRef = boostClampf(iRef, 0.0f, TARGET_CC_CURRENT);

                    float iErr = iRef - i_bat_charge_filt;
                    float dDuty = boostRunPI(iErr, BOOST_CURR_KP, BOOST_CURR_KI, dt,
                                             &boostNewCurrIntegrator, BOOST_CURR_OUT_MIN, BOOST_CURR_OUT_MAX);
                    float dutyTarget = duty_accumulator + dDuty;

                    // Do NOT force minimum duty near CV — that caused overshoot.
                    // Only allow a soft floor while clearly below target and still charging.
                    if (v_bat_filt < (BOOST_CV_TARGET_VOLTAGE - 0.6f) &&
                        i_bat_charge_filt > MIN_CURRENT_FOR_ACTIVE_CHARGE) {
                        int softFloor = boostEstimateDutyRaw(v_solar, v_bat_filt + 1.0f, allowed_max_duty);
                        softFloor = min(softFloor, 180);
                        if (dutyTarget < (float)softFloor) dutyTarget = (float)softFloor;
                    }

                    // If already at/above CV, cut duty aggressively.
                    if (v_bat_filt >= BOOST_CV_TARGET_VOLTAGE) {
                        float over = v_bat_filt - BOOST_CV_TARGET_VOLTAGE;
                        dutyTarget -= (3.0f + over * 12.0f);
                        boostNewCurrIntegrator = 0.0f;
                        boostNewVoltIntegrator *= 0.7f;
                    }
                    if (v_bat > (v_bat_filt + 1.5f)) {
                        // raw sample racing ahead of filter => cut now
                        dutyTarget = min(dutyTarget, duty_accumulator - 8.0f);
                        dutyTarget = max(0.0f, dutyTarget);
                    }

                    dutyTarget = boostClampf(dutyTarget, 0.0f, (float)allowed_max_duty);
                    duty_accumulator = boostApplySlew(dutyTarget, duty_accumulator, BOOST_DUTY_SLEW_UP, BOOST_DUTY_SLEW_DOWN);

                    if (v_bat_filt <= BOOST_CV_EXIT_VOLTAGE &&
                        i_bat_charge_filt < (TARGET_CC_CURRENT - 0.6f)) {
                        if (boostNewCvExitMs == 0) boostNewCvExitMs = now;
                        if (now - boostNewCvExitMs >= BOOST_CV_EXIT_CONFIRM_MS) {
                            boostNewMode = BOOST_NEW_CC_MPPT;
                            boostNewCurrIntegrator = 0.0f;
                            boostNewVoltIntegrator = 0.0f;
                        }
                    } else {
                        boostNewCvExitMs = 0;
                    }

                    bool doneCond = (v_bat_filt >= FULL_DETECT_VOLTAGE) && (i_bat_charge_filt <= FULL_END_CURRENT);
                    if (doneCond) {
                        if (full_condition_start_ms == 0) full_condition_start_ms = now;
                        if (now - full_condition_start_ms >= FULL_CONFIRM_MS) {
                            boostNewMode = BOOST_NEW_DONE;
                            charge_full_hold = true;
                            disablePowerStage();
                            Serial.println("[INFO] Battery FULL detected in BOOST_NEW_CV.");
                        }
                    } else {
                        full_condition_start_ms = 0;
                    }
                } else { // BOOST_NEW_DONE
                    duty_accumulator = 0.0f;
                    if (v_bat_filt <= RESTART_CHARGE_VOLTAGE && v_solar >= MIN_PV_VOLTAGE) {
                        charge_full_hold = false;
                        boostNewMode = BOOST_NEW_CC_MPPT;
                        boostNewCurrIntegrator = 0.0f;
                        boostNewVoltIntegrator = 0.0f;
                    }
                }

                if (i_bat_charge_abs > (TARGET_CC_CURRENT + 0.25f)) {
                    duty_accumulator -= (2.0f + (i_bat_charge_abs - TARGET_CC_CURRENT) * 3.0f);
                    boostNewCurrIntegrator *= 0.8f;
                }
                if (i_solar_mag > BOOST_PV_CURRENT_HARD_A) {
                    duty_accumulator -= 5.0f;
                }
                if (v_solar < (BOOST_VOLTAGE_FLOOR - 0.5f)) {
                    duty_accumulator -= (2.0f + (BOOST_VOLTAGE_FLOOR - v_solar) * 2.5f);
                }
                if (v_bat_filt > BOOST_CV_TARGET_VOLTAGE) {
                    float over_cv = v_bat_filt - BOOST_CV_TARGET_VOLTAGE;
                    duty_accumulator -= (2.5f + over_cv * 10.0f);
                    boostNewCurrIntegrator = 0.0f;
                }
                if (v_bat > (BOOST_CV_TARGET_VOLTAGE + 0.4f)) {
                    duty_accumulator -= 8.0f;
                    boostNewCurrIntegrator = 0.0f;
                }

                // Near BMS zone: hard-cap duty so open-FET fly-up has less energy.
                if (v_bat_filt >= BMS_PREEMPT_ZONE_V || v_bat >= BMS_PREEMPT_ZONE_V) {
                    if (duty_accumulator > BMS_PREEMPT_DUTY_CAP_RAW) {
                        duty_accumulator = BMS_PREEMPT_DUTY_CAP_RAW;
                    }
                    if (v_bat_filt >= BOOST_CV_TARGET_VOLTAGE) {
                        duty_accumulator = min(duty_accumulator, 80.0f);
                    }
                }

                duty_accumulator = boostClampf(duty_accumulator, 0.0f, (float)allowed_max_duty);
            }

            duty_accumulator = constrain(duty_accumulator, 0.0, (float)allowed_max_duty);

            if (currentState == STATE_FORWARD) {
                raw_duty = quantizeDutyWithDither(duty_accumulator, &forward_dither_phase, allowed_max_duty);
                boost_dither_phase = 0.0;
                ledcWrite(PWM_FORWARD_PIN, raw_duty);
                ledcWrite(PWM_BOOST_PIN, 0);
            } else if (currentState == STATE_BOOST) {
                raw_duty = quantizeDutyWithDither(duty_accumulator, &boost_dither_phase, allowed_max_duty);
                forward_dither_phase = 0.0;
                ledcWrite(PWM_BOOST_PIN, raw_duty);
                ledcWrite(PWM_FORWARD_PIN, 0);
            }

            total_Wh += ((v_bat * i_bat_charge_filt) * (now - last_millis)) / 3600000.0;

            if (v_bat_filt >= HIGH_VOLTAGE_STOP_VOLTAGE) {
                if (high_voltage_stop_start_ms == 0) high_voltage_stop_start_ms = now;
                if (now - high_voltage_stop_start_ms >= HIGH_VOLTAGE_STOP_CONFIRM_MS) {
                    charge_full_hold = true;
                    disablePowerStage();
                    Serial.printf("[INFO] High-voltage charge stop at %.2fV. Enter FULL HOLD.\n", v_bat_filt);
                }
            } else {
                high_voltage_stop_start_ms = 0;
            }
        } else {
            ledcWrite(PWM_FORWARD_PIN, 0);
            ledcWrite(PWM_BOOST_PIN, 0);
            full_condition_start_ms = 0;
            high_voltage_stop_start_ms = 0;
        }

        last_millis = now;
        active_duty_percent = round(((float)raw_duty * 100.0) / 1023.0);

        if (ENABLE_DEBUG_VERBOSE && (now - last_debug_time >= DEBUG_PRINT_INTERVAL_MS)) {
            last_debug_time = now;
            const char* state_label = "OFF";
            if (charge_full_hold) {
                state_label = "FULL_HOLD";
            } else if (ovp_latched) {
                state_label = "OVP_LOCK";
            } else if (currentState == STATE_BOOST) {
                if (boostNewMode == BOOST_NEW_SOFTSTART) state_label = "BOOST_SOFT";
                else if (boostNewMode == BOOST_NEW_CC_MPPT) state_label = "BOOST_CCMP";
                else if (boostNewMode == BOOST_NEW_CV) state_label = "BOOST_CV";
                else state_label = "BOOST_DONE";
            } else if (currentState == STATE_FORWARD) {
                state_label = "FORWARD";
            }

            Serial.println("=========================================================================================");
            Serial.printf("[DEBUG] System: %s | State: %s | Duty: %d%%\n",
                          (system_ON ? "ON " : "OFF"), state_label, active_duty_percent);
            Serial.printf("  [PV ] V:%5.1fV I:%5.2fA P:%6.1fW | Vref:%.2fV Iref_mppt:%.2fA\n",
                          v_solar, i_solar_mag, (v_solar * i_solar_mag), boostNewPvRef, boostNewIrefMppt);
            Serial.printf("  [BAT] V:%5.2fV I:%5.2fA (abs:%5.2fA)\n",
                          v_bat_filt, i_bat_filt, i_bat_charge_filt);
            Serial.printf("  [AC ] V:%5.1fV I:%5.2fA\n", v_ac_in, i_ac_in);
            Serial.println("=========================================================================================");
        }

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void TaskLCDLoop(void * pvParameters) {
    bool last_start_state = HIGH, last_stop_state = HIGH;
    bool show_no_power_alert = false;
    bool show_ovp_alert = false;
    unsigned long alert_millis = 0;
    unsigned long last_lcd_recover = 0;
    unsigned long last_lcd_refresh = 0;
    int lcd_mutex_fail_count = 0;

    bool start_raw_last = HIGH, stop_raw_last = HIGH;
    unsigned long start_change_ms = 0, stop_change_ms = 0;
    const unsigned long DEBOUNCE_MS = 80;

    for(;;) {
        unsigned long now = millis();
        bool start_raw = digitalRead(BUTTON_START_PIN);
        bool stop_raw  = digitalRead(BUTTON_STOP_PIN);

        if (start_raw != start_raw_last) {
            start_raw_last = start_raw;
            start_change_ms = now;
        }
        if (stop_raw != stop_raw_last) {
            stop_raw_last = stop_raw;
            stop_change_ms = now;
        }

        bool current_start = last_start_state;
        bool current_stop  = last_stop_state;

        if (now - start_change_ms >= DEBOUNCE_MS) current_start = start_raw;
        if (now - stop_change_ms >= DEBOUNCE_MS) current_stop  = stop_raw;

        bool start_pressed = (current_start == LOW);
        bool stop_pressed  = (current_stop == LOW);
        bool start_edge = (start_pressed && last_start_state == HIGH);
        bool stop_edge  = (stop_pressed && last_stop_state == HIGH);

        if (stop_edge) {
            system_ON = false;
            charge_full_hold = false;
            show_no_power_alert = false;
            show_ovp_alert = false;
        } else if (start_edge) {
            if (ovp_latched) {
                system_ON = false;
                show_no_power_alert = false;
                show_ovp_alert = true;
                alert_millis = now;
            } else if (sensor_init_ok && (v_solar >= MIN_PV_VOLTAGE || v_ac_in >= MIN_AC_VOLTAGE)) {
                system_ON = true;
                charge_full_hold = false;
                show_no_power_alert = false;
                show_ovp_alert = false;
            } else {
                system_ON = false;
                show_no_power_alert = true;
                show_ovp_alert = false;
                alert_millis = now;
            }
        }
        last_start_state = current_start; last_stop_state = current_stop;

        if (system_ON != last_system_state) {
            if (xSemaphoreTake(i2c_Mutex, 50)) {
                lcd.clear();
                xSemaphoreGive(i2c_Mutex);
                lcd_mutex_fail_count = 0;
            } else {
                lcd_mutex_fail_count++;
            }
            if (!system_ON) {
                total_Wh = 0;
            }
            last_system_state = system_ON;
        }

        if (show_no_power_alert && (now - alert_millis > 3000)) {
            show_no_power_alert = false;
            if (xSemaphoreTake(i2c_Mutex, 50)) {
                lcd.clear();
                xSemaphoreGive(i2c_Mutex);
                lcd_mutex_fail_count = 0;
            } else {
                lcd_mutex_fail_count++;
            }
        }
        if (show_ovp_alert && (now - alert_millis > 3000)) {
            show_ovp_alert = false;
            if (xSemaphoreTake(i2c_Mutex, 50)) {
                lcd.clear();
                xSemaphoreGive(i2c_Mutex);
                lcd_mutex_fail_count = 0;
            } else {
                lcd_mutex_fail_count++;
            }
        }

        if (now - last_lcd_refresh >= LCD_REFRESH_INTERVAL_MS) {
            if (xSemaphoreTake(i2c_Mutex, 50)) {
                if (system_ON && charge_full_hold) {
                    lcdPrintLineRaw(0, "BATTERY FULL HOLD");
                    lcdPrintLineFmt(1, "BAT:%5.1fV I:%4.2fA", v_bat_filt, i_bat_filt);
                    lcdPrintLineFmt(2, "Resume <= %5.1fV", RESTART_CHARGE_VOLTAGE);
                    lcdPrintLineRaw(3, "Press STOP to cancel");
                } else if (system_ON) {
                    lcdPrintLineFmt(0, "ACTIVE   DUTY:%3d%%", active_duty_percent);
                    lcdPrintLineFmt(1, "%-8s PWR:%5.1fWh", (currentState == STATE_BOOST ? "BOOST PV" : "FORW AC"), total_Wh);
                    lcdPrintLineFmt(2, "IN :%5.1fV %5.1fA", (currentState == STATE_BOOST ? v_solar : v_ac_in), (currentState == STATE_BOOST ? i_solar : i_ac_in));
                    lcdPrintLineFmt(3, "OUT:%5.1fV %5.1fA", v_bat, i_bat);
                } else if (ovp_latched || show_ovp_alert) {
                    lcdPrintLineRaw(0, "OVP TRIPPED");
                    lcdPrintLineFmt(1, "VBAT:%5.1fV T:%4.1f", v_bat_filt, ovp_trip_voltage);
                    lcdPrintLineFmt(2, "REL <= %5.1fV", HARD_OVP_RELEASE_VOLTAGE);
                    lcdPrintLineRaw(3, "WAIT VOLTAGE DROP");
                } else if (show_no_power_alert) {
                    lcdPrintLineRaw(0, "ERROR");
                    lcdPrintLineRaw(1, "NO INPUT POWER!");
                    lcdPrintLineRaw(2, "Check PV / AC Line");
                    lcdPrintLineRaw(3, "CANNOT ACTIVATE");
                } else {
                    lcdPrintLineRaw(0, "STANDBY");
                    lcdPrintLineFmt(1, "PV :%5.1fV AC:%5.1fV", v_solar, v_ac_in);
                    lcdPrintLineFmt(2, "BATT:%5.1fV", v_bat);
                    lcdPrintLineRaw(3, "");
                }
                xSemaphoreGive(i2c_Mutex);
                lcd_mutex_fail_count = 0;
                last_lcd_refresh = now;
            } else {
                lcd_mutex_fail_count++;
            }
        }

        if (lcd_mutex_fail_count >= 12 && (now - last_lcd_recover > 5000)) {
            last_lcd_recover = now;
            if (xSemaphoreTake(i2c_Mutex, 50)) {
                reinitI2CBusAndLCD();
                xSemaphoreGive(i2c_Mutex);
                lcd_mutex_fail_count = 0;
                last_lcd_refresh = 0;
            }
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
