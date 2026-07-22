#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <stdarg.h>
const char* FW_VERSION_TAG = "cv58-boost-v14-forward-v25";
// Boost path frozen to proven field code: cv58-stability-v14-cv-stable (PV charge OK).
// Forward / mode-select / debug layered on top without changing Boost loop.
// =========================================================================
// Hardware
// =========================================================================
const int RELAY_PV_PIN     = 32;
const int RELAY_AC_PIN     = 33;
const int BUTTON_START_PIN = 25;
const int BUTTON_STOP_PIN  = 26;  // STANDBY: กดสลับโหมด BOOST↔FORWARD / ตอนชาร์จ: หยุด
const int PWM_FORWARD_PIN  = 14;
const int PWM_BOOST_PIN    = 27;
// Boost keeps proven 50 kHz; Forward uses 67 kHz for Nr=Np hardware.
const int PWM_FREQ_BOOST   = 50000;
const int PWM_FREQ_FORWARD = 67000;
const int PWM_RES          = 10;
Adafruit_ADS1115 ads_volt;
Adafruit_ADS1115 ads_curr;
// =========================================================================
// Targets / safety thresholds
// =========================================================================
// CV=56.0V (~3.50V/cell for 16S LFP) — longevity / BMS-friendly cutoff.
const float TARGET_CV_VOLTAGE = 56.00;
const float TARGET_CC_CURRENT = 6.0;       // Boost CC (proven v14)
const float FWD_TARGET_CC_CURRENT = 5.0;   // Forward CC
const float MIN_PV_VOLTAGE = 42.0;
const float UNDER_PV_VOLTAGE_CRIT = 39.0;
// v_ac_in = DC after diode bridge (not VAC RMS). AC 110 V → ~155 Vpeak unloaded,
// typically ~100–140 V under load / with ripple averaging on ADS sample.
const float MIN_AC_VOLTAGE = 95.0;         // post-bridge DC low-line for AC 110 V
const int MAX_DUTY_FORWARD = 460;  // ~45% for Nr=Np reset @ 67 kHz
const int MAX_DUTY_BOOST   = 760;
// Battery must be present and in a safe start window before enabling a power stage.
const float BAT_PRESENT_MIN_V = 40.0;
const float BAT_START_MAX_V = 56.40;
const unsigned long ADC_STALE_TIMEOUT_MS = 700;
const unsigned long SENSOR_ERROR_LOG_MS = 2000;
const float CV_DEADBAND_V = 0.12;
const float FULL_DETECT_VOLTAGE = 55.90;
const float FULL_END_CURRENT = 0.50;
const unsigned long FULL_CONFIRM_MS = 60000;
const float HIGH_VOLTAGE_STOP_VOLTAGE = 56.80;
const unsigned long HIGH_VOLTAGE_STOP_CONFIRM_MS = 300;
const float RESTART_CHARGE_VOLTAGE = 54.0;
// =========================================================================
// Forward (AC) control: SOFTSTART -> CC -> CV -> DONE
// Tuned to mirror proven Boost loop (same PI/CV/slew style); no MPPT.
// =========================================================================
const float FWD_CV_ENTRY_VOLTAGE = 55.50;
const float FWD_CV_FORCE_VOLTAGE = 55.70;
const float FWD_CV_EXIT_VOLTAGE  = 54.80;
const float FWD_CC_TAPER_START_V = 54.80;
const float FWD_CV_IREF_SLEW_A = 0.08;       // same as Boost
const float FWD_CV_NEAR_BAND_V = 0.35;
const float FWD_CV_DUTY_STEP_NEAR = 0.8;
const float FWD_CV_DUTY_STEP_FAR = 2.5;
const unsigned long FWD_SOFTSTART_MS = 2500; // same as Boost
const unsigned long FWD_CV_ENTER_CONFIRM_MS = 200;
const unsigned long FWD_CV_EXIT_CONFIRM_MS = 5000;
// Same PI family as proven Boost (Boost CURR 14/55, VOLT 0.85/0.45).
const float FWD_CURR_KP = 14.0;
const float FWD_CURR_KI = 55.0;
const float FWD_CURR_OUT_MIN = -35.0;
const float FWD_CURR_OUT_MAX = 45.0;
const float FWD_VOLT_KP = 0.85;
const float FWD_VOLT_KI = 0.45;
const float FWD_VOLT_OUT_MIN = 0.0;
const float FWD_VOLT_OUT_MAX = 3.5;
const float FWD_DUTY_SLEW_UP = 4.0;
const float FWD_DUTY_SLEW_DOWN = 6.0;
const float FWD_SOFTSTART_SEED_DUTY = 80.0;  // gentle seed (~8% of 1023)
const float FWD_AC_CURRENT_HARD_A = 2.5;     // soft outer cut (like Boost PV hard)
const float FWD_BAT_CURRENT_HARD_A = 5.75f;  // soft outer cut only (no latch; like Boost)
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
const float BOOST_CV_ENTRY_VOLTAGE = 55.50;
const float BOOST_CV_FORCE_VOLTAGE = 55.70;
const float BOOST_CV_EXIT_VOLTAGE  = 54.80;   // wider hysteresis so CV does not chatter
const float BOOST_CC_TAPER_START_V = 54.80;
const float BMS_OPEN_DETECT_V = 56.30;
const float BMS_OPEN_JUMP_DELTA_V = 1.2;
const float BMS_OPEN_CURRENT_MAX_A = 1.20;
const float BMS_PREEMPT_DUTY_CAP_RAW = 140.0;
const float BMS_PREEMPT_ZONE_V = 55.95;
const float BOOST_CV_IREF_SLEW_A = 0.08;      // A per 20ms control tick
const float BOOST_CV_NEAR_BAND_V = 0.35;      // within this of target => gentle control
const float BOOST_CV_DUTY_STEP_NEAR = 0.8;    // raw duty step limit near target
const float BOOST_CV_DUTY_STEP_FAR = 2.5;
const float BOOST_PV_POWER_LIMIT_W = 650.0;
const float BOOST_PV_CURRENT_HARD_A = 16.3;
const float BOOST_EFF_EST = 0.90;
// Do NOT cap Iref from measured Ppv (causes CC stuck at low current).
// Back off only when PV voltage collapses below this floor.
const float BOOST_PV_COLLAPSE_BACKOFF_V = 41.0;
const float BOOST_MPPT_STEP_V = 0.10;
const float BOOST_MPPT_VREF_MIN = 40.0;
const float BOOST_MPPT_VREF_MAX = 45.0;
const unsigned long BOOST_MPPT_PERIOD_MS = 100;
const unsigned long BOOST_SOFTSTART_MS = 2500;
const unsigned long BOOST_CV_ENTER_CONFIRM_MS = 200;   // was 8000ms (too late)
const float BOOST_CURR_KP = 14.0;
const float BOOST_CURR_KI = 55.0;
const float BOOST_CURR_OUT_MIN = -35.0;
const float BOOST_CURR_OUT_MAX = 45.0;
const float BOOST_VOLT_KP = 0.85;
const float BOOST_VOLT_KI = 0.45;
const float BOOST_VOLT_OUT_MIN = 0.0;
const float BOOST_VOLT_OUT_MAX = 3.5;   // CV should not demand high current near full
const unsigned long BOOST_CV_EXIT_CONFIRM_MS = 5000;
const float BOOST_DUTY_SLEW_UP = 4.0;
const float BOOST_DUTY_SLEW_DOWN = 6.0;
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
// User picks charge path with STOP while STANDBY, then presses START.
enum UserChargeMode { USER_MODE_BOOST = 0, USER_MODE_FORWARD = 1 };
volatile UserChargeMode selectedChargeMode = USER_MODE_BOOST;
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
float boostNewIrefCvCmd = 0.0f;
float boostNewPAvailFilt = 0.0f;
unsigned long boostNewLastMpptMs = 0;
unsigned long boostNewCvEnterMs = 0;
unsigned long boostNewCvExitMs = 0;
unsigned long boost_mode_enter_ms = 0;
enum ForwardMode { FWD_SOFTSTART, FWD_CC, FWD_CV, FWD_DONE };
volatile ForwardMode forwardMode = FWD_SOFTSTART;
float fwdCurrIntegrator = 0.0f;
float fwdVoltIntegrator = 0.0f;
float fwdIrefCvCmd = 0.5f;
unsigned long fwdCvEnterMs = 0;
unsigned long fwdCvExitMs = 0;
unsigned long forward_mode_enter_ms = 0;
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
    boostNewIrefMppt = TARGET_CC_CURRENT;  // start CC seeking full current
    boostNewIrefCvCmd = 0.5f;
    boostNewPAvailFilt = 0.0f;
    boostNewLastMpptMs = 0;
    boostNewCvEnterMs = 0;
    boostNewCvExitMs = 0;
}
static inline void forwardNewResetOnEntry() {
    forwardMode = FWD_SOFTSTART;
    fwdCurrIntegrator = 0.0f;
    fwdVoltIntegrator = 0.0f;
    fwdIrefCvCmd = 0.5f;
    fwdCvEnterMs = 0;
    fwdCvExitMs = 0;
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
    Serial.printf("[BOOT] CFG BOOST_CC=%.2fA FWD_CC=%.2fA CV=%.2fV FWD_CVentry=%.2fV DmaxF=%d fBoost=%dHz fFwd=%dHz\n",
                  TARGET_CC_CURRENT, FWD_TARGET_CC_CURRENT, TARGET_CV_VOLTAGE, FWD_CV_ENTRY_VOLTAGE,
                  MAX_DUTY_FORWARD, PWM_FREQ_BOOST, PWM_FREQ_FORWARD);
    Serial.println("[BOOT] UI: STOP toggles BOOST/FORWARD in STANDBY, then press START.");
    Serial.println("[BOOT] Forward AC sense: diode-bridge DC, AC110V (MIN_AC post-bridge).");
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
    ledcAttach(PWM_FORWARD_PIN, PWM_FREQ_FORWARD, PWM_RES);
    ledcWrite(PWM_FORWARD_PIN, 0);
    ledcAttach(PWM_BOOST_PIN, PWM_FREQ_BOOST, PWM_RES);
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
    unsigned long fwd_full_condition_start_ms = 0;
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
            // BMS open / near-open: kill PWM ASAP (proven Boost path — Boost only).
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
            // (hard latch was killing CV 57V tests with OVP_LOCK). Proven Boost only.
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
                bool selected_input_ok =
                    (selectedChargeMode == USER_MODE_BOOST) ? (v_solar >= MIN_PV_VOLTAGE)
                                                            : (v_ac_in >= MIN_AC_VOLTAGE);
                if ((v_bat_filt <= RESTART_CHARGE_VOLTAGE) && selected_input_ok) {
                    charge_full_hold = false;
                    Serial.println("[INFO] Battery dropped to restart threshold. Charging resumed.");
                }
            }
            if (charge_full_hold) {
                last_millis = now;
                vTaskDelay(20 / portTICK_PERIOD_MS);
                continue;
            }
            if (currentState == STATE_OFF) {
                bool bat_ok = (v_bat_filt >= BAT_PRESENT_MIN_V) && (v_bat_filt <= BAT_START_MAX_V);
                bool want_boost = (selectedChargeMode == USER_MODE_BOOST);
                bool input_ok = want_boost ? (v_solar >= MIN_PV_VOLTAGE)
                                           : (v_ac_in >= MIN_AC_VOLTAGE);
                if (!bat_ok) {
                    system_ON = false;
                    Serial.printf("[CRITICAL] Battery start window fail: Vbat=%.2f (need %.1f..%.1f).\n",
                                  v_bat_filt, BAT_PRESENT_MIN_V, BAT_START_MAX_V);
                } else if (!input_ok) {
                    system_ON = false;
                    Serial.printf("[CRITICAL] Selected mode %s input missing (PV=%.1f AC=%.1f).\n",
                                  want_boost ? "BOOST" : "FORWARD", v_solar, v_ac_in);
                } else if (want_boost) {
                    // BOOST entry — same proven v14 sequence (unchanged control after entry).
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
                } else {
                    ledcWrite(PWM_FORWARD_PIN, 0); ledcWrite(PWM_BOOST_PIN, 0);
                    digitalWrite(RELAY_PV_PIN, LOW);
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    digitalWrite(RELAY_AC_PIN, HIGH);
                    currentState = STATE_FORWARD;
                    forward_mode_enter_ms = now;
                    raw_duty = 0;
                    duty_accumulator = 0.0f;
                    forward_dither_phase = 0.0f;
                    forwardNewResetOnEntry();
                    Serial.println("[INFO] Enter FORWARD SoftStart->CC->CV control.");
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
            forwardMode = FWD_SOFTSTART;
        }
        if (system_ON && currentState != STATE_OFF) {
            int allowed_max_duty = (currentState == STATE_FORWARD) ? MAX_DUTY_FORWARD : MAX_DUTY_BOOST;
            if (currentState == STATE_FORWARD) {
                const float dt = 0.02f;
                // Mirror proven Boost SoftStart→CC→CV→DONE (no MPPT; Vin = bridge DC).
                if (v_ac_in < MIN_AC_VOLTAGE) {
                    duty_accumulator = 0.0f;
                    fwdCurrIntegrator = 0.0f;
                    fwdVoltIntegrator = 0.0f;
                } else if (forwardMode == FWD_SOFTSTART) {
                    // Same ready rule as Boost: current OR timeout (no no-load latch).
                    duty_accumulator = boostApplySlew(FWD_SOFTSTART_SEED_DUTY, duty_accumulator, 2.0f, 5.0f);
                    bool ready = (i_bat_charge_filt >= 0.4f) ||
                                 (now - forward_mode_enter_ms >= FWD_SOFTSTART_MS);
                    if (ready) {
                        forwardMode = FWD_CC;
                        fwdCurrIntegrator = 0.0f;
                        Serial.printf("[INFO] FORWARD SoftStart done -> CC (I=%.2fA duty=%.0f)\n",
                                      i_bat_charge_filt, duty_accumulator);
                    }
                } else if (forwardMode == FWD_CC) {
                    float iRef = FWD_TARGET_CC_CURRENT;
                    // Pre-CV taper (same shape as Boost)
                    if (v_bat_filt >= FWD_CC_TAPER_START_V) {
                        float span = max(0.20f, TARGET_CV_VOLTAGE - FWD_CC_TAPER_START_V);
                        float rem = TARGET_CV_VOLTAGE - v_bat_filt;
                        float taper = boostClampf(rem / span, 0.10f, 1.0f);
                        iRef *= taper;
                    }
                    iRef = boostClampf(iRef, 0.0f, FWD_TARGET_CC_CURRENT);
                    float iErr = iRef - i_bat_charge_filt;
                    float dDuty = boostRunPI(iErr, FWD_CURR_KP, FWD_CURR_KI, dt,
                                             &fwdCurrIntegrator, FWD_CURR_OUT_MIN, FWD_CURR_OUT_MAX);
                    if (iErr > 0.8f && duty_accumulator < 280.0f && v_ac_in >= MIN_AC_VOLTAGE) {
                        dDuty = max(dDuty, 3.0f);
                    }
                    float dutyTarget = duty_accumulator + dDuty;
                    dutyTarget = boostClampf(dutyTarget, 0.0f, (float)allowed_max_duty);
                    duty_accumulator = boostApplySlew(dutyTarget, duty_accumulator,
                                                      FWD_DUTY_SLEW_UP, FWD_DUTY_SLEW_DOWN);
                    if (v_bat_filt >= FWD_CV_FORCE_VOLTAGE || max(v_bat, v_bat_filt) >= FWD_CV_FORCE_VOLTAGE) {
                        forwardMode = FWD_CV;
                        fwdCurrIntegrator = 0.0f;
                        fwdVoltIntegrator = 0.0f;
                        fwdIrefCvCmd = boostClampf(i_bat_charge_filt, 0.3f, 2.0f);
                        fwdCvEnterMs = 0;
                        Serial.printf("[INFO] Force FORWARD CV at Vbat=%.2f / filt=%.2f\n",
                                      v_bat, v_bat_filt);
                    } else if (v_bat_filt >= FWD_CV_ENTRY_VOLTAGE) {
                        if (fwdCvEnterMs == 0) fwdCvEnterMs = now;
                        if (now - fwdCvEnterMs >= FWD_CV_ENTER_CONFIRM_MS) {
                            forwardMode = FWD_CV;
                            fwdCurrIntegrator = 0.0f;
                            fwdVoltIntegrator = 0.0f;
                            fwdIrefCvCmd = boostClampf(i_bat_charge_filt, 0.3f, 2.0f);
                        }
                    } else {
                        fwdCvEnterMs = 0;
                    }
                } else if (forwardMode == FWD_CV) {
                    float vErr = TARGET_CV_VOLTAGE - v_bat_filt;
                    bool nearTarget = (fabsf(vErr) <= FWD_CV_NEAR_BAND_V);
                    if (fabs(vErr) <= CV_DEADBAND_V) {
                        vErr = 0.0f;
                        fwdVoltIntegrator *= 0.92f;
                    }
                    float iReq = boostRunPI(vErr, FWD_VOLT_KP, FWD_VOLT_KI, dt,
                                            &fwdVoltIntegrator, FWD_VOLT_OUT_MIN, FWD_VOLT_OUT_MAX);
                    if (nearTarget) {
                        float nearCap = 1.2f + boostClampf(vErr / FWD_CV_NEAR_BAND_V, 0.0f, 1.0f) * 1.0f;
                        if (iReq > nearCap) iReq = nearCap;
                    }
                    iReq = boostClampf(iReq, 0.0f, FWD_VOLT_OUT_MAX);
                    fwdIrefCvCmd = boostApplySlew(iReq, fwdIrefCvCmd,
                                                  FWD_CV_IREF_SLEW_A, FWD_CV_IREF_SLEW_A);
                    float iRef = fwdIrefCvCmd;
                    float iErr = iRef - i_bat_charge_filt;
                    float dDuty = boostRunPI(iErr, FWD_CURR_KP * (nearTarget ? 0.55f : 0.85f),
                                             FWD_CURR_KI * (nearTarget ? 0.45f : 0.70f),
                                             dt, &fwdCurrIntegrator,
                                             nearTarget ? -8.0f : FWD_CURR_OUT_MIN,
                                             nearTarget ? 8.0f : 18.0f);
                    float dutyStepLimit = nearTarget ? FWD_CV_DUTY_STEP_NEAR : FWD_CV_DUTY_STEP_FAR;
                    if (dDuty > dutyStepLimit) dDuty = dutyStepLimit;
                    if (dDuty < -dutyStepLimit) dDuty = -dutyStepLimit;
                    float dutyTarget = duty_accumulator + dDuty;
                    if (fabsf(TARGET_CV_VOLTAGE - v_bat_filt) <= CV_DEADBAND_V) {
                        dutyTarget = duty_accumulator;
                        fwdCurrIntegrator *= 0.95f;
                    }
                    if (v_bat_filt > TARGET_CV_VOLTAGE) {
                        float over = v_bat_filt - TARGET_CV_VOLTAGE;
                        dutyTarget -= (0.8f + over * 4.0f);
                        fwdVoltIntegrator *= 0.85f;
                    }
                    if (v_bat > (v_bat_filt + 1.5f)) {
                        dutyTarget = min(dutyTarget, duty_accumulator - 6.0f);
                        dutyTarget = max(0.0f, dutyTarget);
                    }
                    dutyTarget = boostClampf(dutyTarget, 0.0f, (float)allowed_max_duty);
                    float cvSlewUp = nearTarget ? 1.2f : 2.5f;
                    float cvSlewDown = nearTarget ? 2.0f : 4.0f;
                    duty_accumulator = boostApplySlew(dutyTarget, duty_accumulator, cvSlewUp, cvSlewDown);
                    if (v_bat_filt <= FWD_CV_EXIT_VOLTAGE) {
                        if (fwdCvExitMs == 0) fwdCvExitMs = now;
                        if (now - fwdCvExitMs >= FWD_CV_EXIT_CONFIRM_MS) {
                            forwardMode = FWD_CC;
                            fwdCurrIntegrator = 0.0f;
                            fwdVoltIntegrator = 0.0f;
                            fwdIrefCvCmd = 0.5f;
                            fwd_full_condition_start_ms = 0;
                        }
                    } else {
                        fwdCvExitMs = 0;
                    }
                    bool doneCond = (v_bat_filt >= FULL_DETECT_VOLTAGE) &&
                                    (i_bat_charge_filt <= FULL_END_CURRENT);
                    if (doneCond) {
                        if (fwd_full_condition_start_ms == 0) fwd_full_condition_start_ms = now;
                        if (now - fwd_full_condition_start_ms >= FULL_CONFIRM_MS) {
                            forwardMode = FWD_DONE;
                            charge_full_hold = true;
                            disablePowerStage();
                            Serial.println("[INFO] Battery FULL detected in FORWARD_CV.");
                        }
                    } else {
                        fwd_full_condition_start_ms = 0;
                    }
                } else { // FWD_DONE
                    duty_accumulator = 0.0f;
                    if (v_bat_filt <= RESTART_CHARGE_VOLTAGE && v_ac_in >= MIN_AC_VOLTAGE) {
                        charge_full_hold = false;
                        forwardMode = FWD_CC;
                        fwdCurrIntegrator = 0.0f;
                        fwdVoltIntegrator = 0.0f;
                        fwdIrefCvCmd = 0.5f;
                        Serial.println("[INFO] FORWARD resume from DONE -> CC.");
                    }
                }
                // Outer safety clamps — soft cuts like Boost (no current latch).
                if (i_bat_charge_abs > (FWD_TARGET_CC_CURRENT + 0.25f)) {
                    duty_accumulator -= (2.0f + (i_bat_charge_abs - FWD_TARGET_CC_CURRENT) * 3.0f);
                    fwdCurrIntegrator *= 0.8f;
                }
                if (fabs(i_ac_in) > FWD_AC_CURRENT_HARD_A) {
                    duty_accumulator -= 5.0f;
                    fwdCurrIntegrator *= 0.85f;
                }
                if (i_bat_charge_abs > FWD_BAT_CURRENT_HARD_A) {
                    duty_accumulator -= 5.0f;
                    fwdCurrIntegrator *= 0.8f;
                }
                if (v_bat_filt > TARGET_CV_VOLTAGE) {
                    float over_cv = v_bat_filt - TARGET_CV_VOLTAGE;
                    float cut = (forwardMode == FWD_CV) ? (0.6f + over_cv * 3.0f)
                                                        : (2.5f + over_cv * 10.0f);
                    duty_accumulator -= cut;
                    if (forwardMode != FWD_CV) fwdCurrIntegrator = 0.0f;
                }
                if (v_bat > (TARGET_CV_VOLTAGE + 0.4f)) {
                    duty_accumulator -= (forwardMode == FWD_CV) ? 3.0f : 8.0f;
                    if (forwardMode != FWD_CV) fwdCurrIntegrator = 0.0f;
                }
                if (v_bat_filt >= BMS_PREEMPT_ZONE_V || v_bat >= BMS_PREEMPT_ZONE_V) {
                    if (duty_accumulator > BMS_PREEMPT_DUTY_CAP_RAW) {
                        duty_accumulator = BMS_PREEMPT_DUTY_CAP_RAW;
                    }
                    if (v_bat_filt >= TARGET_CV_VOLTAGE) {
                        duty_accumulator = min(duty_accumulator, 80.0f);
                    }
                }
                duty_accumulator = boostClampf(duty_accumulator, 0.0f, (float)allowed_max_duty);
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
                    float iRef = min(TARGET_CC_CURRENT, boostNewIrefMppt);
                    // Collapse backoff only: if PV sags hard, pull Iref down.
                    // Never use measured Ppv as a hard current ceiling in CC.
                    if (v_solar < BOOST_PV_COLLAPSE_BACKOFF_V) {
                        float sag = BOOST_PV_COLLAPSE_BACKOFF_V - v_solar;
                        float collapseScale = boostClampf(1.0f - (sag * 0.35f), 0.15f, 1.0f);
                        iRef *= collapseScale;
                    }
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
                    // Help CC climb out of low-duty region when far below target.
                    if (iErr > 0.8f && duty_accumulator < 280.0f && v_solar >= BOOST_VOLTAGE_FLOOR) {
                        dDuty = max(dDuty, 3.0f);
                    }
                    float dutyTarget = duty_accumulator + dDuty;
                    dutyTarget = boostClampf(dutyTarget, 0.0f, (float)allowed_max_duty);
                    duty_accumulator = boostApplySlew(dutyTarget, duty_accumulator, BOOST_DUTY_SLEW_UP, BOOST_DUTY_SLEW_DOWN);
                    // Force CV immediately near BMS threshold; short confirm for soft entry
                    if (v_bat_filt >= BOOST_CV_FORCE_VOLTAGE || max(v_bat, v_bat_filt) >= BOOST_CV_FORCE_VOLTAGE) {
                        boostNewMode = BOOST_NEW_CV;
                        boostNewCurrIntegrator = 0.0f;
                        boostNewVoltIntegrator = 0.0f;
                        boostNewIrefCvCmd = boostClampf(i_bat_charge_filt, 0.3f, 2.0f);
                        boostNewCvEnterMs = 0;
                        Serial.printf("[INFO] Force CV at Vbat=%.2f / filt=%.2f\n", v_bat, v_bat_filt);
                    } else if (v_bat_filt >= BOOST_CV_ENTRY_VOLTAGE) {
                        if (boostNewCvEnterMs == 0) boostNewCvEnterMs = now;
                        if (now - boostNewCvEnterMs >= BOOST_CV_ENTER_CONFIRM_MS) {
                            boostNewMode = BOOST_NEW_CV;
                            boostNewCurrIntegrator = 0.0f;
                            boostNewVoltIntegrator = 0.0f;
                            boostNewIrefCvCmd = boostClampf(i_bat_charge_filt, 0.3f, 2.0f);
                        }
                    } else {
                        boostNewCvEnterMs = 0;
                    }
                } else if (boostNewMode == BOOST_NEW_CV) {
                    // Freeze MPPT Iref hunting in CV — voltage loop owns current request.
                    if (now - boostNewLastMpptMs >= BOOST_MPPT_PERIOD_MS) {
                        boostNewLastMpptMs = now;
                        float pPv = v_solar * i_solar_mag;
                        boostNewPAvailFilt = (boostNewPAvailFilt <= 0.01f) ? pPv : (0.22f * pPv + 0.78f * boostNewPAvailFilt);
                    }
                    float vErr = BOOST_CV_TARGET_VOLTAGE - v_bat_filt;
                    bool nearTarget = (fabsf(vErr) <= BOOST_CV_NEAR_BAND_V);
                    if (fabs(vErr) <= CV_DEADBAND_V) {
                        vErr = 0.0f;
                        boostNewVoltIntegrator *= 0.92f;
                    }
                    float iReq = boostRunPI(vErr, BOOST_VOLT_KP, BOOST_VOLT_KI, dt,
                                            &boostNewVoltIntegrator, BOOST_VOLT_OUT_MIN, BOOST_VOLT_OUT_MAX);
                    // Near target: keep Iref modest so terminal voltage stays calm.
                    if (nearTarget) {
                        float nearCap = 1.2f + boostClampf(vErr / BOOST_CV_NEAR_BAND_V, 0.0f, 1.0f) * 1.0f;
                        if (iReq > nearCap) iReq = nearCap;
                    }
                    if (v_solar < BOOST_PV_COLLAPSE_BACKOFF_V) {
                        float sag = BOOST_PV_COLLAPSE_BACKOFF_V - v_solar;
                        float collapseScale = boostClampf(1.0f - (sag * 0.35f), 0.15f, 1.0f);
                        iReq *= collapseScale;
                    }
                    iReq = boostClampf(iReq, 0.0f, BOOST_VOLT_OUT_MAX);
                    // Slew-limit CV current command to stop Ibat chatter.
                    boostNewIrefCvCmd = boostApplySlew(iReq, boostNewIrefCvCmd,
                                                       BOOST_CV_IREF_SLEW_A, BOOST_CV_IREF_SLEW_A);
                    float iRef = boostNewIrefCvCmd;
                    float iErr = iRef - i_bat_charge_filt;
                    float dDuty = boostRunPI(iErr, BOOST_CURR_KP * (nearTarget ? 0.55f : 0.85f),
                                             BOOST_CURR_KI * (nearTarget ? 0.45f : 0.70f),
                                             dt, &boostNewCurrIntegrator,
                                             nearTarget ? -8.0f : BOOST_CURR_OUT_MIN,
                                             nearTarget ? 8.0f : 18.0f);
                    float dutyStepLimit = nearTarget ? BOOST_CV_DUTY_STEP_NEAR : BOOST_CV_DUTY_STEP_FAR;
                    if (dDuty > dutyStepLimit) dDuty = dutyStepLimit;
                    if (dDuty < -dutyStepLimit) dDuty = -dutyStepLimit;
                    float dutyTarget = duty_accumulator + dDuty;
                    // Hold gently inside deadband.
                    if (fabsf(BOOST_CV_TARGET_VOLTAGE - v_bat_filt) <= CV_DEADBAND_V) {
                        dutyTarget = duty_accumulator;  // freeze duty
                        boostNewCurrIntegrator *= 0.95f;
                    }
                    // Above target: bleed duty gently (not a hard chop).
                    if (v_bat_filt > BOOST_CV_TARGET_VOLTAGE) {
                        float over = v_bat_filt - BOOST_CV_TARGET_VOLTAGE;
                        dutyTarget -= (0.8f + over * 4.0f);
                        boostNewVoltIntegrator *= 0.85f;
                    }
                    if (v_bat > (v_bat_filt + 1.5f)) {
                        dutyTarget = min(dutyTarget, duty_accumulator - 6.0f);
                        dutyTarget = max(0.0f, dutyTarget);
                    }
                    dutyTarget = boostClampf(dutyTarget, 0.0f, (float)allowed_max_duty);
                    float cvSlewUp = nearTarget ? 1.2f : 2.5f;
                    float cvSlewDown = nearTarget ? 2.0f : 4.0f;
                    duty_accumulator = boostApplySlew(dutyTarget, duty_accumulator, cvSlewUp, cvSlewDown);
                    if (v_bat_filt <= BOOST_CV_EXIT_VOLTAGE) {
                        if (boostNewCvExitMs == 0) boostNewCvExitMs = now;
                        if (now - boostNewCvExitMs >= BOOST_CV_EXIT_CONFIRM_MS) {
                            boostNewMode = BOOST_NEW_CC_MPPT;
                            boostNewCurrIntegrator = 0.0f;
                            boostNewVoltIntegrator = 0.0f;
                            boostNewIrefCvCmd = 0.5f;
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
                        boostNewIrefCvCmd = 0.5f;
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
                    // In CV mode the inner loop already bleeds duty; keep outer cut mild.
                    float cut = (boostNewMode == BOOST_NEW_CV) ? (0.6f + over_cv * 3.0f)
                                                              : (2.5f + over_cv * 10.0f);
                    duty_accumulator -= cut;
                    if (boostNewMode != BOOST_NEW_CV) boostNewCurrIntegrator = 0.0f;
                }
                if (v_bat > (BOOST_CV_TARGET_VOLTAGE + 0.4f)) {
                    duty_accumulator -= (boostNewMode == BOOST_NEW_CV) ? 3.0f : 8.0f;
                    if (boostNewMode != BOOST_NEW_CV) boostNewCurrIntegrator = 0.0f;
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
            fwd_full_condition_start_ms = 0;
            high_voltage_stop_start_ms = 0;
        }
        last_millis = now;
        active_duty_percent = round(((float)raw_duty * 100.0) / 1023.0);
        if (ENABLE_DEBUG_VERBOSE && (now - last_debug_time >= DEBUG_PRINT_INTERVAL_MS)) {
            last_debug_time = now;
            const char* sel_label =
                (selectedChargeMode == USER_MODE_BOOST) ? "BOOST" : "FORWARD";
            const char* run_label = "STANDBY";
            if (charge_full_hold) {
                run_label = "FULL_HOLD";
            } else if (ovp_latched) {
                run_label = "OVP_LOCK";
            } else if (!system_ON) {
                run_label = "STANDBY";
            } else if (currentState == STATE_BOOST) {
                if (boostNewMode == BOOST_NEW_SOFTSTART) run_label = "BOOST_SOFT";
                else if (boostNewMode == BOOST_NEW_CC_MPPT) run_label = "BOOST_CCMP";
                else if (boostNewMode == BOOST_NEW_CV) run_label = "BOOST_CV";
                else run_label = "BOOST_DONE";
            } else if (currentState == STATE_FORWARD) {
                if (forwardMode == FWD_SOFTSTART) run_label = "FWD_SOFT";
                else if (forwardMode == FWD_CC) run_label = "FWD_CC";
                else if (forwardMode == FWD_CV) run_label = "FWD_CV";
                else run_label = "FWD_DONE";
            } else if (system_ON) {
                run_label = "STARTING";  // ON but still STATE_OFF, entering selected path
            }
            Serial.println("=========================================================================================");
            Serial.printf("[DEBUG] System: %s | Mode: %s | Run: %s | Duty: %d%% (raw=%d)\n",
                          (system_ON ? "ON " : "OFF"), sel_label, run_label, active_duty_percent, raw_duty);
            // All calibrated sensors
            Serial.printf("  [PV ] V:%6.2fV  I:%6.2fA  |P|:%6.1fW  (mag I:%5.2fA)\n",
                          v_solar, i_solar, (v_solar * i_solar_mag), i_solar_mag);
            Serial.printf("  [AC ] V:%6.2fV  I:%6.2fA  |P|:%6.1fW  (bridge DC)\n",
                          v_ac_in, i_ac_in, fabsf(v_ac_in * i_ac_in));
            Serial.printf("  [BAT] V:%6.2fV  I:%6.2fA  Vf:%6.2fV  If:%6.2fA  Iabs:%5.2fA\n",
                          v_bat, i_bat, v_bat_filt, i_bat_filt, i_bat_charge_abs);
            // ADS raw (mV) — volt ADS 0x48 / curr ADS 0x49
            Serial.printf("  [RAW V mV] PV(ch0):%7.1f  AC(ch2):%7.1f  BAT(ch1):%7.1f\n",
                          raw_mv_v0, raw_mv_v1, raw_mv_v2);
            Serial.printf("  [RAW I mV] PV(ch0):%7.1f  AC(ch1):%7.1f  BAT(ch2):%7.1f\n",
                          raw_mv_i0, raw_mv_i1, raw_mv_i2);
            Serial.printf("  [I ZERO]   PV:%7.1f  AC:%7.1f  BAT:%7.1f  (boot offset mV)\n",
                          current_offset_i0, current_offset_i1, current_offset_i2);
            if (selectedChargeMode == USER_MODE_BOOST || currentState == STATE_BOOST) {
                Serial.printf("  [BOOST] Vref:%.2fV Iref_mppt:%.2fA Iref_cv:%.2fA Pavail:%.1fW\n",
                              boostNewPvRef, boostNewIrefMppt, boostNewIrefCvCmd, boostNewPAvailFilt);
            }
            if (selectedChargeMode == USER_MODE_FORWARD || currentState == STATE_FORWARD) {
                Serial.printf("  [FWD ] Iref_cv:%.2fA duty_acc:%.1f\n",
                              fwdIrefCvCmd, duty_accumulator);
            }
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
            show_no_power_alert = false;
            show_ovp_alert = false;
            if (system_ON || charge_full_hold) {
                // Running / FULL HOLD: STOP ends charge.
                system_ON = false;
                charge_full_hold = false;
            } else if (ovp_latched &&
                       v_bat_filt <= HARD_OVP_RELEASE_VOLTAGE &&
                       v_bat <= (HARD_OVP_RELEASE_VOLTAGE + 0.8f)) {
                // Standby + OVP: STOP clears latch when voltage is safe.
                ovp_latched = false;
                Serial.printf("[INFO] OVP latch cleared by STOP at %.2fV (release=%.2fV).\n",
                              max(v_bat, v_bat_filt), HARD_OVP_RELEASE_VOLTAGE);
            } else if (!ovp_latched) {
                // Standby: STOP toggles selected charge mode before START.
                selectedChargeMode = (selectedChargeMode == USER_MODE_BOOST)
                                         ? USER_MODE_FORWARD
                                         : USER_MODE_BOOST;
                Serial.printf("[INFO] Mode select -> %s (press START to begin)\n",
                              (selectedChargeMode == USER_MODE_BOOST) ? "BOOST PV" : "FORWARD AC");
                last_lcd_refresh = 0;  // force LCD refresh to show new mode
            }
        } else if (start_edge) {
            bool bat_ok = (v_bat_filt >= BAT_PRESENT_MIN_V) && (v_bat_filt <= BAT_START_MAX_V);
            bool selected_input_ok =
                (selectedChargeMode == USER_MODE_BOOST) ? (v_solar >= MIN_PV_VOLTAGE)
                                                        : (v_ac_in >= MIN_AC_VOLTAGE);
            if (ovp_latched) {
                system_ON = false;
                show_no_power_alert = false;
                show_ovp_alert = true;
                alert_millis = now;
            } else if (sensor_init_ok && bat_ok && selected_input_ok) {
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
                    lcdPrintLineRaw(3, "STOP to clear latch");
                } else if (show_no_power_alert) {
                    lcdPrintLineRaw(0, "ERROR");
                    if (selectedChargeMode == USER_MODE_BOOST) {
                        lcdPrintLineRaw(1, "NO PV FOR BOOST");
                        lcdPrintLineFmt(2, "Need PV>=%4.0fV", MIN_PV_VOLTAGE);
                    } else {
                        lcdPrintLineRaw(1, "NO AC FOR FORWARD");
                        lcdPrintLineFmt(2, "Need AC>=%4.0fV", MIN_AC_VOLTAGE);
                    }
                    lcdPrintLineRaw(3, "STOP=mode START=go");
                } else {
                    lcdPrintLineFmt(0, "STANDBY  %s",
                                    (selectedChargeMode == USER_MODE_BOOST) ? "BOOST" : "FORWD");
                    lcdPrintLineFmt(1, "PV :%5.1fV AC:%5.1fV", v_solar, v_ac_in);
                    lcdPrintLineFmt(2, "BATT:%5.1fV", v_bat);
                    lcdPrintLineRaw(3, "STOP=mode START=go");
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
