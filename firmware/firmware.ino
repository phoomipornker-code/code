#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <stdarg.h>
const char* FW_VERSION_TAG = "cv58-boost-v14-forward-v68";
// Boost path frozen to proven field code: cv58-stability-v14-cv-stable (PV charge OK).
// Forward: SoftStart→CC→CV→DONE with step/hysteresis control (no PID).
// Hardware design point: ~5 A at D≈45%; software CC setpoint is FWD_TARGET_CC_CURRENT.
// =========================================================================
// Hardware
// =========================================================================
const int RELAY_PV_PIN     = 32;
const int RELAY_AC_PIN     = 33;
const int BUTTON_START_PIN = 25;
const int BUTTON_STOP_PIN  = 26;  // STANDBY: กดสลับโหมด / ตอนชาร์จ: กดค้าง ~350ms หยุด
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
// Forward CV target (Boost keeps BOOST_CV_TARGET_VOLTAGE=56.00 separately).
// 55.9 V ≈ 3.49 V/cell — slightly under 56 to stay clear of BMS preempt / sense spikes.
const float TARGET_CV_VOLTAGE = 55.90;
const float TARGET_CC_CURRENT = 6.0;       // Boost CC (proven v14)
const float FWD_TARGET_CC_CURRENT = 3.0;   // Forward CC setpoint
const float MIN_PV_VOLTAGE = 42.0;
const float UNDER_PV_VOLTAGE_CRIT = 39.0;
// v_ac_in = DC after diode bridge (not VAC RMS). AC 110 V → ~155 Vpeak unloaded,
// typically ~100–140 V under load / with ripple averaging on ADS sample.
const float MIN_AC_VOLTAGE = 95.0;         // post-bridge DC low-line for AC 110 V
const int MAX_DUTY_FORWARD = 460;  // ~45% for Nr=Np reset @ 67 kHz — HW ~5 A at this D
const int MAX_DUTY_BOOST   = 760;
// Hardware capability (for feedforward): ~5 A near D=45%. SoftStart/CC scale from this.
const float FWD_DESIGN_I_AT_D45 = 5.0f;
const float FWD_DESIGN_DUTY_FRAC = 0.45f;
// Battery must be present and in a safe start window before enabling a power stage.
const float BAT_PRESENT_MIN_V = 40.0;
const float BAT_START_MAX_V = 56.40;
const unsigned long ADC_STALE_TIMEOUT_MS = 2500;  // was 700 — avoid LCD I2C contention trips
const unsigned long ADC_STALE_WARN_MS = 800;
const unsigned long SENSOR_ERROR_LOG_MS = 2000;
const float CV_DEADBAND_V = 0.12;
const float FULL_DETECT_VOLTAGE = 55.90;   // Boost FULL (CV 56.0)
const float FWD_FULL_DETECT_VOLTAGE = 55.80; // Forward FULL near CV 55.9
const float FULL_END_CURRENT = 0.50;
const unsigned long FULL_CONFIRM_MS = 60000;          // Boost
const unsigned long FWD_FULL_CONFIRM_MS = 15000;      // Forward: was 60s — too long near full
const unsigned long FWD_FULL_FAST_CONFIRM_MS = 5000;  // V peak≥CV and Iabs collapsed
const float HIGH_VOLTAGE_STOP_VOLTAGE = 56.80;
const unsigned long HIGH_VOLTAGE_STOP_CONFIRM_MS = 300;
const float RESTART_CHARGE_VOLTAGE = 54.0;
// =========================================================================
// Forward (AC) control: SoftStart → CC → CV → DONE  (NO PID — step/hysteresis)
// Slow duty steps protect Cin; freeze duty-up on AC sag / bus dip.
// =========================================================================
const float FWD_CV_ENTRY_VOLTAGE = 55.00;  // enter CV approaching 55.9
const float FWD_CV_FORCE_VOLTAGE = 55.30;
const float FWD_CV_EXIT_VOLTAGE  = 54.50;  // hysteresis below entry
const float FWD_CC_TAPER_START_V = 54.70;
const float FWD_CV_NEAR_BAND_V = 0.30;
const unsigned long FWD_SOFTSTART_MS = 5000;
const unsigned long FWD_CV_ENTER_CONFIRM_MS = 200;
const unsigned long FWD_CV_EXIT_CONFIRM_MS = 5000;
// Fine step sizes (raw duty per 20 ms) — smoother const-V / CC (was ~1.5–5).
const float FWD_STEP_UP_SOFT = 0.8f;
const float FWD_STEP_UP_CC = 0.6f;
const float FWD_STEP_UP_CC_FAR = 1.2f;
const float FWD_STEP_DOWN_CC = 1.5f;
const float FWD_STEP_DOWN_CC_FINE = 0.6f;
const float FWD_STEP_UP_CV = 0.40f;
const float FWD_STEP_UP_CV_NEAR = 0.20f;
const float FWD_STEP_DOWN_CV = 0.80f;
const float FWD_STEP_DOWN_CV_FINE = 0.35f;
const float FWD_STEP_DOWN_CV_OVER = 1.50f;
const float FWD_CV_HOLD_BAND_V = 0.05f;    // tighter hold around 55.9 V
const float FWD_CV_TAPER_I_A = 0.40f;      // near-full: freeze duty-up (prevents fly-up→BATspike)
const float FWD_CC_HOLD_BAND_A = 0.10f;
const float FWD_CC_FAR_BAND_A = 0.60f;
const float FWD_AC_HOLD_CLIMB_V = 115.0f; // freeze duty-up if bus dips (Cin stress)
const unsigned long FWD_AC_COLLAPSE_CONFIRM_MS = 15000;
const unsigned long FWD_AC_BRIEF_GLITCH_MS = 1500; // hold lone AC=0 blips while BAT OK / still charging
const float FWD_SOFTSTART_SEED_DUTY = 60.0;
const float FWD_AC_CURRENT_HARD_A = 2.5;
const float FWD_BAT_CURRENT_HARD_A = 3.75f;
const float FWD_AC_COLLAPSE_BACKOFF_V = 100.0;
const float FWD_AC_VOLTAGE_FLOOR = 95.0;
const float FWD_NS_NP_EST = 0.70f;
const float FWD_SOFTSTART_READY_DUTY_FRAC = 0.55f;
const float FWD_SOFTSTART_SEED_FRAC = 0.45f;
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
const float CAL_SCALE_V_BAT   = 41.5;
const float CAL_SCALE_I_SOLAR = 42.46;
const float CAL_SCALE_I_AC    = 42.46;
const float CAL_SCALE_I_BAT   = 42.46;
const float NOISE_V_THRESHOLD = 0.5;
const float NOISE_I_THRESHOLD = 0.08;
const float ADC_RAW_MIN_VALID_MV = 80.0;
const float ADC_GLITCH_CURRENT_GATE_A = 0.35;
const unsigned long ADC_GLITCH_LOG_MS = 5000;  // rate-limit glitch WARN spam
// Sudden BAT sense jump up — field CV entry: +2.89 V (≈70 mV raw) passed old 80 mV
// gate then SPIKE-PRECUT latched OVP at 57.92 while filt=55.33. ~50 mV ≈ 2.1 V.
const float ADC_BAT_HIGH_SPIKE_MV = 50.0f;
// 16S pack while charging must stay ~40–57 V ⇒ ADS mV ~955–1362. Field showed
// BATraw=104 mV (V=4.37) with I≈3 A — passed old MIN=80 and poisoned the filter.
const float ADC_BAT_MIN_PLAUSIBLE_MV = 850.0f;   // ~35.6 V
const float ADC_BAT_MAX_PLAUSIBLE_MV = 1500.0f;  // ~62.8 V
const float ADC_BAT_LOW_SPIKE_MV = 80.0f;        // sudden drop vs last good
const float ADC_BAT_V_STEP_SPIKE_V = 2.0f;       // voltage-domain twin of high-spike gate
const float MIN_CURRENT_FOR_ACTIVE_CHARGE = 0.20;
const unsigned long BMS_OPEN_CONFIRM_MS = 200;  // ignore brief V/I blips near CV
const unsigned long HARD_OVP_CONFIRM_MS = 80;   // ignore single-sample HARD OVP
// While charging, require sustained STOP to end (EMI on GPIO can false-edge).
const unsigned long STOP_HOLD_END_MS = 350;
// =========================================================================
// BOOST control (new flow): SOFTSTART -> CC_MPPT -> CV -> DONE
// =========================================================================
const float BOOST_VOLTAGE_FLOOR = 42.0;
const float BOOST_CV_TARGET_VOLTAGE = 56.00;  // conserve pack: stop/hold at 56V
const float BOOST_CV_ENTRY_VOLTAGE = 55.50;
const float BOOST_CV_FORCE_VOLTAGE = 55.70;
const float BOOST_CV_EXIT_VOLTAGE  = 54.80;   // wider hysteresis so CV does not chatter
const float BOOST_CC_TAPER_START_V = 54.80;
const float BMS_OPEN_DETECT_V = 56.80;      // above CV 55.9/56.0 — near-full is not open
const float BMS_OPEN_JUMP_DELTA_V = 1.2;
const float BMS_OPEN_CURRENT_MAX_A = 0.25f; // was 1.20 — taper ~0.7A near full ≠ BMS open
const float BMS_PREEMPT_DUTY_CAP_RAW = 140.0;
const float BMS_PREEMPT_ZONE_V = 55.95;
const float BMS_OPEN_NEAR_FULL_MAX_V = 56.60f; // Forward/Boost CV band: never BMS-OPEN here if I flowing
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
const bool ENABLE_DEBUG_STATUS = true;         // human [STAT] line
const bool ENABLE_DEBUG_CSV = true;            // Excel-ready TSV lines (prefix CSV)
const unsigned long DEBUG_PRINT_INTERVAL_MS = 5000;  // standby [STAT]
const unsigned long DEBUG_PRINT_CHARGE_MS = 3000;    // charge [STAT]
const unsigned long DEBUG_CSV_INTERVAL_MS = 1000;    // denser samples for Excel plots
const unsigned long LCD_REFRESH_INTERVAL_MS = 500;      // standby / FULL
const unsigned long LCD_CHARGE_REFRESH_MS = 2000;       // only used after FULL (or alerts)
// Soft resync kept for FULL/standby recover path — not used while charge-blanked.
const unsigned long LCD_SOFT_RESYNC_MS = 12000;
const unsigned long LCD_MUTEX_WAIT_MS = 80;
const uint32_t I2C_CLOCK_HZ = 100000;
const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;
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
float fwdIrefCcCmd = 0.0f;  // CC current target (debug)
float fwdIrefCvCmd = 0.0f;  // unused for step CV; kept for debug label
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
volatile bool lcd_force_refresh = true;
volatile unsigned long last_lcd_draw_ms = 0;
volatile unsigned long last_lcd_soft_resync_ms = 0;
volatile bool lcd_charge_blanked = false;  // true while charging: LCD off until FULL
volatile bool lcd_show_no_power = false;
volatile bool lcd_show_ovp_alert = false;
volatile unsigned long lcd_alert_until_ms = 0;
void TaskSampleData(void * pvParameters);
void TaskLCDLoop(void * pvParameters);
void calibrateCurrentOffsetsAtBoot();
int quantizeDutyWithDither(float duty_cmd, float *phase, int max_duty);
void lcdPrintLineRaw(uint8_t row, const char *text);
void lcdPrintLineFmt(uint8_t row, const char *fmt, ...);
void lcdDrawBatteryIconLine(uint8_t row, int socPct, bool charging);
void lcdSoftResyncNoBusReset();
void i2cBusSoftUnlock();
void reinitI2CBusAndLCD();
int estimatePackSocPct(float vPack);
void drawLcdScreen();
bool tryDrawLcdScreen();
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
// Duty fraction expected at FWD_TARGET_CC_CURRENT given HW 5 A @ D=45%.
static inline float forwardDutyFracForTargetI() {
    float frac = FWD_DESIGN_DUTY_FRAC * (FWD_TARGET_CC_CURRENT / FWD_DESIGN_I_AT_D45);
    return boostClampf(frac, 0.08f, FWD_DESIGN_DUTY_FRAC);
}
// Forward SoftStart seed: start below CC duty so Cin is not slammed open.
static inline int forwardEstimateDutyRaw(float vin, float vout, int maxDuty) {
    float vinUse = (vin > 80.0f) ? vin : 80.0f;
    float dTarget = forwardDutyFracForTargetI();
    float dV = vout / (vinUse * FWD_NS_NP_EST);
    dV = boostClampf(dV, 0.08f, FWD_DESIGN_DUTY_FRAC);
    float d = max(dV * (FWD_TARGET_CC_CURRENT / FWD_DESIGN_I_AT_D45), dTarget);
    d *= FWD_SOFTSTART_SEED_FRAC;  // only partial open in SoftStart
    d = boostClampf(d, 0.05f, dTarget);
    int raw = (int)roundf(d * 1023.0f);
    raw = constrain(raw, (int)FWD_SOFTSTART_SEED_DUTY, maxDuty);
    return raw;
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
    fwdIrefCcCmd = 0.0f;
    fwdIrefCvCmd = 0.0f;
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
    // Prefer no discard in the hot loop — each conversion ~1.2 ms @ 860 SPS and holds I2C.
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
    lcd_force_refresh = true;  // restore LCD (standby / OVP) after charge blank
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
// Battery icon using CGROM only (0xFF solid block). No CGRAM/createChar —
// EMI while charging corrupts custom glyphs into garbage (field photo).
void lcdDrawBatteryIconLine(uint8_t row, int socPct, bool charging) {
    int fill = (socPct * 10 + 50) / 100;  // 0..10
    if (fill < 0) fill = 0;
    if (fill > 10) fill = 10;
    lcd.setCursor(0, row);
    lcd.print('[');
    for (int i = 0; i < 10; i++) {
        if (i < fill) lcd.write((uint8_t)0xFF);  // solid block
        else lcd.print('-');
    }
    lcd.print(']');
    lcd.print(charging ? '*' : ' ');
    char pct[8];
    snprintf(pct, sizeof(pct), "%3d%%", socPct);
    lcd.print(pct);
    // [ + 10 + ] + mark + NNN% = 17; pad to 20
    for (int c = 17; c < 20; c++) lcd.print(' ');
}
// Re-sync HD44780 over the live I2C bus. Never Wire.end / pin bang while
// system_ON — that glitches ADS + control (see v51/v52).
void lcdSoftResyncNoBusReset() {
    lcd.init();
    lcd.backlight();
    lcd.clear();
}
void i2cBusSoftUnlock() {
    // Clock out stuck slave (SDA low) before Wire.begin — common after EMI.
    Wire.end();
    pinMode(I2C_SDA_PIN, INPUT_PULLUP);
    pinMode(I2C_SCL_PIN, OUTPUT);
    for (int i = 0; i < 16; i++) {
        digitalWrite(I2C_SCL_PIN, HIGH);
        delayMicroseconds(5);
        digitalWrite(I2C_SCL_PIN, LOW);
        delayMicroseconds(5);
    }
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(5);
}
void reinitI2CBusAndLCD() {
    i2cBusSoftUnlock();
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_CLOCK_HZ);
    Wire.setTimeOut(40);
    lcd.init();
    lcd.backlight();
    lcd.clear();
}
// 16S LFP voltage→SOC (display estimate). Charging V is a bit high vs rest.
int estimatePackSocPct(float vPack) {
    static const float vp[] = {40.0f, 48.0f, 49.6f, 51.2f, 52.0f, 52.8f, 53.6f, 54.4f, 55.2f, 55.9f};
    static const float sp[] = { 0.0f, 10.0f, 20.0f, 40.0f, 55.0f, 70.0f, 85.0f, 92.0f, 97.0f, 100.0f};
    const int n = 10;
    if (vPack <= vp[0]) return 0;
    if (vPack >= vp[n - 1]) return 100;
    for (int i = 0; i < n - 1; i++) {
        if (vPack <= vp[i + 1]) {
            float t = (vPack - vp[i]) / (vp[i + 1] - vp[i]);
            float s = sp[i] + t * (sp[i + 1] - sp[i]);
            int pct = (int)lroundf(s);
            if (pct < 0) pct = 0;
            if (pct > 100) pct = 100;
            return pct;
        }
    }
    return 100;
}
void drawLcdScreen() {
    // Caller holds i2c_Mutex. Snapshot locals to keep I2C burst short/consistent.
    const bool on = system_ON;
    const bool full = charge_full_hold;
    const bool ovp = ovp_latched;
    const UserChargeMode mode = selectedChargeMode;
    const float vb = v_bat;
    const float vbf = v_bat_filt;
    const float ibf = i_bat_filt;
    const float vs = v_solar;
    const float vac = v_ac_in;
    const float ovpTrip = ovp_trip_voltage;
    // Prefer filtered pack V for SOC; tiny IR trim while charging.
    float vSoc = vbf;
    if (on && (ibf > 0.3f)) vSoc = vbf - (ibf * 0.04f);
    const int soc = estimatePackSocPct(vSoc);

    // While actively charging: blank LCD (no I2C traffic) until FULL.
    // Still show OVP / error alerts if latched.
    if (on && !full && !ovp && !lcd_show_ovp_alert && !lcd_show_no_power) {
        if (!lcd_charge_blanked) {
            lcd.clear();
            lcd.noBacklight();
            lcd_charge_blanked = true;
        }
        return;
    }

    // Leaving blank period (FULL / STOP / OVP / standby) — restore backlight + content.
    lcd_charge_blanked = false;

    // Soft resync on FULL entry / standby recover (never Wire.end while ON).
    const unsigned long now = millis();
    if (full &&
        (last_lcd_soft_resync_ms == 0 ||
         (now - last_lcd_soft_resync_ms >= LCD_SOFT_RESYNC_MS))) {
        lcdSoftResyncNoBusReset();
        last_lcd_soft_resync_ms = now;
    }

    if (full) {
        lcdPrintLineRaw(0, "BATTERY FULL HOLD");
        lcdDrawBatteryIconLine(1, soc, false);
        lcdPrintLineFmt(2, "BAT:%5.1fV I:%4.2fA", vbf, ibf);
        lcdPrintLineRaw(3, "Hold STOP to end");
    } else if (ovp || lcd_show_ovp_alert) {
        lcdPrintLineRaw(0, "OVP TRIPPED");
        lcdPrintLineFmt(1, "VBAT:%5.1fV T:%4.1f", vbf, ovpTrip);
        lcdPrintLineFmt(2, "REL <= %5.1fV", HARD_OVP_RELEASE_VOLTAGE);
        lcdPrintLineRaw(3, "STOP to clear latch");
    } else if (lcd_show_no_power) {
        lcdPrintLineRaw(0, "ERROR");
        if (mode == USER_MODE_BOOST) {
            lcdPrintLineRaw(1, "NO PV FOR BOOST");
            lcdPrintLineFmt(2, "Need PV>=%4.0fV", MIN_PV_VOLTAGE);
        } else {
            lcdPrintLineRaw(1, "NO AC FOR FORWARD");
            lcdPrintLineFmt(2, "Need AC>=%4.0fV", MIN_AC_VOLTAGE);
        }
        lcdPrintLineRaw(3, "STOP=mode START=go");
    } else {
        lcdPrintLineFmt(0, "STANDBY  %s",
                        (mode == USER_MODE_BOOST) ? "BOOST" : "FORWD");
        lcdPrintLineFmt(1, "PV :%5.1fV AC:%5.1fV", vs, vac);
        lcdPrintLineFmt(2, "BAT:%5.1fV SOC:%3d%%", vb, soc);
        lcdPrintLineRaw(3, "STOP=mode START=go");
    }
    lcd.backlight();
}
bool tryDrawLcdScreen() {
    // Standby / FULL / alerts only. Active charging blanks once then skips I2C.
    if (!xSemaphoreTake(i2c_Mutex, pdMS_TO_TICKS(LCD_MUTEX_WAIT_MS))) {
        return false;
    }
    drawLcdScreen();
    xSemaphoreGive(i2c_Mutex);
    return true;
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
    Serial.printf("[BOOT] %s | B_CC=%.0fA F_CC=%.0fA CV=%.2fV DmaxF=%d\n",
                  FW_VERSION_TAG, TARGET_CC_CURRENT, FWD_TARGET_CC_CURRENT,
                  TARGET_CV_VOLTAGE, MAX_DUTY_FORWARD);
    // Real values + time (no fake Y offsets). Filter lines starting with CSV.
    Serial.println("CSV\ttime\tVin\tVout\tIout\tDuty");
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_CLOCK_HZ);
    Wire.setTimeOut(40);
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
    if (xSemaphoreTake(i2c_Mutex, pdMS_TO_TICKS(200))) {
        lcdPrintLineRaw(0, "BOOT OK");
        lcdPrintLineFmt(1, "%s", FW_VERSION_TAG);
        lcdPrintLineRaw(2, "LCD+ADS I2C ready");
        lcdPrintLineRaw(3, "STOP=mode START=go");
        xSemaphoreGive(i2c_Mutex);
    }
    if (!sensor_init_ok) {
        Serial.println("[FATAL] ADS1115 init failed. System is locked in safe standby.");
        forceSafeShutdown();
    } else {
        calibrateCurrentOffsetsAtBoot();
        last_adc_sample_ms = millis();
    }
    xTaskCreatePinnedToCore(TaskSampleData, "ADC_PWM_Task", 8192, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(TaskLCDLoop, "Button_Task", 4096, NULL, 2, NULL, 1);
}
void loop() { vTaskDelay(1000); }
void TaskSampleData(void * pvParameters) {
    unsigned long pv_collapse_start_time = 0;
    bool pv_is_collapsing = false;
    unsigned long ac_collapse_start_time = 0;
    bool ac_is_collapsing = false;
    unsigned long last_debug_time = 0;
    unsigned long last_csv_time = 0;
    int last_stat_sel = (int)USER_MODE_BOOST;  // match default — no STANDBY spam until toggle
    unsigned long last_sensor_error_log = 0;
    unsigned long full_condition_start_ms = 0;
    unsigned long fwd_full_condition_start_ms = 0;
    unsigned long high_voltage_stop_start_ms = 0;
    float last_valid_raw_mv_v0 = NAN;
    float last_valid_raw_mv_v1 = NAN;  // AC bridge
    float last_valid_raw_mv_v2 = NAN;
    float last_plausible_bat_mv = NAN;  // only updated with in-range BAT sense
    float last_good_v_bat = NAN;
    unsigned long ac_brief_low_since_ms = 0;
    unsigned long last_adc_glitch_log = 0;
    unsigned long bms_open_suspect_ms = 0;
    unsigned long hard_ovp_suspect_ms = 0;
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
        // Hold I2C mutex only for short ADS bursts; release between chips so LCD can run.
        // (Long single hold + EMI reinit was freezing the display while Forward charged.)
        if (xSemaphoreTake(i2c_Mutex, pdMS_TO_TICKS(80))) {
            raw_mv_v0 = readADCStable(ads_volt, 0) * 0.1875;
            raw_mv_v1 = readADCStable(ads_volt, 2) * 0.1875;
            raw_mv_v2 = readADCStable(ads_volt, 1) * 0.1875;
            xSemaphoreGive(i2c_Mutex);
            if (xSemaphoreTake(i2c_Mutex, pdMS_TO_TICKS(80))) {
                raw_mv_i0 = readADCStable(ads_curr, 0) * 0.1875;
                raw_mv_i1 = readADCStable(ads_curr, 1) * 0.1875;
                raw_mv_i2 = readADCStable(ads_curr, 2) * 0.1875;
                xSemaphoreGive(i2c_Mutex);
                sample_ok = true;
            }
        }
        if (sample_ok) {

            bool power_stage_active = (raw_duty > 0);
            // FORWARD leaves PV sense open/zero — not an ADC glitch.
            bool forward_active = (currentState == STATE_FORWARD);
            float pv_raw_before = raw_mv_v0;
            float ac_raw_before = raw_mv_v1;
            float bat_raw_before = raw_mv_v2;
            // Same ~40mV on BAT+AC while charging ⇒ ADS bus glitch.
            // Sudden AC→~0 while BAT still valid: brief glitch/dropout — hold AC short time.
            bool multi_ch_bus_glitch =
                power_stage_active &&
                (raw_mv_v2 < ADC_RAW_MIN_VALID_MV) &&
                (raw_mv_v1 < ADC_RAW_MIN_VALID_MV) &&
                (fabsf(raw_mv_v1 - raw_mv_v2) < 15.0f);
            bool ac_sudden_alone =
                forward_active && power_stage_active &&
                (raw_mv_v1 < ADC_RAW_MIN_VALID_MV) &&
                (raw_mv_v2 >= ADC_RAW_MIN_VALID_MV) &&
                !isnan(last_valid_raw_mv_v1) &&
                (last_valid_raw_mv_v1 >= ADC_RAW_MIN_VALID_MV);
            // Still delivering charge current with BAT sense OK ⇒ AC=0 is almost certainly ADC glitch.
            bool ac_glitch_while_charging =
                ac_sudden_alone &&
                (i_bat_charge_filt > 0.35f || i_bat_charge_abs > 0.35f);
            if (ac_sudden_alone && !ac_glitch_while_charging) {
                if (ac_brief_low_since_ms == 0) ac_brief_low_since_ms = now;
            } else if (!ac_sudden_alone) {
                ac_brief_low_since_ms = 0;
            }
            bool ac_brief_glitch = ac_glitch_while_charging ||
                (ac_sudden_alone &&
                 (now - ac_brief_low_since_ms < FWD_AC_BRIEF_GLITCH_MS));
            bool solar_raw_glitch = !forward_active &&
                                    (selectedChargeMode != USER_MODE_FORWARD) &&
                                    (raw_mv_v0 < ADC_RAW_MIN_VALID_MV) &&
                                    (power_stage_active ||
                                     i_solar_mag > ADC_GLITCH_CURRENT_GATE_A ||
                                     i_bat_charge_filt > ADC_GLITCH_CURRENT_GATE_A);
            bool ac_raw_glitch = multi_ch_bus_glitch || ac_brief_glitch;
            bool bat_raw_low_glitch = (raw_mv_v2 < ADC_RAW_MIN_VALID_MV) &&
                                      (power_stage_active ||
                                       i_bat_charge_filt > ADC_GLITCH_CURRENT_GATE_A ||
                                       multi_ch_bus_glitch);
            // Field: BAT jumped to ~91.6 V / ~100 V (BATraw~2408) — always reject out-of-range
            // when we already have a plausible pack reading (including after STOP).
            bool bat_mv_in_pack_range =
                (raw_mv_v2 >= ADC_BAT_MIN_PLAUSIBLE_MV) &&
                (raw_mv_v2 <= ADC_BAT_MAX_PLAUSIBLE_MV);
            bool bat_raw_high_spike =
                !isnan(last_plausible_bat_mv) &&
                (raw_mv_v2 > (last_plausible_bat_mv + ADC_BAT_HIGH_SPIKE_MV));
            bool bat_raw_low_spike =
                !isnan(last_plausible_bat_mv) &&
                (raw_mv_v2 < (last_plausible_bat_mv - ADC_BAT_LOW_SPIKE_MV));
            // Always implausible if outside 16S window once we know a good pack reading.
            bool bat_raw_implausible =
                !bat_mv_in_pack_range &&
                (!isnan(last_plausible_bat_mv) ||
                 ((power_stage_active || system_ON) &&
                  (raw_duty > 20 ||
                   i_bat_charge_filt > ADC_GLITCH_CURRENT_GATE_A ||
                   i_bat_charge_abs > ADC_GLITCH_CURRENT_GATE_A)));
            bool bat_raw_glitch = bat_raw_low_glitch || bat_raw_high_spike ||
                                 bat_raw_low_spike || bat_raw_implausible;
            if (solar_raw_glitch && !isnan(last_valid_raw_mv_v0)) {
                raw_mv_v0 = last_valid_raw_mv_v0;
            } else if (!solar_raw_glitch) {
                last_valid_raw_mv_v0 = raw_mv_v0;
            }
            if (ac_raw_glitch && !isnan(last_valid_raw_mv_v1)) {
                raw_mv_v1 = last_valid_raw_mv_v1;
            } else if (!ac_raw_glitch && raw_mv_v1 >= ADC_RAW_MIN_VALID_MV) {
                last_valid_raw_mv_v1 = raw_mv_v1;
            }
            if (bat_raw_glitch) {
                if (!isnan(last_plausible_bat_mv)) {
                    raw_mv_v2 = last_plausible_bat_mv;
                } else if (!isnan(last_valid_raw_mv_v2) &&
                           (last_valid_raw_mv_v2 >= ADC_BAT_MIN_PLAUSIBLE_MV) &&
                           (last_valid_raw_mv_v2 <= ADC_BAT_MAX_PLAUSIBLE_MV)) {
                    raw_mv_v2 = last_valid_raw_mv_v2;
                }
                // else: leave raw; post-convert path will avoid filter poison
            } else if (bat_mv_in_pack_range) {
                last_valid_raw_mv_v2 = raw_mv_v2;
                last_plausible_bat_mv = raw_mv_v2;
            }
            if ((solar_raw_glitch || ac_raw_glitch || bat_raw_glitch) &&
                (now - last_adc_glitch_log >= ADC_GLITCH_LOG_MS)) {
                last_adc_glitch_log = now;
                Serial.printf("[WARN] ADC glitch filtered: PVraw=%.1f ACraw=%.1f BATraw=%.1fmV duty=%d Ib=%.2fA%s%s%s%s\n",
                              pv_raw_before, ac_raw_before, bat_raw_before, raw_duty, i_bat_charge_filt,
                              multi_ch_bus_glitch ? " BUS!" : "",
                              ac_brief_glitch ? " ACblip!" : "",
                              bat_raw_high_spike ? " BATspike!" : "",
                              (bat_raw_implausible || bat_raw_low_spike) ? " BATbad!" : "");
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
            // Refuse to poison Vbat filter with out-of-range samples (4.4 V or ~100 V spikes).
            // Always — including after STOP (field: OVP at filt=74 after spike leaked in).
            const bool bat_v_implausible_now = (v_bat < 35.0f || v_bat > 62.0f);
            // Voltage-domain spike (field: +2.89 V ≈70 mV passed old 80 mV gate → false OVP).
            const bool bat_v_step_spike =
                !isnan(last_good_v_bat) &&
                (v_bat > (last_good_v_bat + ADC_BAT_V_STEP_SPIKE_V));
            if (bat_v_implausible_now || bat_v_step_spike) {
                if (!isnan(last_good_v_bat)) {
                    v_bat = last_good_v_bat;
                } else if (v_bat_filt >= 35.0f && v_bat_filt <= 62.0f) {
                    v_bat = v_bat_filt;
                }
            } else {
                last_good_v_bat = v_bat;
            }
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
            // Heal filter if it was already poisoned by a prior spike leak.
            if ((v_bat_filt < 35.0f || v_bat_filt > 62.0f) && !isnan(last_good_v_bat)) {
                for (int i = 0; i < 8; i++) vbat_filter_buf[i] = last_good_v_bat;
                vbat_filter_sum = last_good_v_bat * 8.0f;
                filter_count = 8;
                v_bat_filt = last_good_v_bat;
                v_bat = last_good_v_bat;
            }
            // Field near full: V raw≈56.4 while Vf lagged ~55.2 after spike holds — pull Vf up.
            static uint8_t v_catchup_hits = 0;
            if (!isnan(v_bat) && v_bat >= 35.0f && v_bat <= 62.0f &&
                (v_bat > (v_bat_filt + 0.40f)) && (v_bat - v_bat_filt) < 3.0f) {
                if (++v_catchup_hits >= 3) {
                    for (int i = 0; i < 8; i++) vbat_filter_buf[i] = v_bat;
                    vbat_filter_sum = v_bat * 8.0f;
                    filter_count = 8;
                    v_bat_filt = v_bat;
                    v_catchup_hits = 0;
                }
            } else {
                v_catchup_hits = 0;
            }
            // Field: Iabs≈0 but If stuck ~2 A (boxcar stale) → FULL never trips. Snap filt to raw.
            static uint8_t i_zero_hits = 0;
            if (fabsf(i_bat) <= NOISE_I_THRESHOLD && fabsf(i_bat_filt) > 0.40f) {
                if (++i_zero_hits >= 5) {
                    for (int i = 0; i < 8; i++) ibat_filter_buf[i] = i_bat;
                    ibat_filter_sum = i_bat * 8.0f;
                    filter_count = 8;
                    i_bat_filt = i_bat;
                    i_zero_hits = 0;
                }
            } else {
                i_zero_hits = 0;
            }
            i_solar_mag = fabs(i_solar);
            i_bat_charge_filt = fabs(i_bat_filt);
            i_bat_charge_abs = fabs(i_bat);
            float vbat_step = (last_vbat_sample > 0.0f) ? (v_bat - last_vbat_sample) : 0.0f;
            float vbat_filt_step = (last_vbat_filt_sample > 0.0f) ? (v_bat_filt - last_vbat_filt_sample) : 0.0f;
            // BMS open: real open raises V hard AND collapses I near zero.
            // Field: CV near full Vf~56.2 I~0.68A was false-tripped (I≤1.2A looked "collapsed").
            bool bms_zone = (v_bat_filt >= BMS_PREEMPT_ZONE_V || v_bat >= BMS_PREEMPT_ZONE_V);
            bool i_collapsed = (i_bat_charge_filt <= BMS_OPEN_CURRENT_MAX_A) &&
                               (i_bat_charge_abs <= (BMS_OPEN_CURRENT_MAX_A + 0.15f));
            bool bms_v_jump =
                (vbat_step >= BMS_OPEN_JUMP_DELTA_V) ||
                (v_bat > (v_bat_filt + 1.8f));
            bool bms_v_high =
                (v_bat >= BMS_OPEN_DETECT_V) &&
                (v_bat_filt >= (BMS_OPEN_DETECT_V - 0.25f));
            bool bms_v_event = bms_v_jump || bms_v_high;
            // Near-full CV taper (V≤56.6 with charge current still flowing) is NOT BMS open.
            bool near_full_taper =
                (v_bat_filt <= BMS_OPEN_NEAR_FULL_MAX_V) &&
                (v_bat <= (BMS_OPEN_NEAR_FULL_MAX_V + 0.30f)) &&
                (i_bat_charge_filt > BMS_OPEN_CURRENT_MAX_A);
            bool bms_suspect = !ovp_latched &&
                               system_ON &&
                               (currentState == STATE_BOOST || currentState == STATE_FORWARD) &&
                               raw_duty > 0 &&
                               bms_zone &&
                               bms_v_event &&
                               !near_full_taper;
            if (bms_suspect && i_collapsed) {
                if (bms_open_suspect_ms == 0) bms_open_suspect_ms = now;
                if (now - bms_open_suspect_ms >= BMS_OPEN_CONFIRM_MS) {
                    ovp_latched = true;
                    ovp_trip_voltage = max(v_bat, v_bat_filt);
                    ovp_trip_ms = now;
                    forceSafeShutdown();
                    // Do not pretend this is a normal FULL — latch OVP for STOP clear.
                    bms_open_suspect_ms = 0;
                    last_lcd_soft_resync_ms = 0;
                    lcd_force_refresh = true;
                    Serial.printf("[CRITICAL] BMS-OPEN/preempt at raw=%.2f filt=%.2f I=%.2fA step=%.2f. PWM off.\n",
                                  v_bat, v_bat_filt, i_bat_charge_filt, vbat_step);
                }
            } else {
                bms_open_suspect_ms = 0;
            }
            if (!ovp_latched &&
                (v_bat >= HARD_OVP_TRIP_VOLTAGE || v_bat_filt >= HARD_OVP_TRIP_VOLTAGE)) {
                // Field after STOP: spike leaked → filt=74 / trip 80.52 — not a real pack OVP.
                // Only latch when readings stay inside a physical 16S window.
                const bool filt_pack_ok = (v_bat_filt >= 35.0f && v_bat_filt <= 62.0f);
                const bool raw_pack_ok = (v_bat >= 35.0f && v_bat <= 62.0f);
                bool filt_trip = filt_pack_ok && (v_bat_filt >= HARD_OVP_TRIP_VOLTAGE);
                bool raw_trip_plausible =
                    raw_pack_ok &&
                    (v_bat >= HARD_OVP_TRIP_VOLTAGE) &&
                    filt_pack_ok &&
                    (v_bat_filt >= (HARD_OVP_TRIP_VOLTAGE - 1.5f)) &&
                    (vbat_step < 8.0f);
                if (filt_trip || raw_trip_plausible) {
                    if (hard_ovp_suspect_ms == 0) hard_ovp_suspect_ms = now;
                    if (now - hard_ovp_suspect_ms >= HARD_OVP_CONFIRM_MS) {
                        ovp_latched = true;
                        ovp_trip_voltage = max(v_bat, v_bat_filt);
                        ovp_trip_ms = now;
                        forceSafeShutdown();
                        hard_ovp_suspect_ms = 0;
                        Serial.printf("[CRITICAL] HARD OVP TRIP at %.2fV (filt=%.2f trip=%.2fV). Output disabled.\n",
                                      ovp_trip_voltage, v_bat_filt, HARD_OVP_TRIP_VOLTAGE);
                    }
                } else {
                    hard_ovp_suspect_ms = 0;
                }
            } else {
                hard_ovp_suspect_ms = 0;
            }
            // Near CV runaway soft-cut — proven Boost logic, also applied to Forward.
            // Never latch OVP from a lone raw spike while filt is still far below trip
            // (field: SPIKE-PRECUT 57.92 / filt 55.33 right after CC→CV with I still flowing).
            if (!ovp_latched &&
                system_ON &&
                (currentState == STATE_BOOST || currentState == STATE_FORWARD) &&
                raw_duty > 0 &&
                v_bat_filt >= BOOST_CV_ENTRY_VOLTAGE &&
                i_bat_charge_filt < MIN_CURRENT_FOR_ACTIVE_CHARGE &&
                v_bat > (v_bat_filt + 3.0f)) {
                const bool runaway_ovp =
                    (v_bat >= HARD_OVP_TRIP_VOLTAGE) &&
                    (v_bat_filt >= (HARD_OVP_TRIP_VOLTAGE - 1.0f));
                if (runaway_ovp) {
                    ovp_latched = true;
                    ovp_trip_voltage = v_bat;
                    ovp_trip_ms = now;
                    forceSafeShutdown();
                    Serial.printf("[CRITICAL] RUNAWAY-CUT at %.2fV (filt=%.2fV, duty=%d).\n",
                                  v_bat, v_bat_filt, raw_duty);
                } else {
                    duty_accumulator = max(0.0f, duty_accumulator - 15.0f);
                    if (currentState == STATE_BOOST) {
                        boostNewCurrIntegrator = 0.0f;
                        if (boostNewMode != BOOST_NEW_CV) {
                            boostNewMode = BOOST_NEW_CV;
                            boostNewVoltIntegrator = 0.0f;
                        }
                    } else {
                        if (forwardMode != FWD_CV) {
                            forwardMode = FWD_CV;
                        }
                    }
                    Serial.printf("[WARN] Runaway soft-cut duty at %.2fV (filt=%.2fV).\n",
                                  v_bat, v_bat_filt);
                }
            }
            if (!ovp_latched &&
                system_ON &&
                (currentState == STATE_BOOST || currentState == STATE_FORWARD) &&
                raw_duty > 0 &&
                v_bat_filt >= (BOOST_CV_ENTRY_VOLTAGE - 0.2f) &&
                (vbat_step > BOOST_VBAT_SPIKE_PRECUT_DELTA_V ||
                 (vbat_step > 0.7f && vbat_filt_step > 0.25f)) &&
                v_bat > (v_bat_filt + BOOST_VBAT_SPIKE_PRECUT_RAW_ABOVE_FILT_V)) {
                // Latch only if filt also near HARD OVP (real runaway), not mux ghost.
                const bool spike_ovp =
                    (v_bat >= HARD_OVP_TRIP_VOLTAGE) &&
                    (v_bat_filt >= (HARD_OVP_TRIP_VOLTAGE - 1.0f)) &&
                    (i_bat_charge_filt <= 0.35f) &&
                    (i_bat_charge_abs <= 0.50f);
                if (spike_ovp) {
                    ovp_latched = true;
                    ovp_trip_voltage = v_bat;
                    ovp_trip_ms = now;
                    forceSafeShutdown();
                    Serial.printf("[CRITICAL] SPIKE-PRECUT at %.2fV (step=%.2fV, filt=%.2fV, duty=%d).\n",
                                  v_bat, vbat_step, v_bat_filt, raw_duty);
                } else {
                    duty_accumulator = max(0.0f, duty_accumulator - 12.0f);
                    if (currentState == STATE_BOOST) {
                        boostNewCurrIntegrator = 0.0f;
                        if (boostNewMode != BOOST_NEW_CV) {
                            boostNewMode = BOOST_NEW_CV;
                            boostNewVoltIntegrator = 0.0f;
                        }
                    } else {
                        if (forwardMode != FWD_CV) {
                            forwardMode = FWD_CV;
                        }
                    }
                    Serial.printf("[WARN] Spike soft-cut duty at %.2fV (step=%.2fV filt=%.2f I=%.2fA).\n",
                                  v_bat, vbat_step, v_bat_filt, i_bat_charge_filt);
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
        }
        if (!sample_ok && system_ON) {
            unsigned long stale_ms = now - last_adc_sample_ms;
            if (stale_ms > ADC_STALE_WARN_MS &&
                (now - last_sensor_error_log >= SENSOR_ERROR_LOG_MS)) {
                last_sensor_error_log = now;
                Serial.printf("[WARN] ADC bus busy/stale %lums (LCD contention?).\n", stale_ms);
            }
            if (stale_ms > ADC_STALE_TIMEOUT_MS) {
                forceSafeShutdown();
                Serial.println("[CRITICAL] ADC sample timeout. Auto-shutdown for safety.");
            }
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
                // Freeze duty-up on real sag only. AC=0 while Ibat still flows = sense glitch.
                bool ac_sense_dead = (v_ac_in < MIN_AC_VOLTAGE);
                bool still_charging = (i_bat_charge_filt > 0.35f) || (i_bat_charge_abs > 0.35f);
                if (ac_sense_dead && still_charging) {
                    ac_is_collapsing = false;
                } else if (ac_sense_dead) {
                    if (!ac_is_collapsing) {
                        ac_is_collapsing = true;
                        ac_collapse_start_time = now;
                        Serial.printf("[WARN] AC sag %.1fV — freeze duty-up (no pause). Shutdown if >%lums.\n",
                                      v_ac_in, FWD_AC_COLLAPSE_CONFIRM_MS);
                    }
                    if (now - ac_collapse_start_time >= FWD_AC_COLLAPSE_CONFIRM_MS) {
                        system_ON = false;
                        Serial.println("[CRITICAL] AC bridge lost (sustained). Auto-Shutdown.");
                    }
                } else {
                    ac_is_collapsing = false;
                }
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
            ac_is_collapsing = false;
            boostNewMode = BOOST_NEW_SOFTSTART;
            forwardMode = FWD_SOFTSTART;
        }
        if (system_ON && currentState != STATE_OFF) {
            int allowed_max_duty = (currentState == STATE_FORWARD) ? MAX_DUTY_FORWARD : MAX_DUTY_BOOST;
            if (currentState == STATE_FORWARD) {
                // No PID: SoftStart ramp + CC/CV hysteresis steps. Freeze up on Cin/AC stress.
                // Do not freeze on AC=0 while battery current still flows (ADC blip).
                const bool ac_fake_low = (v_ac_in < MIN_AC_VOLTAGE) &&
                                         ((i_bat_charge_filt > 0.35f) || (i_bat_charge_abs > 0.35f));
                const bool freezeDutyUp = (!ac_fake_low && ((v_ac_in < MIN_AC_VOLTAGE) || ac_is_collapsing)) ||
                                          (v_ac_in < FWD_AC_HOLD_CLIMB_V && raw_duty > 40 && !ac_fake_low);
                if (forwardMode == FWD_SOFTSTART) {
                    int seedDuty = forwardEstimateDutyRaw(v_ac_in, TARGET_CV_VOLTAGE, allowed_max_duty);
                    if (!freezeDutyUp && duty_accumulator < (float)seedDuty) {
                        duty_accumulator += FWD_STEP_UP_SOFT;
                    }
                    if (duty_accumulator > (float)seedDuty) duty_accumulator = (float)seedDuty;
                    bool nearSeed = (duty_accumulator >= ((float)seedDuty * FWD_SOFTSTART_READY_DUTY_FRAC));
                    bool ready = (nearSeed && (i_bat_charge_filt >= 0.25f)) ||
                                 (now - forward_mode_enter_ms >= FWD_SOFTSTART_MS);
                    if (ready) {
                        forwardMode = FWD_CC;
                        Serial.printf("[INFO] FORWARD SoftStart done -> CC step (I=%.2fA duty=%.0f seed=%d)\n",
                                      i_bat_charge_filt, duty_accumulator, seedDuty);
                    }
                } else if (forwardMode == FWD_CC) {
                    float iRef = FWD_TARGET_CC_CURRENT;
                    if (v_ac_in < FWD_AC_COLLAPSE_BACKOFF_V) {
                        float sag = FWD_AC_COLLAPSE_BACKOFF_V - v_ac_in;
                        float collapseScale = boostClampf(1.0f - (sag * 0.35f), 0.15f, 1.0f);
                        iRef *= collapseScale;
                    }
                    if (v_bat_filt >= FWD_CC_TAPER_START_V) {
                        float span = max(0.20f, TARGET_CV_VOLTAGE - FWD_CC_TAPER_START_V);
                        float rem = TARGET_CV_VOLTAGE - v_bat_filt;
                        float taper = boostClampf(rem / span, 0.10f, 1.0f);
                        iRef *= taper;
                    }
                    iRef = boostClampf(iRef, 0.0f, FWD_TARGET_CC_CURRENT);
                    fwdIrefCcCmd = iRef;
                    float iErr = iRef - i_bat_charge_filt;
                    if (iErr > FWD_CC_HOLD_BAND_A) {
                        // Climb until I hits band or Dmax — no FF ceiling (Vac high made
                        // dutyFf~276 and stuck at ~317 with I still ~0.4A).
                        if (!freezeDutyUp) {
                            float step = (iErr > FWD_CC_FAR_BAND_A) ? FWD_STEP_UP_CC_FAR : FWD_STEP_UP_CC;
                            duty_accumulator += step;
                        }
                    } else if (iErr < -FWD_CC_HOLD_BAND_A) {
                        float stepDn = (iErr < -FWD_CC_FAR_BAND_A) ? FWD_STEP_DOWN_CC
                                                                    : FWD_STEP_DOWN_CC_FINE;
                        duty_accumulator -= stepDn;
                    }
                    // else hold duty (hysteresis band)
                    // Enter CV earlier (field: Vbat~55.2–55.4 stuck in tapered CC).
                    float vBatPeak = max(v_bat, v_bat_filt);
                    if (vBatPeak >= FWD_CV_FORCE_VOLTAGE) {
                        forwardMode = FWD_CV;
                        fwdCvEnterMs = 0;
                        Serial.printf("[INFO] Force FORWARD CV at Vbat=%.2f / filt=%.2f\n",
                                      v_bat, v_bat_filt);
                    } else if (vBatPeak >= FWD_CV_ENTRY_VOLTAGE) {
                        if (fwdCvEnterMs == 0) fwdCvEnterMs = now;
                        if (now - fwdCvEnterMs >= FWD_CV_ENTER_CONFIRM_MS) {
                            forwardMode = FWD_CV;
                            Serial.printf("[INFO] FORWARD CC -> CV at Vbat=%.2f / filt=%.2f\n",
                                          v_bat, v_bat_filt);
                        }
                    } else {
                        fwdCvEnterMs = 0;
                    }
                } else if (forwardMode == FWD_CV) {
                    // Fine const-V: small steps; never slam duty (BMS cap 140 killed CV at ~55.9).
                    float vReg = v_bat_filt;
                    float vPeak = max(v_bat, v_bat_filt);
                    float vErr = TARGET_CV_VOLTAGE - vReg;
                    fwdIrefCvCmd = TARGET_CV_VOLTAGE;
                    if (vPeak > (TARGET_CV_VOLTAGE + 0.25f)) {
                        float over = vPeak - TARGET_CV_VOLTAGE;
                        duty_accumulator -= (FWD_STEP_DOWN_CV_OVER + over * 1.5f);
                    } else if (vErr > FWD_CV_HOLD_BAND_V) {
                        // Below target — raise duty, but freeze climb when current has already
                        // tapered (near full). Field: duty ran to Dmax@I≈0.16A → sense fly-up spikes.
                        if (!freezeDutyUp && i_bat_charge_abs < FWD_TARGET_CC_CURRENT) {
                            if (i_bat_charge_filt <= FWD_CV_TAPER_I_A && vErr < 0.60f) {
                                // hold duty — do not open toward Dmax into a tapering pack
                            } else {
                                float step = (vErr > FWD_CV_NEAR_BAND_V) ? FWD_STEP_UP_CV
                                                                         : FWD_STEP_UP_CV_NEAR;
                                duty_accumulator += step;
                            }
                        } else if (i_bat_charge_abs > (FWD_TARGET_CC_CURRENT + 0.15f) &&
                                   vErr < FWD_CV_NEAR_BAND_V) {
                            duty_accumulator -= FWD_STEP_DOWN_CV_FINE;
                        }
                    } else if (vErr < -FWD_CV_HOLD_BAND_V) {
                        float over = -vErr;
                        float stepDn = (over > FWD_CV_NEAR_BAND_V) ? FWD_STEP_DOWN_CV_OVER
                                       : (over > 0.12f)            ? FWD_STEP_DOWN_CV
                                                                   : FWD_STEP_DOWN_CV_FINE;
                        duty_accumulator -= stepDn;
                    }
                    // else in hold band → constant voltage
                    if (v_bat > (v_bat_filt + 1.5f)) {
                        duty_accumulator -= FWD_STEP_DOWN_CV_OVER;
                    }
                    if (v_bat_filt <= FWD_CV_EXIT_VOLTAGE) {
                        if (fwdCvExitMs == 0) fwdCvExitMs = now;
                        if (now - fwdCvExitMs >= FWD_CV_EXIT_CONFIRM_MS) {
                            forwardMode = FWD_CC;
                            fwd_full_condition_start_ms = 0;
                        }
                    } else {
                        fwdCvExitMs = 0;
                    }
                    // Field: Vf lagged at 55.21 while Vraw=56.38 and Iabs=0 / If stuck 2A
                    // → never reached old (Vf≥55.8 && If≤0.5 for 60s). Use peak V + min I.
                    const float vFull = max(v_bat, v_bat_filt);
                    const float iFull = min(i_bat_charge_filt, i_bat_charge_abs);
                    bool doneSlow = (vFull >= FWD_FULL_DETECT_VOLTAGE) &&
                                    (iFull <= FULL_END_CURRENT);
                    bool doneFast = (vFull >= TARGET_CV_VOLTAGE) &&
                                    (i_bat_charge_abs <= 0.35f) &&
                                    (iFull <= 1.00f);
                    bool doneCond = doneSlow || doneFast;
                    unsigned long needMs = doneFast ? FWD_FULL_FAST_CONFIRM_MS
                                                     : FWD_FULL_CONFIRM_MS;
                    if (doneCond) {
                        if (fwd_full_condition_start_ms == 0) fwd_full_condition_start_ms = now;
                        if (now - fwd_full_condition_start_ms >= needMs) {
                            forwardMode = FWD_DONE;
                            charge_full_hold = true;
                            disablePowerStage();
                            last_lcd_soft_resync_ms = 0;
                            lcd_force_refresh = true;
                            Serial.printf("[INFO] Battery FULL (FORWARD CV) V=%.2f/%.2f I=%.2f/%.2fA\n",
                                          v_bat, v_bat_filt, i_bat_charge_abs, i_bat_charge_filt);
                        }
                    } else {
                        fwd_full_condition_start_ms = 0;
                    }
                } else { // FWD_DONE
                    duty_accumulator = 0.0f;
                    if (v_bat_filt <= RESTART_CHARGE_VOLTAGE && v_ac_in >= MIN_AC_VOLTAGE) {
                        charge_full_hold = false;
                        forwardMode = FWD_CC;
                        Serial.println("[INFO] FORWARD resume from DONE -> CC.");
                    }
                }
                // Outer safety clamps (step cuts — no PID unwind).
                if (i_bat_charge_abs > (FWD_TARGET_CC_CURRENT + 0.25f) && forwardMode != FWD_CV) {
                    duty_accumulator -= (2.0f + (i_bat_charge_abs - FWD_TARGET_CC_CURRENT) * 3.0f);
                }
                if (fabs(i_ac_in) > FWD_AC_CURRENT_HARD_A) {
                    duty_accumulator -= 2.0f;
                }
                if (i_bat_charge_abs > FWD_BAT_CURRENT_HARD_A) {
                    duty_accumulator -= (forwardMode == FWD_CV) ? 1.0f : 5.0f;
                }
                // Do NOT dump duty on AC sag — freeze duty-up only.
                // Outer OVP: in Forward CV, leave fine const-V to the phase loop unless clearly over.
                if (forwardMode == FWD_CV) {
                    if (v_bat_filt > (TARGET_CV_VOLTAGE + 0.15f)) {
                        float over_cv = v_bat_filt - TARGET_CV_VOLTAGE;
                        duty_accumulator -= (0.3f + over_cv * 1.5f);
                    }
                    if (v_bat > (TARGET_CV_VOLTAGE + 0.50f)) {
                        duty_accumulator -= 1.2f;
                    }
                } else {
                    if (v_bat_filt > TARGET_CV_VOLTAGE) {
                        float over_cv = v_bat_filt - TARGET_CV_VOLTAGE;
                        duty_accumulator -= (2.5f + over_cv * 10.0f);
                    }
                    if (v_bat > (TARGET_CV_VOLTAGE + 0.4f)) {
                        duty_accumulator -= 8.0f;
                    }
                }
                if (v_bat_filt >= BMS_PREEMPT_ZONE_V || v_bat >= BMS_PREEMPT_ZONE_V) {
                    if (forwardMode == FWD_CV) {
                        // Field: slam to 140 at ~55.9 killed CV (duty 380→142).
                        // Const-V already regulates; hard cap only near BMS-open.
                        if (v_bat >= BMS_OPEN_DETECT_V || v_bat_filt >= BMS_OPEN_DETECT_V) {
                            if (duty_accumulator > BMS_PREEMPT_DUTY_CAP_RAW) {
                                duty_accumulator = BMS_PREEMPT_DUTY_CAP_RAW;
                            }
                            if (v_bat_filt >= TARGET_CV_VOLTAGE) {
                                duty_accumulator = min(duty_accumulator, 80.0f);
                            }
                        }
                    } else {
                        if (duty_accumulator > BMS_PREEMPT_DUTY_CAP_RAW) {
                            duty_accumulator = BMS_PREEMPT_DUTY_CAP_RAW;
                        }
                        if (v_bat_filt >= TARGET_CV_VOLTAGE) {
                            duty_accumulator = min(duty_accumulator, 80.0f);
                        }
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
                            last_lcd_soft_resync_ms = 0;
                            lcd_force_refresh = true;
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
                // No dither on Forward — 1-LSB toggling looked like signal cuts on the scope.
                raw_duty = constrain((int)lroundf(duty_accumulator), 0, allowed_max_duty);
                forward_dither_phase = 0.0;
                ledcWrite(PWM_FORWARD_PIN, raw_duty);
                ledcWrite(PWM_BOOST_PIN, 0);
            } else if (currentState == STATE_BOOST) {
                raw_duty = quantizeDutyWithDither(duty_accumulator, &boost_dither_phase, allowed_max_duty);
                forward_dither_phase = 0.0;
                ledcWrite(PWM_BOOST_PIN, raw_duty);
                ledcWrite(PWM_FORWARD_PIN, 0);
            }
            total_Wh += ((v_bat * i_bat_charge_filt) * (now - last_millis)) / 3600000.0;
            // High-V stop: use peak so lagged Vf cannot block cut near full.
            const float vStop = max(v_bat, v_bat_filt);
            if (vStop >= HIGH_VOLTAGE_STOP_VOLTAGE) {
                if (high_voltage_stop_start_ms == 0) high_voltage_stop_start_ms = now;
                if (now - high_voltage_stop_start_ms >= HIGH_VOLTAGE_STOP_CONFIRM_MS) {
                    charge_full_hold = true;
                    disablePowerStage();
                    last_lcd_soft_resync_ms = 0;
                    lcd_force_refresh = true;
                    Serial.printf("[INFO] High-voltage charge stop at %.2fV (filt=%.2f). Enter FULL HOLD.\n",
                                  vStop, v_bat_filt);
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
        // LCD: blank while charging; refresh standby / FULL / alerts only.
        const bool chargeActive =
            system_ON && !charge_full_hold && !ovp_latched && !lcd_show_ovp_alert;
        if (chargeActive) {
            // One-shot blank on entry; then no LCD I2C until FULL/STOP/OVP.
            if (!lcd_charge_blanked || lcd_force_refresh) {
                if (tryDrawLcdScreen()) {
                    last_lcd_draw_ms = now;
                    lcd_force_refresh = false;
                }
            }
        } else {
            const unsigned long lcdPeriod =
                charge_full_hold ? LCD_CHARGE_REFRESH_MS : LCD_REFRESH_INTERVAL_MS;
            if (lcd_force_refresh || (now - last_lcd_draw_ms >= lcdPeriod)) {
                if (tryDrawLcdScreen()) {
                    last_lcd_draw_ms = now;
                    lcd_force_refresh = false;
                }
            }
        }
        const bool chargingNow =
            system_ON && (currentState == STATE_FORWARD || currentState == STATE_BOOST);
        const unsigned long dbgPeriod =
            chargingNow ? DEBUG_PRINT_CHARGE_MS : DEBUG_PRINT_INTERVAL_MS;

        // Build run label once for STAT + CSV.
        const char* run_label = "STANDBY";
        if (charge_full_hold) {
            run_label = "FULL";
        } else if (ovp_latched) {
            run_label = "OVP";
        } else if (!system_ON) {
            run_label = "STANDBY";
        } else if (currentState == STATE_BOOST) {
            if (boostNewMode == BOOST_NEW_SOFTSTART) run_label = "B_SOFT";
            else if (boostNewMode == BOOST_NEW_CC_MPPT) run_label = "B_CC";
            else if (boostNewMode == BOOST_NEW_CV) run_label = "B_CV";
            else run_label = "B_DONE";
        } else if (currentState == STATE_FORWARD) {
            if (forwardMode == FWD_SOFTSTART) run_label = "F_SOFT";
            else if (forwardMode == FWD_CC) run_label = "F_CC";
            else if (forwardMode == FWD_CV) run_label = "F_CV";
            else run_label = "F_DONE";
        } else if (system_ON) {
            run_label = "START";
        }
        const char* sel_label =
            (selectedChargeMode == USER_MODE_BOOST) ? "BOOST" : "FORW";
        const float vin_now =
            (currentState == STATE_BOOST ||
             (!system_ON && selectedChargeMode == USER_MODE_BOOST))
                ? v_solar
                : v_ac_in;

        // Excel TSV — real Vin/Vout/Iout/Duty + HH:MM:SS (only while active).
        const bool csvActive = system_ON || charge_full_hold || ovp_latched;
        if (ENABLE_DEBUG_CSV && csvActive &&
            (now - last_csv_time >= DEBUG_CSV_INTERVAL_MS)) {
            last_csv_time = now;
            const unsigned long sec = now / 1000UL;
            const unsigned int hh = (unsigned int)((sec / 3600UL) % 100UL);
            const unsigned int mm = (unsigned int)((sec / 60UL) % 60UL);
            const unsigned int ss = (unsigned int)(sec % 60UL);
            Serial.printf("CSV\t%02u:%02u:%02u\t%.1f\t%.2f\t%.2f\t%d\n",
                          hh, mm, ss, vin_now, v_bat_filt, i_bat_charge_filt,
                          active_duty_percent);
        }

        // [STAT]: periodic while charging/FULL/OVP; STANDBY only on mode change.
        if (ENABLE_DEBUG_STATUS) {
            const int sel_now = (int)selectedChargeMode;
            const bool mode_changed = (sel_now != last_stat_sel);
            const unsigned long sec = now / 1000UL;
            const unsigned int hh = (unsigned int)((sec / 3600UL) % 100UL);
            const unsigned int mm = (unsigned int)((sec / 60UL) % 60UL);
            const unsigned int ss = (unsigned int)(sec % 60UL);
            if (system_ON || charge_full_hold || ovp_latched) {
                if (now - last_debug_time >= dbgPeriod) {
                    last_debug_time = now;
                    Serial.printf("[STAT] %02u:%02u:%02u %s D=%d%% BAT %.2fV/%.2fV I=%.2fA IN=%.1fV%s\n",
                                  hh, mm, ss, run_label, active_duty_percent,
                                  v_bat, v_bat_filt, i_bat_charge_filt,
                                  vin_now,
                                  ac_is_collapsing ? " AC_SAG" : "");
                }
            } else if (mode_changed) {
                last_stat_sel = sel_now;
                Serial.printf("[STAT] %02u:%02u:%02u STANDBY %s BAT %.2fV PV %.1f AC %.1f\n",
                              hh, mm, ss, sel_label, v_bat_filt, v_solar, v_ac_in);
            }
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}
void TaskLCDLoop(void * pvParameters) {
    // Buttons only — LCD pixels are drawn from TaskSampleData to avoid I2C fights.
    bool last_start_state = HIGH, last_stop_state = HIGH;
    bool start_raw_last = HIGH, stop_raw_last = HIGH;
    unsigned long start_change_ms = 0, stop_change_ms = 0;
    unsigned long stop_held_since_ms = 0;
    bool stop_end_armed = false;
    unsigned long last_standby_reinit_ms = 0;
    const unsigned long DEBOUNCE_MS = 80;
    for (;;) {
        unsigned long now = millis();
        if (lcd_alert_until_ms != 0 && now >= lcd_alert_until_ms) {
            lcd_show_no_power = false;
            lcd_show_ovp_alert = false;
            lcd_alert_until_ms = 0;
            lcd_force_refresh = true;
        }
        bool start_raw = digitalRead(BUTTON_START_PIN);
        bool stop_raw = digitalRead(BUTTON_STOP_PIN);
        if (start_raw != start_raw_last) {
            start_raw_last = start_raw;
            start_change_ms = now;
        }
        if (stop_raw != stop_raw_last) {
            stop_raw_last = stop_raw;
            stop_change_ms = now;
        }
        bool current_start = last_start_state;
        bool current_stop = last_stop_state;
        if (now - start_change_ms >= DEBOUNCE_MS) current_start = start_raw;
        if (now - stop_change_ms >= DEBOUNCE_MS) current_stop = stop_raw;
        bool start_pressed = (current_start == LOW);
        bool stop_pressed = (current_stop == LOW);
        bool start_edge = (start_pressed && last_start_state == HIGH);
        bool stop_edge = (stop_pressed && last_stop_state == HIGH);
        if (system_ON || charge_full_hold) {
            if (stop_pressed) {
                if (stop_held_since_ms == 0) stop_held_since_ms = now;
                if (!stop_end_armed && (now - stop_held_since_ms >= STOP_HOLD_END_MS)) {
                    stop_end_armed = true;
                    system_ON = false;
                    charge_full_hold = false;
                    lcd_force_refresh = true;
                    Serial.printf("[INFO] STOP held %lums — charge ended (was %s).\n",
                                  (unsigned long)STOP_HOLD_END_MS,
                                  (currentState == STATE_FORWARD) ? "FORWARD" :
                                  (currentState == STATE_BOOST) ? "BOOST" : "ON");
                }
            } else {
                stop_held_since_ms = 0;
                stop_end_armed = false;
            }
        } else if (stop_edge) {
            stop_held_since_ms = 0;
            stop_end_armed = false;
            if (ovp_latched &&
                v_bat_filt <= HARD_OVP_RELEASE_VOLTAGE &&
                v_bat <= (HARD_OVP_RELEASE_VOLTAGE + 0.8f)) {
                ovp_latched = false;
                lcd_force_refresh = true;
                Serial.printf("[INFO] OVP latch cleared by STOP at %.2fV (release=%.2fV).\n",
                              max(v_bat, v_bat_filt), HARD_OVP_RELEASE_VOLTAGE);
            } else if (!ovp_latched) {
                selectedChargeMode = (selectedChargeMode == USER_MODE_BOOST)
                                         ? USER_MODE_FORWARD
                                         : USER_MODE_BOOST;
                lcd_force_refresh = true;
                Serial.printf("[INFO] Mode select -> %s (press START to begin)\n",
                              (selectedChargeMode == USER_MODE_BOOST) ? "BOOST PV" : "FORWARD AC");
            }
        } else if (!stop_pressed) {
            stop_held_since_ms = 0;
            stop_end_armed = false;
        }
        if (start_edge) {
            bool bat_ok = (v_bat_filt >= BAT_PRESENT_MIN_V) && (v_bat_filt <= BAT_START_MAX_V);
            bool selected_input_ok =
                (selectedChargeMode == USER_MODE_BOOST) ? (v_solar >= MIN_PV_VOLTAGE)
                                                        : (v_ac_in >= MIN_AC_VOLTAGE);
            if (ovp_latched) {
                system_ON = false;
                lcd_show_ovp_alert = true;
                lcd_show_no_power = false;
                lcd_alert_until_ms = now + 3000;
                lcd_force_refresh = true;
            } else if (sensor_init_ok && bat_ok && selected_input_ok) {
                system_ON = true;
                charge_full_hold = false;
                lcd_show_no_power = false;
                lcd_show_ovp_alert = false;
                lcd_charge_blanked = false;   // force one blank on next draw
                last_lcd_soft_resync_ms = 0;
                lcd_force_refresh = true;
            } else {
                system_ON = false;
                lcd_show_no_power = true;
                lcd_show_ovp_alert = false;
                lcd_alert_until_ms = now + 3000;
                lcd_force_refresh = true;
            }
        }
        last_start_state = current_start;
        last_stop_state = current_stop;
        if (system_ON != last_system_state) {
            if (!system_ON) total_Wh = 0;
            last_system_state = system_ON;
            lcd_force_refresh = true;
        }
        // Never Wire.end / lcd.init while system_ON — even at duty=0 SoftStart entry
        // (old charge_active required raw_duty>0 and allowed reinit during relay delay).
        if (!system_ON && !charge_full_hold &&
            lcd_force_refresh &&
            (now - last_lcd_draw_ms > 2000) &&
            (now - last_standby_reinit_ms > 5000)) {
            last_standby_reinit_ms = now;
            if (xSemaphoreTake(i2c_Mutex, pdMS_TO_TICKS(200))) {
                reinitI2CBusAndLCD();
                drawLcdScreen();
                xSemaphoreGive(i2c_Mutex);
                last_lcd_draw_ms = now;
                lcd_force_refresh = false;
                Serial.println("[WARN] LCD I2C recovered (standby reinit).");
            }
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}
