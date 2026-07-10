#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <stdarg.h>

// =========================================================================
// ตั้งค่า Hardware & ขาต่อใช้งาน PWM แยก 2 วงจร
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
// ตั้งค่าเป้าหมาย และ เกณฑ์ความปลอดภัยขั้นต่ำ (Safety Thresholds)
// =========================================================================
const float TARGET_CV_VOLTAGE = 58.0;           // ผู้ใช้ต้องการ CV คงที่ที่ 58V
const float TARGET_CC_CURRENT = 3.0;

const float MIN_PV_VOLTAGE = 42.0;         // เริ่มทำงานเมื่อแผงถึง 42V
const float UNDER_PV_VOLTAGE_CRIT = 39.0;  // ต่ำกว่า 39V เกิน 2 วินาที สั่งตัด
const float MIN_AC_VOLTAGE = 140.0;

const int MAX_DUTY_FORWARD = 490;
const int MAX_DUTY_BOOST   = 760;

// ถ้าอ่าน ADC ไม่สำเร็จเกินช่วงนี้ ให้เข้าสู่โหมดปลอดภัย
const unsigned long ADC_STALE_TIMEOUT_MS = 700;
const unsigned long SENSOR_ERROR_LOG_MS = 2000;
const float CV_DEADBAND_V = 0.10;
const float FULL_DETECT_VOLTAGE = 57.8;
const float FULL_END_CURRENT = 0.45;              // 15% ของกระแส CC (3A)
const unsigned long FULL_CONFIRM_MS = 300000;     // เงื่อนไข FULL ต้องต่อเนื่อง 5 นาที
const float HIGH_VOLTAGE_STOP_VOLTAGE = 58.4;     // pre-OVP stop ก่อน hard OVP
const unsigned long HIGH_VOLTAGE_STOP_CONFIRM_MS = 1500;
const float RESTART_CHARGE_VOLTAGE = 55.8;        // ฮิสเทอรีซิสหลังเต็มสำหรับ CV 58V

// =========================================================================
// ตัวแปรและค่าคงที่สำหรับ PID Control (โหมด BOOST คุมแรงดันแผงโซล่าเซลล์)
// =========================================================================
const float Kp = 1.0;
const float Ki = 0.05;
const float Kd = 0.02;

float pid_error = 0.0;
float pid_last_error = 0.0;
float pid_integral = 0.0;
float pid_derivative = 0.0;
float v_solar_target = 42.0;

// =========================================================================
// ตัวแปรและค่าคงที่สำหรับ PID Control (โหมด CC/CV ในการชาร์จแบตเตอรี่ลิเธียม)
// =========================================================================
const float Kp_cc = 0.15;  // แบตเตอรี่ลิเธียมความต้านทานต่ำมาก Gain ต้องน้อยเพื่อกันกระแสกระชาก
const float Ki_cc = 0.01;
const float Kd_cc = 0.005;

const float Kp_cv = 0.5;   // ลด Gain เพื่อให้ช่วงใกล้ CV แกว่งน้อยลง
const float Ki_cv = 0.015;
const float Kd_cv = 0.005;

float pid_error_cc = 0.0, pid_last_error_cc = 0.0, pid_integral_cc = 0.0;
float pid_error_cv = 0.0, pid_last_error_cv = 0.0, pid_integral_cv = 0.0;

// =========================================================================
// 🔍 ส่วนปรับแต่งการเพี้ยนและขจัดสัญญาณรบกวน (Calibration Zone)
// =========================================================================
const float OFFSET_V_SOLAR = 0.0;
const float CAL_SCALE_V_SOLAR = 41.9;

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
const float BOOST_VOLTAGE_FLOOR = 42.0;
const float BOOST_BAT_VOLTAGE_LIMIT = 58.2;
const float BOOST_CV_TARGET_VOLTAGE = 58.0;
const float BOOST_START_I_TARGET = 1.0;
const float BOOST_START_EFF_EST = 0.90;
const int BOOST_START_DUTY_MIN_RAW = 60;
const int BOOST_START_DUTY_MAX_RAW = 320;
const int BOOST_START_DUTY_NEAR_FULL_CAP_RAW = 130;
const float BOOST_CV_ENTRY_VOLTAGE = 56.2;
const float BOOST_CV_EXIT_VOLTAGE = 55.8;
const unsigned long BOOST_RAMP_DURATION_MS = 2500;
const float BOOST_RAMP_STEP = 1.2;
const float BOOST_CV_KP = 1.1;
const float BOOST_CV_KI = 0.03;
const float BOOST_CV_KD = 0.02;
const float BOOST_OC_SOFT_MARGIN_A = 0.05;
const float BOOST_OC_HARD_MARGIN_A = 0.80;
const float BOOST_OC_SOFT_DOWN_BASE = 1.0;
const float BOOST_OC_SOFT_DOWN_GAIN = 1.6;
const float BOOST_OC_HARD_DOWN_BASE = 3.0;
const float BOOST_OC_HARD_DOWN_GAIN = 2.0;
const float BOOST_MIN_DUTY_WHILE_LIMITING = 20.0;
const float BOOST_CEILING_RELEASE_STEP = 1.6;
const float BOOST_VBAT_HARD_OVERSHOOT_MARGIN = 0.20;
const int BOOST_CEILING_FLOOR_RAW = 24;
const float BOOST_CURRENT_CAP_V1 = 55.8;
const float BOOST_CURRENT_CAP_V2 = 56.2;
const float BOOST_CURRENT_CAP_V3 = 56.6;
const float BOOST_CURRENT_CAP_A1 = 1.2;
const float BOOST_CURRENT_CAP_A2 = 1.0;
const float BOOST_CURRENT_CAP_A3 = 0.8;
const float BOOST_NEAR_FULL_V0 = 55.0;
const float BOOST_NEAR_FULL_V1 = 55.8;
const float BOOST_NEAR_FULL_V2 = 56.2;
const float BOOST_NEAR_FULL_V3 = 56.6;
const int BOOST_DUTY_CAP_V0_RAW = 140;
const int BOOST_DUTY_CAP_V1_RAW = 120;
const int BOOST_DUTY_CAP_V2_RAW = 100;
const int BOOST_DUTY_CAP_V3_RAW = 85;
const float BOOST_FORCE_CV_VOLTAGE = 55.2;
const float BOOST_FORCE_CV_RELEASE = 54.8;
const float BOOST_DIRECT_CV_START_VOLTAGE = 55.2;
const int BOOST_START_DUTY_SEED_RAW = 20;
const int BOOST_START_DUTY_SEED_NEAR_FULL_RAW = 0;
const int BOOST_START_TARGET_NEAR_FULL_MAX_RAW = 80;
const unsigned long BOOST_START_SETTLE_MS = 350;
const float BOOST_VBAT_SPIKE_PRECUT_DELTA_V = 1.2;
const float BOOST_VBAT_SPIKE_PRECUT_RAW_ABOVE_FILT_V = 1.5;
const float BOOST_CV_UP_STEP_V1 = 0.20;
const float BOOST_CV_UP_STEP_V2 = 0.05;
const float BOOST_CV_UP_STEP_V3 = 0.00;
const float BOOST_MPPT_MIN_CURRENT_FOR_UPDATE = 0.20;
const unsigned long BOOST_MPPT_LOW_CURRENT_FALLBACK_MS = 2500;
const float BOOST_RECOVERY_TARGET_V = 42.3;
const float BOOST_SAFE_V_HEADROOM = 0.4;
const float BOOST_SAFE_I_HEADROOM = 0.15;
const float BOOST_SAFE_BAT_HEADROOM = 0.3;
const float HARD_OVP_TRIP_VOLTAGE = 58.8;
const float HARD_OVP_RELEASE_VOLTAGE = 57.8;
const bool ENABLE_DEBUG_VERBOSE = true;           // ดีบักเดิมหลายบรรทัด
const unsigned long DEBUG_PRINT_INTERVAL_MS = 500;
const unsigned long LCD_REFRESH_INTERVAL_MS = 180;
const uint32_t I2C_CLOCK_HZ = 100000;

// =========================================================================
// ตัวแปรระบบ
// =========================================================================
volatile float v_solar = 0, v_ac_in = 0, v_bat = 0;
volatile float i_solar = 0, i_ac_in = 0, i_bat = 0;
volatile float v_bat_filt = 0, i_bat_filt = 0;
volatile float i_solar_mag = 0;
volatile float i_bat_charge_filt = 0;  // กระแสชาร์จใช้ค่าบวกเสมอเพื่อกันทิศเซนเซอร์กลับด้าน
volatile float i_bat_charge_abs = 0;   // กระแสชาร์จแบบไม่ฟิลเตอร์สำหรับกัน overshoot เร็ว
volatile bool ovp_latched = false;
volatile float ovp_trip_voltage = 0.0;
volatile bool system_ON = false;
volatile bool charge_full_hold = false;
volatile int active_duty_percent = 0;
volatile int boost_start_duty_raw = 20;
volatile float boost_duty_ceiling = MAX_DUTY_BOOST;
int raw_duty = 0;
float duty_accumulator = 0.0;          // เก็บ duty แบบทศนิยม เพื่อลด dead-zone จาก round()
float boost_dither_phase = 0.0;        // สะสมเศษ duty เพื่อทำ sub-LSB averaging
float forward_dither_phase = 0.0;
float total_Wh = 0;
unsigned long last_millis = 0;

volatile bool sensor_init_ok = false;
volatile unsigned long last_adc_sample_ms = 0;

enum SystemState { STATE_BOOST, STATE_FORWARD, STATE_OFF };
volatile SystemState currentState = STATE_OFF;
enum BoostControlMode { BOOST_RAMP, BOOST_MPPT, BOOST_CV_HOLD };
volatile BoostControlMode boostMode = BOOST_RAMP;
bool last_system_state = false;

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
int calcInitialBoostDutyRaw(float v_pv_now, float v_bat_now);
int calcBoostDutyCapRaw(float v_bat_now, int default_cap);
int quantizeDutyWithDither(float duty_cmd, float *phase, int max_duty);
void lcdPrintLineRaw(uint8_t row, const char *text);
void lcdPrintLineFmt(uint8_t row, const char *fmt, ...);
void reinitI2CBusAndLCD();

int calcInitialBoostDutyRaw(float v_pv_now, float v_bat_now) {
    float vout = (v_bat_now > (BOOST_VOLTAGE_FLOOR + 0.8)) ? v_bat_now : (BOOST_VOLTAGE_FLOOR + 0.8);
    float vin_ref = (v_pv_now > BOOST_VOLTAGE_FLOOR) ? v_pv_now : BOOST_VOLTAGE_FLOOR;

    float d_est = 1.0 - (vin_ref / vout);
    if (d_est < 0.0) d_est = 0.0;

    // ประมาณกำลังเริ่มต้นที่ต้องการจาก I_start แล้วลด duty ลงถ้ากระแสแผงที่ต้องการสูงเกินกรอบ CC
    float p_out_start = vout * BOOST_START_I_TARGET;
    float p_in_start = p_out_start / BOOST_START_EFF_EST;
    float i_pv_req = p_in_start / vin_ref;
    if (i_pv_req > TARGET_CC_CURRENT) {
        float scale = TARGET_CC_CURRENT / i_pv_req;
        d_est *= scale;
    }

    int raw = (int)roundf(d_est * 1023.0);
    int max_start_duty = BOOST_START_DUTY_MAX_RAW;
    if (v_bat_now >= BOOST_CV_ENTRY_VOLTAGE) {
        max_start_duty = BOOST_START_DUTY_NEAR_FULL_CAP_RAW;
    } else if (v_bat_now >= (BOOST_CV_ENTRY_VOLTAGE - 0.6)) {
        max_start_duty = min(BOOST_START_DUTY_MAX_RAW, BOOST_START_DUTY_NEAR_FULL_CAP_RAW + 20);
    }
    return constrain(raw, BOOST_START_DUTY_MIN_RAW, max_start_duty);
}

int calcBoostDutyCapRaw(float v_bat_now, int default_cap) {
    int cap = default_cap;
    if (v_bat_now >= BOOST_NEAR_FULL_V3) {
        cap = BOOST_DUTY_CAP_V3_RAW;
    } else if (v_bat_now >= BOOST_NEAR_FULL_V2) {
        cap = BOOST_DUTY_CAP_V2_RAW;
    } else if (v_bat_now >= BOOST_NEAR_FULL_V1) {
        cap = BOOST_DUTY_CAP_V1_RAW;
    } else if (v_bat_now >= BOOST_NEAR_FULL_V0) {
        cap = BOOST_DUTY_CAP_V0_RAW;
    }
    if (cap > default_cap) cap = default_cap;
    if (cap < 0) cap = 0;
    return cap;
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
    // Fast read path: one conversion is usually enough at high data-rate.
    // If a corrupted negative code appears on single-ended channel, retry once.
    int16_t sample = adc.readADC_SingleEnded(channel);
    if (sample < 0) {
        sample = adc.readADC_SingleEnded(channel);
        if (sample < 0) sample = 0;
    }
    return sample;
}

static inline void disablePowerStage() {
    currentState = STATE_OFF;
    boostMode = BOOST_RAMP;
    boost_duty_ceiling = MAX_DUTY_BOOST;
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
    // Re-init I2C/LCD together to recover from occasional LCD bus lockups.
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

    // Calibrate zero-current baseline while power stage is disabled.
    disablePowerStage();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    for (int i = 0; i < CAL_SAMPLES; i++) {
        // Keep calibration robust against mux-settling artifacts.
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

// =========================================================================
// SETUP & MAIN LOOP
// =========================================================================
void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22);
    Wire.setClock(I2C_CLOCK_HZ);
    Wire.setTimeOut(25);

    bool volt_ok = ads_volt.begin(0x48);
    bool curr_ok = ads_curr.begin(0x49);
    sensor_init_ok = (volt_ok && curr_ok);

    if (sensor_init_ok) {
        // Reduce ADC conversion latency to avoid blocking LCD task on I2C mutex.
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

// =========================================================================
// TASK 1: การคำนวณข้อมูลสัญญานทางไฟฟ้าและลูป PID ควบคุมแปลงผันกำลังไฟฟ้า
// =========================================================================
void TaskSampleData(void * pvParameters) {
    float p_solar_old = 0.0;
    float v_solar_old = 0.0;
    int mppt_direction = 1;
    const float MPPT_V_STEP = 0.15;
    unsigned long last_mppt_time = 0;

    unsigned long pv_collapse_start_time = 0;
    bool pv_is_collapsing = false;
    unsigned long boost_mode_enter_ms = 0;
    unsigned long mppt_low_current_start_ms = 0;
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
            // Voltage channels are more sensitive to mux-settling; discard-first keeps readings stable.
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

            v_solar = (mv_pure_v0 / 1000.0) * CAL_SCALE_V_SOLAR;
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

            // ใช้ moving average ช่วยลดการสั่นของ CV รอบแรงดันใกล้เต็ม
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

            // Hard OVP: treat high battery voltage as real event and cut power immediately.
            if (!ovp_latched &&
                (v_bat >= HARD_OVP_TRIP_VOLTAGE || v_bat_filt >= HARD_OVP_TRIP_VOLTAGE)) {
                ovp_latched = true;
                ovp_trip_voltage = max(v_bat, v_bat_filt);
                forceSafeShutdown();
                Serial.printf("[CRITICAL] HARD OVP TRIP at %.2fV (trip=%.2fV). Output disabled.\n",
                              ovp_trip_voltage, HARD_OVP_TRIP_VOLTAGE);
            }

            // Fast runaway detection: แรงดันแบตพุ่งเร็วผิดปกติพร้อมกระแสหาย ให้ตัดทันที
            if (!ovp_latched &&
                system_ON &&
                currentState == STATE_BOOST &&
                raw_duty > 0 &&
                v_bat_filt >= BOOST_CV_ENTRY_VOLTAGE &&
                i_bat_charge_filt < MIN_CURRENT_FOR_ACTIVE_CHARGE &&
                v_bat > (v_bat_filt + 3.0f)) {
                ovp_latched = true;
                ovp_trip_voltage = v_bat;
                forceSafeShutdown();
                Serial.printf("[CRITICAL] RUNAWAY-CUT at %.2fV (filt=%.2fV, duty=%d).\n",
                              v_bat, v_bat_filt, raw_duty);
            }

            // Fast spike pre-cut: ถ้าแรงดันพุ่งเร็วผิดปกติในโซนใกล้เต็ม ให้ตัดก่อนรอ OVP
            if (!ovp_latched &&
                system_ON &&
                currentState == STATE_BOOST &&
                raw_duty > 0 &&
                v_bat_filt >= (BOOST_CV_ENTRY_VOLTAGE - 0.2f) &&
                (vbat_step > BOOST_VBAT_SPIKE_PRECUT_DELTA_V ||
                 (vbat_step > 0.7f && vbat_filt_step > 0.25f)) &&
                v_bat > (v_bat_filt + BOOST_VBAT_SPIKE_PRECUT_RAW_ABOVE_FILT_V)) {
                ovp_latched = true;
                ovp_trip_voltage = v_bat;
                forceSafeShutdown();
                Serial.printf("[CRITICAL] SPIKE-PRECUT at %.2fV (step=%.2fV, filt=%.2fV, duty=%d).\n",
                              v_bat, vbat_step, v_bat_filt, raw_duty);
            }

            if (ovp_latched &&
                v_bat <= HARD_OVP_RELEASE_VOLTAGE &&
                v_bat_filt <= HARD_OVP_RELEASE_VOLTAGE) {
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

        // stale-data protection: อย่าคุมกำลังด้วยค่าที่อ่านค้าง
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
                    boostMode = BOOST_RAMP;
                    boost_mode_enter_ms = now;

                    v_solar_old = v_solar;
                    p_solar_old = v_solar * i_solar_mag;
                    v_solar_target = v_solar - 1.0;
                    if (v_solar_target < BOOST_VOLTAGE_FLOOR) v_solar_target = BOOST_VOLTAGE_FLOOR;
                    pid_integral = 0; pid_last_error = 0;
                    pid_integral_cc = 0; pid_last_error_cc = 0;
                    pid_integral_cv = 0; pid_last_error_cv = 0;
                    float vbat_for_start = (v_bat_filt > NOISE_V_THRESHOLD) ? v_bat_filt : v_bat;
                    int boost_start_target_raw = calcInitialBoostDutyRaw(v_solar, vbat_for_start); // target duty สำหรับ RAMP
                    bool start_in_cv_hold = (vbat_for_start >= BOOST_DIRECT_CV_START_VOLTAGE);
                    if (start_in_cv_hold) {
                        boostMode = BOOST_CV_HOLD;
                        if (boost_start_target_raw > BOOST_START_TARGET_NEAR_FULL_MAX_RAW) {
                            boost_start_target_raw = BOOST_START_TARGET_NEAR_FULL_MAX_RAW;
                        }
                        pid_integral_cv = 0;
                        pid_last_error_cv = 0;
                    }
                    boost_start_duty_raw = boost_start_target_raw;
                    int start_seed = BOOST_START_DUTY_SEED_RAW;
                    if (vbat_for_start >= BOOST_NEAR_FULL_V0) {
                        start_seed = BOOST_START_DUTY_SEED_NEAR_FULL_RAW;
                    }
                    raw_duty = start_seed;
                    if (raw_duty > boost_start_target_raw) {
                        raw_duty = boost_start_target_raw;
                    }
                    duty_accumulator = (float)raw_duty;
                    int start_duty_cap = calcBoostDutyCapRaw(vbat_for_start, MAX_DUTY_BOOST);
                    boost_duty_ceiling = (float)start_duty_cap;
                    if (duty_accumulator > boost_duty_ceiling) {
                        duty_accumulator = boost_duty_ceiling;
                        raw_duty = (int)roundf(duty_accumulator);
                    }
                    pv_is_collapsing = false;
                    Serial.printf("[INFO] BOOST start seed=%d target=%d mode=%s.\n",
                                  raw_duty, boost_start_duty_raw,
                                  (boostMode == BOOST_CV_HOLD ? "CV_HOLD" : "RAMP"));
                }
                else if (v_ac_in >= MIN_AC_VOLTAGE) {
                    ledcWrite(PWM_FORWARD_PIN, 0); ledcWrite(PWM_BOOST_PIN, 0);
                    digitalWrite(RELAY_PV_PIN, LOW);
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    digitalWrite(RELAY_AC_PIN, HIGH);
                    currentState = STATE_FORWARD;
                    boostMode = BOOST_RAMP;
                    boost_mode_enter_ms = 0;
                    pid_integral_cc = 0; pid_last_error_cc = 0;
                    pid_integral_cv = 0; pid_last_error_cv = 0;
                    raw_duty = 10;
                    duty_accumulator = 10.0;
                    boost_start_duty_raw = 20;
                    boost_duty_ceiling = MAX_DUTY_BOOST;
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
                boostMode = BOOST_RAMP;
                boost_mode_enter_ms = 0;
            }
        }

        if (!system_ON) {
            digitalWrite(RELAY_PV_PIN, LOW);
            digitalWrite(RELAY_AC_PIN, LOW);
            currentState = STATE_OFF;
            boostMode = BOOST_RAMP;
            boost_mode_enter_ms = 0;
            boost_start_duty_raw = 20;
            boost_duty_ceiling = MAX_DUTY_BOOST;
            raw_duty = 0;
            duty_accumulator = 0.0;
            boost_dither_phase = 0.0;
            forward_dither_phase = 0.0;
            pv_is_collapsing = false;
        }

        if (system_ON && currentState != STATE_OFF) {
            int allowed_max_duty = (currentState == STATE_FORWARD) ? MAX_DUTY_FORWARD : MAX_DUTY_BOOST;
            float duty_before_control = duty_accumulator;

            if (currentState == STATE_FORWARD) {
                // =================================================================
                // 🔋 โหมด AC: ระบบควบคุม DUAL-LOOP PID (CC/CV CHARGING CONTROL)
                // =================================================================

                // 1. ลูปควบคุมกระแสคงที่ (Constant Current Loop - CC) เป้าหมาย 3.0A
                // ใช้ค่าบวกของกระแสชาร์จ เพื่อให้ไม่ขึ้นกับทิศเซนเซอร์ Hall
                pid_error_cc = TARGET_CC_CURRENT - i_bat_charge_filt;
                pid_integral_cc += pid_error_cc;
                pid_integral_cc = constrain(pid_integral_cc, -100, 100);
                float delta_error_cc = pid_error_cc - pid_last_error_cc;
                float pid_out_cc = (Kp_cc * pid_error_cc) + (Ki_cc * pid_integral_cc) + (Kd_cc * delta_error_cc);
                pid_last_error_cc = pid_error_cc;

                // 2. ลูปควบคุมแรงดันคงที่ (Constant Voltage Loop - CV) เป้าหมาย 58.4V
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

                // เลือกค่าเอาต์พุตจากวงจร PID ที่ปลอดภัยและมีค่าต่ำที่สุด ป้องกัน Overshoot
                float final_battery_pid = min(pid_out_cc, pid_out_cv);

                // จำกัดความเร็วการเร่ง/ลด ในหนึ่งรอบลูป (Slew-Rate Limit ฝั่งแบตเตอรี่)
                if (final_battery_pid > 1.5) final_battery_pid = 1.5;
                if (final_battery_pid < -4.0) final_battery_pid = -4.0;

                duty_accumulator += final_battery_pid;
            }
            else if (currentState == STATE_BOOST) {
                // =================================================================
                // ☀️ โหมด PV: แยก 3 ช่วง RAMP / MPPT / CV_HOLD
                // =================================================================

                if (v_solar == 0.0) {
                    duty_accumulator = 0.0;
                    pid_integral = 0;
                    pid_integral_cv = 0;
                }
                else {
                    if ((boost_mode_enter_ms > 0) && (now - boost_mode_enter_ms < BOOST_START_SETTLE_MS)) {
                        duty_accumulator = 0.0;
                        pid_integral = 0;
                        pid_integral_cv = 0;
                    } else
                    if (boostMode == BOOST_RAMP) {
                        mppt_low_current_start_ms = 0;
                        if (v_bat_filt >= BOOST_FORCE_CV_VOLTAGE || v_bat_filt >= BOOST_CV_ENTRY_VOLTAGE) {
                            boostMode = BOOST_CV_HOLD;
                            pid_integral_cv = 0;
                            pid_last_error_cv = 0;
                        } else {
                            if (duty_accumulator < (float)boost_start_duty_raw) {
                                duty_accumulator += BOOST_RAMP_STEP;
                            }
                            bool ramp_ready_for_mppt =
                                (i_solar_mag >= MIN_CURRENT_FOR_ACTIVE_CHARGE) ||
                                (i_bat_charge_filt >= MIN_CURRENT_FOR_ACTIVE_CHARGE) ||
                                ((boost_mode_enter_ms > 0) &&
                                 (now - boost_mode_enter_ms >= BOOST_RAMP_DURATION_MS));
                            if (ramp_ready_for_mppt) {
                                boostMode = BOOST_MPPT;
                                pid_integral = 0;
                                pid_last_error = 0;
                            }
                        }
                    }
                    else if (boostMode == BOOST_MPPT) {
                        if (v_bat_filt >= BOOST_FORCE_CV_VOLTAGE) {
                            boostMode = BOOST_CV_HOLD;
                            pid_integral_cv = 0;
                            pid_last_error_cv = 0;
                            mppt_low_current_start_ms = 0;
                        } else {
                        bool mppt_has_useful_current =
                            (i_solar_mag >= BOOST_MPPT_MIN_CURRENT_FOR_UPDATE) ||
                            (i_bat_charge_filt >= BOOST_MPPT_MIN_CURRENT_FOR_UPDATE);

                        if ((now - last_mppt_time >= 100) && mppt_has_useful_current) {
                            last_mppt_time = now;
                            float p_solar = v_solar * i_solar_mag;
                            float delta_p = p_solar - p_solar_old;
                            float delta_v = v_solar - v_solar_old;

                            if (delta_p != 0) {
                                if (delta_p > 0) {
                                    if (delta_v > 0) mppt_direction = 1;
                                    else             mppt_direction = -1;
                                } else {
                                    if (delta_v > 0) mppt_direction = -1;
                                    else             mppt_direction = 1;
                                }
                            }

                            v_solar_target += (mppt_direction * MPPT_V_STEP);
                            if (v_solar_target < BOOST_VOLTAGE_FLOOR) v_solar_target = BOOST_VOLTAGE_FLOOR;
                            if (v_solar_target > 48.0) v_solar_target = 48.0;

                            p_solar_old = p_solar; v_solar_old = v_solar;
                        }

                        if (!mppt_has_useful_current) {
                            if (mppt_low_current_start_ms == 0) mppt_low_current_start_ms = now;
                            if (now - mppt_low_current_start_ms >= BOOST_MPPT_LOW_CURRENT_FALLBACK_MS) {
                                boostMode = BOOST_RAMP;
                                v_solar_target = BOOST_RECOVERY_TARGET_V;
                                pid_integral = 0;
                                pid_last_error = 0;
                                if (duty_accumulator < BOOST_CEILING_FLOOR_RAW) {
                                    duty_accumulator = BOOST_CEILING_FLOOR_RAW;
                                }
                                mppt_low_current_start_ms = 0;
                                Serial.println("[INFO] MPPT low-current fallback -> BOOST_RAMP.");
                            }
                        } else {
                            mppt_low_current_start_ms = 0;
                        }

                        pid_error = v_solar - v_solar_target;
                        if (v_solar < BOOST_VOLTAGE_FLOOR) {
                            pid_integral = 0;
                        } else {
                            pid_integral += pid_error;
                            pid_integral = constrain(pid_integral, -50, 50);
                        }
                        pid_derivative = pid_error - pid_last_error;

                        float pid_output = (Kp * pid_error) + (Ki * pid_integral) + (Kd * pid_derivative);

                        if (duty_accumulator < 30.0 && v_solar > BOOST_VOLTAGE_FLOOR) {
                            pid_output = 2.0;
                        } else if (pid_output > 1.5) {
                            pid_output = 1.5;
                        }

                        if (v_solar <= (BOOST_VOLTAGE_FLOOR - 0.5)) {
                            if (pid_output > 0) pid_output = 0;
                            if (v_solar <= (BOOST_VOLTAGE_FLOOR - 1.0)) pid_output = -5.0;
                            else if (pid_output < -2.0) pid_output = -2.0;
                        }

                        duty_accumulator += pid_output;
                        pid_last_error = pid_error;

                        if (v_bat_filt >= BOOST_CV_ENTRY_VOLTAGE) {
                            boostMode = BOOST_CV_HOLD;
                            pid_integral_cv = 0;
                            pid_last_error_cv = 0;
                            mppt_low_current_start_ms = 0;
                        }
                        }
                    }
                    else { // BOOST_CV_HOLD
                        mppt_low_current_start_ms = 0;
                        pid_error_cv = BOOST_CV_TARGET_VOLTAGE - v_bat_filt;
                        if (fabs(pid_error_cv) <= CV_DEADBAND_V) {
                            pid_error_cv = 0.0;
                            pid_integral_cv *= 0.90;
                        }
                        pid_integral_cv += pid_error_cv;
                        pid_integral_cv = constrain(pid_integral_cv, -120, 120);
                        float delta_error_cv = pid_error_cv - pid_last_error_cv;
                        float cv_hold_output = (BOOST_CV_KP * pid_error_cv) +
                                               (BOOST_CV_KI * pid_integral_cv) +
                                               (BOOST_CV_KD * delta_error_cv);
                        pid_last_error_cv = pid_error_cv;

                        float cv_up_limit = 1.0;
                        if (v_bat_filt >= BOOST_NEAR_FULL_V3) cv_up_limit = BOOST_CV_UP_STEP_V3;
                        else if (v_bat_filt >= BOOST_NEAR_FULL_V2) cv_up_limit = BOOST_CV_UP_STEP_V2;
                        else if (v_bat_filt >= BOOST_NEAR_FULL_V1) cv_up_limit = BOOST_CV_UP_STEP_V1;
                        if (cv_hold_output > cv_up_limit) cv_hold_output = cv_up_limit;

                        float cv_down_limit = (v_bat_filt >= BOOST_NEAR_FULL_V2) ? -4.2 : -3.0;
                        if (cv_hold_output < cv_down_limit) cv_hold_output = cv_down_limit;

                        duty_accumulator += cv_hold_output;

                        if (v_bat_filt <= BOOST_CV_EXIT_VOLTAGE &&
                            v_bat_filt <= BOOST_FORCE_CV_RELEASE &&
                            i_bat_charge_filt < (TARGET_CC_CURRENT - 0.3)) {
                            boostMode = BOOST_MPPT;
                            pid_integral = 0;
                            pid_last_error = 0;
                        }
                    }
                }
            }

            // Guardrail ตอนเริ่มและขณะบูสต์: รักษา I<=3A, Vpv>=42V และกันแรงดันแบตพุ่ง
            if (currentState == STATE_BOOST) {
                float boost_current_limit = TARGET_CC_CURRENT;
                if (v_bat_filt >= BOOST_CURRENT_CAP_V3) {
                    boost_current_limit = BOOST_CURRENT_CAP_A3;
                } else if (v_bat_filt >= BOOST_CURRENT_CAP_V2) {
                    boost_current_limit = BOOST_CURRENT_CAP_A2;
                } else if (v_bat_filt >= BOOST_CURRENT_CAP_V1) {
                    boost_current_limit = BOOST_CURRENT_CAP_A1;
                }
                float i_charge_for_guardrail = max(i_bat_charge_filt, i_bat_charge_abs);

                float vbat_for_cap = max(v_bat_filt, v_bat);
                int vbat_duty_cap = calcBoostDutyCapRaw(vbat_for_cap, allowed_max_duty);

                bool near_v_limit = (v_solar <= (BOOST_VOLTAGE_FLOOR + BOOST_SAFE_V_HEADROOM));
                bool near_i_limit = (i_charge_for_guardrail >= (boost_current_limit - BOOST_SAFE_I_HEADROOM));
                bool near_bat_limit = (v_bat_filt >= (BOOST_CV_TARGET_VOLTAGE - BOOST_SAFE_BAT_HEADROOM));
                bool hold_duty_ceiling = near_v_limit || near_i_limit || near_bat_limit;
                bool hard_limit_active =
                    (i_charge_for_guardrail > (boost_current_limit + BOOST_OC_HARD_MARGIN_A)) ||
                    (v_bat_filt > (BOOST_BAT_VOLTAGE_LIMIT + BOOST_VBAT_HARD_OVERSHOOT_MARGIN));

                // เข้าใกล้ลิมิต: ล็อกเพดาน duty ไม่ให้เพิ่มต่อ; ออกจากลิมิตแล้วค่อยคืนเพดานช้าๆ
                if (hold_duty_ceiling) {
                    float ceiling_lock_target = duty_before_control;
                    if (!hard_limit_active && ceiling_lock_target < BOOST_CEILING_FLOOR_RAW) {
                        ceiling_lock_target = BOOST_CEILING_FLOOR_RAW;
                    }
                    if (ceiling_lock_target < boost_duty_ceiling) {
                        boost_duty_ceiling = ceiling_lock_target;
                    }
                } else {
                    boost_duty_ceiling += BOOST_CEILING_RELEASE_STEP;
                    if (boost_duty_ceiling > (float)allowed_max_duty) {
                        boost_duty_ceiling = (float)allowed_max_duty;
                    }
                }
                if (boost_duty_ceiling > (float)vbat_duty_cap) {
                    boost_duty_ceiling = (float)vbat_duty_cap;
                }

                if (v_solar < BOOST_VOLTAGE_FLOOR) {
                    float v_under = BOOST_VOLTAGE_FLOOR - v_solar;
                    duty_accumulator -= (3.0 + (v_under * 3.5));
                }
                if (i_charge_for_guardrail > (boost_current_limit + BOOST_OC_SOFT_MARGIN_A)) {
                    float over_current = i_charge_for_guardrail - boost_current_limit;
                    float duty_down = BOOST_OC_SOFT_DOWN_BASE + (over_current * BOOST_OC_SOFT_DOWN_GAIN);
                    if (over_current > BOOST_OC_HARD_MARGIN_A) {
                        duty_down = BOOST_OC_HARD_DOWN_BASE + (over_current * BOOST_OC_HARD_DOWN_GAIN);
                    }
                    if (v_bat_filt >= BOOST_NEAR_FULL_V2) {
                        duty_down *= 1.5;
                    }
                    duty_accumulator -= duty_down;

                    // กัน duty ตกเป็นศูนย์ทันทีจากโอเวอร์คเรนต์ชั่วคราวระหว่างที่แรงดันยังอยู่ในโซนใช้งาน
                    bool can_hold_min_duty = (over_current <= BOOST_OC_HARD_MARGIN_A) &&
                                             (v_solar > (BOOST_VOLTAGE_FLOOR + 0.8)) &&
                                             (v_bat_filt < (BOOST_CV_TARGET_VOLTAGE - 0.8));
                    if (can_hold_min_duty && duty_accumulator < BOOST_MIN_DUTY_WHILE_LIMITING) {
                        duty_accumulator = BOOST_MIN_DUTY_WHILE_LIMITING;
                    }

                    pid_integral = 0;
                    pid_integral_cv *= 0.8;
                }
                if (v_bat_filt > BOOST_CV_TARGET_VOLTAGE) {
                    float over_cv_target = v_bat_filt - BOOST_CV_TARGET_VOLTAGE;
                    duty_accumulator -= (2.2 + (over_cv_target * 7.5));
                    pid_integral = 0;
                    pid_integral_cv *= 0.75;
                }
                if (v_bat_filt > (BOOST_CV_ENTRY_VOLTAGE - 0.6) &&
                    i_bat_charge_filt < MIN_CURRENT_FOR_ACTIVE_CHARGE &&
                    duty_accumulator > (float)BOOST_START_DUTY_NEAR_FULL_CAP_RAW) {
                    duty_accumulator = (float)BOOST_START_DUTY_NEAR_FULL_CAP_RAW;
                }
                // หากอยู่แรงดันสูงแต่กระแสชาร์จหายไป ให้ดึง duty ลงเร็ว
                if (v_bat_filt > (BOOST_CV_ENTRY_VOLTAGE + 0.4) &&
                    i_bat_charge_filt < MIN_CURRENT_FOR_ACTIVE_CHARGE &&
                    duty_accumulator > 120.0) {
                    duty_accumulator = 60.0;
                    pid_integral = 0;
                    pid_integral_cv = 0;
                }
                if (v_bat_filt > BOOST_BAT_VOLTAGE_LIMIT) {
                    float over_bat_v = v_bat_filt - BOOST_BAT_VOLTAGE_LIMIT;
                    if (over_bat_v > BOOST_VBAT_HARD_OVERSHOOT_MARGIN) {
                        duty_accumulator -= (4.0 + (over_bat_v * 9.0));
                    } else {
                        duty_accumulator -= (1.5 + (over_bat_v * 6.0));
                    }
                    pid_integral = 0;
                }
                if (v_bat > (v_bat_filt + 0.9f)) {
                    duty_accumulator -= 10.0;
                    pid_integral = 0;
                    pid_integral_cv *= 0.7;
                }

                // จำกัดความเร็วขาขึ้นเมื่อเข้าใกล้ข้อจำกัดเพื่อกัน overshoot
                float duty_delta = duty_accumulator - duty_before_control;
                if (duty_delta > 0.0 && hold_duty_ceiling) {
                    const float LIMITED_UP_STEP = 0.35;
                    if (duty_delta > LIMITED_UP_STEP) {
                        duty_accumulator = duty_before_control + LIMITED_UP_STEP;
                    }
                }

                // เพดาน duty แบบ dynamic: ถึงเงื่อนไขแล้วไม่ให้เพิ่มต่อจนกว่าจะหลุดจากโซนลิมิต
                if (duty_accumulator > boost_duty_ceiling) {
                    duty_accumulator = boost_duty_ceiling;
                }
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

            bool full_window_current = (i_bat_charge_filt <= FULL_END_CURRENT);
            if ((v_bat_filt >= FULL_DETECT_VOLTAGE) && full_window_current) {
                if (full_condition_start_ms == 0) full_condition_start_ms = now;
                if (now - full_condition_start_ms >= FULL_CONFIRM_MS) {
                    charge_full_hold = true;
                    disablePowerStage();
                    Serial.println("[INFO] Battery FULL detected. Hold charging until restart threshold.");
                }
            } else {
                full_condition_start_ms = 0;
            }

            // ถ้าแรงดันแบตสูงค้างนาน ให้ตัดชาร์จแบบ pre-stop ก่อนชน OVP
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
                if (boostMode == BOOST_RAMP) state_label = "BOOST_RAMP";
                else if (boostMode == BOOST_MPPT) state_label = "BOOST_MPPT";
                else state_label = "BOOST_CV";
            } else if (currentState == STATE_FORWARD) {
                state_label = "FORWARD";
            }
            Serial.println("=========================================================================================");
            Serial.printf("[DEBUG INTERFACE] System: %s | State: %s | Active Duty: %d%% | DutyCeil:%3d%%\n",
                          (system_ON ? "ON " : "OFF"),
                          state_label,
                          active_duty_percent,
                          (int)roundf((boost_duty_ceiling * 100.0f) / 1023.0f));
            Serial.printf("  [PV SOLAR] Calc Volt: %5.1f V | RAW Pin A0: %7.1f mV | Target: %.2f V\n", v_solar, raw_mv_v0, v_solar_target);
            Serial.printf("  [PV CURR ] Calc Amps: %5.2f A (|I|=%5.2fA) | RAW Pin A0: %7.1f mV\n", i_solar, i_solar_mag, raw_mv_i0);
            Serial.printf("  [BATTERY ] Calc Volt: %5.1f V | RAW Pin A1: %7.1f mV\n", v_bat, raw_mv_v2);
            Serial.printf("  [BAT CURR] Calc Amps: %5.2f A | RAW Pin A2: %7.1f mV\n", i_bat, raw_mv_i2);
            Serial.printf("  [BAT FILT] Volt/Amps: %5.2f V / %5.2f A (|I|=%5.2fA)\n", v_bat_filt, i_bat_filt, i_bat_charge_filt);
            Serial.printf("  [AC VOLT ] Calc Volt: %5.1f V | RAW Pin A2: %7.1f mV\n", v_ac_in, raw_mv_v1);
            Serial.println("=========================================================================================");
        }

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

// =========================================================================
// TASK 2: การตรวจสอบสถานะปุ่มกดและการอัปเดตหน้าจอแสดงผล LCD 20x4
// =========================================================================
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

        // โหมด latching: กด START ติดค้าง, กด STOP ถึงดับ
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

        // กู้ LCD เฉพาะเมื่อมีอาการค้างจริง ไม่ init ทุกคาบเวลา
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
