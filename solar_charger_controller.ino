#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

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
const float TARGET_CV_VOLTAGE = 58.4;
const float TARGET_CC_CURRENT = 3.0;

const float MIN_PV_VOLTAGE_START = 40.0;   // เกณฑ์เริ่มทำงานฝั่ง PV (ผ่อนเล็กน้อยให้เริ่มติดง่ายขึ้น)
const float UNDER_PV_VOLTAGE_CRIT = 39.0;  // ต่ำกว่า 39V เกิน 2 วินาที สั่งตัด
const float MIN_AC_VOLTAGE_START = 120.0;  // เกณฑ์เริ่มทำงานฝั่ง AC (ช่วยกรณีคาลิเบรตต่ำกว่าจริง)
const float MIN_AC_VOLTAGE_KEEP = 110.0;   // เกณฑ์คงการทำงานฝั่ง AC

const int MAX_DUTY_FORWARD = 490;
const int MAX_DUTY_BOOST   = 760;

// ถ้าอ่าน ADC ไม่สำเร็จเกินช่วงนี้ ให้เข้าสู่โหมดปลอดภัย
const unsigned long ADC_STALE_TIMEOUT_MS = 700;
const unsigned long SENSOR_ERROR_LOG_MS = 2000;
const TickType_t I2C_MUTEX_TIMEOUT_SAMPLE_TICKS = pdMS_TO_TICKS(50);
const TickType_t I2C_MUTEX_TIMEOUT_LCD_TICKS = pdMS_TO_TICKS(120);
const unsigned long LCD_RECOVER_RETRY_MS = 1500;
const unsigned long LCD_MUTEX_WARN_MS = 2000;
const float CV_DEADBAND_V = 0.12;
const float CV_SOFT_OVERVOLTAGE_V = 58.6;         // เข้าโซนนี้ให้กด Duty ลงแรงขึ้น
const float BAT_OVERVOLTAGE_CUTOFF_V = 59.0;      // เกินค่านี้ให้สั่ง Duty = 0%
const float BAT_OVERVOLTAGE_RECOVER_V = 58.6;     // ต้องลดต่ำกว่านี้จึงออกจากโหมดป้องกัน
const unsigned long BAT_OVERVOLTAGE_CONFIRM_MS = 3000; // ถ้ายังเกินต่อเนื่องค่อยตัดระบบ
const float CV_FINE_ZONE_V = 57.8;                 // ใกล้เต็มเริ่มเข้าโหมดปรับละเอียด
const float CV_FINE_STEP_UP = 0.02;                // เพิ่ม duty ให้ละเอียดขึ้น ลดอาการพุ่งในโซน 0-2%
const float CV_FINE_STEP_DOWN = -0.10;
const float CV_ULTRA_FINE_ZONE_V = 58.2;           // ช่วงท้ายก่อนเต็ม ใช้ step ละเอียดพิเศษ
const float CV_ULTRA_FINE_STEP_UP = 0.005;
const float CV_ULTRA_FINE_STEP_DOWN = -0.06;
const float CV_RISE_LOCK_V = 58.20;                // สูงกว่าโซนนี้ห้ามเพิ่ม duty
const unsigned long CV_FINE_UP_STEP_INTERVAL_MS = 1200;      // หน่วงการเพิ่ม duty ขาขึ้น (อย่าตั้งสูงมากจนชาร์จไม่เข้า)
const unsigned long CV_ULTRA_FINE_UP_STEP_INTERVAL_MS = 2600;
const float MIN_REASONABLE_KP_CV = 0.10;
const float MAX_REASONABLE_KI_CC = 0.05;
const unsigned long MAX_REASONABLE_CV_FINE_UP_INTERVAL_MS = 3000;
const unsigned long BOOST_MPPT_INTERVAL_MS = 150;
const float BOOST_MPPT_V_STEP_NORMAL = 0.08;
const float BOOST_MPPT_V_STEP_LOW_SUN = 0.05;
const float BOOST_MPPT_MIN_DELTA_P_W = 0.25;
const float BOOST_MPPT_TARGET_MIN_NORMAL = 41.0;
const float BOOST_MPPT_TARGET_MIN_LOW_SUN = 38.5;
const float BOOST_LOW_SUN_ENTRY_V = 40.8;
const float BOOST_LOW_SUN_EXIT_V = 41.6;
const float BOOST_BAT_CURRENT_LIMIT_MARGIN_A = 0.20;
const float BOOST_PID_POS_LIMIT = 1.0;
const float BOOST_PID_NEG_LIMIT = -2.0;
const float BOOST_SOFTSTART_STEP = 1.0;
const float BOOST_MIN_VALID_PV_V = 5.0;
const float BOOST_PV_SHUTDOWN_LOW_SUN_V = 37.5;
const unsigned long BOOST_PV_SHUTDOWN_NORMAL_MS = 2000;
const unsigned long BOOST_PV_SHUTDOWN_LOW_SUN_MS = 7000;
const float FULL_DETECT_VOLTAGE = 58.3;
const float FULL_END_CURRENT = 0.45;              // 15% ของกระแส CC (3A)
const unsigned long FULL_CONFIRM_MS = 300000;     // เงื่อนไข FULL ต้องต่อเนื่อง 5 นาที
const float RESTART_CHARGE_VOLTAGE = 55.2;        // แรงดันตกต่ำกว่านี้จึงกลับมาชาร์จใหม่

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

const float Kp_cv = 0.42;   // ถ้าต่ำเกินไปจะเร่ง duty ไม่พอจนดูเหมือนไม่ชาร์จ
const float Ki_cv = 0.010;
const float Kd_cv = 0.004;

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

// =========================================================================
// ตัวแปรระบบ
// =========================================================================
volatile float v_solar = 0, v_ac_in = 0, v_bat = 0;
volatile float i_solar = 0, i_ac_in = 0, i_bat = 0;
volatile float v_bat_filt = 0, i_bat_filt = 0;
volatile bool system_ON = false;
volatile bool charge_full_hold = false;
volatile int active_duty_percent = 0;
int raw_duty = 0;
float total_Wh = 0;
unsigned long last_millis = 0;

volatile bool sensor_init_ok = false;
volatile unsigned long last_adc_sample_ms = 0;

enum SystemState { STATE_BOOST, STATE_FORWARD, STATE_OFF };
volatile SystemState currentState = STATE_OFF;
bool last_system_state = false;

LiquidCrystal_I2C lcd(0x27, 20, 4);
SemaphoreHandle_t i2c_Mutex;

float raw_mv_v0 = 0, raw_mv_v1 = 0, raw_mv_v2 = 0;
float raw_mv_i0 = 0, raw_mv_i1 = 0, raw_mv_i2 = 0;
float vbat_filter_buf[8] = {0};
float ibat_filter_buf[8] = {0};
float vbat_filter_sum = 0;
float ibat_filter_sum = 0;
int filter_index = 0;
int filter_count = 0;

void TaskSampleData(void * pvParameters);
void TaskLCDLoop(void * pvParameters);

static inline void disablePowerStage() {
    currentState = STATE_OFF;
    raw_duty = 0;
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

// =========================================================================
// SETUP & MAIN LOOP
// =========================================================================
void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22);
    Wire.setTimeOut(25);
    Wire.setClock(400000);  // ลดเวลาจับ bus ของทั้ง ADS/LCD ลดอาการจอค้างจากการแย่ง I2C
    if (Kp_cv < MIN_REASONABLE_KP_CV) {
        Serial.printf("[WARN] Kp_cv=%.3f too low. Fallback to 0.42 will be used.\n", Kp_cv);
    }
    if (Ki_cc > MAX_REASONABLE_KI_CC) {
        Serial.printf("[WARN] Ki_cc=%.3f too high. Fallback to 0.01 will be used.\n", Ki_cc);
    }
    if (CV_FINE_UP_STEP_INTERVAL_MS > MAX_REASONABLE_CV_FINE_UP_INTERVAL_MS) {
        Serial.printf("[WARN] CV_FINE_UP_STEP_INTERVAL_MS=%lu too long. Fallback to 1200ms will be used.\n", CV_FINE_UP_STEP_INTERVAL_MS);
    }

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
    lcd.init();
    lcd.backlight();

    if (!sensor_init_ok) {
        Serial.println("[FATAL] ADS1115 init failed. System is locked in safe standby.");
        forceSafeShutdown();
    } else {
        last_adc_sample_ms = millis();
    }

    xTaskCreatePinnedToCore(TaskSampleData, "ADC_PWM_Task", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(TaskLCDLoop, "LCD_Task", 4096, NULL, 1, NULL, 1);
}

void loop() { vTaskDelay(1000); }

// =========================================================================
// TASK 1: การคำนวณข้อมูลสัญญานทางไฟฟ้าและลูป PID ควบคุมแปลงผันกำลังไฟฟ้า
// =========================================================================
void TaskSampleData(void * pvParameters) {
    float p_solar_old = 0.0;
    float v_solar_old = 0.0;
    int mppt_direction = 1;
    unsigned long last_mppt_time = 0;
    bool low_sun_mode = false;

    unsigned long pv_collapse_start_time = 0;
    bool pv_is_collapsing = false;
    unsigned long last_debug_time = 0;
    unsigned long last_sensor_error_log = 0;
    unsigned long full_condition_start_ms = 0;
    unsigned long overvoltage_start_ms = 0;
    bool overvoltage_duty_zero_active = false;
    float duty_step_accumulator = 0.0;
    unsigned long last_forward_up_step_ms = 0;
    const float kp_cv_effective = (Kp_cv < MIN_REASONABLE_KP_CV) ? 0.42f : Kp_cv;
    const float ki_cc_effective = (Ki_cc > MAX_REASONABLE_KI_CC) ? 0.01f : Ki_cc;
    const unsigned long cv_fine_up_interval_effective =
        (CV_FINE_UP_STEP_INTERVAL_MS > MAX_REASONABLE_CV_FINE_UP_INTERVAL_MS) ? 1200UL : CV_FINE_UP_STEP_INTERVAL_MS;

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
        if (xSemaphoreTake(i2c_Mutex, I2C_MUTEX_TIMEOUT_SAMPLE_TICKS)) {
            raw_mv_v0 = ads_volt.readADC_SingleEnded(0) * 0.1875;
            raw_mv_v1 = ads_volt.readADC_SingleEnded(2) * 0.1875;
            raw_mv_v2 = ads_volt.readADC_SingleEnded(1) * 0.1875;

            raw_mv_i0 = ads_curr.readADC_SingleEnded(0) * 0.1875;
            raw_mv_i1 = ads_curr.readADC_SingleEnded(1) * 0.1875;
            raw_mv_i2 = ads_curr.readADC_SingleEnded(2) * 0.1875;

            float mv_pure_v0 = raw_mv_v0 - OFFSET_V_SOLAR; if (mv_pure_v0 < 0.0) mv_pure_v0 = 0.0;
            float mv_pure_v1 = raw_mv_v1 - OFFSET_V_AC;    if (mv_pure_v1 < 0.0) mv_pure_v1 = 0.0;
            float mv_pure_v2 = raw_mv_v2 - OFFSET_V_BAT;   if (mv_pure_v2 < 0.0) mv_pure_v2 = 0.0;

            float mv_pure_i0 = raw_mv_i0 - OFFSET_I_SOLAR;
            float mv_pure_i1 = raw_mv_i1 - OFFSET_I_AC;
            float mv_pure_i2 = raw_mv_i2 - OFFSET_I_BAT;

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

            last_adc_sample_ms = now;
            sample_ok = true;
            xSemaphoreGive(i2c_Mutex);
        }

        // stale-data protection: อย่าคุมกำลังด้วยค่าที่อ่านค้าง
        if (!sample_ok && system_ON && (now - last_adc_sample_ms > ADC_STALE_TIMEOUT_MS)) {
            forceSafeShutdown();
            Serial.println("[CRITICAL] ADC sample timeout. Auto-shutdown for safety.");
        }

        // hard over-voltage protection แบบ 2 ชั้น:
        // 1) เกินแรงดันให้ลด Duty = 0% ก่อน
        // 2) ถ้ายังเกินต่อเนื่องนาน จึงค่อยตัดระบบ
        if (system_ON && !charge_full_hold) {
            if (v_bat_filt >= BAT_OVERVOLTAGE_CUTOFF_V) {
                if (!overvoltage_duty_zero_active) {
                    overvoltage_duty_zero_active = true;
                    overvoltage_start_ms = now;
                    raw_duty = 0;
                    pid_integral = 0.0;
                    pid_integral_cc = 0.0;
                    pid_integral_cv = 0.0;
                    pid_last_error = 0.0;
                    pid_last_error_cc = 0.0;
                    pid_last_error_cv = 0.0;
                    Serial.printf("[WARN] Over-voltage %.2fV -> Force Duty 0%% and monitor.\n", v_bat_filt);
                }
            } else if (overvoltage_duty_zero_active && v_bat_filt <= BAT_OVERVOLTAGE_RECOVER_V) {
                overvoltage_duty_zero_active = false;
                overvoltage_start_ms = 0;
                Serial.printf("[INFO] Voltage recovered %.2fV -> Exit over-voltage guard.\n", v_bat_filt);
            }

            if (overvoltage_duty_zero_active) {
                raw_duty = 0;
                ledcWrite(PWM_FORWARD_PIN, 0);
                ledcWrite(PWM_BOOST_PIN, 0);
                duty_step_accumulator = 0.0;

                if ((now - overvoltage_start_ms >= BAT_OVERVOLTAGE_CONFIRM_MS) &&
                    (v_bat_filt >= BAT_OVERVOLTAGE_CUTOFF_V)) {
                    forceSafeShutdown();
                    overvoltage_duty_zero_active = false;
                    overvoltage_start_ms = 0;
                    Serial.printf("[CRITICAL] Over-voltage persisted at %.2fV. Shutdown.\n", v_bat_filt);
                }

                last_millis = now;
                active_duty_percent = 0;
                vTaskDelay(20 / portTICK_PERIOD_MS);
                continue;
            }
        } else {
            overvoltage_duty_zero_active = false;
            overvoltage_start_ms = 0;
        }

        if (system_ON) {
            if (charge_full_hold) {
                disablePowerStage();
                if ((v_bat_filt <= RESTART_CHARGE_VOLTAGE) &&
                    (v_solar >= MIN_PV_VOLTAGE_START || v_ac_in >= MIN_AC_VOLTAGE_START)) {
                    charge_full_hold = false;
                    Serial.println("[INFO] Battery dropped to restart threshold. Charging resumed.");
                }
            }

            if (charge_full_hold) {
                vTaskDelay(20 / portTICK_PERIOD_MS);
                continue;
            }

            if (currentState == STATE_OFF) {
                if (v_solar >= MIN_PV_VOLTAGE_START) {
                    ledcWrite(PWM_FORWARD_PIN, 0); ledcWrite(PWM_BOOST_PIN, 0);
                    digitalWrite(RELAY_AC_PIN, LOW);
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    digitalWrite(RELAY_PV_PIN, HIGH);
                    currentState = STATE_BOOST;
                    low_sun_mode = (v_solar <= BOOST_LOW_SUN_ENTRY_V);

                    v_solar_old = v_solar;
                    p_solar_old = v_solar * i_solar;
                    float target_floor = low_sun_mode ? BOOST_MPPT_TARGET_MIN_LOW_SUN : BOOST_MPPT_TARGET_MIN_NORMAL;
                    v_solar_target = v_solar - 0.8;
                    if (v_solar_target < target_floor) v_solar_target = target_floor;
                    pid_integral = 0; pid_last_error = 0;
                    pid_integral_cc = 0; pid_last_error_cc = 0;
                    pid_integral_cv = 0; pid_last_error_cv = 0;
                    raw_duty = 20;
                    pv_is_collapsing = false;
                }
                else if (v_ac_in >= MIN_AC_VOLTAGE_START) {
                    ledcWrite(PWM_FORWARD_PIN, 0); ledcWrite(PWM_BOOST_PIN, 0);
                    digitalWrite(RELAY_PV_PIN, LOW);
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    digitalWrite(RELAY_AC_PIN, HIGH);
                    currentState = STATE_FORWARD;
                    pid_integral_cc = 0; pid_last_error_cc = 0;
                    pid_integral_cv = 0; pid_last_error_cv = 0;
                    raw_duty = 10;
                }
                else {
                    Serial.printf("[INFO] Start source not ready. PV=%.1fV (<%.1f) AC=%.1fV (<%.1f)\n",
                                  v_solar, MIN_PV_VOLTAGE_START, v_ac_in, MIN_AC_VOLTAGE_START);
                    system_ON = false;
                }
            }
            else if (currentState == STATE_BOOST) {
                bool prev_low_sun_mode = low_sun_mode;
                if (v_solar <= BOOST_LOW_SUN_ENTRY_V) low_sun_mode = true;
                else if (v_solar >= BOOST_LOW_SUN_EXIT_V) low_sun_mode = false;
                if (prev_low_sun_mode != low_sun_mode) {
                    Serial.printf("[INFO] BOOST mode=%s (PV=%.1fV)\n", low_sun_mode ? "LOW_SUN" : "NORMAL", v_solar);
                }

                float pv_shutdown_v = low_sun_mode ? BOOST_PV_SHUTDOWN_LOW_SUN_V : UNDER_PV_VOLTAGE_CRIT;
                unsigned long pv_shutdown_confirm_ms = low_sun_mode ? BOOST_PV_SHUTDOWN_LOW_SUN_MS : BOOST_PV_SHUTDOWN_NORMAL_MS;
                if (v_solar < pv_shutdown_v) {
                    if (!pv_is_collapsing) {
                        pv_is_collapsing = true;
                        pv_collapse_start_time = now;
                    }

                    if (now - pv_collapse_start_time >= pv_shutdown_confirm_ms) {
                        system_ON = false;
                        Serial.printf("[CRITICAL] Solar collapsed below %.1fV for %lums. Auto-Shutdown.\n",
                                      pv_shutdown_v, pv_shutdown_confirm_ms);
                    }
                } else {
                    pv_is_collapsing = false;
                }
            }
            else if (currentState == STATE_FORWARD) {
                if (v_ac_in < MIN_AC_VOLTAGE_KEEP) { system_ON = false; }
            }
        }

        if (!system_ON) {
            digitalWrite(RELAY_PV_PIN, LOW);
            digitalWrite(RELAY_AC_PIN, LOW);
            currentState = STATE_OFF;
            raw_duty = 0;
            pv_is_collapsing = false;
            low_sun_mode = false;
        }

        if (system_ON && currentState != STATE_OFF) {
            int allowed_max_duty = (currentState == STATE_FORWARD) ? MAX_DUTY_FORWARD : MAX_DUTY_BOOST;
            if (currentState != STATE_FORWARD) {
                duty_step_accumulator = 0.0;
                last_forward_up_step_ms = 0;
            }

            if (currentState == STATE_FORWARD) {
                // =================================================================
                // 🔋 โหมด AC: ระบบควบคุม DUAL-LOOP PID (CC/CV CHARGING CONTROL)
                // =================================================================

                // 1. ลูปควบคุมกระแสคงที่ (Constant Current Loop - CC) เป้าหมาย 3.0A
                pid_error_cc = TARGET_CC_CURRENT - i_bat_filt;
                pid_integral_cc += pid_error_cc;
                pid_integral_cc = constrain(pid_integral_cc, -100, 100);
                float delta_error_cc = pid_error_cc - pid_last_error_cc;
                float pid_out_cc = (Kp_cc * pid_error_cc) + (ki_cc_effective * pid_integral_cc) + (Kd_cc * delta_error_cc);
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
                float pid_out_cv = (kp_cv_effective * pid_error_cv) + (Ki_cv * pid_integral_cv) + (Kd_cv * delta_error_cv);
                pid_last_error_cv = pid_error_cv;

                // เลือกค่าเอาต์พุตจากวงจร PID ที่ปลอดภัยและมีค่าต่ำที่สุด ป้องกัน Overshoot
                float final_battery_pid = min(pid_out_cc, pid_out_cv);

                // เข้าใกล้แรงดัน CV แล้ว จำกัดการเร่ง Duty ให้เบาลง
                if (v_bat_filt > (TARGET_CV_VOLTAGE - 0.6) && final_battery_pid > 0.25) {
                    final_battery_pid = 0.25;
                }

                // หากแรงดันเริ่มสูงกว่าเป้า ให้เร่งลด Duty ทันที
                if (v_bat_filt >= CV_SOFT_OVERVOLTAGE_V) {
                    final_battery_pid = -6.0;
                    pid_integral_cc = 0.0;
                    pid_integral_cv = 0.0;
                }

                // จำกัดความเร็วการเร่ง/ลด ในหนึ่งรอบลูป (Slew-Rate Limit ฝั่งแบตเตอรี่)
                if (final_battery_pid > 1.5) final_battery_pid = 1.5;
                if (final_battery_pid < -4.0) final_battery_pid = -4.0;
                
                // โหมดละเอียดช่วงใกล้เต็มและ duty ต่ำ: สะสมเศษเพื่อลด step jump ที่ 0-2%
                if (v_bat_filt >= CV_FINE_ZONE_V && raw_duty <= 25) {
                    if (final_battery_pid > CV_FINE_STEP_UP) final_battery_pid = CV_FINE_STEP_UP;
                    if (final_battery_pid < CV_FINE_STEP_DOWN) final_battery_pid = CV_FINE_STEP_DOWN;
                }
                if (v_bat_filt >= CV_ULTRA_FINE_ZONE_V && raw_duty <= 15) {
                    if (final_battery_pid > CV_ULTRA_FINE_STEP_UP) final_battery_pid = CV_ULTRA_FINE_STEP_UP;
                    if (final_battery_pid < CV_ULTRA_FINE_STEP_DOWN) final_battery_pid = CV_ULTRA_FINE_STEP_DOWN;
                }
                if (v_bat_filt >= CV_RISE_LOCK_V && final_battery_pid > 0.0) {
                    final_battery_pid = 0.0;
                }

                duty_step_accumulator += final_battery_pid;
                int duty_step = 0;
                if (duty_step_accumulator >= 1.0) {
                    duty_step = (int)floor(duty_step_accumulator);
                } else if (duty_step_accumulator <= -1.0) {
                    duty_step = (int)ceil(duty_step_accumulator);
                }

                // โซนปลาย CV: ขาขึ้นต้องช้ากว่าขาลงเพื่อลดการกระชากแรงดัน
                if (duty_step > 0 && v_bat_filt >= CV_FINE_ZONE_V && raw_duty <= 25) {
                    unsigned long min_up_interval = cv_fine_up_interval_effective;
                    if (v_bat_filt >= CV_ULTRA_FINE_ZONE_V && raw_duty <= 15) {
                        min_up_interval = CV_ULTRA_FINE_UP_STEP_INTERVAL_MS;
                    }
                    if (now - last_forward_up_step_ms < min_up_interval) {
                        duty_step = 0;
                        if (duty_step_accumulator > 0.95) duty_step_accumulator = 0.95;
                    } else {
                        last_forward_up_step_ms = now;
                    }
                }

                raw_duty += duty_step;
                duty_step_accumulator -= duty_step;
            }
            else if (currentState == STATE_BOOST) {
                // =================================================================
                // ☀️ โหมด PV: ระบบควบคุมแผงและระบบป้องกันฝั่งเอาต์พุตขั้นเด็ดขาด
                // =================================================================

                if (v_bat_filt >= CV_SOFT_OVERVOLTAGE_V) {
                    raw_duty -= 10;
                    pid_integral = 0;
                }
                else if (v_bat_filt >= TARGET_CV_VOLTAGE || i_bat_filt >= (TARGET_CC_CURRENT + BOOST_BAT_CURRENT_LIMIT_MARGIN_A)) {
                    raw_duty -= 3;
                    pid_integral = 0;
                }
                else if (v_solar <= BOOST_MIN_VALID_PV_V) {
                    raw_duty = 0; pid_integral = 0;
                }
                else {
                    if (now - last_mppt_time >= BOOST_MPPT_INTERVAL_MS) {
                        last_mppt_time = now;
                        float p_solar = v_solar * i_solar;
                        float delta_p = p_solar - p_solar_old;
                        float delta_v = v_solar - v_solar_old;

                        if (fabs(delta_p) >= BOOST_MPPT_MIN_DELTA_P_W) {
                            if (delta_p > 0) {
                                if (delta_v > 0) mppt_direction = 1;
                                else             mppt_direction = -1;
                            } else {
                                if (delta_v > 0) mppt_direction = -1;
                                else             mppt_direction = 1;
                            }
                        }

                        float step_v = low_sun_mode ? BOOST_MPPT_V_STEP_LOW_SUN : BOOST_MPPT_V_STEP_NORMAL;
                        float target_floor = low_sun_mode ? BOOST_MPPT_TARGET_MIN_LOW_SUN : BOOST_MPPT_TARGET_MIN_NORMAL;
                        v_solar_target += (mppt_direction * step_v);
                        if (v_solar_target < target_floor) v_solar_target = target_floor;
                        if (v_solar_target > 48.0) v_solar_target = 48.0;

                        p_solar_old = p_solar; v_solar_old = v_solar;
                    }

                    float target_floor = low_sun_mode ? BOOST_MPPT_TARGET_MIN_LOW_SUN : BOOST_MPPT_TARGET_MIN_NORMAL;
                    pid_error = v_solar - v_solar_target;
                    if (v_solar < (target_floor - 0.3)) {
                        pid_integral = 0;
                    } else {
                        pid_integral += pid_error;
                        pid_integral = constrain(pid_integral, -50, 50);
                    }
                    pid_derivative = pid_error - pid_last_error;

                    float pid_output = (Kp * pid_error) + (Ki * pid_integral) + (Kd * pid_derivative);

                    // ระบบแก้ล็อกช่วงเริ่มต้น (Soft-start ในโหมดแผง)
                    if (raw_duty < 30 && v_solar > target_floor) {
                        pid_output = BOOST_SOFTSTART_STEP;
                    } else {
                        if (pid_output > BOOST_PID_POS_LIMIT) pid_output = BOOST_PID_POS_LIMIT;
                    }
                    if (pid_output < BOOST_PID_NEG_LIMIT) pid_output = BOOST_PID_NEG_LIMIT;

                    if (v_solar <= 40.5) {
                        if (pid_output > 0) pid_output = 0;
                        if (v_solar <= 40.0) pid_output = -5.0;
                        else if (pid_output < BOOST_PID_NEG_LIMIT) pid_output = BOOST_PID_NEG_LIMIT;
                    }

                    raw_duty += (int)round(pid_output);
                    pid_last_error = pid_error;
                }
            }

            raw_duty = constrain(raw_duty, 0, allowed_max_duty);

            if (currentState == STATE_FORWARD) {
                ledcWrite(PWM_FORWARD_PIN, raw_duty);
                ledcWrite(PWM_BOOST_PIN, 0);
            } else if (currentState == STATE_BOOST) {
                ledcWrite(PWM_BOOST_PIN, raw_duty);
                ledcWrite(PWM_FORWARD_PIN, 0);
            }

            total_Wh += ((v_bat * i_bat) * (now - last_millis)) / 3600000.0;

            if ((v_bat_filt >= FULL_DETECT_VOLTAGE) && (fabs(i_bat_filt) <= FULL_END_CURRENT)) {
                if (full_condition_start_ms == 0) full_condition_start_ms = now;
                if (now - full_condition_start_ms >= FULL_CONFIRM_MS) {
                    charge_full_hold = true;
                    disablePowerStage();
                    Serial.println("[INFO] Battery FULL detected. Hold charging until restart threshold.");
                }
            } else {
                full_condition_start_ms = 0;
            }
        } else {
            ledcWrite(PWM_FORWARD_PIN, 0);
            ledcWrite(PWM_BOOST_PIN, 0);
            full_condition_start_ms = 0;
            overvoltage_duty_zero_active = false;
            overvoltage_start_ms = 0;
            duty_step_accumulator = 0.0;
        }

        last_millis = now;
        active_duty_percent = round(((float)raw_duty * 100.0) / 1023.0);

        if (now - last_debug_time >= 500) {
            last_debug_time = now;
            const char* state_label = charge_full_hold
                ? "FULL_HOLD"
                : (currentState == STATE_BOOST ? (low_sun_mode ? "BOOST_LOW" : "BOOST") : (currentState == STATE_FORWARD ? "FORWARD" : "OFF"));
            Serial.println("=========================================================================================");
            Serial.printf("[DEBUG INTERFACE] System: %s | State: %s | Active Duty: %d%%\n",
                          (system_ON ? "ON " : "OFF"),
                          state_label,
                          active_duty_percent);
            Serial.printf("  [PV SOLAR] Calc Volt: %5.1f V | RAW Pin A0: %7.1f mV | Target: %.2f V\n", v_solar, raw_mv_v0, v_solar_target);
            Serial.printf("  [PV CURR ] Calc Amps: %5.2f A | RAW Pin A0: %7.1f mV\n", i_solar, raw_mv_i0);
            Serial.printf("  [BATTERY ] Calc Volt: %5.1f V | RAW Pin A1: %7.1f mV\n", v_bat, raw_mv_v2);
            Serial.printf("  [BAT CURR] Calc Amps: %5.2f A | RAW Pin A2: %7.1f mV\n", i_bat, raw_mv_i2);
            Serial.printf("  [BAT FILT] Volt/Amps: %5.2f V / %5.2f A\n", v_bat_filt, i_bat_filt);
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
    unsigned long alert_millis = 0;
    unsigned long last_lcd_recover = 0;
    unsigned long last_lcd_warn = 0;
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
        } else if (start_edge) {
            if (sensor_init_ok && (v_solar >= MIN_PV_VOLTAGE_START || v_ac_in >= MIN_AC_VOLTAGE_START)) {
                system_ON = true;
                charge_full_hold = false;
                show_no_power_alert = false;
                Serial.printf("[INFO] START accepted. PV=%.1fV AC=%.1fV\n", v_solar, v_ac_in);
            } else {
                system_ON = false;
                show_no_power_alert = true;
                alert_millis = now;
                Serial.printf("[WARN] START blocked. sensor=%d PV=%.1fV (need %.1f) AC=%.1fV (need %.1f)\n",
                              sensor_init_ok ? 1 : 0,
                              v_solar, MIN_PV_VOLTAGE_START,
                              v_ac_in, MIN_AC_VOLTAGE_START);
            }
        }
        last_start_state = current_start; last_stop_state = current_stop;

        if (system_ON != last_system_state) {
            if (xSemaphoreTake(i2c_Mutex, I2C_MUTEX_TIMEOUT_LCD_TICKS)) {
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
            if (xSemaphoreTake(i2c_Mutex, I2C_MUTEX_TIMEOUT_LCD_TICKS)) {
                lcd.clear();
                xSemaphoreGive(i2c_Mutex);
                lcd_mutex_fail_count = 0;
            } else {
                lcd_mutex_fail_count++;
            }
        }

        if (xSemaphoreTake(i2c_Mutex, I2C_MUTEX_TIMEOUT_LCD_TICKS)) {
            if (system_ON && charge_full_hold) {
                lcd.setCursor(0, 0); lcd.print("BATTERY FULL HOLD    ");
                lcd.setCursor(0, 1); lcd.printf("BAT:%5.1fV I:%4.2fA  ", v_bat_filt, i_bat_filt);
                lcd.setCursor(0, 2); lcd.printf("Resume <= %5.1fV     ", RESTART_CHARGE_VOLTAGE);
                lcd.setCursor(0, 3); lcd.print("Press STOP to cancel ");
            } else if (system_ON) {
                lcd.setCursor(0, 0); lcd.printf("ACTIVE   DUTY:%3d%%   ", active_duty_percent);
                lcd.setCursor(0, 1); lcd.printf("%-8s   PWR:%5.1fWh ", (currentState == STATE_BOOST ? "BOOST PV" : "FORW AC"), total_Wh);
                lcd.setCursor(0, 2); lcd.printf("IN :%5.1fV %5.1fA   ", (currentState == STATE_BOOST ? v_solar : v_ac_in), (currentState == STATE_BOOST ? i_solar : i_ac_in));
                lcd.setCursor(0, 3); lcd.printf("OUT:%5.1fV %5.1fA   ", v_bat, i_bat);
            } else if (show_no_power_alert) {
                lcd.setCursor(0, 0); lcd.print("      ERROR      ");
                lcd.setCursor(0, 1); lcd.print("  NO INPUT POWER!   ");
                lcd.setCursor(0, 2); lcd.print(" Check PV / AC Line ");
                lcd.setCursor(0, 3); lcd.print("  CANNOT ACTIVATE   ");
            } else {
                lcd.setCursor(0, 0); lcd.print("STANDBY             ");
                lcd.setCursor(0, 1); lcd.printf("PV :%5.1fV AC:%5.1fV", v_solar, v_ac_in);
                lcd.setCursor(0, 2); lcd.printf("BATT:%5.1fV         ", v_bat);
                lcd.setCursor(0, 3); lcd.print("                    ");
            }
            xSemaphoreGive(i2c_Mutex);
            lcd_mutex_fail_count = 0;
        } else {
            lcd_mutex_fail_count++;
        }

        if (lcd_mutex_fail_count > 0 && (now - last_lcd_warn >= LCD_MUTEX_WARN_MS)) {
            last_lcd_warn = now;
            Serial.printf("[WARN] LCD mutex contention count=%d\n", lcd_mutex_fail_count);
        }

        // กู้ LCD เฉพาะเมื่อมีอาการค้างจริง ไม่ init ทุกคาบเวลา
        if (lcd_mutex_fail_count >= 8 && (now - last_lcd_recover > LCD_RECOVER_RETRY_MS)) {
            last_lcd_recover = now;
            if (xSemaphoreTake(i2c_Mutex, I2C_MUTEX_TIMEOUT_LCD_TICKS)) {
                lcd.init();
                lcd.backlight();
                lcd.clear();
                xSemaphoreGive(i2c_Mutex);
                lcd_mutex_fail_count = 0;
                Serial.println("[INFO] LCD recovered by re-init.");
            }
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
