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

const float MIN_PV_VOLTAGE = 42.0;         // เริ่มทำงานเมื่อแผงถึง 42V
const float UNDER_PV_VOLTAGE_CRIT = 39.0;  // ต่ำกว่า 39V เกิน 2 วินาที สั่งตัด
const float MIN_AC_VOLTAGE = 140.0;

const int MAX_DUTY_FORWARD = 490;
const int MAX_DUTY_BOOST   = 760;

// ถ้าอ่าน ADC ไม่สำเร็จเกินช่วงนี้ ให้เข้าสู่โหมดปลอดภัย
const unsigned long ADC_STALE_TIMEOUT_MS = 700;
const unsigned long SENSOR_ERROR_LOG_MS = 2000;
const float CV_DEADBAND_V = 0.10;
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
const float MIN_CURRENT_FOR_ACTIVE_CHARGE = 0.20;
const float BOOST_VOLTAGE_FLOOR = 42.0;
const float BOOST_BAT_VOLTAGE_LIMIT = 58.0;
const float BOOST_SAFE_V_HEADROOM = 0.4;
const float BOOST_SAFE_I_HEADROOM = 0.15;
const float BOOST_SAFE_BAT_HEADROOM = 0.3;
const float HARD_OVP_TRIP_VOLTAGE = 60.5;
const float HARD_OVP_RELEASE_VOLTAGE = 58.0;

// =========================================================================
// ตัวแปรระบบ
// =========================================================================
volatile float v_solar = 0, v_ac_in = 0, v_bat = 0;
volatile float i_solar = 0, i_ac_in = 0, i_bat = 0;
volatile float v_bat_filt = 0, i_bat_filt = 0;
volatile float i_solar_mag = 0;
volatile float i_bat_charge_filt = 0;  // กระแสชาร์จใช้ค่าบวกเสมอเพื่อกันทิศเซนเซอร์กลับด้าน
volatile bool ovp_latched = false;
volatile float ovp_trip_voltage = 0.0;
volatile bool system_ON = false;
volatile bool charge_full_hold = false;
volatile int active_duty_percent = 0;
int raw_duty = 0;
float duty_accumulator = 0.0;          // เก็บ duty แบบทศนิยม เพื่อลด dead-zone จาก round()
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
float current_offset_i0 = OFFSET_I_SOLAR;
float current_offset_i1 = OFFSET_I_AC;
float current_offset_i2 = OFFSET_I_BAT;
float vbat_filter_buf[8] = {0};
float ibat_filter_buf[8] = {0};
float vbat_filter_sum = 0;
float ibat_filter_sum = 0;
int filter_index = 0;
int filter_count = 0;

void TaskSampleData(void * pvParameters);
void TaskLCDLoop(void * pvParameters);
void calibrateCurrentOffsetsAtBoot();

static inline int16_t readADCStable(Adafruit_ADS1115 &adc, uint8_t channel) {
    // Discard first conversion after channel switch to reduce mux settling artifacts.
    (void)adc.readADC_SingleEnded(channel);
    return adc.readADC_SingleEnded(channel);
}

static inline void disablePowerStage() {
    currentState = STATE_OFF;
    raw_duty = 0;
    duty_accumulator = 0.0;
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

void calibrateCurrentOffsetsAtBoot() {
    const int CAL_SAMPLES = 80;
    float sum_i0 = 0.0;
    float sum_i1 = 0.0;
    float sum_i2 = 0.0;

    // Calibrate zero-current baseline while power stage is disabled.
    disablePowerStage();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    for (int i = 0; i < CAL_SAMPLES; i++) {
        sum_i0 += readADCStable(ads_curr, 0) * 0.1875;
        sum_i1 += readADCStable(ads_curr, 1) * 0.1875;
        sum_i2 += readADCStable(ads_curr, 2) * 0.1875;
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
    Wire.setTimeOut(25);

    bool volt_ok = ads_volt.begin(0x48);
    bool curr_ok = ads_curr.begin(0x49);
    sensor_init_ok = (volt_ok && curr_ok);

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
        calibrateCurrentOffsetsAtBoot();
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
    const float MPPT_V_STEP = 0.15;
    unsigned long last_mppt_time = 0;

    unsigned long pv_collapse_start_time = 0;
    bool pv_is_collapsing = false;
    unsigned long last_debug_time = 0;
    unsigned long last_sensor_error_log = 0;
    unsigned long full_condition_start_ms = 0;

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
            raw_mv_v0 = readADCStable(ads_volt, 0) * 0.1875;
            raw_mv_v1 = readADCStable(ads_volt, 2) * 0.1875;
            raw_mv_v2 = readADCStable(ads_volt, 1) * 0.1875;

            raw_mv_i0 = readADCStable(ads_curr, 0) * 0.1875;
            raw_mv_i1 = readADCStable(ads_curr, 1) * 0.1875;
            raw_mv_i2 = readADCStable(ads_curr, 2) * 0.1875;

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

            // Hard OVP: treat high battery voltage as real event and cut power immediately.
            if (!ovp_latched &&
                (v_bat >= HARD_OVP_TRIP_VOLTAGE || v_bat_filt >= HARD_OVP_TRIP_VOLTAGE)) {
                ovp_latched = true;
                ovp_trip_voltage = max(v_bat, v_bat_filt);
                forceSafeShutdown();
                Serial.printf("[CRITICAL] HARD OVP TRIP at %.2fV (trip=%.2fV). Output disabled.\n",
                              ovp_trip_voltage, HARD_OVP_TRIP_VOLTAGE);
            }

            if (ovp_latched &&
                v_bat <= HARD_OVP_RELEASE_VOLTAGE &&
                v_bat_filt <= HARD_OVP_RELEASE_VOLTAGE) {
                ovp_latched = false;
                Serial.printf("[INFO] OVP latch cleared at %.2fV (release=%.2fV).\n",
                              max(v_bat, v_bat_filt), HARD_OVP_RELEASE_VOLTAGE);
            }

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

                    v_solar_old = v_solar;
                    p_solar_old = v_solar * i_solar_mag;
                    v_solar_target = v_solar - 1.0;
                    if (v_solar_target < BOOST_VOLTAGE_FLOOR) v_solar_target = BOOST_VOLTAGE_FLOOR;
                    pid_integral = 0; pid_last_error = 0;
                    pid_integral_cc = 0; pid_last_error_cc = 0;
                    pid_integral_cv = 0; pid_last_error_cv = 0;
                    raw_duty = 20;
                    duty_accumulator = 20.0;
                    pv_is_collapsing = false;
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
                // ☀️ โหมด PV: ระบบควบคุมแผงและระบบป้องกันฝั่งเอาต์พุตขั้นเด็ดขาด
                // =================================================================

                bool charge_is_active = (i_bat_charge_filt >= MIN_CURRENT_FOR_ACTIVE_CHARGE) ||
                                        (i_solar_mag >= MIN_CURRENT_FOR_ACTIVE_CHARGE);
                if ((v_bat_filt >= BOOST_BAT_VOLTAGE_LIMIT && charge_is_active) ||
                    (i_bat_charge_filt >= TARGET_CC_CURRENT)) {
                    duty_accumulator -= 5.0;
                    pid_integral = 0;
                }
                else if (v_solar == 0.0) {
                    duty_accumulator = 0.0;
                    pid_integral = 0;
                }
                else {
                    if ((now - last_mppt_time >= 100) && (i_solar_mag > 0.0)) {
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

                    pid_error = v_solar - v_solar_target;
                    if (v_solar < BOOST_VOLTAGE_FLOOR) {
                        pid_integral = 0;
                    } else {
                        pid_integral += pid_error;
                        pid_integral = constrain(pid_integral, -50, 50);
                    }
                    pid_derivative = pid_error - pid_last_error;

                    float pid_output = (Kp * pid_error) + (Ki * pid_integral) + (Kd * pid_derivative);

                    // ระบบแก้ล็อกช่วงเริ่มต้น (Soft-start ในโหมดแผง)
                    if (duty_accumulator < 30.0 && v_solar > BOOST_VOLTAGE_FLOOR) {
                        pid_output = 2.0;
                    } else {
                        if (pid_output > 1.5) pid_output = 1.5;
                    }

                    if (v_solar <= (BOOST_VOLTAGE_FLOOR - 0.5)) {
                        if (pid_output > 0) pid_output = 0;
                        if (v_solar <= (BOOST_VOLTAGE_FLOOR - 1.0)) pid_output = -5.0;
                        else if (pid_output < -2.0) pid_output = -2.0;
                    }

                    duty_accumulator += pid_output;
                    pid_last_error = pid_error;
                }
            }

            // Guardrail ตอนเริ่มและขณะบูสต์: รักษา I<=3A, Vpv>=42V และ Vbatt<=58V
            if (currentState == STATE_BOOST) {
                if (v_solar < BOOST_VOLTAGE_FLOOR) {
                    float v_under = BOOST_VOLTAGE_FLOOR - v_solar;
                    duty_accumulator -= (3.0 + (v_under * 3.5));
                }
                if (i_bat_charge_filt > TARGET_CC_CURRENT) {
                    float over_current = i_bat_charge_filt - TARGET_CC_CURRENT;
                    duty_accumulator -= (5.0 + (over_current * 3.0));
                    pid_integral = 0;
                }
                if (v_bat_filt > BOOST_BAT_VOLTAGE_LIMIT) {
                    float over_bat_v = v_bat_filt - BOOST_BAT_VOLTAGE_LIMIT;
                    duty_accumulator -= (5.0 + (over_bat_v * 10.0));
                    pid_integral = 0;
                }

                // จำกัดความเร็วขาขึ้นของ duty เมื่อเข้าใกล้ข้อจำกัด 42V/3A
                float duty_delta = duty_accumulator - duty_before_control;
                if (duty_delta > 0.0) {
                    bool near_v_limit = (v_solar <= (BOOST_VOLTAGE_FLOOR + BOOST_SAFE_V_HEADROOM));
                    bool near_i_limit = (i_bat_charge_filt >= (TARGET_CC_CURRENT - BOOST_SAFE_I_HEADROOM));
                    bool near_bat_limit = (v_bat_filt >= (BOOST_BAT_VOLTAGE_LIMIT - BOOST_SAFE_BAT_HEADROOM));
                    if (near_v_limit || near_i_limit || near_bat_limit) {
                        const float LIMITED_UP_STEP = 0.6;
                        if (duty_delta > LIMITED_UP_STEP) {
                            duty_accumulator = duty_before_control + LIMITED_UP_STEP;
                        }
                    }
                }
            }

            duty_accumulator = constrain(duty_accumulator, 0.0, (float)allowed_max_duty);
            raw_duty = (int)roundf(duty_accumulator);

            if (currentState == STATE_FORWARD) {
                ledcWrite(PWM_FORWARD_PIN, raw_duty);
                ledcWrite(PWM_BOOST_PIN, 0);
            } else if (currentState == STATE_BOOST) {
                ledcWrite(PWM_BOOST_PIN, raw_duty);
                ledcWrite(PWM_FORWARD_PIN, 0);
            }

            total_Wh += ((v_bat * i_bat_charge_filt) * (now - last_millis)) / 3600000.0;

            bool full_window_current = (i_bat_charge_filt >= NOISE_I_THRESHOLD) &&
                                       (i_bat_charge_filt <= FULL_END_CURRENT);
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
        } else {
            ledcWrite(PWM_FORWARD_PIN, 0);
            ledcWrite(PWM_BOOST_PIN, 0);
            full_condition_start_ms = 0;
        }

        last_millis = now;
        active_duty_percent = round(((float)raw_duty * 100.0) / 1023.0);

        if (now - last_debug_time >= 500) {
            last_debug_time = now;
            const char* state_label = charge_full_hold
                ? "FULL_HOLD"
                : (ovp_latched
                    ? "OVP_LOCK"
                    : (currentState == STATE_BOOST ? "BOOST" : (currentState == STATE_FORWARD ? "FORWARD" : "OFF")));
            Serial.println("=========================================================================================");
            Serial.printf("[DEBUG INTERFACE] System: %s | State: %s | Active Duty: %d%%\n",
                          (system_ON ? "ON " : "OFF"),
                          state_label,
                          active_duty_percent);
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

        if (xSemaphoreTake(i2c_Mutex, 50)) {
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
            } else if (ovp_latched || show_ovp_alert) {
                lcd.setCursor(0, 0); lcd.print("    OVP TRIPPED      ");
                lcd.setCursor(0, 1); lcd.printf("VBAT:%5.1fV TRIP:%4.1f", v_bat_filt, ovp_trip_voltage);
                lcd.setCursor(0, 2); lcd.printf("REL <= %5.1fV        ", HARD_OVP_RELEASE_VOLTAGE);
                lcd.setCursor(0, 3); lcd.print("WAIT VOLTAGE DROP    ");
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

        // กู้ LCD เฉพาะเมื่อมีอาการค้างจริง ไม่ init ทุกคาบเวลา
        if (lcd_mutex_fail_count >= 5 && (now - last_lcd_recover > 2000)) {
            last_lcd_recover = now;
            if (xSemaphoreTake(i2c_Mutex, 50)) {
                lcd.init();
                lcd.backlight();
                lcd.clear();
                xSemaphoreGive(i2c_Mutex);
                lcd_mutex_fail_count = 0;
            }
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
