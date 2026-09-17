#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

// Forward-only charger controller derived from the supplied project.
// Outer voltage PI produces a 0..5 A current request.
// Inner current PI produces a 0..45% PWM duty request.

const char *FW_VERSION_TAG = "forward-cascade-pi-v1";

// Hardware
const int RELAY_PV_PIN     = 32;
const int RELAY_AC_PIN     = 33;
const int BUTTON_START_PIN = 25;
const int BUTTON_STOP_PIN  = 26;
const int PWM_FORWARD_PIN  = 14;
const int PWM_BOOST_PIN    = 27;
const int PWM_FORWARD_FREQ = 67000;
const int PWM_BOOST_FREQ   = 50000;
const int PWM_RES          = 10;
const int PWM_FORWARD_CHANNEL = 0;
const int PWM_BOOST_CHANNEL   = 2; // Separate timer from channel 0
const int PWM_FULL_SCALE   = (1 << PWM_RES) - 1; // 1023

Adafruit_ADS1115 ads_volt;
Adafruit_ADS1115 ads_curr;
LiquidCrystal_I2C lcd(0x27, 20, 4);
SemaphoreHandle_t i2cMutex;

// Charging target and limits
// Kept at 56 V so it remains below the supplied 56.8 V stop and 57.8 V OVP.
// Do not change this to 58.4 V without validating the battery, BMS and OVP.
const float FORWARD_VREF_V        = 56.0f;
const float FORWARD_CURRENT_MAX_A = 5.0f;
const float FORWARD_DUTY_MAX      = 0.45f;
const int MAX_DUTY_FORWARD        = 460; // round(1023 * 0.45)

const float MIN_AC_VOLTAGE             = 140.0f;
const float HIGH_VOLTAGE_STOP_VOLTAGE  = 56.80f;
const float HARD_OVP_TRIP_VOLTAGE      = 57.80f;
const float HARD_OVP_RELEASE_VOLTAGE   = 55.80f;
const float RESTART_CHARGE_VOLTAGE     = 54.0f;
const unsigned long OVP_RELEASE_MS     = 2500;
const unsigned long ADC_STALE_MS       = 700;

// Simulink gains
const float FORWARD_VOLT_KP = 15.0f;
const float FORWARD_VOLT_KI = 1.0f;
const float FORWARD_CURR_KP = 0.5f;
const float FORWARD_CURR_KI = 23.0f;

// Hardware duty slew limits, in 10-bit PWM counts per control update
const float FORWARD_SLEW_UP_RAW   = 3.0f;
const float FORWARD_SLEW_DOWN_RAW = 12.0f;

// Calibration copied from the supplied program
const float OFFSET_V_AC         = 0.0f;
const float OFFSET_V_BAT        = 0.0f;
const float OFFSET_I_AC         = 1646.9f;
const float OFFSET_I_BAT        = 1646.9f;
const float CAL_SCALE_V_AC      = 71.43f;
const float CAL_SCALE_V_BAT     = 41.85f;
const float CAL_SCALE_I_AC      = 42.46f;
const float CAL_SCALE_I_BAT     = 42.46f;
const float NOISE_V_THRESHOLD   = 0.5f;
const float NOISE_I_THRESHOLD   = 0.08f;
const uint32_t I2C_CLOCK_HZ     = 100000;

volatile float v_ac_in = 0.0f;
volatile float v_bat = 0.0f;
volatile float i_ac_in = 0.0f;
volatile float i_bat = 0.0f;
volatile float v_bat_filt = 0.0f;
volatile float i_bat_charge_filt = 0.0f;
volatile bool sensor_init_ok = false;
volatile bool system_ON = false;
volatile bool charge_full_hold = false;
volatile bool ovp_latched = false;
volatile int active_duty_percent = 0;

float currentOffsetAc = OFFSET_I_AC;
float currentOffsetBat = OFFSET_I_BAT;
float forwardVoltIntegrator = 0.0f;
float forwardCurrIntegrator = 0.0f;
float forwardCurrentRef = 0.0f;
float dutyAccumulator = 0.0f;
unsigned long forwardLastControlUs = 0;
unsigned long lastAdcSampleMs = 0;
unsigned long ovpTripMs = 0;

float vbatFilter[8] = {0};
float ibatFilter[8] = {0};
float vbatFilterSum = 0.0f;
float ibatFilterSum = 0.0f;
int filterIndex = 0;
int filterCount = 0;

static inline float clampValue(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline float applySlew(float target, float current,
                              float upStep, float downStep) {
    float change = target - current;
    if (change > upStep) return current + upStep;
    if (change < -downStep) return current - downStep;
    return target;
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

static void resetForwardController() {
    forwardVoltIntegrator = 0.0f;
    forwardCurrIntegrator = 0.0f;
    forwardCurrentRef = 0.0f;
    dutyAccumulator = 0.0f;
    forwardLastControlUs = 0;
}

static void disablePowerStage() {
    resetForwardController();
    digitalWrite(RELAY_PV_PIN, LOW);
    digitalWrite(RELAY_AC_PIN, LOW);
    ledcWrite(PWM_FORWARD_PIN, 0);
    ledcWrite(PWM_BOOST_PIN, 0);
    active_duty_percent = 0;
}

static void resetForwardForStart() {
    resetForwardController();
    dutyAccumulator = 10.0f;
}

static float forwardCascadePI(float vOut, float iOut, float dt) {
    dt = clampValue(dt, 0.005f, 0.100f);

    // Outer voltage loop: CV is a current request in amperes (0..5 A).
    float voltageError = FORWARD_VREF_V - vOut;
    float cvUnclamped =
        FORWARD_VOLT_KP * voltageError + forwardVoltIntegrator;
    forwardCurrentRef =
        clampValue(cvUnclamped, 0.0f, FORWARD_CURRENT_MAX_A);

    // Conditional anti-windup. Saturation still matches the model output.
    bool voltageCanIntegrate =
        (cvUnclamped > 0.0f && cvUnclamped < FORWARD_CURRENT_MAX_A) ||
        (cvUnclamped >= FORWARD_CURRENT_MAX_A && voltageError < 0.0f) ||
        (cvUnclamped <= 0.0f && voltageError > 0.0f);
    if (voltageCanIntegrate) {
        forwardVoltIntegrator += FORWARD_VOLT_KI * voltageError * dt;
    }

    // Inner current loop: CC is normalized PWM duty (0..0.45).
    float currentError = forwardCurrentRef - iOut;
    float dutyUnclamped =
        FORWARD_CURR_KP * currentError + forwardCurrIntegrator;
    float duty =
        clampValue(dutyUnclamped, 0.0f, FORWARD_DUTY_MAX);

    bool currentCanIntegrate =
        (dutyUnclamped > 0.0f && dutyUnclamped < FORWARD_DUTY_MAX) ||
        (dutyUnclamped >= FORWARD_DUTY_MAX && currentError < 0.0f) ||
        (dutyUnclamped <= 0.0f && currentError > 0.0f);
    if (currentCanIntegrate) {
        forwardCurrIntegrator += FORWARD_CURR_KI * currentError * dt;
    }

    return duty;
}

static void calibrateCurrentOffsetsAtBoot() {
    const int samples = 80;
    float sumAc = 0.0f;
    float sumBat = 0.0f;

    disablePowerStage();
    vTaskDelay(pdMS_TO_TICKS(100));

    for (int i = 0; i < samples; ++i) {
        sumAc += readADCStable(ads_curr, 1, true) * 0.1875f;
        sumBat += readADCStable(ads_curr, 2, true) * 0.1875f;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    currentOffsetAc = sumAc / samples;
    currentOffsetBat = sumBat / samples;
    Serial.printf("[CAL] Iac=%.2fmV Ibat=%.2fmV\n",
                  currentOffsetAc, currentOffsetBat);
}

static void updateMeasurements() {
    float rawVac = readADCStable(ads_volt, 2, true) * 0.1875f;
    float rawVbat = readADCStable(ads_volt, 1, true) * 0.1875f;
    float rawIac = readADCStable(ads_curr, 1) * 0.1875f;
    float rawIbat = readADCStable(ads_curr, 2) * 0.1875f;

    float pureVac = fmaxf(0.0f, rawVac - OFFSET_V_AC);
    float pureVbat = fmaxf(0.0f, rawVbat - OFFSET_V_BAT);

    v_ac_in = (pureVac / 1000.0f) * CAL_SCALE_V_AC;
    v_bat = (pureVbat / 1000.0f) * CAL_SCALE_V_BAT;
    i_ac_in = ((rawIac - currentOffsetAc) / 1000.0f) * CAL_SCALE_I_AC;
    i_bat = ((rawIbat - currentOffsetBat) / 1000.0f) * CAL_SCALE_I_BAT;

    if (v_ac_in < NOISE_V_THRESHOLD) v_ac_in = 0.0f;
    if (v_bat < NOISE_V_THRESHOLD) v_bat = 0.0f;
    if (fabsf(i_ac_in) < NOISE_I_THRESHOLD) i_ac_in = 0.0f;
    if (fabsf(i_bat) < NOISE_I_THRESHOLD) i_bat = 0.0f;

    vbatFilterSum -= vbatFilter[filterIndex];
    ibatFilterSum -= ibatFilter[filterIndex];
    vbatFilter[filterIndex] = v_bat;
    ibatFilter[filterIndex] = fabsf(i_bat);
    vbatFilterSum += vbatFilter[filterIndex];
    ibatFilterSum += ibatFilter[filterIndex];

    filterIndex = (filterIndex + 1) % 8;
    if (filterCount < 8) ++filterCount;
    v_bat_filt = vbatFilterSum / filterCount;
    i_bat_charge_filt = ibatFilterSum / filterCount;
    lastAdcSampleMs = millis();
}

static void controlTask(void *parameter) {
    unsigned long lastDebugMs = 0;

    for (;;) {
        unsigned long nowMs = millis();

        if (!sensor_init_ok) {
            system_ON = false;
            disablePowerStage();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        bool sampled = false;
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            updateMeasurements();
            xSemaphoreGive(i2cMutex);
            sampled = true;
        }

        if (!sampled && nowMs - lastAdcSampleMs > ADC_STALE_MS) {
            Serial.println("[FAULT] ADC timeout");
            system_ON = false;
            disablePowerStage();
        }

        if (!ovp_latched &&
            (v_bat >= HARD_OVP_TRIP_VOLTAGE ||
             v_bat_filt >= HARD_OVP_TRIP_VOLTAGE)) {
            ovp_latched = true;
            ovpTripMs = nowMs;
            system_ON = false;
            charge_full_hold = true;
            disablePowerStage();
            Serial.printf("[FAULT] OVP %.2fV\n", fmaxf(v_bat, v_bat_filt));
        }

        if (ovp_latched &&
            nowMs - ovpTripMs >= OVP_RELEASE_MS &&
            v_bat < HARD_OVP_RELEASE_VOLTAGE &&
            v_bat_filt < HARD_OVP_RELEASE_VOLTAGE) {
            ovp_latched = false;
            Serial.println("[INFO] OVP released");
        }

        if (charge_full_hold &&
            v_bat_filt <= RESTART_CHARGE_VOLTAGE &&
            v_ac_in >= MIN_AC_VOLTAGE &&
            !ovp_latched) {
            charge_full_hold = false;
            Serial.println("[INFO] Charge restart available");
        }

        if (system_ON && !charge_full_hold && !ovp_latched) {
            if (v_ac_in < MIN_AC_VOLTAGE) {
                Serial.println("[FAULT] AC input low");
                system_ON = false;
                disablePowerStage();
            } else {
                digitalWrite(RELAY_PV_PIN, LOW);
                digitalWrite(RELAY_AC_PIN, HIGH);
                ledcWrite(PWM_BOOST_PIN, 0);

                unsigned long nowUs = micros();
                float dt = 0.020f;
                if (forwardLastControlUs != 0) {
                    dt = (float)(nowUs - forwardLastControlUs) * 1.0e-6f;
                }
                forwardLastControlUs = nowUs;

                float duty = forwardCascadePI(
                    v_bat_filt,
                    i_bat_charge_filt,
                    dt
                );

                float dutyTargetRaw = duty * PWM_FULL_SCALE;
                dutyAccumulator = applySlew(
                    dutyTargetRaw,
                    dutyAccumulator,
                    FORWARD_SLEW_UP_RAW,
                    FORWARD_SLEW_DOWN_RAW
                );

                // Independent current safety override.
                if (i_bat_charge_filt > FORWARD_CURRENT_MAX_A + 0.25f) {
                    dutyAccumulator -=
                        8.0f +
                        (i_bat_charge_filt - FORWARD_CURRENT_MAX_A) * 8.0f;
                    forwardCurrIntegrator *= 0.8f;
                }

                dutyAccumulator =
                    clampValue(dutyAccumulator, 0.0f, MAX_DUTY_FORWARD);
                int rawDuty = (int)lroundf(dutyAccumulator);
                ledcWrite(PWM_FORWARD_PIN, rawDuty);
                active_duty_percent =
                    (int)lroundf(rawDuty * 100.0f / PWM_FULL_SCALE);

                if (v_bat_filt >= HIGH_VOLTAGE_STOP_VOLTAGE) {
                    charge_full_hold = true;
                    disablePowerStage();
                    Serial.printf("[INFO] High-voltage stop %.2fV\n",
                                  v_bat_filt);
                }
            }
        } else {
            disablePowerStage();
        }

        if (nowMs - lastDebugMs >= 500) {
            lastDebugMs = nowMs;
            Serial.printf(
                "[%s] Vac=%.1fV Vbat=%.2fV Ibat=%.2fA "
                "CV=%.2fA Duty=%d%%\n",
                system_ON ? "RUN" : (charge_full_hold ? "HOLD" : "OFF"),
                v_ac_in,
                v_bat_filt,
                i_bat_charge_filt,
                forwardCurrentRef,
                active_duty_percent
            );
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void lcdTask(void *parameter) {
    bool previousStart = HIGH;
    bool previousStop = HIGH;
    unsigned long lastRefreshMs = 0;

    for (;;) {
        bool start = digitalRead(BUTTON_START_PIN);
        bool stop = digitalRead(BUTTON_STOP_PIN);

        if (stop == LOW && previousStop == HIGH) {
            system_ON = false;
            charge_full_hold = false;
            disablePowerStage();
        }

        if (start == LOW && previousStart == HIGH) {
            if (!ovp_latched &&
                sensor_init_ok &&
                v_ac_in >= MIN_AC_VOLTAGE) {
                resetForwardForStart();
                charge_full_hold = false;
                system_ON = true;
            }
        }

        previousStart = start;
        previousStop = stop;

        unsigned long now = millis();
        if (now - lastRefreshMs >= 200 &&
            xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            lastRefreshMs = now;
            lcd.setCursor(0, 0);
            if (ovp_latched) {
                lcd.printf("%-20s", "OVP TRIPPED");
            } else if (charge_full_hold) {
                lcd.printf("%-20s", "BATTERY FULL HOLD");
            } else if (system_ON) {
                lcd.printf("FORWARD DUTY:%3d%%  ", active_duty_percent);
            } else {
                lcd.printf("%-20s", "STANDBY");
            }

            lcd.setCursor(0, 1);
            lcd.printf("AC :%5.1fV %5.1fA ", v_ac_in, i_ac_in);
            lcd.setCursor(0, 2);
            lcd.printf("BAT:%5.1fV %5.2fA ", v_bat_filt,
                       i_bat_charge_filt);
            lcd.setCursor(0, 3);
            lcd.printf("CV :%4.2fA MAX:5.0A", forwardCurrentRef);
            xSemaphoreGive(i2cMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void setup() {
    Serial.begin(115200);
    Serial.printf("[BOOT] %s\n", FW_VERSION_TAG);

    pinMode(RELAY_PV_PIN, OUTPUT);
    pinMode(RELAY_AC_PIN, OUTPUT);
    pinMode(BUTTON_START_PIN, INPUT_PULLUP);
    pinMode(BUTTON_STOP_PIN, INPUT_PULLUP);

    bool forwardPwmOk = ledcAttachChannel(
        PWM_FORWARD_PIN, PWM_FORWARD_FREQ, PWM_RES, PWM_FORWARD_CHANNEL
    );
    bool boostPwmOk = ledcAttachChannel(
        PWM_BOOST_PIN, PWM_BOOST_FREQ, PWM_RES, PWM_BOOST_CHANNEL
    );
    if (!forwardPwmOk || !boostPwmOk) {
        Serial.println("[FATAL] LEDC PWM initialization failed");
    }
    disablePowerStage();

    Wire.begin(21, 22);
    Wire.setClock(I2C_CLOCK_HZ);
    Wire.setTimeOut(25);

    bool voltOk = ads_volt.begin(0x48);
    bool currOk = ads_curr.begin(0x49);
    sensor_init_ok = voltOk && currOk && forwardPwmOk && boostPwmOk;

    i2cMutex = xSemaphoreCreateMutex();
    if (i2cMutex == NULL) {
        sensor_init_ok = false;
    }

    lcd.init();
    lcd.backlight();
    lcd.clear();

    if (sensor_init_ok) {
        ads_volt.setDataRate(RATE_ADS1115_860SPS);
        ads_curr.setDataRate(RATE_ADS1115_860SPS);
        calibrateCurrentOffsetsAtBoot();
        lastAdcSampleMs = millis();
    } else {
        Serial.println("[FATAL] ADS1115 initialization failed");
        lcd.setCursor(0, 0);
        lcd.print("ADS1115 ERROR");
    }

    xTaskCreatePinnedToCore(
        controlTask, "ForwardControl", 6144, NULL, 2, NULL, 0
    );
    xTaskCreatePinnedToCore(
        lcdTask, "DisplayButtons", 4096, NULL, 1, NULL, 1
    );
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
