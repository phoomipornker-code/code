/*
  Arduino Battery Charger Controller

  This sketch controls a low-voltage DC charger stage with a PWM-driven
  MOSFET/driver. It is intended for learning and prototyping. Do not connect an
  Arduino directly to mains power or to a battery pack without a proper charger
  power stage, fuse, thermal protection, and battery-specific safety hardware.
*/

enum ChargerState {
  IDLE,
  BULK_CHARGE,
  TAPER_CHARGE,
  COMPLETE,
  FAULT
};

// ---------------------- Pin configuration ----------------------
const byte PWM_PIN = 9;
const byte ENABLE_PIN = 8;
const byte STATUS_LED_PIN = 13;
const byte BATTERY_VOLTAGE_PIN = A0;
const byte CHARGE_CURRENT_PIN = A1;
const byte TEMPERATURE_PIN = A2;

// ---------------------- Battery configuration ----------------------
// Defaults are for a single-cell Li-ion battery. Adjust for your battery.
const float TARGET_VOLTAGE = 4.20;
const float MAX_VOLTAGE = 4.25;
const float RESTART_VOLTAGE = 4.05;
const float BULK_CURRENT_MA = 1000.0;
const float TAPER_CURRENT_MA = 150.0;
const float CURRENT_TOLERANCE_MA = 25.0;
const float MIN_TEMPERATURE_C = 0.0;
const float MAX_TEMPERATURE_C = 45.0;
const unsigned long COMPLETE_HOLD_MS = 30000UL;

// ---------------------- Sensor calibration ----------------------
const float ADC_REFERENCE_V = 5.0;
const float ADC_MAX = 1023.0;

// Voltage divider: battery positive -> R1 -> A0 -> R2 -> GND.
const float VOLTAGE_DIVIDER_R1 = 30000.0;
const float VOLTAGE_DIVIDER_R2 = 10000.0;

// ACS712-05B style current sensor defaults. Calibrate for your module.
const float CURRENT_SENSOR_ZERO_V = 2.50;
const float CURRENT_SENSOR_MV_PER_AMP = 185.0;

// 10k NTC thermistor defaults.
const float THERMISTOR_SERIES_RESISTOR = 10000.0;
const float THERMISTOR_NOMINAL = 10000.0;
const float THERMISTOR_NOMINAL_TEMP_C = 25.0;
const float THERMISTOR_BETA = 3950.0;

// ---------------------- Control loop configuration ----------------------
const unsigned long SAMPLE_INTERVAL_MS = 250UL;
const byte PWM_MIN = 0;
const byte PWM_MAX = 255;
const byte PWM_STEP = 2;

ChargerState state = IDLE;
byte pwmValue = 0;
unsigned long lastSampleMs = 0;
unsigned long taperStartMs = 0;

float batteryVoltage = 0.0;
float chargeCurrentMa = 0.0;
float temperatureC = 25.0;

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);

  Serial.begin(9600);
  disableOutput();
  state = BULK_CHARGE;

  Serial.println(F("Arduino battery charger controller started"));
}

void loop() {
  unsigned long now = millis();
  if (now - lastSampleMs < SAMPLE_INTERVAL_MS) {
    return;
  }
  lastSampleMs = now;

  readSensors();
  updateSafetyState();
  updateChargerState(now);
  applyControl();
  printStatus();
}

void readSensors() {
  batteryVoltage = readBatteryVoltage();
  chargeCurrentMa = readChargeCurrentMa();
  temperatureC = readTemperatureC();
}

float readBatteryVoltage() {
  int raw = analogRead(BATTERY_VOLTAGE_PIN);
  float pinVoltage = raw * ADC_REFERENCE_V / ADC_MAX;
  float dividerRatio = (VOLTAGE_DIVIDER_R1 + VOLTAGE_DIVIDER_R2) / VOLTAGE_DIVIDER_R2;
  return pinVoltage * dividerRatio;
}

float readChargeCurrentMa() {
  int raw = analogRead(CHARGE_CURRENT_PIN);
  float sensorVoltage = raw * ADC_REFERENCE_V / ADC_MAX;
  float amps = (sensorVoltage - CURRENT_SENSOR_ZERO_V) / (CURRENT_SENSOR_MV_PER_AMP / 1000.0);
  return max(0.0, amps * 1000.0);
}

float readTemperatureC() {
  int raw = analogRead(TEMPERATURE_PIN);
  if (raw <= 0 || raw >= 1023) {
    return 999.0;
  }

  float resistance = THERMISTOR_SERIES_RESISTOR / ((ADC_MAX / raw) - 1.0);
  float steinhart = resistance / THERMISTOR_NOMINAL;
  steinhart = log(steinhart);
  steinhart /= THERMISTOR_BETA;
  steinhart += 1.0 / (THERMISTOR_NOMINAL_TEMP_C + 273.15);
  steinhart = 1.0 / steinhart;
  return steinhart - 273.15;
}

void updateSafetyState() {
  if (batteryVoltage <= 0.1 || batteryVoltage > MAX_VOLTAGE) {
    setFault(F("Battery voltage is outside safe range"));
    return;
  }

  if (temperatureC < MIN_TEMPERATURE_C || temperatureC > MAX_TEMPERATURE_C) {
    setFault(F("Battery temperature is outside safe range"));
  }
}

void updateChargerState(unsigned long now) {
  if (state == FAULT) {
    return;
  }

  if (state == COMPLETE) {
    if (batteryVoltage <= RESTART_VOLTAGE) {
      state = BULK_CHARGE;
      taperStartMs = 0;
    }
    return;
  }

  if (batteryVoltage >= TARGET_VOLTAGE) {
    if (state != TAPER_CHARGE) {
      state = TAPER_CHARGE;
      taperStartMs = now;
    }

    if (chargeCurrentMa <= TAPER_CURRENT_MA && now - taperStartMs >= COMPLETE_HOLD_MS) {
      state = COMPLETE;
    }
    return;
  }

  state = BULK_CHARGE;
}

void applyControl() {
  switch (state) {
    case BULK_CHARGE:
      enableOutput();
      regulateCurrent(BULK_CURRENT_MA);
      break;
    case TAPER_CHARGE:
      enableOutput();
      regulateVoltage(TARGET_VOLTAGE);
      break;
    case COMPLETE:
    case IDLE:
    case FAULT:
      disableOutput();
      break;
  }

  analogWrite(PWM_PIN, pwmValue);
  digitalWrite(STATUS_LED_PIN, state == FAULT ? HIGH : LOW);
}

void regulateCurrent(float targetCurrentMa) {
  if (chargeCurrentMa < targetCurrentMa - CURRENT_TOLERANCE_MA) {
    increasePwm();
  } else if (chargeCurrentMa > targetCurrentMa + CURRENT_TOLERANCE_MA) {
    decreasePwm();
  }
}

void regulateVoltage(float targetVoltage) {
  if (batteryVoltage < targetVoltage && chargeCurrentMa > TAPER_CURRENT_MA) {
    increasePwm();
  } else {
    decreasePwm();
  }
}

void increasePwm() {
  pwmValue = min(PWM_MAX, pwmValue + PWM_STEP);
}

void decreasePwm() {
  if (pwmValue <= PWM_STEP) {
    pwmValue = PWM_MIN;
  } else {
    pwmValue -= PWM_STEP;
  }
}

void enableOutput() {
  digitalWrite(ENABLE_PIN, HIGH);
}

void disableOutput() {
  pwmValue = 0;
  analogWrite(PWM_PIN, pwmValue);
  digitalWrite(ENABLE_PIN, LOW);
}

void setFault(const __FlashStringHelper *message) {
  if (state != FAULT) {
    Serial.print(F("FAULT: "));
    Serial.println(message);
  }
  state = FAULT;
}

void printStatus() {
  Serial.print(F("state="));
  Serial.print(stateName(state));
  Serial.print(F(" voltage="));
  Serial.print(batteryVoltage, 3);
  Serial.print(F("V current="));
  Serial.print(chargeCurrentMa, 1);
  Serial.print(F("mA temperature="));
  Serial.print(temperatureC, 1);
  Serial.print(F("C pwm="));
  Serial.println(pwmValue);
}

const char *stateName(ChargerState chargerState) {
  switch (chargerState) {
    case IDLE:
      return "idle";
    case BULK_CHARGE:
      return "bulk";
    case TAPER_CHARGE:
      return "taper";
    case COMPLETE:
      return "complete";
    case FAULT:
      return "fault";
  }
  return "unknown";
}
