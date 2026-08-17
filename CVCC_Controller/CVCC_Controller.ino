/*
 * Constant-voltage / constant-current controller, ported from a Simulink block
 * diagram. Signal path, left to right in the model:
 *
 *   58 V ---(-)--- ZOH [V_OUT] --> Kp=0.1 -> sat -+
 *                                 Ki -> K*Ts/(z-1) +--> [CV] --+
 *                                                              (min) -> sat -> D->P -> [PWM]
 *    5 A ---(-)--- ZOH [I_OUT] --> Kp=1.0 -> sat -+             |
 *                                 Ki -> K*Ts/(z-1) +--> [CC] --+
 *
 * All tunables live in config.h.
 */

#include "CVCCController.h"
#include "config.h"
#include "pwm.h"

static CVCCController controller;

static unsigned long nextStepUs = 0;
static unsigned long nextPrintMs = 0;

static float vOut = 0.0f;
static float iOut = 0.0f;
static CVCCOutput result = {0.0f, 0.0f, 0.0f};

static float readVolts(uint8_t pin) {
  uint32_t accumulator = 0;
  for (uint8_t sample = 0; sample < ADC_OVERSAMPLE; ++sample) {
    accumulator += analogRead(pin);
  }
  const float counts = (float)accumulator / (float)ADC_OVERSAMPLE;
  return counts * (ADC_VREF / ADC_COUNTS);
}

/* One tick of the discrete model: sample, solve, actuate. */
static void controlStep() {
  vOut = (readVolts(V_SENSE_PIN) - V_SENSE_OFFSET_V) * V_SENSE_GAIN;
  iOut = (readVolts(I_SENSE_PIN) - I_SENSE_OFFSET_V) * I_SENSE_GAIN;

  result = controller.update(vOut, iOut);

  pwmWriteDuty(result.duty);
}

static void reportTelemetry() {
  const unsigned long now = millis();
  if ((long)(now - nextPrintMs) < 0) return;
  nextPrintMs = now + TELEMETRY_PERIOD_MS;

  Serial.print(F("V="));
  Serial.print(vOut, 2);
  Serial.print(F(" I="));
  Serial.print(iOut, 3);
  Serial.print(F(" CV="));
  Serial.print(result.cv, 3);
  Serial.print(F(" CC="));
  Serial.print(result.cc, 3);
  Serial.print(F(" Duty="));
  Serial.print(result.duty, 3);
  Serial.println(controller.currentLimited ? F(" mode=CC") : F(" mode=CV"));
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  /* Bring the gate low before anything else so a reset cannot leave the
   * converter latched at whatever duty it was running at. */
  pwmBegin();
  adcSpeedUp();
  controller.begin();

  nextStepUs = micros();
  nextPrintMs = millis();
}

void loop() {
  const unsigned long now = micros();

  if ((long)(now - nextStepUs) >= 0) {
    nextStepUs += TS_MICROS;

    /* If a step was missed the schedule is behind reality; realign instead of
     * running a burst of catch-up steps with a wrong effective Ts. */
    if ((long)(now - nextStepUs) >= 0) {
      nextStepUs = now + TS_MICROS;
    }

    controlStep();
  }

  reportTelemetry();
}
