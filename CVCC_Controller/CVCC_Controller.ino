/*
 * Constant-voltage / constant-current controller, ported from a Simulink block
 * diagram. It regulates a 67 kHz forward converter running from 155 V that
 * charges a battery at up to 58 V and 5 A.
 *
 * Signal path, left to right in the model:
 *
 *   58 V ---(-)--- ZOH [V_OUT] --> Kp=0.1 -> sat -+
 *                                 Ki -> K*Ts/(z-1) -(-)-> [CV] --+
 *                                                                (min) -> sat -> D->P -> [PWM]
 *    5 A ---(-)--- ZOH [I_OUT] --> Kp=1.0 -> sat -+               |
 *                                 Ki -> K*Ts/(z-1) -(-)-> [CC] --+
 *
 * Every sat block limits to 0 .. 0.41, so 0.41 is the maximum duty and also the
 * value the Display block reads whenever a branch is railed.
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

/* Worst-case time one control step has taken. Compare it against TS_MICROS to
 * see how much headroom the board really has. */
static uint16_t peakStepUs = 0;

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
  const unsigned long entered = micros();

  vOut = (readVolts(V_SENSE_PIN) - V_SENSE_OFFSET_V) * V_SENSE_GAIN;
  iOut = (readVolts(I_SENSE_PIN) - I_SENSE_OFFSET_V) * I_SENSE_GAIN;

  result = controller.update(vOut, iOut);

  pwmWriteDuty(result.duty);

  const uint16_t elapsed = (uint16_t)(micros() - entered);
  if (elapsed > peakStepUs) peakStepUs = elapsed;
}

/* Stand-in for the Display block. Rows are kept short enough to fit the 64-byte
 * serial buffer, so printing never stalls the control loop. */
static void reportTelemetry() {
  const unsigned long now = millis();
  if ((long)(now - nextPrintMs) < 0) return;
  nextPrintMs = now + TELEMETRY_PERIOD_MS;

  Serial.print(vOut, 2);
  Serial.print(',');
  Serial.print(iOut, 3);
  Serial.print(',');
  Serial.print(result.cv, 3);
  Serial.print(',');
  Serial.print(result.cc, 3);
  Serial.print(',');
  Serial.print(result.duty, 3);
  Serial.print(controller.currentLimited ? F(",CC,") : F(",CV,"));
  Serial.println(peakStepUs);
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  /* Bring the gate low before anything else so a reset cannot leave the
   * converter latched at whatever duty it was running at. */
  pwmBegin();
  adcSpeedUp();
  controller.begin();

  Serial.print(F("# pwm "));
  Serial.print(pwmActualFrequencyHz());
  Serial.print(F(" Hz, "));
  Serial.print(pwmDutySteps());
  Serial.print(F(" steps, step budget "));
  Serial.print(TS_MICROS);
  Serial.println(F(" us"));
  Serial.println(F("# V,I,CV,CC,Duty,mode,peakStepUs"));

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
