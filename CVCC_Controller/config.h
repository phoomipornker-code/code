#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Every value here has a one-to-one counterpart in the Simulink diagram.
 * Change the numbers to match your own model; the code does not need edits.
 * ------------------------------------------------------------------------- */

/* Discrete solver step. Drives both the Zero-Order Hold blocks on the two
 * feedback signals and the Ts used by the K*Ts/(z-1) integrators. */
constexpr unsigned long TS_MICROS = 1000UL;
constexpr float TS = 1.0e-6f * (float)TS_MICROS;

/* Constant blocks feeding the two error summing junctions. */
constexpr float V_REF = 58.0f; /* volts */
constexpr float I_REF = 5.0f;  /* amps  */

/* Gain blocks of the constant-voltage branch. */
constexpr float KP_V = 0.1f;
constexpr float KI_V = 0.0f;

/* Gain blocks of the constant-current branch. */
constexpr float KP_I = 1.0f;
constexpr float KI_I = 0.0f;

/* Saturation block sitting on each proportional path. The diagram does not
 * reveal its limits, so these mirror the duty-cycle domain the branch feeds. */
constexpr float P_SAT_MIN = -1.0f;
constexpr float P_SAT_MAX = 1.0f;

/* Clamp applied to the discrete integrator state (anti-windup). A plain
 * Simulink integrator has no limits, but an unbounded state on real hardware
 * takes seconds to unwind after a fault. */
constexpr float I_SAT_MIN = -1.0f;
constexpr float I_SAT_MAX = 1.0f;

/* Saturation block after the min selector, i.e. the legal duty-cycle range. */
constexpr float DUTY_MIN = 0.0f;
constexpr float DUTY_MAX = 1.0f;

/* --- Sensor scaling: engineering units = (adcVolts - offset) * gain --------
 * V_SENSE_GAIN is the inverse of the resistor divider ratio, e.g. 150k/10k
 * divides by 16. I_SENSE assumes a bidirectional hall sensor such as the
 * ACS712-20A: 2.5 V at zero current, 100 mV per amp. */
constexpr float V_SENSE_OFFSET_V = 0.0f;
constexpr float V_SENSE_GAIN = 16.0f;
constexpr float I_SENSE_OFFSET_V = 2.5f;
constexpr float I_SENSE_GAIN = 10.0f;

constexpr float ADC_VREF = 5.0f;
constexpr float ADC_COUNTS = 1023.0f;
constexpr uint8_t ADC_OVERSAMPLE = 4;

#if defined(ARDUINO)

constexpr uint8_t V_SENSE_PIN = A0; /* replaces the [V_OUT] from-tag */
constexpr uint8_t I_SENSE_PIN = A1; /* replaces the [I_OUT] from-tag */

/* Carrier frequency of the D->P block. The output pin is not a free choice, so
 * pwm.h derives it from the board instead of listing it here. */
constexpr unsigned long PWM_FREQ_HZ = 20000UL;

/* Stand-in for the Display block: how often the loop reports over serial. */
constexpr unsigned long TELEMETRY_PERIOD_MS = 100UL;
constexpr unsigned long SERIAL_BAUD = 115200UL;

#endif /* ARDUINO */

#endif /* CONFIG_H */
