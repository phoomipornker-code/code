#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Every value here has a one-to-one counterpart in the Simulink diagram.
 * Change the numbers to match your own model; the code does not need edits.
 *
 * Power stage the model drives: a forward converter switching at 67 kHz, fed
 * from 155 V, charging a battery at up to 58 V and 5 A.
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

/* The summing junction that recombines the proportional and integral paths is a
 * "+-" block, so the integral term is subtracted rather than added. Note what
 * that means: a positive Ki would be positive feedback, because a sustained
 * error would drive the output away from the setpoint instead of toward it. The
 * model only stays stable because both Ki are zero. Fix the sign in Simulink
 * before tuning the integral gain. */
constexpr float I_TERM_SIGN = -1.0f;

/* All three Saturation blocks in the model are set to the same 0 .. 0.41 range,
 * so 0.41 is the highest duty the converter is allowed to run at. The zero
 * lower limit means neither proportional path can push a negative contribution.
 *
 * Saturation blocks sitting on the two proportional paths. */
constexpr float P_SAT_MIN = 0.0f;
constexpr float P_SAT_MAX = 0.41f;

/* Clamp applied to the discrete integrator state (anti-windup), kept on the
 * same range. A plain Simulink integrator has no limits, but an unbounded state
 * on real hardware takes seconds to unwind after a fault. */
constexpr float I_SAT_MIN = 0.0f;
constexpr float I_SAT_MAX = 0.41f;

/* Saturation block after the min selector, i.e. the legal duty-cycle range.
 * 0.41 is not an arbitrary tuning choice: a forward converter has to reset its
 * transformer core during the off time, which caps the duty below 0.5, and 0.41
 * keeps margin under that hard limit. */
constexpr float DUTY_MIN = 0.0f;
constexpr float DUTY_MAX = 0.41f;
static_assert(DUTY_MAX < 0.5f,
              "a forward converter cannot reset its transformer core above 50% "
              "duty; raise this only after changing topology");

/* --- Sensor scaling: engineering units = (adcVolts - offset) * gain --------
 * V_SENSE_GAIN is the inverse of the resistor divider ratio: 150k/10k divides
 * 58 V down to 3.6 V, which leaves headroom without wasting ADC range.
 * I_SENSE assumes an ACS712-05B, the +-5 A variant that suits a 5 A design:
 * 2.5 V at zero current and 185 mV per amp, so 1/0.185 = 5.405 A per volt. */
constexpr float V_SENSE_OFFSET_V = 0.0f;
constexpr float V_SENSE_GAIN = 16.0f;
constexpr float I_SENSE_OFFSET_V = 2.5f;
constexpr float I_SENSE_GAIN = 5.405f;

constexpr float ADC_VREF = 5.0f;
constexpr float ADC_COUNTS = 1023.0f;
constexpr uint8_t ADC_OVERSAMPLE = 4;

#if defined(ARDUINO)

constexpr uint8_t V_SENSE_PIN = A0; /* replaces the [V_OUT] from-tag */
constexpr uint8_t I_SENSE_PIN = A1; /* replaces the [I_OUT] from-tag */

/* Switching frequency of the power stage, i.e. the carrier of the D->P block.
 * The output pin is not a free choice, so pwm.h derives it from the board.
 * Beware that at 67 kHz a 16 MHz AVR only has 238 timer counts per period; see
 * the startup banner for the resolution actually achieved. */
constexpr unsigned long PWM_FREQ_HZ = 67000UL;

/* Stand-in for the Display block: how often the loop reports over serial. */
constexpr unsigned long TELEMETRY_PERIOD_MS = 100UL;
constexpr unsigned long SERIAL_BAUD = 115200UL;

#endif /* ARDUINO */

#endif /* CONFIG_H */
