#ifndef PWM_H
#define PWM_H

#include <Arduino.h>

#include "config.h"
#include "util.h"

/* The D->P block. analogWrite would work, but its ~490 Hz carrier is far too
 * slow for a switching converter, so on AVR parts Timer1 is reprogrammed for a
 * fast PWM with ICR1 as TOP. That gives an exact frequency and roughly ten bits
 * of duty resolution instead of eight. */

/* Timer1 can only reach its OC1A pin, so the board dictates the output. */
#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
#define PWM_BACKEND_TIMER1 1
constexpr uint8_t PWM_PIN = 11;
#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || \
    defined(__AVR_ATmega168__) || defined(__AVR_ATmega32U4__)
#define PWM_BACKEND_TIMER1 1
constexpr uint8_t PWM_PIN = 9;
#elif defined(ARDUINO_ARCH_ESP32)
/* LEDC divides the 80 MHz APB clock and takes a power-of-two resolution, so
 * 67 kHz caps out at 10 bits: 67000 * 1024 = 68.6 MHz still fits. A fractional
 * divider gets the carrier within a fraction of a percent of the target.
 * MCPWM would give 2388 counts from its 160 MHz clock plus dead-time
 * generation and a fault input, at the cost of leaving the Arduino API. */
#define PWM_BACKEND_LEDC 1
constexpr uint8_t PWM_PIN = 25;
constexpr uint8_t PWM_LEDC_BITS = 10;
constexpr uint8_t PWM_LEDC_CHANNEL = 0;
constexpr unsigned long PWM_LEDC_STEPS = 1UL << PWM_LEDC_BITS;
static_assert(PWM_FREQ_HZ * PWM_LEDC_STEPS <= 80000000UL,
              "PWM_LEDC_BITS too high for this frequency on the 80 MHz APB clock");
#else
constexpr uint8_t PWM_PIN = 9;
#warning "Unknown board: using analogWrite, too slow to switch a converter."
#endif

#if defined(PWM_BACKEND_TIMER1)
constexpr unsigned long PWM_TOP = (F_CPU / PWM_FREQ_HZ) - 1UL;
static_assert(PWM_TOP <= 65535UL, "PWM_FREQ_HZ too low for a prescaler of 1");
static_assert(PWM_TOP >= 100UL, "PWM_FREQ_HZ leaves less than 1% duty resolution");
#endif

inline void pwmBegin() {
#if defined(PWM_BACKEND_TIMER1)
  pinMode(PWM_PIN, OUTPUT);
  /* Waveform mode 14 (fast PWM, TOP = ICR1), non-inverting OC1A, prescaler 1. */
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
  ICR1 = (uint16_t)PWM_TOP;
  OCR1A = 0;
#elif defined(PWM_BACKEND_LEDC)
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PWM_PIN, PWM_FREQ_HZ, PWM_LEDC_BITS);
  ledcWrite(PWM_PIN, 0);
#else
  ledcSetup(PWM_LEDC_CHANNEL, PWM_FREQ_HZ, PWM_LEDC_BITS);
  ledcAttachPin(PWM_PIN, PWM_LEDC_CHANNEL);
  ledcWrite(PWM_LEDC_CHANNEL, 0);
#endif
#else
  pinMode(PWM_PIN, OUTPUT);
  analogWrite(PWM_PIN, 0);
#endif
}

inline void pwmWriteDuty(float duty) {
  duty = clampf(duty, 0.0f, 1.0f);
#if defined(PWM_BACKEND_TIMER1)
  OCR1A = (uint16_t)(duty * (float)PWM_TOP + 0.5f);
#elif defined(PWM_BACKEND_LEDC)
  const uint32_t value = (uint32_t)(duty * (float)PWM_LEDC_STEPS + 0.5f);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PWM_PIN, value);
#else
  ledcWrite(PWM_LEDC_CHANNEL, value);
#endif
#else
  analogWrite(PWM_PIN, (int)(duty * 255.0f + 0.5f));
#endif
}

/* The carrier lands near, not exactly on, PWM_FREQ_HZ, and at 67 kHz there are
 * few counts left to resolve duty with. Both are worth reporting at startup
 * rather than assuming. */
inline unsigned long pwmActualFrequencyHz() {
#if defined(PWM_BACKEND_TIMER1)
  return F_CPU / (PWM_TOP + 1UL);
#elif defined(PWM_BACKEND_LEDC)
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  /* The fractional divider lands within about 0.2% of the target; ask the
   * peripheral rather than repeating the requested value back. */
  return ledcReadFreq(PWM_PIN);
#else
  return PWM_FREQ_HZ;
#endif
#else
  return 490UL;
#endif
}

inline unsigned int pwmDutySteps() {
#if defined(PWM_BACKEND_TIMER1)
  return (unsigned int)(PWM_TOP + 1UL);
#elif defined(PWM_BACKEND_LEDC)
  return (unsigned int)PWM_LEDC_STEPS;
#else
  return 256U;
#endif
}

/* A default-speed AVR ADC needs about 112 us per reading, which would eat most
 * of a 1 ms step once both channels are oversampled. A /32 prescaler brings
 * that down to roughly 26 us; the cost is a fraction of a bit of accuracy. */
inline void adcSpeedUp() {
#if defined(ADCSRA) && defined(ADPS2)
  ADCSRA = (ADCSRA & ~0x07) | _BV(ADPS2) | _BV(ADPS0);
#endif
}

#endif /* PWM_H */
