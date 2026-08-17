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
#define PWM_USE_TIMER1 1
constexpr uint8_t PWM_PIN = 11;
#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || \
    defined(__AVR_ATmega168__) || defined(__AVR_ATmega32U4__)
#define PWM_USE_TIMER1 1
constexpr uint8_t PWM_PIN = 9;
#else
#define PWM_USE_TIMER1 0
constexpr uint8_t PWM_PIN = 9;
#warning "Unknown board: using analogWrite, too slow to switch a converter."
#endif

#if PWM_USE_TIMER1
constexpr unsigned long PWM_TOP = (F_CPU / PWM_FREQ_HZ) - 1UL;
static_assert(PWM_TOP <= 65535UL, "PWM_FREQ_HZ too low for a prescaler of 1");
static_assert(PWM_TOP >= 100UL, "PWM_FREQ_HZ leaves less than 1% duty resolution");
#endif

inline void pwmBegin() {
  pinMode(PWM_PIN, OUTPUT);
#if PWM_USE_TIMER1
  /* Waveform mode 14 (fast PWM, TOP = ICR1), non-inverting OC1A, prescaler 1. */
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
  ICR1 = (uint16_t)PWM_TOP;
  OCR1A = 0;
#else
  analogWrite(PWM_PIN, 0);
#endif
}

inline void pwmWriteDuty(float duty) {
  duty = clampf(duty, 0.0f, 1.0f);
#if PWM_USE_TIMER1
  OCR1A = (uint16_t)(duty * (float)PWM_TOP + 0.5f);
#else
  analogWrite(PWM_PIN, (int)(duty * 255.0f + 0.5f));
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
