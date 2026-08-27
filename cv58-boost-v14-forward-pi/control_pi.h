#ifndef CONTROL_PI_H
#define CONTROL_PI_H

/* Shared PI helpers for Boost and Forward. Arduino-free so host tests can
 * compile the same math that runs on the ESP32.
 *
 * Old Forward dual-PID (min of CC PID and CV PID, plus Kd) is not used:
 * the two loops fought, the integral was not dt-scaled, and derivative
 * amplified current-sense noise. Both converters now use this PI with
 * clamping anti-windup.
 */

static inline float boostClampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline float boostMaxf(float a, float b) {
    return (a > b) ? a : b;
}

static inline float boostMinf(float a, float b) {
    return (a < b) ? a : b;
}

static inline float boostApplySlew(float target, float current, float upStep, float downStep) {
    float d = target - current;
    if (d > upStep) return current + upStep;
    if (d < -downStep) return current - downStep;
    return target;
}

/* Standard PI with clamping anti-windup: integral is updated only when the
 * unsaturated output is inside [outMin, outMax]. dt is in seconds. */
static inline float boostRunPI(float err, float kp, float ki, float dt,
                               float *integ, float outMin, float outMax) {
    float p = kp * err;
    float iCandidate = *integ + (ki * err * dt);
    float out = p + iCandidate;
    if (out > outMax) out = outMax;
    else if (out < outMin) out = outMin;
    else *integ = iCandidate;
    return out;
}

#endif /* CONTROL_PI_H */
