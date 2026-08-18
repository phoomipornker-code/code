#ifndef FORWARD_CONTROL_H
#define FORWARD_CONTROL_H

/* Forward (AC) regulation law, restructured from the v83 mode machine into the
 * min-select form the Simulink model uses.
 *
 * v83 ran either a CC block or a CV block, switching between them on entry and
 * exit thresholds with confirmation timers. Here both loops state the duty step
 * they want on every tick and the more restrictive one is applied, so the
 * handover from current limit to voltage limit happens on its own with no mode,
 * no threshold and no timer. A loop still far from its setpoint reports
 * FWD_STEP_NOT_LIMITING rather than a step, otherwise the slow-moving voltage
 * loop would hold back the current loop during the early part of a charge.
 *
 * Steps are raw LEDC counts per control tick (20 ms). Every value below is the
 * one v83 tuned in the field; the restructuring did not retune them.
 *
 * Kept free of Arduino calls so test/test_forward_control.cpp can exercise the
 * same code that runs on the board.
 */

const float FWD_STEP_UP_SOFT = 0.8f;
const float FWD_STEP_UP_CC = 0.6f;
const float FWD_STEP_UP_CC_FAR = 1.2f;
const float FWD_STEP_DOWN_CC = 1.5f;
const float FWD_STEP_DOWN_CC_FINE = 0.6f;
const float FWD_STEP_UP_CV = 0.40f;
const float FWD_STEP_UP_CV_NEAR = 0.20f;
const float FWD_STEP_DOWN_CV = 0.80f;
const float FWD_STEP_DOWN_CV_FINE = 0.35f;
const float FWD_STEP_DOWN_CV_OVER = 1.50f;

const float FWD_CV_HOLD_BAND_V = 0.05f; /* inside this band duty is held */
const float FWD_CV_NEAR_BAND_V = 0.30f; /* final gentle approach to the target */
/* Where the voltage loop starts to limit at all. 0.60 V under a 55.90 V target
 * is 55.30 V, which is the point v83 used to force its CV mode. */
const float FWD_CV_APPROACH_BAND_V = 0.60f;
const float FWD_CV_TAPER_I_A = 0.40f; /* near full: do not open toward Dmax */
const float FWD_CC_HOLD_BAND_A = 0.10f;
const float FWD_CC_FAR_BAND_A = 0.60f;

/* Reported by a loop that is far enough from its setpoint to have no business
 * limiting the duty. Larger than any real step, so the min ignores it. */
const float FWD_STEP_NOT_LIMITING = 1000.0f;

struct ForwardStepRequest {
  float voltageStep;
  float currentStep;
  float step; /* what to add to the duty accumulator this tick */
  bool voltageLimiting;
  bool currentLimiting;
};

/* Over-voltage response, folded into one place. v83 spread it across three
 * independent if-blocks that all fired on the same tick (the CV chain plus two
 * outer clamps), so the step written on any one line was never the step that
 * reached the accumulator. Beyond 0.25 V of overshoot this reproduces the
 * harsher of the two responses v83 could produce, since an overshoot that large
 * is a fault rather than a regulation error; inside 0.25 V it keeps the fine
 * const-V steps, which is what v83 used once its CV mode was active. */
inline float forwardVoltageStep(float vTarget, float vFilt, float vPeak,
                                float iAbs, float iFilt, float iLimit,
                                bool freezeUp) {
  const float overPeak = vPeak - vTarget;
  if (overPeak > 0.40f) return -(2.5f + 10.0f * overPeak + 8.0f);
  if (overPeak > 0.25f) return -(2.5f + 10.0f * overPeak);

  const float overFilt = vFilt - vTarget;
  if (overFilt > 0.15f) return -(FWD_STEP_DOWN_CV + 0.3f + 1.5f * overFilt);
  if (overFilt > 0.12f) return -FWD_STEP_DOWN_CV;
  if (overFilt > FWD_CV_HOLD_BAND_V) return -FWD_STEP_DOWN_CV_FINE;

  const float vErr = vTarget - vFilt;
  if (vErr > FWD_CV_APPROACH_BAND_V) return FWD_STEP_NOT_LIMITING;
  if (vErr > FWD_CV_HOLD_BAND_V) {
    if (freezeUp) return 0.0f;
    /* Let the current loop own the climb while it is at its limit, and hold
     * still once the pack has tapered, which is what kept v83 from running to
     * Dmax into a nearly full pack and provoking sense fly-up. */
    if (iAbs >= iLimit) return 0.0f;
    if (iFilt <= FWD_CV_TAPER_I_A) return 0.0f;
    return (vErr > FWD_CV_NEAR_BAND_V) ? FWD_STEP_UP_CV : FWD_STEP_UP_CV_NEAR;
  }
  return 0.0f;
}

inline float forwardCurrentStep(float iLimit, float iFilt, bool freezeUp) {
  const float iErr = iLimit - iFilt;
  if (iErr > FWD_CC_FAR_BAND_A) return FWD_STEP_NOT_LIMITING;
  if (iErr > FWD_CC_HOLD_BAND_A) return freezeUp ? 0.0f : FWD_STEP_UP_CC;
  if (iErr < -FWD_CC_FAR_BAND_A) return -FWD_STEP_DOWN_CC;
  if (iErr < -FWD_CC_HOLD_BAND_A) return -FWD_STEP_DOWN_CC_FINE;
  return 0.0f;
}

inline ForwardStepRequest forwardSelectStep(float vTarget, float vFilt,
                                            float vPeak, float iLimit,
                                            float iFilt, float iAbs,
                                            bool freezeUp) {
  ForwardStepRequest request;
  request.voltageStep =
      forwardVoltageStep(vTarget, vFilt, vPeak, iAbs, iFilt, iLimit, freezeUp);
  request.currentStep = forwardCurrentStep(iLimit, iFilt, freezeUp);
  request.voltageLimiting = (request.voltageStep < FWD_STEP_NOT_LIMITING);
  request.currentLimiting = (request.currentStep < FWD_STEP_NOT_LIMITING);

  if (!request.voltageLimiting && !request.currentLimiting) {
    /* Neither loop is near its setpoint, so climb at the rate v83 used when it
     * was far from the current limit. */
    request.step = freezeUp ? 0.0f : FWD_STEP_UP_CC_FAR;
  } else {
    request.step = (request.voltageStep < request.currentStep)
                       ? request.voltageStep
                       : request.currentStep;
  }
  return request;
}

#endif /* FORWARD_CONTROL_H */
