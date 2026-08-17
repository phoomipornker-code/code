/* Checks for the forward regulation law in firmware/forward_control.h.
 *
 * Two things matter here. First, that the min-select really does hand over from
 * current limit to voltage limit on its own, since that is what replaced v83's
 * mode machine. Second, that folding v83's three stacked over-voltage clamps
 * into one function did not make the response to an overshoot any weaker — the
 * old aggregate is reimplemented below and compared against.
 *
 *   make -C test
 */

#include <cmath>
#include <cstdio>
#include <string>

#include "forward_control.h"

static const float V_TARGET = 55.90f;
static const float I_LIMIT = 3.0f;

static int failures = 0;

static void expectNear(const std::string& what, float actual, float expected) {
  if (std::fabs(actual - expected) > 1e-4f) {
    std::printf("FAIL %s: got %.4f, expected %.4f\n", what.c_str(), actual,
                expected);
    ++failures;
  } else {
    std::printf("ok   %s = %.4f\n", what.c_str(), actual);
  }
}

static void expectTrue(const std::string& what, bool condition) {
  if (!condition) {
    std::printf("FAIL %s\n", what.c_str());
    ++failures;
  } else {
    std::printf("ok   %s\n", what.c_str());
  }
}

static ForwardStepRequest at(float vBat, float iBat, bool freezeUp = false) {
  return forwardSelectStep(V_TARGET, vBat, vBat, I_LIMIT, iBat, iBat, freezeUp);
}

/* Early in a charge neither loop is near its setpoint, so the duty must climb at
 * the rate v83 used when it was far from the current limit. A voltage loop that
 * reported a step here would throttle the whole charge. */
static void testFreeClimbFarFromBoth() {
  const ForwardStepRequest r = at(48.0f, 1.0f);
  expectTrue("voltage loop stands aside", !r.voltageLimiting);
  expectTrue("current loop stands aside", !r.currentLimiting);
  expectNear("free climb rate", r.step, FWD_STEP_UP_CC_FAR);
}

static void testCurrentLoopTakesOver() {
  const ForwardStepRequest approaching = at(48.0f, 2.6f);
  expectTrue("current loop limits inside its far band",
             approaching.currentLimiting);
  expectNear("approach step", approaching.step, FWD_STEP_UP_CC);

  const ForwardStepRequest holding = at(48.0f, 2.95f);
  expectNear("duty held at the current limit", holding.step, 0.0f);

  const ForwardStepRequest over = at(48.0f, 3.3f);
  expectNear("small overshoot backs off", over.step, -FWD_STEP_DOWN_CC_FINE);

  const ForwardStepRequest wayOver = at(48.0f, 4.0f);
  expectNear("large overshoot backs off harder", wayOver.step,
             -FWD_STEP_DOWN_CC);
}

/* The handover v83 needed entry thresholds and a confirm timer for. */
static void testVoltageLoopTakesOverWithoutAMode() {
  const ForwardStepRequest approaching = at(55.40f, 1.0f);
  expectTrue("voltage loop limits inside its approach band",
             approaching.voltageLimiting);
  expectNear("approach step", approaching.step, FWD_STEP_UP_CV);

  const ForwardStepRequest near = at(55.75f, 1.0f);
  expectNear("final approach is gentler", near.step, FWD_STEP_UP_CV_NEAR);

  const ForwardStepRequest holding = at(55.88f, 1.0f);
  expectNear("duty held at the voltage target", holding.step, 0.0f);
  expectTrue("voltage loop reported in charge", holding.voltageLimiting);
}

/* Both loops limiting at once: the more restrictive one must win. */
static void testMinSelectPicksTheSmallerStep() {
  const ForwardStepRequest r = at(55.40f, 2.6f);
  expectTrue("both loops limiting", r.voltageLimiting && r.currentLimiting);
  expectNear("current loop wants", r.currentStep, FWD_STEP_UP_CC);
  expectNear("voltage loop wants", r.voltageStep, FWD_STEP_UP_CV);
  expectNear("smaller step applied", r.step, FWD_STEP_UP_CV);
}

/* Once the pack has tapered, opening further only provokes sense fly-up. */
static void testNearFullHoldsInsteadOfClimbing() {
  const ForwardStepRequest r = at(55.70f, 0.30f);
  expectTrue("voltage loop in charge", r.voltageLimiting);
  expectNear("duty held with a tapered pack", r.step, 0.0f);
}

static void testFreezeBlocksClimbsButNotCuts() {
  expectNear("free climb frozen", at(48.0f, 1.0f, true).step, 0.0f);
  expectNear("current climb frozen", at(48.0f, 2.6f, true).step, 0.0f);
  expectNear("voltage climb frozen", at(55.40f, 1.0f, true).step, 0.0f);
  expectNear("over-current cut still applies", at(48.0f, 4.0f, true).step,
             -FWD_STEP_DOWN_CC);
  expectTrue("over-voltage cut still applies", at(56.30f, 1.0f, true).step < 0.0f);
}

/* v83's response to an overshoot, as it actually landed: the CV chain plus the
 * two outer clamps, all in the same tick, with raw and filtered equal. */
static float v83CvModeStep(float over) {
  float total = 0.0f;
  if (over > 0.25f) {
    total -= (FWD_STEP_DOWN_CV_OVER + over * 1.5f);
  } else if (over > FWD_CV_HOLD_BAND_V) {
    total -= (over > 0.12f) ? FWD_STEP_DOWN_CV : FWD_STEP_DOWN_CV_FINE;
  }
  if (over > 0.15f) total -= (0.3f + over * 1.5f);
  if (over > 0.50f) total -= 1.2f;
  return total;
}

/* And the response outside its CV mode, which was the harsher of the two. */
static float v83CcModeStep(float over) {
  float total = 0.0f;
  if (over > 0.0f) total -= (2.5f + over * 10.0f);
  if (over > 0.40f) total -= 8.0f;
  return total;
}

static void testOvervoltageIsNeverWeakerThanV83() {
  for (int milli = 60; milli <= 1000; milli += 10) {
    const float vBat = V_TARGET + (float)milli * 0.001f;
    /* Compare against the overshoot the law itself sees. Subtracting 55.9 leaves
     * only about 4 uV of float resolution, which is enough to land on the other
     * side of a band edge and has nothing to do with the response. */
    const float over = vBat - V_TARGET;
    const float now = at(vBat, 1.0f).step;
    const float cv = v83CvModeStep(over);
    const float cc = v83CcModeStep(over);
    const float harsher = (cv < cc) ? cv : cc;

    if (over > 0.25f) {
      /* Beyond a quarter volt an overshoot is a fault, so match the harsher
       * of the two paths v83 could take. */
      if (now > harsher + 1e-4f) {
        std::printf("FAIL over=%.3f: %.3f is weaker than v83's %.3f\n", over,
                    now, harsher);
        ++failures;
        return;
      }
    } else if (std::fabs(now - cv) > 1e-4f) {
      /* Inside a quarter volt this is regulation, and must still be the fine
       * const-V response v83 used once its CV mode was active. */
      std::printf("FAIL over=%.3f: %.3f does not match v83 CV %.3f\n", over, now,
                  cv);
      ++failures;
      return;
    }
  }
  std::printf("ok   over-voltage response matches or beats v83 from 0.06 to 1.00 V\n");
}

/* Nothing should make the duty climb faster as the pack gets closer to full. */
static void testApproachIsMonotonic() {
  float previous = FWD_STEP_NOT_LIMITING;
  for (int centi = 4800; centi <= 5650; centi += 5) {
    const float v = (float)centi * 0.01f;
    const float step = at(v, 1.0f).step;
    if (step > previous + 1e-4f) {
      std::printf("FAIL step rose to %.3f at %.2f V (was %.3f)\n", step, v,
                  previous);
      ++failures;
      return;
    }
    previous = step;
  }
  std::printf("ok   step never rises as the pack approaches the target\n");
}

int main() {
  testFreeClimbFarFromBoth();
  testCurrentLoopTakesOver();
  testVoltageLoopTakesOverWithoutAMode();
  testMinSelectPicksTheSmallerStep();
  testNearFullHoldsInsteadOfClimbing();
  testFreezeBlocksClimbsButNotCuts();
  testOvervoltageIsNeverWeakerThanV83();
  testApproachIsMonotonic();

  if (failures != 0) {
    std::printf("\n%d check(s) failed\n", failures);
    return 1;
  }
  std::printf("\nall checks passed\n");
  return 0;
}
