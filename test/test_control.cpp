/* Host-side checks for the control math. The controller headers are free of
 * Arduino calls, so the exact code that runs on the board is exercised here and
 * compared against values worked out by hand from the Simulink diagram.
 *
 *   make -C test
 */

#include <cmath>
#include <cstdio>
#include <string>

#include "CVCCController.h"

static int failures = 0;

static void expectNear(const std::string& what, float actual, float expected) {
  const float tolerance = 1e-5f;
  if (std::fabs(actual - expected) > tolerance) {
    std::printf("FAIL %s: got %.6f, expected %.6f\n", what.c_str(), actual,
                expected);
    ++failures;
  } else {
    std::printf("ok   %s = %.6f\n", what.c_str(), actual);
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

/* Far below both setpoints every branch railes against the 0.41 upper limit,
 * which is the state the model screenshot was captured in. */
static void testRailedAtStartup() {
  CVCCController c;
  c.begin();
  const CVCCOutput out = c.update(0.0f, 0.0f);
  expectNear("CV branch railed", out.cv, P_SAT_MAX);
  expectNear("CC branch railed", out.cc, P_SAT_MAX);
  expectNear("duty at the maximum the model allows", out.duty, 0.41f);
}

/* Close to the voltage setpoint the CV branch comes off its limit and takes
 * over, because the lightly loaded CC branch is still railed. */
static void testVoltageLoopInCharge() {
  CVCCController c;
  c.begin();
  const CVCCOutput out = c.update(56.0f, 4.0f);
  expectNear("CV branch", out.cv, 0.2f);
  expectNear("CC branch still railed", out.cc, P_SAT_MAX);
  expectNear("duty follows the smaller branch", out.duty, 0.2f);
  expectTrue("voltage loop reported as active", !c.currentLimited);
}

/* Pull close to the current limit and the current branch must take over. */
static void testCurrentLimitTakesOver() {
  CVCCController c;
  c.begin();
  const CVCCOutput out = c.update(50.0f, 4.9f);
  expectNear("CV branch railed", out.cv, P_SAT_MAX);
  expectNear("CC branch", out.cc, 0.1f);
  expectNear("duty follows the current branch", out.duty, 0.1f);
  expectTrue("current loop reported as active", c.currentLimited);
}

/* Above the setpoint the error goes negative, and because the saturation blocks
 * have a lower limit of zero the branch has to sit at zero rather than demand a
 * negative duty cycle. */
static void testOvervoltageFloorsDuty() {
  CVCCController c;
  c.begin();
  const CVCCOutput out = c.update(62.0f, 0.0f);
  expectNear("CV branch floored", out.cv, 0.0f);
  expectNear("duty clamped to DUTY_MIN", out.duty, DUTY_MIN);
}

/* With Ki = 0, as the diagram has it, there is nothing to hold duty up once the
 * error reaches zero, so the output settles below the setpoint. */
static void testProportionalOnlyHasNoHoldup() {
  CVCCController c;
  c.begin();
  const CVCCOutput out = c.update(V_REF, 0.0f);
  expectNear("duty at zero voltage error", out.cv, 0.0f);
}

/* K*Ts/(z-1) is Forward Euler: step k must not yet contain sample k. */
static void testForwardEulerDelay() {
  PIController pi{0.0f, 2.0f, 0.001f, P_SAT_MIN, P_SAT_MAX,
                  I_SAT_MIN, I_SAT_MAX, 0.0f};
  expectNear("integrator step 1", pi.update(1.0f), 0.000f);
  expectNear("integrator step 2", pi.update(1.0f), 0.002f);
  expectNear("integrator step 3", pi.update(1.0f), 0.004f);
}

static void testProportionalSaturation() {
  PIController pi{1.0f, 0.0f, 0.001f, P_SAT_MIN, P_SAT_MAX,
                  I_SAT_MIN, I_SAT_MAX, 0.0f};
  expectNear("large positive error clamped", pi.update(40.0f), 0.41f);
  expectNear("negative error clamped at the lower limit", pi.update(-40.0f), 0.0f);
}

static void testIntegratorAntiWindup() {
  PIController pi{0.0f, 100.0f, 0.001f, P_SAT_MIN, P_SAT_MAX,
                  I_SAT_MIN, I_SAT_MAX, 0.0f};
  for (int step = 0; step < 200; ++step) {
    pi.update(1.0f);
  }
  expectNear("integrator state clamped at I_SAT_MAX", pi.iState, 0.41f);
  expectNear("output clamped too", pi.update(1.0f), 0.41f);

  pi.reset();
  expectNear("reset clears the state", pi.iState, 0.0f);
}

int main() {
  testRailedAtStartup();
  testVoltageLoopInCharge();
  testCurrentLimitTakesOver();
  testOvervoltageFloorsDuty();
  testProportionalOnlyHasNoHoldup();
  testForwardEulerDelay();
  testProportionalSaturation();
  testIntegratorAntiWindup();

  if (failures != 0) {
    std::printf("\n%d check(s) failed\n", failures);
    return 1;
  }
  std::printf("\nall checks passed\n");
  return 0;
}
