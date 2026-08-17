#ifndef CVCC_CONTROLLER_H
#define CVCC_CONTROLLER_H

#include "PIController.h"
#include "config.h"

struct CVCCOutput {
  float cv;   /* the [CV] goto-tag  */
  float cc;   /* the [CC] goto-tag  */
  float duty; /* the [Duty] goto-tag, after the output saturation */
};

/* The whole diagram minus the hardware: two PI branches, the min selector and
 * the output saturation. Keeping it free of Arduino calls lets the same code
 * run under the host test in test/. */
struct CVCCController {
  PIController cvLoop;
  PIController ccLoop;
  float vRef;
  float iRef;
  float dutyMin;
  float dutyMax;
  bool currentLimited;

  void begin() {
    cvLoop = PIController{KP_V, KI_V, TS, P_SAT_MIN, P_SAT_MAX,
                          I_SAT_MIN, I_SAT_MAX, I_TERM_SIGN, 0.0f};
    ccLoop = PIController{KP_I, KI_I, TS, P_SAT_MIN, P_SAT_MAX,
                          I_SAT_MIN, I_SAT_MAX, I_TERM_SIGN, 0.0f};
    vRef = V_REF;
    iRef = I_REF;
    dutyMin = DUTY_MIN;
    dutyMax = DUTY_MAX;
    currentLimited = false;
  }

  CVCCOutput update(float vOut, float iOut) {
    CVCCOutput out;
    out.cv = cvLoop.update(vRef - vOut);
    out.cc = ccLoop.update(iRef - iOut);

    /* The min block: whichever branch asks for less duty is the one currently
     * in control, which is how a supply slides between CV and CC operation. */
    currentLimited = (out.cc < out.cv);
    const float selected = currentLimited ? out.cc : out.cv;
    out.duty = clampf(selected, dutyMin, dutyMax);
    return out;
  }
};

#endif /* CVCC_CONTROLLER_H */
