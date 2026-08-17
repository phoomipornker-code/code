#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

#include "util.h"

/* One branch of the Simulink diagram:
 *
 *   err --+--> [Kp] --> [Saturation] --+
 *         |                            (+)--> out
 *         +--> [Ki] --> [K*Ts/(z-1)] --+
 *
 * K*Ts/(z-1) is the Forward Euler form of the Discrete-Time Integrator, so its
 * output at step k reflects samples up to k-1 only. Reading the state before
 * accumulating reproduces that one-step delay; accumulating first would give
 * the Backward Euler block K*Ts*z/(z-1) instead and shift the phase.
 */
struct PIController {
  float kp;
  float ki;
  float ts;
  float pMin; /* Saturation block on the proportional path */
  float pMax;
  float iMin; /* integrator state clamp */
  float iMax;
  float iState;

  void reset() { iState = 0.0f; }

  float update(float err) {
    const float p = clampf(kp * err, pMin, pMax);
    const float i = iState;
    iState = clampf(iState + ki * ts * err, iMin, iMax);
    return p + i;
  }
};

#endif /* PI_CONTROLLER_H */
