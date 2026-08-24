"""Discrete PI with clamping anti-windup.

Matches the charger firmware helper `boostRunPI`:

    u = Kp * e + integral
    integral += Ki * e * dt   only while u is inside [out_min, out_max]

dt is the control sample time in seconds (20 ms on the ESP32 charger).
"""

from __future__ import annotations


def clamp(x: float, lo: float, hi: float) -> float:
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def apply_slew(target: float, current: float, up_step: float, down_step: float) -> float:
    """Limit how far a command may move in one sample."""
    delta = target - current
    if delta > up_step:
        return current + up_step
    if delta < -down_step:
        return current - down_step
    return target


def run_pi(
    err: float,
    kp: float,
    ki: float,
    dt: float,
    integ: float,
    out_min: float,
    out_max: float,
) -> tuple[float, float]:
    """One PI sample.

    Returns (saturated_output, possibly_updated_integral).
    When the unsaturated output would leave [out_min, out_max], the
    integral is frozen so it cannot wind up against a hard limit.
    """
    proportional = kp * err
    integ_candidate = integ + ki * err * dt
    unsaturated = proportional + integ_candidate
    if unsaturated > out_max:
        return out_max, integ
    if unsaturated < out_min:
        return out_min, integ
    return unsaturated, integ_candidate


class PiController:
    """Stateful wrapper around :func:`run_pi`."""

    def __init__(self, kp: float, ki: float, out_min: float, out_max: float) -> None:
        self.kp = kp
        self.ki = ki
        self.out_min = out_min
        self.out_max = out_max
        self.integ = 0.0

    def reset(self, integ: float = 0.0) -> None:
        self.integ = integ

    def step(self, err: float, dt: float) -> float:
        out, self.integ = run_pi(
            err, self.kp, self.ki, dt, self.integ, self.out_min, self.out_max
        )
        return out
