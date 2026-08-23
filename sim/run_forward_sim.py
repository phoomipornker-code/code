"""Average-value check of forward_pi_tick.m (no MATLAB required)."""
from __future__ import annotations

import math


def clampf(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def apply_slew(target, current, up_step, down_step):
    d = target - current
    if d > up_step:
        return current + up_step
    if d < -down_step:
        return current - down_step
    return target


def run_pi(err, kp, ki, dt, integ, lo, hi):
    p = kp * err
    i_cand = integ + ki * err * dt
    out = p + i_cand
    if out > hi:
        return hi, integ
    if out < lo:
        return lo, integ
    return out, i_cand


def simulate(t_stop=180.0):
    ts = 0.02
    n = int(t_stop / ts)
    vac = 220.0
    r_esr = 0.040
    tau_i = 0.04
    # Equivalent capacitance so 5 A raises ~0.02 V/s (CC→CV inside 3 min)
    c_eq = 5.0 / 0.02
    v_nl = 54.0
    vbat = v_nl
    ibat = 0.0

    mode = 0
    duty = 0.0
    integ_i = 0.0
    integ_v = 0.0
    iref = 5.0
    t_enter = 0.0
    t_cv_enter = 0.0
    t_cv_exit = 0.0
    t_full = 0.0

    last_mode = 0
    saw = {0: False, 1: False, 2: False}

    for k in range(n):
        now_ms = k * ts * 1000.0
        v_filt = vbat
        i_cc = 5.0
        v_cv = 57.60
        dmax = 460.0

        if vac < 100:
            duty = 0.0
            integ_i = 0.0
            integ_v = 0.0
        elif mode == 0:
            duty = apply_slew(80.0, duty, 1.5, 4.0)
            if ibat >= 0.35 or (now_ms - t_enter) >= 2000:
                mode = 1
                integ_i = 0.0
                iref = i_cc
        elif mode == 1:
            i_ref = i_cc
            if v_filt >= 56.40:
                span = max(0.20, v_cv - 56.40)
                i_ref *= clampf((v_cv - v_filt) / span, 0.10, 1.0)
            i_ref = clampf(i_ref, 0.0, i_cc)
            iref = i_ref
            i_err = i_ref - abs(ibat)
            d_duty, integ_i = run_pi(i_err, 8.0, 35.0, ts, integ_i, -20.0, 25.0)
            if i_err > 0.8 and duty < 200 and vac >= 140:
                d_duty = max(d_duty, 2.0)
            duty = apply_slew(clampf(duty + d_duty, 0.0, dmax), duty, 3.0, 5.0)
            if v_filt >= 57.30:
                mode = 2
                integ_i = 0.0
                integ_v = 0.0
                iref = clampf(abs(ibat), 0.3, 2.0)
                t_cv_enter = 0.0
            elif v_filt >= 57.10:
                if t_cv_enter == 0:
                    t_cv_enter = now_ms
                if now_ms - t_cv_enter >= 200:
                    mode = 2
                    integ_i = 0.0
                    integ_v = 0.0
                    iref = clampf(abs(ibat), 0.3, 2.0)
                    t_cv_enter = 0.0
            else:
                t_cv_enter = 0.0
        elif mode == 2:
            v_err = v_cv - v_filt
            near = abs(v_err) <= 0.35
            if abs(v_err) <= 0.12:
                v_err = 0.0
                integ_v *= 0.92
            i_req, integ_v = run_pi(v_err, 0.70, 0.35, ts, integ_v, 0.0, 3.0)
            if near:
                i_req = min(i_req, 1.0 + clampf(v_err / 0.35, 0.0, 1.0))
            i_req = clampf(i_req, 0.0, 3.0)
            iref = apply_slew(i_req, iref, 0.06, 0.06)
            i_err = iref - abs(ibat)
            kp = 8.0 * (0.50 if near else 0.80)
            ki = 35.0 * (0.40 if near else 0.65)
            lo, hi = (-6.0, 6.0) if near else (-20.0, 15.0)
            d_duty, integ_i = run_pi(i_err, kp, ki, ts, integ_i, lo, hi)
            step = 0.6 if near else 2.0
            d_duty = clampf(d_duty, -step, step)
            duty_t = duty + d_duty
            if abs(v_cv - v_filt) <= 0.12:
                duty_t = duty
                integ_i *= 0.95
            if v_filt > v_cv:
                duty_t -= 0.6 + (v_filt - v_cv) * 3.5
                integ_v *= 0.85
            duty = apply_slew(clampf(duty_t, 0.0, dmax), duty, 1.0 if near else 2.0, 1.8 if near else 3.5)
            if v_filt >= 57.40 and abs(ibat) <= 0.50:
                if t_full == 0:
                    t_full = now_ms
                if now_ms - t_full >= 60000:
                    mode = 3
                    duty = 0.0
            else:
                t_full = 0.0
        else:
            duty = 0.0

        duty = clampf(duty, 0.0, dmax)
        dfrac = duty / 1023.0
        i_ss = clampf(5.0 * (dfrac / 0.22), 0.0, 8.0)  # ~5 A near duty 225
        ibat += ts * (i_ss - ibat) / tau_i
        v_nl += ibat * ts / c_eq
        vbat = v_nl + ibat * r_esr

        saw[mode] = True
        last_mode = mode

    return {
        "vbat": vbat,
        "ibat": ibat,
        "duty": duty,
        "mode": last_mode,
        "saw_soft": saw[0],
        "saw_cc": saw[1],
        "saw_cv": saw[2],
    }


if __name__ == "__main__":
    r = simulate()
    print(
        f"end V={r['vbat']:.2f} I={r['ibat']:.2f} duty={r['duty']:.0f} mode={r['mode']} "
        f"saw SOFT/CC/CV={r['saw_soft']}/{r['saw_cc']}/{r['saw_cv']}"
    )
    assert r["saw_cc"], "CC never entered"
    assert r["saw_cv"], "CV never entered"
    assert r["vbat"] > 56.5
    assert r["duty"] <= 460.0
    print("forward sim smoke check passed")
