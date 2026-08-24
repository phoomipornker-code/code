"""CC/CV cascade PI charger simulation for the learning lesson.

Two labs:

  python3 learn/simulate_cc_cv.py              # full SoftStart → CC → CV charge
  python3 learn/simulate_cc_cv.py --lab p-vs-pi  # why the I term exists

Control sample time is 20 ms, matching the ESP32 charger. PWM (67 kHz) is
not modelled — only the average duty that the PI actually commands.
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from dataclasses import dataclass, field

from pi import PiController, apply_slew, clamp
from svgplot import HLine, Panel, Series, Span, write_stack

DT = 0.02
I_CC = 5.0
V_CV = 57.60
DMAX = 460.0
PWM_FULL = 1023.0
SOFT_SEED = 80.0
CV_ENTRY = 57.10
CV_FORCE = 57.30
CV_TAPER_START = 56.40
FULL_V = 57.40
FULL_I = 0.50
MODE_NAME = {0: "SOFT", 1: "CC", 2: "CV", 3: "DONE"}


@dataclass
class Sample:
    t: float
    vbat: float
    ibat: float
    iref: float
    duty: float
    mode: int
    integ_i: float
    integ_v: float


@dataclass
class ChargeResult:
    samples: list[Sample] = field(default_factory=list)
    entered: set[int] = field(default_factory=set)

    @property
    def last(self) -> Sample:
        return self.samples[-1]


def _duty_to_current(duty: float, vbat: float) -> float:
    """Average-value plant: more duty → more current, less headroom as V rises."""
    dfrac = duty / PWM_FULL
    headroom = max(0.15, (62.0 - vbat) / 8.0)
    return clamp(28.0 * dfrac * headroom, 0.0, 8.0)


def simulate_charge(
    t_stop: float = 180.0,
    v_start: float = 54.0,
    kp_i: float = 8.0,
    ki_i: float = 35.0,
    kp_v: float = 0.70,
    ki_v: float = 0.35,
    # ~90 s of CC at 5 A from 54 V to 57.1 V, then ~90 s of CV in a 180 s run
    c_eq: float = 150.0,
) -> ChargeResult:
    """SoftStart → CC current PI → CV voltage-over-current cascade."""
    n = int(t_stop / DT)
    voc = v_start
    ibat = 0.0
    r_esr = 0.040
    tau_i = 0.04

    mode = 0
    duty = 0.0
    iref = I_CC
    t_enter = 0.0
    t_cv_enter = 0.0
    t_full = 0.0

    curr = PiController(kp_i, ki_i, -20.0, 25.0)
    volt = PiController(kp_v, ki_v, 0.0, 3.0)

    out = ChargeResult()

    for k in range(n):
        t = k * DT
        now_ms = t * 1000.0
        vbat = voc + ibat * r_esr

        if mode == 0:
            duty = apply_slew(SOFT_SEED, duty, 1.5, 4.0)
            iref = I_CC
            if (now_ms - t_enter) >= 2000.0:
                mode = 1
                curr.reset()
                iref = I_CC
        elif mode == 1:
            i_ref = I_CC
            if vbat >= CV_TAPER_START:
                span = max(0.20, V_CV - CV_TAPER_START)
                i_ref *= clamp((V_CV - vbat) / span, 0.10, 1.0)
            i_ref = clamp(i_ref, 0.0, I_CC)
            iref = i_ref
            d_duty = curr.step(i_ref - abs(ibat), DT)
            if (i_ref - abs(ibat)) > 0.8 and duty < 200.0:
                d_duty = max(d_duty, 2.0)
            duty = apply_slew(clamp(duty + d_duty, 0.0, DMAX), duty, 3.0, 5.0)
            if vbat >= CV_FORCE:
                mode = 2
                curr.reset()
                volt.reset()
                iref = clamp(abs(ibat), 0.3, 2.0)
                t_cv_enter = 0.0
            elif vbat >= CV_ENTRY:
                if t_cv_enter == 0.0:
                    t_cv_enter = now_ms
                if now_ms - t_cv_enter >= 200.0:
                    mode = 2
                    curr.reset()
                    volt.reset()
                    iref = clamp(abs(ibat), 0.3, 2.0)
                    t_cv_enter = 0.0
            else:
                t_cv_enter = 0.0
        elif mode == 2:
            v_err = V_CV - vbat
            near = abs(v_err) <= 0.35
            if abs(v_err) <= 0.12:
                v_err = 0.0
                volt.integ *= 0.92
            i_req = volt.step(v_err, DT)
            if near:
                i_req = min(i_req, 1.0 + clamp(v_err / 0.35, 0.0, 1.0))
            iref = apply_slew(clamp(i_req, 0.0, 3.0), iref, 0.06, 0.06)
            d_duty = curr.step(iref - abs(ibat), DT)
            d_duty = clamp(d_duty, -2.0, 2.0)
            duty_t = clamp(duty + d_duty, 0.0, DMAX)
            if vbat > V_CV:
                duty_t -= 0.6 + (vbat - V_CV) * 3.5
                volt.integ *= 0.85
            duty = apply_slew(clamp(duty_t, 0.0, DMAX), duty, 2.0, 3.5)
            if vbat >= FULL_V and abs(ibat) <= FULL_I:
                if t_full == 0.0:
                    t_full = now_ms
                if now_ms - t_full >= 20000.0:
                    mode = 3
                    duty = 0.0
            else:
                t_full = 0.0
        else:
            duty = 0.0
            iref = 0.0

        duty = clamp(duty, 0.0, DMAX)
        i_ss = _duty_to_current(duty, voc)
        ibat += DT * (i_ss - ibat) / tau_i
        voc += ibat * DT / c_eq
        vbat = voc + ibat * r_esr

        out.entered.add(mode)
        out.samples.append(
            Sample(t, vbat, ibat, iref, duty, mode, curr.integ, volt.integ)
        )

    return out


def simulate_current_step(
    use_integral: bool,
    t_stop: float = 8.0,
    kp: float = 40.0,
    ki: float = 60.0,
    v_hold: float = 54.0,
) -> list[Sample]:
    """Current loop only, battery voltage frozen.

    Uses a *position-form* PI (output = duty, not Δduty) so the I term is
    visible. The charge lab uses incremental Δduty; that accumulator is
    already an integrator, which hides why Ki exists.
    """
    n = int(t_stop / DT)
    ibat = 0.0
    tau_i = 0.04
    curr = PiController(kp, ki if use_integral else 0.0, 0.0, DMAX)
    samples: list[Sample] = []
    for k in range(n):
        t = k * DT
        iref = 0.0 if t < 0.4 else I_CC
        duty = curr.step(iref - abs(ibat), DT)
        i_ss = _duty_to_current(duty, v_hold)
        ibat += DT * (i_ss - ibat) / tau_i
        samples.append(Sample(t, v_hold, ibat, iref, duty, 1, curr.integ, 0.0))
    return samples


def _mode_spans(samples: list[Sample]) -> list[Span]:
    colors = {0: "#9aa0a6", 1: "#4c8bf5", 2: "#f5a524", 3: "#34a853"}
    spans: list[Span] = []
    start = samples[0]
    for prev, cur in zip(samples, samples[1:]):
        if cur.mode != prev.mode:
            spans.append(
                Span(start.t, prev.t, colors[prev.mode], MODE_NAME[prev.mode])
            )
            start = cur
    last = samples[-1]
    spans.append(Span(start.t, last.t, colors[last.mode], MODE_NAME[last.mode]))
    return spans


def _write_csv(path: str, samples: list[Sample]) -> None:
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["t_s", "Vbat", "Ibat", "Iref", "duty", "mode", "mode_name"])
        for s in samples:
            w.writerow(
                [
                    f"{s.t:.3f}",
                    f"{s.vbat:.4f}",
                    f"{s.ibat:.4f}",
                    f"{s.iref:.4f}",
                    f"{s.duty:.2f}",
                    s.mode,
                    MODE_NAME[s.mode],
                ]
            )


def _downsample(samples: list[Sample], max_points: int = 900) -> list[Sample]:
    if len(samples) <= max_points:
        return samples
    step = max(1, len(samples) // max_points)
    sliced = samples[::step]
    if sliced[-1] is not samples[-1]:
        sliced.append(samples[-1])
    return sliced


def plot_charge(result: ChargeResult, path: str) -> None:
    s = _downsample(result.samples)
    t = [x.t for x in s]
    panels = [
        Panel(
            title="Battery voltage",
            ylabel="V",
            series=[Series("Vbat", t, [x.vbat for x in s], "#1a73e8")],
            hlines=[HLine(V_CV, "#d93025", label=f"Vcv={V_CV:.2f}")],
        ),
        Panel(
            title="Charge current",
            ylabel="A",
            series=[
                Series("Ibat", t, [x.ibat for x in s], "#188038"),
                Series("Iref", t, [x.iref for x in s], "#e37400", width=1.2),
            ],
            hlines=[HLine(I_CC, "#188038", label=f"Icc={I_CC:.1f} A")],
            y_min=-0.2,
            y_max=6.2,
        ),
        Panel(
            title="Duty (LEDC counts)",
            ylabel="counts",
            series=[Series("duty", t, [x.duty for x in s], "#7b1fa2")],
            hlines=[HLine(DMAX, "#7b1fa2", label=f"Dmax={DMAX:.0f}")],
            y_min=0.0,
            y_max=500.0,
        ),
        Panel(
            title="Mode  0=SOFT  1=CC  2=CV  3=DONE",
            ylabel="mode",
            series=[Series("mode", t, [float(x.mode) for x in s], "#202124")],
            y_min=-0.2,
            y_max=3.4,
        ),
    ]
    write_stack(
        path,
        panels,
        _mode_spans(s),
        xlabel="time (s)",
        title="CC/CV cascade PI  —  16S LFP pack, Forward 5 A / 57.6 V",
        t_max=s[-1].t if s else 1.0,
    )


def plot_p_vs_pi(p_samples: list[Sample], pi_samples: list[Sample], path: str) -> None:
    t = [x.t for x in p_samples]
    panels = [
        Panel(
            title="Current tracking after Iref steps 0 → 5 A",
            ylabel="A",
            series=[
                Series("Iref", t, [x.iref for x in pi_samples], "#5f6368", width=1.2),
                Series("P only", t, [x.ibat for x in p_samples], "#d93025"),
                Series("PI", t, [x.ibat for x in pi_samples], "#188038"),
            ],
            hlines=[HLine(I_CC, "#188038", label="5 A")],
            y_min=-0.2,
            y_max=6.2,
        ),
        Panel(
            title="Duty the current PI commands",
            ylabel="counts",
            series=[
                Series("P only", t, [x.duty for x in p_samples], "#d93025"),
                Series("PI", t, [x.duty for x in pi_samples], "#188038"),
            ],
            y_min=0.0,
            y_max=500.0,
        ),
    ]
    write_stack(
        path,
        panels,
        [],
        xlabel="time (s)",
        title="Why the I term exists — P-only leaves a standing current error",
        t_max=t[-1] if t else 1.0,
    )


def _cv_window(result: ChargeResult) -> list[Sample]:
    return [s for s in result.samples if s.mode == 2]


def print_charge_lesson(result: ChargeResult) -> None:
    last = result.last
    cc = [s for s in result.samples if s.mode == 1 and 8.0 < s.t < 40.0]
    cv = _cv_window(result)
    t_cv = cv[0].t if cv else None
    print("=== CC/CV cascade PI  (charge lab) ===")
    print(f"end:  V={last.vbat:.2f} V   I={last.ibat:.2f} A   duty={last.duty:.0f}   mode={MODE_NAME[last.mode]}")
    print(f"modes seen: {', '.join(MODE_NAME[m] for m in sorted(result.entered))}")
    if cc:
        i_avg = sum(s.ibat for s in cc) / len(cc)
        print(f"CC hold (8–40 s):  Ibat avg={i_avg:.2f} A  (target {I_CC:.1f} A)")
    if t_cv is not None:
        v_end = cv[-1].vbat
        i_end = cv[-1].ibat
        print(f"CV from t={t_cv:.1f} s:  last V={v_end:.2f} V (target {V_CV:.2f})  I tapered to {i_end:.2f} A")
    print()
    print("Read the plot bottom-to-top:")
    print("  1. SOFT  duty ramps from 0 so the transformer is not slammed.")
    print("  2. CC    current PI holds Ibat ≈ 5 A. Voltage is still free to climb.")
    print("  3. CV    voltage PI takes over Iref and current falls (taper).")
    print("  4. DONE  V high and I low long enough — charge current to zero.")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="CC/CV PI learning labs")
    parser.add_argument("--lab", choices=("charge", "p-vs-pi"), default="charge")
    parser.add_argument("--tstop", type=float, default=None)
    parser.add_argument("--kp-i", type=float, default=8.0)
    parser.add_argument("--ki-i", type=float, default=35.0)
    parser.add_argument("--kp-v", type=float, default=0.70)
    parser.add_argument("--ki-v", type=float, default=0.35)
    parser.add_argument("--out-dir", default=None)
    args = parser.parse_args(argv)

    here = os.path.dirname(os.path.abspath(__file__))
    out_dir = args.out_dir or os.path.join(here, "out")
    os.makedirs(out_dir, exist_ok=True)

    if args.lab == "p-vs-pi":
        p_s = simulate_current_step(False)
        pi_s = simulate_current_step(True, kp=args.kp_i, ki=args.ki_i)
        svg = os.path.join(out_dir, "p_vs_pi.svg")
        plot_p_vs_pi(p_s, pi_s, svg)
        p_ss = p_s[-1].ibat
        pi_ss = pi_s[-1].ibat
        print("=== P vs PI  (current-step lab) ===")
        print(f"P-only  I(end)={p_ss:.2f} A   error={I_CC - p_ss:.2f} A")
        print(f"PI      I(end)={pi_ss:.2f} A   error={I_CC - pi_ss:.2f} A")
        print(f"wrote {svg}")
        print("P alone cannot cancel the standing error of the plant. The I term can.")
        return 0

    t_stop = 180.0 if args.tstop is None else args.tstop
    result = simulate_charge(
        t_stop=t_stop,
        kp_i=args.kp_i,
        ki_i=args.ki_i,
        kp_v=args.kp_v,
        ki_v=args.ki_v,
    )
    svg = os.path.join(out_dir, "cc_cv_charge.svg")
    csv_path = os.path.join(out_dir, "cc_cv_charge.csv")
    plot_charge(result, svg)
    _write_csv(csv_path, result.samples)
    print_charge_lesson(result)
    print(f"wrote {svg}")
    print(f"wrote {csv_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
