"""100 Hz bus ripple into battery current — matches the 5 ms/div scope shot."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .design import design_240vac_58v_5a
from .svg import Svg


LINE_HZ = 50.0
VBAT = 54.4
IREF = 2.12
R_SERIES = 0.80  # สาย + DCR ของ L + เซนส์ — ให้ ΔI ใกล้สโคป ~1.9 A
ETA = 0.88
DMAX = 0.45
DUTY_OPEN = 0.392


@dataclass
class RippleRun:
    t_ms: np.ndarray
    vin: np.ndarray
    ibat: np.ndarray
    duty: np.ndarray
    i_mean: float
    i_pp: float
    i_max: float
    f_ripple_hz: float
    label: str


def _dominant_hz(t: np.ndarray, y: np.ndarray, t_skip: float = 0.02) -> float:
    mask = t >= t_skip
    yy = y[mask] - np.mean(y[mask])
    dt = float(t[1] - t[0])
    spec = np.fft.rfft(yy)
    freq = np.fft.rfftfreq(yy.size, dt)
    spec[freq < 20.0] = 0.0
    return float(freq[int(np.argmax(np.abs(spec)))])


def simulate_ibat(
    *,
    loop_dt: float | None,
    kp: float = 0.0,
    ki: float = 0.0,
    slew: float | None = None,
    feedforward: bool = False,
    t_end: float = 0.08,
    dt: float = 20e-6,
    cin_f: float | None = None,
    label: str = "",
) -> RippleRun:
    """Average-value forward into a stiff battery, with cap-input 50 Hz rectifier."""
    dsgn = design_240vac_58v_5a()
    cin = dsgn.cin_f if cin_f is None else cin_f
    n = dsgn.n
    l = dsgn.l_h
    vac_peak = dsgn.vac_nom * np.sqrt(2.0)

    nstep = int(t_end / dt)
    t = np.arange(nstep) * dt
    vin_a = np.empty(nstep)
    i_a = np.empty(nstep)
    d_a = np.empty(nstep)

    vin = vac_peak * 0.92
    i = IREF
    d = DUTY_OPEN
    integ = d
    t_loop = 0.0

    for k in range(nstep):
        tt = t[k]
        vac = vac_peak * np.sin(2.0 * np.pi * LINE_HZ * tt)
        i_in = (i * VBAT) / max(vin, 40.0) / ETA
        if abs(vac) > vin:
            vin = abs(vac)
        else:
            vin = max(40.0, vin - i_in * dt / cin)

        if loop_dt is not None and tt - t_loop >= loop_dt - 1e-15:
            err = IREF - i
            integ = float(np.clip(integ + ki * err * loop_dt, -0.2, DMAX))
            d_cmd = kp * err + integ
            if feedforward:
                d_cmd += DUTY_OPEN * 340.0 / max(vin, 80.0) - DUTY_OPEN
            d_cmd = float(np.clip(d_cmd, 0.05, DMAX))
            if slew is not None:
                d_cmd = float(np.clip(d_cmd, d - slew, d + slew))
            d = d_cmd
            t_loop = tt
        elif feedforward and loop_dt is None:
            d = float(np.clip(DUTY_OPEN * 340.0 / max(vin, 80.0), 0.05, DMAX))

        v_sec = vin * n * d - dsgn.vf
        i = max(0.0, i + dt * (v_sec - VBAT - i * R_SERIES) / l)
        vin_a[k] = vin
        i_a[k] = i
        d_a[k] = d

    settle = t >= 0.03
    i_s = i_a[settle]
    return RippleRun(
        t_ms=t * 1e3,
        vin=vin_a,
        ibat=i_a,
        duty=d_a,
        i_mean=float(i_s.mean()),
        i_pp=float(i_s.max() - i_s.min()),
        i_max=float(i_s.max()),
        f_ripple_hz=_dominant_hz(t, i_a),
        label=label,
    )


def open_loop() -> RippleRun:
    return simulate_ibat(loop_dt=None, label="duty คงที่ / ลูปช้า")


def slow_pi_20ms() -> RippleRun:
    # 5/1023 counts per 20 ms — สลูว์ของเฟิร์มแวร์เดิม แทบไม่ไล่ 100 Hz ได้
    return simulate_ibat(
        loop_dt=0.020,
        kp=0.03,
        ki=0.6,
        slew=5.0 / 1023.0,
        label="PI ทุก 20 ms (เฟิร์มแวร์เดิม)",
    )


def fast_pi_5khz() -> RippleRun:
    return simulate_ibat(
        loop_dt=50e-6,
        kp=0.004,
        ki=12.0,
        feedforward=True,
        label="D ∝ 1/Vin + PI กระแส",
    )


def plot_ripple(open_r: RippleRun, slow: RippleRun, fast: RippleRun, path) -> None:
    W, H = 1100, 980
    svg = Svg(W, H, bg="#f4efe4")
    svg.rect(18, 16, W - 36, H - 32, fill="#fbf7ee", stroke="#d7ccba", sw=1.2, rx=14)
    svg.text(40, 50, "กระแสขาแบตแกว่งเพราะริปเปิลบัส 100 Hz — ไม่ใช่ D2", size=22, weight="700")
    svg.text(
        40,
        74,
        f"สโคป: V≈{VBAT:.1f} V, Imean≈{IREF:.2f} A, Ipp≈1.92 A, 5 ms/div  →  ครอบครัว 50 Hz ไม่ใช่ 67 kHz",
        size=13,
        fill="#5c564c",
    )

    def window(run: RippleRun) -> tuple[np.ndarray, np.ndarray]:
        m = (run.t_ms >= 30.0) & (run.t_ms <= 70.0)
        return run.t_ms[m] - 30.0, run.ibat[m]

    def panel(
        y: float,
        xs: np.ndarray,
        ys: np.ndarray,
        color: str,
        ylabel: str,
        title: str,
        stats: str,
    ) -> None:
        x, w, h = 90, 960, 165
        svg.rect(x, y, w, h, fill="#fffdf8", stroke="#d9d0c0", sw=1.0, rx=6)
        svg.text(x, y - 8, title, size=13, weight="700", fill="#3f3a33")
        svg.text(x + w, y - 8, stats, size=12, fill="#5c564c", anchor="end")
        y0, y1 = float(ys.min()), float(ys.max())
        pad = 0.12 * max(y1 - y0, 0.2)
        y0 -= pad
        y1 += pad
        x0, x1 = float(xs[0]), float(xs[-1])

        def px(v: float) -> float:
            return x + (v - x0) / (x1 - x0) * w

        def py(v: float) -> float:
            return y + h - (v - y0) / (y1 - y0) * h

        step = max(1, len(xs) // 800)
        svg.polyline([(px(a), py(b)) for a, b in zip(xs[::step], ys[::step])], stroke=color, sw=1.8)
        svg.text(x - 8, y + 14, ylabel, size=12, fill="#4a4338", anchor="end")
        svg.text(x - 8, y + h - 4, f"{y0:.1f}", size=10, fill="#8a8174", anchor="end")
        svg.text(x - 8, y + 26, f"{y1:.1f}", size=10, fill="#8a8174", anchor="end")

    tx, iy = window(open_r)
    panel(
        108,
        tx,
        iy,
        "#1aa6c1",
        "I (A)",
        open_r.label,
        f"mean {open_r.i_mean:.2f} A   pp {open_r.i_pp:.2f} A   {open_r.f_ripple_hz:.0f} Hz",
    )
    tx, iy = window(slow)
    panel(
        318,
        tx,
        iy,
        "#c45c26",
        "I (A)",
        slow.label,
        f"mean {slow.i_mean:.2f} A   pp {slow.i_pp:.2f} A   {slow.f_ripple_hz:.0f} Hz",
    )
    tx, iy = window(fast)
    panel(
        528,
        tx,
        iy,
        "#1f7a4d",
        "I (A)",
        fast.label,
        f"mean {fast.i_mean:.2f} A   pp {fast.i_pp:.2f} A",
    )
    m = (open_r.t_ms >= 30.0) & (open_r.t_ms <= 70.0)
    panel(
        738,
        open_r.t_ms[m] - 30.0,
        open_r.vin[m],
        "#a33b20",
        "Vin",
        "บัส DC หลัง Cin (ต้นเหตุ)",
        "100 Hz จากเรกติไฟเออร์ 50 Hz",
    )
    svg.text(570, 940, "เวลา 40 ms  (เทียบสโคป 5 ms/div × 8 ช่อง)", size=12, fill="#4a4338", anchor="middle")
    svg.save(path)


def main() -> None:
    from pathlib import Path

    open_r = open_loop()
    slow = slow_pi_20ms()
    fast = fast_pi_5khz()
    out = Path(__file__).resolve().parents[1] / "artifacts" / "ibat_100hz.svg"
    plot_ripple(open_r, slow, fast, out)
    for run in (open_r, slow, fast):
        print(f"{run.label:36}  mean={run.i_mean:.2f} A  pp={run.i_pp:.2f} A  f={run.f_ripple_hz:.0f} Hz")
    print(f"saved {out}")


if __name__ == "__main__":
    main()
