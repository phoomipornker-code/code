"""AC input of the bridge + Cin stage (no PFC) — matches the 5 ms/div line-side scope."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .design import design_240vac_58v_5a
from .svg import Svg


@dataclass
class LineWave:
    t_ms: np.ndarray
    vac: np.ndarray
    iac: np.ndarray
    vbus: np.ndarray
    v_rms: float
    i_rms: float
    i_peak: float
    i_pp: float
    pf: float
    p_avg: float
    crest: float
    vbus_min: float
    vbus_max: float
    vo_max_at_dmax: float


def simulate_line(
    *,
    vac_rms: float = 110.0,
    p_load: float = 115.0,
    rs: float = 5.5,
    cin_f: float | None = None,
    t_end: float = 0.08,
    dt: float = 20e-6,
) -> LineWave:
    """Cap-input rectifier: line current is pulses at the AC peaks."""
    dsgn = design_240vac_58v_5a()
    cin = dsgn.cin_f if cin_f is None else cin_f
    w = 2.0 * np.pi * 50.0
    vp = vac_rms * np.sqrt(2.0)
    nstep = int(t_end / dt)
    t = np.arange(nstep) * dt
    vac_a = vp * np.sin(w * t)
    i_a = np.empty(nstep)
    bus_a = np.empty(nstep)
    vbus = vp * 0.9

    for k in range(nstep):
        vac = float(vac_a[k])
        i_load = p_load / max(vbus, 40.0)
        if abs(vac) > vbus:
            i_chg = (abs(vac) - vbus) / rs
            vbus = vbus + dt * (i_chg - i_load) / cin
            i_line = i_chg if vac >= 0.0 else -i_chg
        else:
            vbus = max(40.0, vbus - dt * i_load / cin)
            i_line = 0.0
        i_a[k] = i_line
        bus_a[k] = vbus

    settle = t >= 0.04
    vac_s = vac_a[settle]
    i_s = i_a[settle]
    bus_s = bus_a[settle]
    v_rms = float(np.sqrt(np.mean(vac_s**2)))
    i_rms = float(np.sqrt(np.mean(i_s**2)))
    p_avg = float(np.mean(vac_s * i_s))
    pf = p_avg / max(v_rms * i_rms, 1e-9)
    i_peak = float(np.max(np.abs(i_s)))
    vo_max = float(bus_s.min() * dsgn.n * dsgn.d_max_use - dsgn.vf)
    return LineWave(
        t_ms=t * 1e3,
        vac=vac_a,
        iac=i_a,
        vbus=bus_a,
        v_rms=v_rms,
        i_rms=i_rms,
        i_peak=i_peak,
        i_pp=float(i_s.max() - i_s.min()),
        pf=pf,
        p_avg=p_avg,
        crest=i_peak / max(i_rms, 1e-9),
        vbus_min=float(bus_s.min()),
        vbus_max=float(bus_s.max()),
        vo_max_at_dmax=vo_max,
    )


def plot_line(w110: LineWave, w240: LineWave, path) -> None:
    W, H = 1100, 920
    svg = Svg(W, H, bg="#f4efe4")
    svg.rect(18, 16, W - 36, H - 32, fill="#fbf7ee", stroke="#d7ccba", sw=1.2, rx=14)
    svg.text(40, 48, "ฝั่งขาเข้า: บริดจ์ + Cin ไม่มี PFC", size=22, weight="700")
    svg.text(
        40,
        72,
        "สโคป 5 ms/div  ·  Vac RMS 110 V, Irms 1.71 A, Ipk 3.84 A  ·  พัลส์ที่ยอดไซน์ = ชาร์จ Cin",
        size=13,
        fill="#5c564c",
    )

    def window(w: LineWave) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        m = (w.t_ms >= 40.0) & (w.t_ms <= 80.0)
        return w.t_ms[m] - 40.0, w.vac[m], w.iac[m]

    def panel(y: float, xs, ys, color: str, ylabel: str, title: str, stats: str, y0=None, y1=None) -> None:
        x, ww, h = 90, 960, 155
        svg.rect(x, y, ww, h, fill="#fffdf8", stroke="#d9d0c0", sw=1.0, rx=6)
        svg.text(x, y - 8, title, size=13, weight="700")
        svg.text(x + ww, y - 8, stats, size=12, fill="#5c564c", anchor="end")
        ya, yb = (float(ys.min()), float(ys.max())) if y0 is None else (y0, y1)
        pad = 0.10 * max(yb - ya, 0.2)
        ya -= pad
        yb += pad
        x0, x1 = float(xs[0]), float(xs[-1])

        def px(v: float) -> float:
            return x + (v - x0) / (x1 - x0) * ww

        def py(v: float) -> float:
            return y + h - (v - ya) / (yb - ya) * h

        if ya < 0 < yb:
            svg.line(x, py(0), x + ww, py(0), stroke="#e2d8c8", sw=1.0)
        step = max(1, len(xs) // 900)
        svg.polyline([(px(a), py(b)) for a, b in zip(xs[::step], ys[::step])], stroke=color, sw=1.7)
        svg.text(x - 8, y + 14, ylabel, size=12, fill="#4a4338", anchor="end")

    tx, vac, iac = window(w110)
    panel(108, tx, vac, "#c9a227", "V", "CH1  แรงดันสาย (110 V RMS) — ยอดแบนเล็กน้อยเพราะชาร์จ Cin",
          f"RMS {w110.v_rms:.0f} V   pk {w110.vac.max():.0f} V")
    panel(300, tx, iac, "#1aa6c1", "A", "CH2  กระแสสาย — พัลส์แคบที่ยอดบวกและยอดลบ (ปกติของ non-PFC)",
          f"RMS {w110.i_rms:.2f} A   pk {w110.i_peak:.2f} A   PF {w110.pf:.2f}")
    m = (w110.t_ms >= 40.0) & (w110.t_ms <= 80.0)
    panel(492, w110.t_ms[m] - 40.0, w110.vbus[m], "#a33b20", "V", "บัส DC หลัง Cin — หุบ 100 Hz ตัวเดียวกับที่ไป Ibat",
          f"{w110.vbus_min:.0f}–{w110.vbus_max:.0f} V   Vo,max@D=0.45 ≈ {w110.vo_max_at_dmax:.0f} V")
    tx, _vac, _iac = window(w240)
    panel(684, tx, w240.iac, "#2a6b48", "A", "เทียบที่ 240 V RMS กำลังเท่ากัน — พัลส์เตี้ยลง PF ดีขึ้นเล็กน้อย",
          f"RMS {w240.i_rms:.2f} A   pk {w240.i_peak:.2f} A   PF {w240.pf:.2f}")
    svg.text(570, 880, "เวลา 40 ms  (5 ms/div × 8 ช่อง)   ·   ขด 48:21 ที่ 110 V สร้าง 58 V ไม่ถึง", size=13, fill="#4a4338", anchor="middle")
    svg.save(path)


def main() -> None:
    from pathlib import Path

    w110 = simulate_line(vac_rms=110.0, p_load=115.0)
    w240 = simulate_line(vac_rms=240.0, p_load=115.0, rs=6.5)
    out = Path(__file__).resolve().parents[1] / "artifacts" / "input_line.svg"
    plot_line(w110, w240, out)
    for name, w in (("110 V", w110), ("240 V", w240)):
        print(
            f"{name}: Vrms={w.v_rms:.1f}  Irms={w.i_rms:.2f}  Ipk={w.i_peak:.2f}  "
            f"PF={w.pf:.2f}  P={w.p_avg:.0f} W  bus={w.vbus_min:.0f}–{w.vbus_max:.0f}  "
            f"Vo,max={w.vo_max_at_dmax:.1f} V"
        )
    print(f"saved {out}")


if __name__ == "__main__":
    main()
