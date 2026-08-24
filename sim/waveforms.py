"""Ideal CCM waveforms for a single-switch forward + Nr."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .design import ForwardDesign, design_240vac_58v_5a
from .svg import Svg


@dataclass
class Waveforms:
    t_us: np.ndarray
    v_gs: np.ndarray
    v_ds: np.ndarray
    v_rect: np.ndarray
    i_l: np.ndarray
    i_d1: np.ndarray
    i_d2: np.ndarray
    i_m: np.ndarray
    vin: float
    vo: float
    vs: float
    d: float
    d_max: float
    fs: float
    di: float
    vds_reset: float
    im_peak: float


def ideal_ccm(
    design: ForwardDesign | None = None,
    vin: float | None = None,
    n_cycles: int = 3,
    points_per_cycle: int = 800,
) -> Waveforms:
    dsgn = design or design_240vac_58v_5a()
    vin = dsgn.vin_nom if vin is None else vin
    d = dsgn.duty(vin)
    d_max = dsgn.d_max_reset
    if not 0.0 < d < d_max:
        raise ValueError(f"D={d:.3f} must be in (0, Dmax={d_max:.3f})")

    ts = 1.0 / dsgn.fs
    vs = vin * dsgn.n
    vo = dsgn.vo
    t = np.linspace(0.0, n_cycles * ts, n_cycles * points_per_cycle, endpoint=False)
    phase = np.mod(t, ts)
    t_on = d * ts
    t_reset = (dsgn.nr / dsgn.np) * t_on
    on = phase < t_on

    v_gs = np.where(on, 1.0, 0.0)
    v_rect = np.where(on, vs, 0.0)

    v_ds = np.empty_like(t)
    im = np.empty_like(t)
    i_l = np.empty_like(t)
    di = ((vs - vo) * t_on) / dsgn.l_h
    im_peak = (vin / dsgn.lm) * t_on
    i0 = dsgn.io - 0.5 * di

    for i, ph in enumerate(phase):
        if ph < t_on:
            v_ds[i] = 0.0
            im[i] = (vin / dsgn.lm) * ph
            i_l[i] = i0 + ((vs - vo) / dsgn.l_h) * ph
        elif ph < t_on + t_reset:
            v_ds[i] = vin * (1.0 + dsgn.np / dsgn.nr)
            im[i] = im_peak - (vin / dsgn.lm) * (dsgn.np / dsgn.nr) * (ph - t_on)
            i_l[i] = (i0 + di) - (vo / dsgn.l_h) * (ph - t_on)
        else:
            v_ds[i] = vin
            im[i] = 0.0
            i_l[i] = (i0 + di) - (vo / dsgn.l_h) * (ph - t_on)

    return Waveforms(
        t_us=t * 1e6,
        v_gs=v_gs,
        v_ds=v_ds,
        v_rect=v_rect,
        i_l=i_l,
        i_d1=np.where(on, i_l, 0.0),
        i_d2=np.where(~on, i_l, 0.0),
        i_m=im,
        vin=vin,
        vo=vo,
        vs=vs,
        d=d,
        d_max=d_max,
        fs=dsgn.fs,
        di=di,
        vds_reset=vin * (1.0 + dsgn.np / dsgn.nr),
        im_peak=im_peak,
    )


def _axis(
    svg: Svg,
    x: float,
    y: float,
    w: float,
    h: float,
    xs: np.ndarray,
    ys: np.ndarray,
    color: str,
    ylabel: str,
    y_min: float | None = None,
    y_max: float | None = None,
    extra: list[tuple[np.ndarray, str, float]] | None = None,
) -> None:
    svg.rect(x, y, w, h, fill="#fffdf8", stroke="#d9d0c0", sw=1.0, rx=6)
    y0 = float(ys.min()) if y_min is None else y_min
    y1 = float(ys.max()) if y_max is None else y_max
    if y1 <= y0:
        y1 = y0 + 1.0
    pad = 0.08 * (y1 - y0)
    if y_min is None:
        y0 -= pad
    y1 += pad
    x0, x1 = float(xs[0]), float(xs[-1])

    def px(xv: float) -> float:
        return x + (xv - x0) / (x1 - x0) * w

    def py(yv: float) -> float:
        return y + h - (yv - y0) / (y1 - y0) * h

    # zero line if in range
    if y0 < 0 < y1:
        svg.line(x, py(0), x + w, py(0), stroke="#e2d8c8", sw=1.0)

    def stroke_series(xv: np.ndarray, yv: np.ndarray, col: str, sw: float) -> None:
        pts = [(px(a), py(b)) for a, b in zip(xv[::2], yv[::2])]
        svg.polyline(pts, stroke=col, sw=sw)

    stroke_series(xs, ys, color, 1.8)
    if extra:
        for yv, col, sw in extra:
            stroke_series(xs, yv, col, sw)

    svg.text(x - 8, y + 14, ylabel, size=12, fill="#4a4338", anchor="end")
    svg.text(x - 8, y + h - 4, f"{y0:.0f}", size=10, fill="#8a8174", anchor="end")
    svg.text(x - 8, y + 26, f"{y1:.0f}", size=10, fill="#8a8174", anchor="end")


def plot_waveforms(wf: Waveforms, path) -> None:
    W, H = 1100, 920
    svg = Svg(W, H, bg="#f4efe4")
    svg.rect(18, 16, W - 36, H - 32, fill="#fbf7ee", stroke="#d7ccba", sw=1.2, rx=14)
    svg.text(40, 52, "คลื่นอุดมคติ CCM — Single-Switch Forward + Nr", size=22, weight="700")
    svg.text(
        40,
        76,
        f"Vin={wf.vin:.0f} V   Vo={wf.vo:.0f} V   D={wf.d:.3f} (Dmax={wf.d_max:.2f})   "
        f"fs={wf.fs/1e3:.0f} kHz   ΔI_L={wf.di:.2f} A   Vds,reset={wf.vds_reset:.0f} V",
        size=13,
        fill="#5c564c",
    )

    left, width, top, panel_h, gap = 90, 960, 100, 140, 16
    xs = wf.t_us
    _axis(svg, left, top + 0 * (panel_h + gap), width, panel_h, xs, wf.v_gs, "#0b3d5c", "Gate", -0.15, 1.2)
    _axis(svg, left, top + 1 * (panel_h + gap), width, panel_h, xs, wf.v_ds, "#a33b20", "v_DS (V)", y_min=0.0)
    _axis(svg, left, top + 2 * (panel_h + gap), width, panel_h, xs, wf.v_rect, "#c45c26", "v_rect (V)", y_min=0.0)
    _axis(
        svg,
        left,
        top + 3 * (panel_h + gap),
        width,
        panel_h,
        xs,
        wf.i_l,
        "#1f7a4d",
        "i (A)",
        extra=[(wf.i_d1, "#3a7ca5", 1.2), (wf.i_d2, "#8b5e34", 1.2)],
    )
    last_y = top + 4 * (panel_h + gap)
    _axis(svg, left, last_y, width, panel_h, xs, wf.i_m, "#5c4d7a", "i_m (A)")
    svg.text(left + width / 2, last_y + panel_h + 28, "เวลา (µs)", size=13, fill="#4a4338", anchor="middle")
    svg.text(left + 8, top + 3 * (panel_h + gap) + 18, "i_L", size=11, fill="#1f7a4d")
    svg.text(left + 40, top + 3 * (panel_h + gap) + 18, "i_D1", size=11, fill="#3a7ca5")
    svg.text(left + 88, top + 3 * (panel_h + gap) + 18, "i_D2", size=11, fill="#8b5e34")
    svg.save(path)


def main() -> None:
    from pathlib import Path

    out = Path(__file__).resolve().parents[1] / "artifacts" / "forward_waveforms.svg"
    wf = ideal_ccm()
    plot_waveforms(wf, out)
    print(f"D={wf.d:.3f}  Vs={wf.vs:.1f} V  Vds,reset={wf.vds_reset:.0f} V")
    print(f"ΔI_L={wf.di:.3f} A  Im,pk={wf.im_peak:.3f} A")
    print(f"saved {out}")


if __name__ == "__main__":
    main()
