"""110 VAC + switching transformer 1:1.3 — can it explain the scope shots?"""

from __future__ import annotations

from dataclasses import dataclass

from .bus_ripple import simulate_ibat
from .design import design_240vac_58v_5a
from .input_line import simulate_line
from .svg import Svg

N_UP = 1.3  # Np:Ns = 1:1.3  (ทุติยภูมิมากกว่า)
N_DOWN = 1.0 / 1.3  # Np:Ns = 1.3:1
VBAT = 54.4
DMAX = 0.45


@dataclass
class TurnsCase:
    name: str
    n: float
    vin_min: float
    vin_max: float
    d_needed: float
    vo_max: float
    can_hold_vbat: bool
    i_pp: float
    i_mean: float


def evaluate(n: float, vac_rms: float = 110.0, name: str = "") -> TurnsCase:
    dsgn = design_240vac_58v_5a()
    line = simulate_line(vac_rms=vac_rms, p_load=VBAT * 2.12)
    vin_avg = 0.5 * (line.vbus_min + line.vbus_max)
    d_needed = (VBAT + dsgn.vf) / max(vin_avg * n, 1.0)
    vo_max = line.vbus_min * n * DMAX - dsgn.vf
    d_use = min(d_needed, DMAX)
    run = simulate_ibat(
        loop_dt=None,
        vac_rms=vac_rms,
        n_ratio=n,
        duty_init=d_use,
        label=name,
    )
    return TurnsCase(
        name=name,
        n=n,
        vin_min=line.vbus_min,
        vin_max=line.vbus_max,
        d_needed=d_needed,
        vo_max=vo_max,
        can_hold_vbat=vo_max >= VBAT and d_needed <= DMAX,
        i_pp=run.i_pp,
        i_mean=run.i_mean,
    )


def plot_cases(up: TurnsCase, down: TurnsCase, path) -> None:
    run_up = simulate_ibat(loop_dt=None, vac_rms=110.0, n_ratio=N_UP, duty_init=min(up.d_needed, DMAX), label="1:1.3")
    run_dn = simulate_ibat(loop_dt=None, vac_rms=110.0, n_ratio=N_DOWN, duty_init=min(down.d_needed, DMAX), label="1.3:1")
    run_240 = simulate_ibat(loop_dt=None, label="240 V  48:21")

    W, H = 1100, 780
    svg = Svg(W, H, bg="#f4efe4")
    svg.rect(18, 16, W - 36, H - 32, fill="#fbf7ee", stroke="#d7ccba", sw=1.2, rx=14)
    svg.text(40, 48, "110 V + หม้อแปลง 1/1.3 อธิบาย 54 V ได้ — ริปเปิล 100 Hz แย่ลง", size=20, weight="700")
    svg.text(40, 72, "พัลส์กระแสสายมาจากบริดจ์+Cin อยู่แล้ว  ไม่เกี่ยวกับอัตรารอบ", size=13, fill="#5c564c")

    def window(run):
        m = (run.t_ms >= 30.0) & (run.t_ms <= 70.0)
        return run.t_ms[m] - 30.0, run.ibat[m]

    def panel(y, run, color, title, stats):
        xs, ys = window(run)
        x, w, h = 90, 960, 165
        svg.rect(x, y, w, h, fill="#fffdf8", stroke="#d9d0c0", sw=1.0, rx=6)
        svg.text(x, y - 8, title, size=13, weight="700")
        svg.text(x + w, y - 8, stats, size=12, fill="#5c564c", anchor="end")
        y0, y1 = float(ys.min()), float(ys.max())
        pad = 0.12 * max(y1 - y0, 0.3)
        y0 -= pad
        y1 += pad
        x0, x1 = float(xs[0]), float(xs[-1])
        step = max(1, len(xs) // 800)
        pts = [
            (x + (a - x0) / (x1 - x0) * w, y + h - (b - y0) / (y1 - y0) * h)
            for a, b in zip(xs[::step], ys[::step])
        ]
        svg.polyline(pts, stroke=color, sw=1.8)
        svg.text(x - 8, y + 14, "I (A)", size=12, fill="#4a4338", anchor="end")

    panel(110, run_up, "#1aa6c1", "110 V  และ  n = Ns/Np = 1.3  (1:1.3  สเตปอัพ)",
          f"D≈{min(up.d_needed, DMAX):.2f}   Vo,max≈{up.vo_max:.0f} V   Imean {run_up.i_mean:.2f} A  pp {run_up.i_pp:.2f} A")
    panel(320, run_dn, "#c45c26", "110 V  และ  n = 1/1.3 ≈ 0.77  (1.3:1  สเตปดาวน์)",
          f"D ที่ต้อง≈{down.d_needed:.2f} (>0.45)   Vo,max≈{down.vo_max:.0f} V   ค้าง 54 V ไม่ได้")
    panel(530, run_240, "#2a6b48", "เทียบออกแบบ 240 V  48:21  (n=0.44)",
          f"Imean {run_240.i_mean:.2f} A   pp {run_240.i_pp:.2f} A")
    svg.text(570, 740, "เวลา 40 ms   ·   ริปเปิลยังเป็น 100 Hz ทุกกรณี", size=13, fill="#4a4338", anchor="middle")
    svg.save(path)


def main() -> None:
    from pathlib import Path

    up = evaluate(N_UP, name="n=1.3")
    down = evaluate(N_DOWN, name="n=1/1.3")
    for c in (up, down):
        print(
            f"{c.name:12} n={c.n:.3f}  bus={c.vin_min:.0f}–{c.vin_max:.0f} V  "
            f"Dneed={c.d_needed:.3f}  Vo,max={c.vo_max:.1f} V  "
            f"hold54={c.can_hold_vbat}  Imean={c.i_mean:.2f} A  Ipp={c.i_pp:.2f} A"
        )
    out = Path(__file__).resolve().parents[1] / "artifacts" / "hw_110_n13.svg"
    plot_cases(up, down, out)
    print(f"saved {out}")


if __name__ == "__main__":
    main()
