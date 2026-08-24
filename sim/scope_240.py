"""Predicted Vout / Iout on the scope: 240 VAC, transformer 48:21."""

from __future__ import annotations

from .bus_ripple import simulate_ibat
from .design import design_240vac_58v_5a
from .svg import Svg
from .waveforms import ideal_ccm


VBAT = 54.4  # CC เข้าแบต 16S — โหมดเดียวกับสโคปเดิม
IOUT = 5.0
R_BATT = 0.040  # 16S pack ESR — Vout แทบไม่ขยับที่ 50 V/div


def _duty_for(vbat: float, iout: float, vin: float = 340.0) -> float:
    dsgn = design_240vac_58v_5a()
    vsec = vbat + dsgn.vf + iout * 0.80
    return vsec / (vin * dsgn.n)


def line_scale(feedforward: bool):
    d0 = _duty_for(VBAT, IOUT)
    return simulate_ibat(
        loop_dt=50e-6 if feedforward else None,
        kp=0.004 if feedforward else 0.0,
        ki=12.0 if feedforward else 0.0,
        feedforward=feedforward,
        duty_init=d0,
        vff_nom=340.0,
        vbat=VBAT,
        iref=IOUT,
        label="Vff" if feedforward else "open",
    )


def plot_scope(path) -> None:
    dsgn = design_240vac_58v_5a()
    open_r = line_scale(False)
    vff_r = line_scale(True)
    sw = ideal_ccm(dsgn, vin=dsgn.vin_nom, n_cycles=4)
    esr_c = 0.035
    v_sw = dsgn.vo + (sw.i_l - dsgn.io) * esr_c

    W, H = 1100, 980
    svg = Svg(W, H, bg="#1a2330")
    svg.rect(18, 16, W - 36, H - 32, fill="#0f1824", stroke="#3d5a73", sw=1.2, rx=14)
    svg.text(40, 48, "240 V  ·  48:21  ·  สัญญาณ Vout / Iout", size=22, weight="700", fill="#e8eef4")
    svg.text(
        40,
        72,
        f"จุดทำงาน CC  {dsgn.vin_nom:.0f} Vbus  n={dsgn.n:.3f}  D≈0.40  "
        f"54.4 V / 5 A   (CV 58 V รัด Dmax ที่หุบบัสถ้า Cin 220 µF)",
        size=13,
        fill="#9bb0c2",
    )

    def panel(y, xs, y1, y2, c1, c2, title, stats, ylab1, ylab2, lim1, lim2):
        x, w, h = 80, 980, 175
        svg.rect(x, y, w, h, fill="#0b121c", stroke="#2c4458", sw=1.0, rx=6)
        svg.text(x, y - 8, title, size=13, weight="700", fill="#d5e4f0")
        svg.text(x + w, y - 8, stats, size=12, fill="#8fa3b5", anchor="end")
        x0, x1 = float(xs[0]), float(xs[-1])
        step = max(1, len(xs) // 900)

        def px(v: float) -> float:
            return x + (v - x0) / (x1 - x0) * w

        def draw(ys, col, lo, hi):
            def py(v: float) -> float:
                return y + h - (v - lo) / (hi - lo) * h

            svg.polyline(
                [(px(a), py(b)) for a, b in zip(xs[::step], ys[::step])],
                stroke=col,
                sw=1.8,
            )

        draw(y1, c1, lim1[0], lim1[1])
        draw(y2, c2, lim2[0], lim2[1])
        svg.text(x - 6, y + 16, ylab1, size=11, fill=c1, anchor="end")
        svg.text(x - 6, y + 32, ylab2, size=11, fill=c2, anchor="end")

    m = (open_r.t_ms >= 30.0) & (open_r.t_ms <= 70.0)
    t5 = open_r.t_ms[m] - 30.0
    v_open = VBAT + (open_r.ibat[m] - IOUT) * R_BATT
    v_vff = VBAT + (vff_r.ibat[m] - IOUT) * R_BATT
    panel(
        110,
        t5,
        v_open,
        open_r.ibat[m],
        "#e6c35c",
        "#3ec6e0",
        "5 ms/div   duty คงที่ (ยังมี 100 Hz จากบัส)   CH1 50 V/div  CH2 2 A/div",
        f"V≈{v_open.mean():.1f} V   I mean {open_r.i_mean:.2f} A   I pp {open_r.i_pp:.2f} A",
        "V",
        "A",
        (8.0, 108.0),
        (0.0, 12.0),
    )
    panel(
        330,
        t5,
        v_vff,
        vff_r.ibat[m],
        "#e6c35c",
        "#3ec6e0",
        "5 ms/div   D ∝ 1/Vin   CH1 50 V/div  CH2 2 A/div",
        f"V≈{v_vff.mean():.1f} V   I mean {vff_r.i_mean:.2f} A   I pp {vff_r.i_pp:.2f} A",
        "V",
        "A",
        (8.0, 108.0),
        (0.0, 12.0),
    )
    panel(
        550,
        sw.t_us,
        v_sw,
        sw.i_l,
        "#e6c35c",
        "#3ec6e0",
        "10 µs/div   ริปเปิลสวิตช์ 67 kHz   CH1 100 mV/div  CH2 1 A/div",
        f"ΔI_L={sw.di:.2f} A   ΔV≈{(v_sw.max()-v_sw.min())*1e3:.0f} mV",
        "V",
        "A",
        (57.70, 58.30),
        (3.5, 6.5),
    )
    svg.rect(80, 780, 980, 140, fill="#0b121c", stroke="#2c4458", sw=1.0, rx=6)
    svg.text(100, 812, "เทียบกับสโคป 110 V + 1:1.3 ที่เคยจับ", size=14, weight="700", fill="#d5e4f0")
    svg.text(100, 838, "Vout ที่ 50 V/div จะดูเป็นเส้นตรงทั้งคู่  (แบตยึดโวลต์)", size=13, fill="#9bb0c2")
    svg.text(100, 860, "Iout ที่ 5 ms/div:  110 V+1.3  แกว่งแรง 100 Hz   ·   240 V+48:21  เบากว่า  และเรียบเกือบหมดถ้าชดเชย Vin", size=13, fill="#9bb0c2")
    svg.text(100, 882, "Iout ที่ 10 µs/div:  สามเหลี่ยม ~1 A รอบ 5 A  ที่ 67 kHz  — นี่คือของ L 520 µH ถือว่าปกติ", size=13, fill="#9bb0c2")
    svg.save(path)


def main() -> None:
    from pathlib import Path

    dsgn = design_240vac_58v_5a()
    open_r = line_scale(False)
    vff_r = line_scale(True)
    sw = ideal_ccm()
    print(f"240 V  48:21  D={sw.d:.3f}  Vo={sw.vo:.1f} V  ΔI_L={sw.di:.2f} A")
    print(f"5 ms  open-loop   Imean={open_r.i_mean:.2f} A  Ipp={open_r.i_pp:.2f} A")
    print(f"5 ms  Vin-FF      Imean={vff_r.i_mean:.2f} A  Ipp={vff_r.i_pp:.2f} A")
    out = Path(__file__).resolve().parents[1] / "artifacts" / "scope_240_48_21.svg"
    plot_scope(out)
    print(f"saved {out}")
    _ = dsgn


if __name__ == "__main__":
    main()
