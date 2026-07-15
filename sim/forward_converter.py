#!/usr/bin/env python3
"""จำลอง waveform อุดมคติของ Single-Switch Forward + Nr (CCM)."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

# --- พารามิเตอร์เริ่มต้น (ตัวอย่างแรงดันต่ำ) ---
VIN = 48.0
NP, NS, NR = 2.0, 1.0, 2.0
D = 0.40
FS = 100e3
L = 47e-6
IO = 2.0
LM = 200e-6
N_CYCLES = 3
POINTS_PER_CYCLE = 1000

OUT_DIR = Path(__file__).resolve().parents[1] / "artifacts"
OUT_PNG = OUT_DIR / "forward_converter_waveforms.png"
OUT_PNG_240 = OUT_DIR / "forward_240vac_58v_5a_nr_waveforms.png"


def preset_240vac_nr() -> dict[str, float]:
    """จุดทำงานปกติ: Vin≈340 V, n=0.44, Nr=Np, Vo≈58 V / 5 A."""
    vin = 340.0
    n = 0.44
    d = (58.0 + 0.7) / (vin * n)  # ≈ 0.393
    return {
        "vin": vin,
        "np_": 25.0,
        "ns": 11.0,
        "nr": 25.0,
        "d": d,
        "fs": 100e3,
        "l": 360e-6,
        "io": 5.0,
        "lm": 2.0e-3,  # Lm ปฐมภูมิประมาณสำหรับออฟไลน์
        "vo_target": 58.0,
    }


def ideal_forward_waveforms(
    vin: float = VIN,
    np_: float = NP,
    ns: float = NS,
    nr: float = NR,
    d: float = D,
    fs: float = FS,
    l: float = L,
    io: float = IO,
    lm: float = LM,
    n_cycles: int = N_CYCLES,
    points_per_cycle: int = POINTS_PER_CYCLE,
    vo_target: float | None = None,
) -> dict[str, np.ndarray | float]:
    """สร้างคลื่นเวลาแบบชิ้นส่วนเชิงเส้นสำหรับ forward + Nr อุดมคติ."""
    d_max = 1.0 / (1.0 + nr / np_)
    if not 0.0 < d < d_max:
        raise ValueError(f"ต้องมี 0 < D < Dmax={d_max:.3f} สำหรับ Nr/Np={nr/np_:.2f}")

    ts = 1.0 / fs
    n_ratio = ns / np_
    vs = vin * n_ratio
    vo = vo_target if vo_target is not None else vs * d
    r_load = vo / io

    t = np.linspace(0.0, n_cycles * ts, n_cycles * points_per_cycle, endpoint=False)
    phase = np.mod(t, ts)
    on = phase < (d * ts)

    v_gs = np.where(on, 1.0, 0.0)
    v_rect = np.where(on, vs, 0.0)

    # Vds อุดมคติ: ตอน ON ≈ 0, ตอนรีเซ็ต ≈ Vin*(1+Np/Nr), หลังรีเซ็ต ≈ Vin
    t_on = d * ts
    t_reset = (nr / np_) * t_on
    v_ds = np.empty_like(t)
    for i, ph in enumerate(phase):
        if ph < t_on:
            v_ds[i] = 0.0
        elif ph < t_on + t_reset:
            v_ds[i] = vin * (1.0 + np_ / nr)
        else:
            v_ds[i] = vin

    di = ((vs - vo) * d * ts) / l
    i_l = np.empty_like(t)
    for i, ph in enumerate(phase):
        if ph < t_on:
            i_l[i] = (io - 0.5 * di) + ((vs - vo) / l) * ph
        else:
            i_l[i] = (io + 0.5 * di) - (vo / l) * (ph - t_on)

    i_d1 = np.where(on, i_l, 0.0)
    i_d2 = np.where(~on, i_l, 0.0)

    # กระแสมากเนไทซิ่ง + รีเซ็ตด้วย Nr
    im = np.empty_like(t)
    for i, ph in enumerate(phase):
        if ph < t_on:
            im[i] = (vin / lm) * ph
        else:
            im_peak = (vin / lm) * t_on
            t_off = ph - t_on
            # รีเซ็ต: dφ/dt = Vin/Nr → di_m/dt สะท้อนที่ปฐมภูมิ
            # i_m ลดด้วยความชัน -(vin/lm)*(np_/nr)
            if t_off <= t_reset:
                im[i] = im_peak - (vin / lm) * (np_ / nr) * t_off
            else:
                im[i] = 0.0

    return {
        "t_us": t * 1e6,
        "v_gs": v_gs,
        "v_rect": v_rect,
        "v_ds": v_ds,
        "i_l": i_l,
        "i_d1": i_d1,
        "i_d2": i_d2,
        "i_m": im,
        "vo": vo,
        "vs": vs,
        "r_load": r_load,
        "di": di,
        "fs": fs,
        "d": d,
        "vin": vin,
        "n_ratio": n_ratio,
        "nr_over_np": nr / np_,
        "d_max": d_max,
        "vds_reset": vin * (1.0 + np_ / nr),
    }


def plot_waveforms(
    data: dict[str, np.ndarray | float],
    out_path: Path = OUT_PNG,
    title_prefix: str = "Single-Switch Forward + Nr",
) -> Path:
    """วาดคลื่นหลักและบันทึกเป็น PNG."""
    t = data["t_us"]
    fig, axes = plt.subplots(5, 1, figsize=(10, 11), sharex=True)

    axes[0].plot(t, data["v_gs"], color="#0b3d5c", lw=1.6)
    axes[0].set_ylabel("Gate (norm.)")
    axes[0].set_ylim(-0.1, 1.2)
    axes[0].set_title(
        f"{title_prefix} — Ideal CCM\n"
        f"Vin={data['vin']:.0f} V, Ns/Np={data['n_ratio']:.2f}, "
        f"Nr/Np={data['nr_over_np']:.2f}, D={data['d']:.3f}, "
        f"fs={data['fs']/1e3:.0f} kHz → Vo≈{data['vo']:.2f} V"
    )
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, data["v_ds"], color="#a33b20", lw=1.6)
    axes[1].set_ylabel("v_DS (V)")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t, data["v_rect"], color="#c45c26", lw=1.6)
    axes[2].set_ylabel("v_rect (V)")
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(t, data["i_l"], color="#1f7a4d", lw=1.6, label="i_L")
    axes[3].plot(t, data["i_d1"], color="#3a7ca5", lw=1.0, alpha=0.85, label="i_D1")
    axes[3].plot(t, data["i_d2"], color="#8b5e34", lw=1.0, alpha=0.85, label="i_D2")
    axes[3].set_ylabel("Current (A)")
    axes[3].legend(loc="upper right", fontsize=8, ncol=3)
    axes[3].grid(True, alpha=0.3)

    axes[4].plot(t, data["i_m"], color="#5c4d7a", lw=1.6)
    axes[4].set_ylabel("i_m (A)")
    axes[4].set_xlabel("Time (µs)")
    axes[4].grid(True, alpha=0.3)

    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def main() -> None:
    parser = argparse.ArgumentParser(description="Forward converter ideal waveforms")
    parser.add_argument(
        "--design",
        choices=("default", "240vac-nr"),
        default="default",
        help="เลือกชุดพารามิเตอร์",
    )
    args = parser.parse_args()

    if args.design == "240vac-nr":
        kw = preset_240vac_nr()
        data = ideal_forward_waveforms(**kw)
        path = plot_waveforms(
            data,
            out_path=OUT_PNG_240,
            title_prefix="AC240→DC58V/5A  Single-Switch + Nr",
        )
    else:
        data = ideal_forward_waveforms()
        path = plot_waveforms(data)

    print("Single-Switch Forward + Nr (ideal CCM)")
    print(f"  Vin               = {data['vin']:.1f} V")
    print(f"  Ns/Np, Nr/Np      = {data['n_ratio']:.3f}, {data['nr_over_np']:.3f}")
    print(f"  D / Dmax          = {data['d']:.3f} / {data['d_max']:.3f}")
    print(f"  Vs (secondary ON) = {data['vs']:.2f} V")
    print(f"  Vo                = {data['vo']:.2f} V")
    print(f"  Vds during reset  = {data['vds_reset']:.1f} V")
    print(f"  R_load            = {data['r_load']:.2f} Ω")
    print(f"  ΔI_L              = {data['di']:.3f} A")
    print(f"  Waveform saved → {path}")


if __name__ == "__main__":
    main()
