#!/usr/bin/env python3
"""จำลอง waveform อุดมคติของ Single-Ended Forward Converter (CCM)."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

# --- พารามิเตอร์วงจร (อุดมคติ, CCM) ---
VIN = 48.0  # V
NP, NS = 2.0, 1.0  # อัตราส่วนหม้อแปลง
D = 0.40  # duty cycle (< 0.5 สำหรับ reset 1:1)
FS = 100e3  # Hz
L = 47e-6  # H ตัวเหนี่ยวนำเอาต์พุต
IO = 2.0  # A กระแสโหลดเฉลี่ย
N_CYCLES = 3
POINTS_PER_CYCLE = 1000

OUT_DIR = Path(__file__).resolve().parents[1] / "artifacts"
OUT_PNG = OUT_DIR / "forward_converter_waveforms.png"


def ideal_forward_waveforms(
    vin: float = VIN,
    np_: float = NP,
    ns: float = NS,
    d: float = D,
    fs: float = FS,
    l: float = L,
    io: float = IO,
    n_cycles: int = N_CYCLES,
    points_per_cycle: int = POINTS_PER_CYCLE,
) -> dict[str, np.ndarray | float]:
    """สร้างคลื่นเวลาแบบชิ้นส่วนเชิงเส้นสำหรับ forward converter อุดมคติ."""
    if not 0.0 < d < 0.5:
        raise ValueError("สำหรับ reset winding 1:1 ควรใช้ 0 < D < 0.5")

    ts = 1.0 / fs
    vs = vin * (ns / np_)  # แรงดันทุติยภูมิตอน ON
    vo = vs * d  # อุดมคติ CCM
    r_load = vo / io

    t = np.linspace(0.0, n_cycles * ts, n_cycles * points_per_cycle, endpoint=False)
    phase = np.mod(t, ts)
    on = phase < (d * ts)

    v_gs = np.where(on, 1.0, 0.0)
    v_sec = np.where(on, vs, 0.0)  # หลังเรกติไฟเออร์เข้าโหนด LC (ก่อน L)
    # ตอน OFF โหนดหลัง D1 ถูกดึงลงใกล้ 0 ผ่าน D2 (อุดมคติ)
    v_rect = np.where(on, vs, 0.0)

    # กระแสใน L: สามเหลี่ยมรอบค่าเฉลี่ย IO
    di = ((vs - vo) * d * ts) / l
    # ในช่วง ON: เพิ่มขึ้นจาก IO - di/2, ช่วง OFF: ลดลง
    t_on = d * ts
    i_l = np.empty_like(t)
    for i, ph in enumerate(phase):
        if ph < t_on:
            i_l[i] = (io - 0.5 * di) + ((vs - vo) / l) * ph
        else:
            i_l[i] = (io + 0.5 * di) - (vo / l) * (ph - t_on)

    i_d1 = np.where(on, i_l, 0.0)
    i_d2 = np.where(~on, i_l, 0.0)

    # กระแสมากเนไทซิ่งปฐมภูมิ (สมมติ Lm)
    lm = 200e-6
    im = np.empty_like(t)
    for i, ph in enumerate(phase):
        if ph < t_on:
            im[i] = (vin / lm) * ph
        else:
            # รีเซ็ตด้วย -Vin ผ่าน reset winding 1:1 จนกลับใกล้ 0
            im_peak = (vin / lm) * t_on
            t_off = ph - t_on
            t_reset = t_on  # reset 1:1
            if t_off <= t_reset:
                im[i] = im_peak - (vin / lm) * t_off
            else:
                im[i] = 0.0

    return {
        "t_us": t * 1e6,
        "v_gs": v_gs,
        "v_rect": v_rect,
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
    }


def plot_waveforms(data: dict[str, np.ndarray | float], out_path: Path = OUT_PNG) -> Path:
    """วาดคลื่นหลักและบันทึกเป็น PNG."""
    t = data["t_us"]
    fig, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True)

    axes[0].plot(t, data["v_gs"], color="#0b3d5c", lw=1.6)
    axes[0].set_ylabel("Gate (norm.)")
    axes[0].set_ylim(-0.1, 1.2)
    axes[0].set_title(
        "Forward Converter — Ideal CCM Waveforms\n"
        f"Vin={data['vin']:.0f} V, Ns/Np={NS/NP:.2f}, D={data['d']:.2f}, "
        f"fs={data['fs']/1e3:.0f} kHz → Vo≈{data['vo']:.2f} V"
    )
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, data["v_rect"], color="#c45c26", lw=1.6)
    axes[1].set_ylabel("v_rect (V)")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t, data["i_l"], color="#1f7a4d", lw=1.6, label="i_L")
    axes[2].plot(t, data["i_d1"], color="#3a7ca5", lw=1.0, alpha=0.85, label="i_D1")
    axes[2].plot(t, data["i_d2"], color="#8b5e34", lw=1.0, alpha=0.85, label="i_D2")
    axes[2].set_ylabel("Current (A)")
    axes[2].legend(loc="upper right", fontsize=8, ncol=3)
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(t, data["i_m"], color="#5c4d7a", lw=1.6)
    axes[3].set_ylabel("i_m (A)")
    axes[3].set_xlabel("Time (µs)")
    axes[3].grid(True, alpha=0.3)

    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def main() -> None:
    data = ideal_forward_waveforms()
    path = plot_waveforms(data)
    print("Forward Converter (ideal CCM)")
    print(f"  Vs (secondary ON) = {data['vs']:.2f} V")
    print(f"  Vo                = {data['vo']:.2f} V")
    print(f"  R_load            = {data['r_load']:.2f} Ω")
    print(f"  ΔI_L              = {data['di']:.3f} A")
    print(f"  Waveform saved → {path}")


if __name__ == "__main__":
    main()
