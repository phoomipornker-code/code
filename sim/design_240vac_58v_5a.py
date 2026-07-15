#!/usr/bin/env python3
"""ออกแบบ Single-Switch Forward + Nr: AC 240 V → DC 58 V / 5 A."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Spec:
    vac_nom: float = 240.0
    vac_tol: float = 0.10
    vo: float = 58.0
    io: float = 5.0
    vf: float = 0.7
    eta: float = 0.90
    fs: float = 100e3
    d_max: float = 0.45
    nr_over_np: float = 1.0  # Nr/Np = 1 → reset 1:1
    vin_ripple_margin: float = 5.0
    di_l_ratio: float = 0.20
    line_freq: float = 50.0
    vin_delta: float = 20.0


def ac_to_vdc_peak(vac: float) -> float:
    return vac * math.sqrt(2.0)


def design(spec: Spec = Spec()) -> dict[str, float]:
    vac_min = spec.vac_nom * (1.0 - spec.vac_tol)
    vac_max = spec.vac_nom * (1.0 + spec.vac_tol)
    vin_min = ac_to_vdc_peak(vac_min) - spec.vin_ripple_margin
    vin_nom = ac_to_vdc_peak(spec.vac_nom)
    vin_max = ac_to_vdc_peak(vac_max)

    po = spec.vo * spec.io
    pin = po / spec.eta
    v_out_need = spec.vo + spec.vf

    # Dmax จาก Nr: D <= 1/(1+Nr/Np)
    d_max_theory = 1.0 / (1.0 + spec.nr_over_np)
    d_max_use = min(spec.d_max, 0.95 * d_max_theory)

    n_calc = v_out_need / (vin_min * d_max_use)
    # ปัดขึ้น 2 ตำแหน่ง เพื่อให้ D ที่ Vin_min ไม่เกิน Dmax
    n_use = math.ceil(n_calc * 100.0 - 1e-12) / 100.0

    d_at_vin_min = v_out_need / (vin_min * n_use)
    d_at_vin_nom = v_out_need / (vin_nom * n_use)
    d_at_vin_max = v_out_need / (vin_max * n_use)

    vs_nom = vin_nom * n_use
    vs_max = vin_max * n_use
    di_l = spec.io * spec.di_l_ratio
    l_out = spec.vo * (1.0 - d_at_vin_nom) / (spec.fs * di_l)

    ip_sec_reflected = spec.io * n_use
    ip_peak = ip_sec_reflected / spec.eta + 0.15 * ip_sec_reflected
    ip_rms_approx = ip_peak * math.sqrt(d_at_vin_nom)

    # Single-switch: Vds ≈ Vin*(1 + Np/Nr) = Vin*(1 + 1/(Nr/Np))
    vds_ideal = vin_max * (1.0 + 1.0 / spec.nr_over_np)
    vds_with_margin = vds_ideal * 1.15  # leakage / spike margin ~15%

    c_in = pin / (2.0 * math.pi * spec.line_freq * vin_nom * spec.vin_delta)

    # ตัวอย่างจำนวนรอบ (Np อ้างอิง)
    np_turns = 25.0
    ns_turns = round(np_turns * n_use)
    nr_turns = round(np_turns * spec.nr_over_np)

    return {
        "vac_min": vac_min,
        "vac_max": vac_max,
        "vin_min": vin_min,
        "vin_nom": vin_nom,
        "vin_max": vin_max,
        "po": po,
        "pin": pin,
        "nr_over_np": spec.nr_over_np,
        "d_max_theory": d_max_theory,
        "d_max_use": d_max_use,
        "n_calc": n_calc,
        "n_use": n_use,
        "np_turns": np_turns,
        "ns_turns": ns_turns,
        "nr_turns": nr_turns,
        "d_at_vin_min": d_at_vin_min,
        "d_at_vin_nom": d_at_vin_nom,
        "d_at_vin_max": d_at_vin_max,
        "vs_nom": vs_nom,
        "vs_max": vs_max,
        "di_l": di_l,
        "l_out_h": l_out,
        "ip_peak": ip_peak,
        "ip_rms_approx": ip_rms_approx,
        "c_in_f": c_in,
        "vds_ideal": vds_ideal,
        "vds_with_margin": vds_with_margin,
        "diode_vrrm": vs_max,
        "d1_iavg": spec.io * d_at_vin_nom,
        "d2_iavg": spec.io * (1.0 - d_at_vin_nom),
        "fs": spec.fs,
        "vo": spec.vo,
        "io": spec.io,
    }


def print_report(d: dict[str, float]) -> None:
    print("=" * 62)
    print("Single-Switch Forward + Nr reset")
    print("AC 240 V → DC 58 V / 5 A")
    print("=" * 62)
    print(f"Po / Pin (@η)          = {d['po']:.1f} W / {d['pin']:.1f} W")
    print()
    print("--- DC bus ---")
    print(f"Vac range              = {d['vac_min']:.1f} … {d['vac_max']:.1f} V")
    print(
        f"Vin min/nom/max        = "
        f"{d['vin_min']:.1f} / {d['vin_nom']:.1f} / {d['vin_max']:.1f} V"
    )
    print(f"Cin (ΔV=20V, 50Hz)     ≈ {d['c_in_f'] * 1e6:.0f} µF / 400–450 V")
    print()
    print("--- Reset winding Nr ---")
    print(f"Nr/Np                  = {d['nr_over_np']:.2f}")
    print(f"Dmax theory / use      = {d['d_max_theory']:.3f} / {d['d_max_use']:.3f}")
    print(
        f"Vds ideal / +15%       = "
        f"{d['vds_ideal']:.0f} / {d['vds_with_margin']:.0f} V  → MOSFET 900–1000 V"
    )
    print()
    print("--- Transformer ---")
    print(f"n=Ns/Np (calc→use)     = {d['n_calc']:.4f} → {d['n_use']:.2f}")
    print(
        f"Example turns Np:Ns:Nr = "
        f"{d['np_turns']:.0f}:{d['ns_turns']:.0f}:{d['nr_turns']:.0f}"
    )
    print(
        f"D @ Vin min/nom/max    = "
        f"{d['d_at_vin_min']:.3f} / {d['d_at_vin_nom']:.3f} / {d['d_at_vin_max']:.3f}"
    )
    print(f"Vs nom / max           = {d['vs_nom']:.1f} / {d['vs_max']:.1f} V")
    print()
    print("--- Output LC ---")
    print(f"ΔI_L                   = {d['di_l']:.2f} A")
    print(f"L_out                  ≈ {d['l_out_h'] * 1e6:.0f} µH  (choose 330–390 µH)")
    print("C_out                  = 470–1000 µF / ≥80 V + MLCC")
    print()
    print("--- Device stress ---")
    print(f"Q1 Ipeak / Irms≈       = {d['ip_peak']:.2f} / {d['ip_rms_approx']:.2f} A")
    print(f"D1/D2 VRRM             ≥ {d['diode_vrrm']:.0f} V  → use 200–300 V")
    print(f"D1 / D2 Iavg           ≈ {d['d1_iavg']:.2f} / {d['d2_iavg']:.2f} A")
    print(f"Dr                     = ultrafast, V >= Vin_max (~{d['vin_max']:.0f} V)")
    print(f"fs                     = {d['fs'] / 1e3:.0f} kHz")
    print("=" * 62)
    print("หมายเหตุ: ต้องมี RCD/snubber ที่ drain ของ Q1 ตัด leakage spike")


def main() -> None:
    print_report(design())


if __name__ == "__main__":
    main()
