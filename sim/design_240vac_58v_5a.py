#!/usr/bin/env python3
"""ออกแบบ Single-Switch Forward + Nr บนแกน ETD49: AC 240 V → DC 58 V / 5 A."""

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
    # ETD49/25/16 (N87)
    core_name: str = "ETD49/25/16"
    ae_m2: float = 211e-6  # 211 mm²
    ie_m: float = 114e-3  # 114 mm
    ve_m3: float = 24100e-9  # 24100 mm³
    b_max_t: float = 0.20  # T @ 100 kHz (N87)
    j_a_per_mm2: float = 4.5  # ความหนาแน่นกระแสลวด
    # STW20N95K5 (MDmesh K5, TO-247)
    mosfet: str = "STW20N95K5"
    mosfet_vdss: float = 950.0
    mosfet_rds_typ: float = 0.275
    mosfet_rds_max: float = 0.330
    mosfet_rds_hot: float = 0.45  # ประมาณที่ Tj สูง
    mosfet_id_25c: float = 17.5
    mosfet_id_100c: float = 11.0
    mosfet_qg_nc: float = 40.0
    mosfet_vdrive: float = 10.0
    # ไดโอดรีเซ็ต Dr
    dr_part: str = "UF4007"
    dr_vrrm: float = 1000.0
    dr_if: float = 1.0
    dr_vf: float = 1.7  # V @ IF ประมาณ
    # RCD snubber (สมมติ leakage หลัง interleaved)
    ll_h: float = 20e-6  # H
    v_clamp: float = 880.0  # V เป้า spike สูงสุด
    rcd_cs_pick_f: float = 1.0e-9
    rcd_rs_pick_ohm: float = 100.0
    rcd_ds_part: str = "UF4007"


def ac_to_vdc_peak(vac: float) -> float:
    return vac * math.sqrt(2.0)


def awg_from_area_mm2(area_mm2: float) -> str:
    """เลือก AWG เล็กสุดที่พื้นที่ทองแดงยังพอ (แล้วแนะนำ Litz @ 100 kHz)."""
    # AWG → mm² (หนา → บาง)
    table = [
        (15, 1.65),
        (16, 1.31),
        (17, 1.04),
        (18, 0.823),
        (19, 0.653),
        (20, 0.518),
        (21, 0.410),
        (22, 0.326),
        (23, 0.258),
        (24, 0.205),
        (26, 0.129),
        (28, 0.081),
    ]
    choice = None
    for awg, a in table:
        if a >= area_mm2:
            choice = awg
        else:
            break
    if choice is None:
        return f"Litz / หลายเส้น รวม ≥ {area_mm2:.2f} mm²"
    return f"≥ AWG {choice} หรือ Litz รวม ≥ {area_mm2:.2f} mm²"


def design(spec: Spec = Spec()) -> dict[str, float | str]:
    vac_min = spec.vac_nom * (1.0 - spec.vac_tol)
    vac_max = spec.vac_nom * (1.0 + spec.vac_tol)
    vin_min = ac_to_vdc_peak(vac_min) - spec.vin_ripple_margin
    vin_nom = ac_to_vdc_peak(spec.vac_nom)
    vin_max = ac_to_vdc_peak(vac_max)

    po = spec.vo * spec.io
    pin = po / spec.eta
    v_out_need = spec.vo + spec.vf

    d_max_theory = 1.0 / (1.0 + spec.nr_over_np)
    d_max_use = min(spec.d_max, 0.95 * d_max_theory)

    n_calc = v_out_need / (vin_min * d_max_use)
    n_use = math.ceil(n_calc * 100.0 - 1e-12) / 100.0

    # จำนวนรอบจาก ETD49: Np >= Vin*D / (Ae * Bmax * fs)
    np_min = (vin_min * d_max_use) / (spec.ae_m2 * spec.b_max_t * spec.fs)
    # ถ้าใกล้จำนวนเต็ม (เช่น 32.04) ให้ใช้ค่ากลม เพื่อได้ 32:14:32
    if abs(np_min - round(np_min)) < 0.08:
        np_turns = int(round(np_min))
    else:
        np_turns = int(math.ceil(np_min))

    ns_turns = max(1, int(round(np_turns * n_use)))
    n_actual = ns_turns / np_turns
    d_at_vin_min = v_out_need / (vin_min * n_actual)
    while d_at_vin_min > d_max_use:
        ns_turns += 1
        n_actual = ns_turns / np_turns
        d_at_vin_min = v_out_need / (vin_min * n_actual)

    nr_turns = int(round(np_turns * spec.nr_over_np))
    n_actual = ns_turns / np_turns

    d_at_vin_min = v_out_need / (vin_min * n_actual)
    d_at_vin_nom = v_out_need / (vin_nom * n_actual)
    d_at_vin_max = v_out_need / (vin_max * n_actual)

    # B จริงที่ Vin_max, D_max_use (worst flux)
    b_at_vin_min = (vin_min * d_at_vin_min) / (np_turns * spec.ae_m2 * spec.fs)
    b_at_vin_nom = (vin_nom * d_at_vin_nom) / (np_turns * spec.ae_m2 * spec.fs)

    vs_nom = vin_nom * n_actual
    vs_max = vin_max * n_actual
    di_l = spec.io * spec.di_l_ratio
    l_out = spec.vo * (1.0 - d_at_vin_nom) / (spec.fs * di_l)

    ip_sec_reflected = spec.io * n_actual
    ip_peak = ip_sec_reflected / spec.eta + 0.15 * ip_sec_reflected
    ip_rms_approx = ip_peak * math.sqrt(d_at_vin_nom)
    is_rms_approx = spec.io * math.sqrt(d_at_vin_nom)

    vds_ideal = vin_max * (1.0 + 1.0 / spec.nr_over_np)
    vds_with_margin = vds_ideal * 1.15

    c_in = pin / (2.0 * math.pi * spec.line_freq * vin_nom * spec.vin_delta)

    # พื้นที่ทองแดงจาก J
    ap_cu = ip_rms_approx / spec.j_a_per_mm2
    as_cu = is_rms_approx / spec.j_a_per_mm2
    # ขดรีเซ็ตกระแสต่ำ — ใช้ลวดบางกว่าปฐมภูมิได้
    ar_cu = max(0.1, ap_cu * 0.35)

    # Lm ประมาณสำหรับ ungapped N87: AL ~ 3200–3800 nH/N² → ใช้ 3500
    al_nh = 3500.0
    lm_h = al_nh * 1e-9 * (np_turns**2)
    im_peak = (vin_nom * d_at_vin_nom) / (lm_h * spec.fs)

    # สูญเสีย STW20N95K5 ประมาณ
    p_cond = (ip_rms_approx**2) * spec.mosfet_rds_hot
    p_gate = spec.mosfet_qg_nc * 1e-9 * spec.mosfet_vdrive * spec.fs
    # สวิตชิ่งหยาบสมมติ tr+tf ≈ 40 ns
    p_sw_est = 0.5 * vin_nom * ip_peak * 40e-9 * spec.fs
    p_fet_est = p_cond + p_sw_est + p_gate
    vds_margin = spec.mosfet_vdss / vds_ideal

    # ไดโอดรีเซ็ต Dr: VR ≈ Vin*(Nr/Np), I ≈ Im*(Np/Nr)
    dr_vr = vin_max * spec.nr_over_np
    dr_i_peak = im_peak * (1.0 / spec.nr_over_np)
    # avg ≈ (Im_peak/2) * (t_reset/Ts), t_reset/Ts = D*(Nr/Np)
    dr_i_avg = 0.5 * dr_i_peak * d_at_vin_nom * spec.nr_over_np
    dr_p_est = dr_i_avg * spec.dr_vf
    dr_v_margin = spec.dr_vrrm / dr_vr

    # RCD: จำกัด spike เหนือ 2*Vin
    v_off = vds_ideal  # ≈ 2*Vin เมื่อ Nr=Np
    ip_rcd = max(ip_peak, 3.0)  # มาร์จิ้นออกแบบ
    den_c = spec.v_clamp**2 - v_off**2
    cs_min = (spec.ll_h * ip_rcd**2) / den_c if den_c > 0 else float("inf")
    p_rcd = 0.5 * spec.ll_h * ip_rcd**2 * spec.fs
    # R ให้ tau ≈ 0.2–0.5 ของคาบ; ค่าเริ่มจากกำลังและ ΔV
    # R ≈ (Vclamp - Voff)^2 / P_rcd  (ประมาณการคายพลังงานต่อรอบ)
    dv = spec.v_clamp - v_off
    rs_est = (dv**2) / p_rcd if p_rcd > 0 else 0.0

    return {
        "core_name": spec.core_name,
        "ae_mm2": spec.ae_m2 * 1e6,
        "ie_mm": spec.ie_m * 1e3,
        "ve_mm3": spec.ve_m3 * 1e9,
        "b_max_t": spec.b_max_t,
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
        "n_actual": n_actual,
        "np_min": np_min,
        "np_turns": float(np_turns),
        "ns_turns": float(ns_turns),
        "nr_turns": float(nr_turns),
        "b_at_vin_min": b_at_vin_min,
        "b_at_vin_nom": b_at_vin_nom,
        "d_at_vin_min": d_at_vin_min,
        "d_at_vin_nom": d_at_vin_nom,
        "d_at_vin_max": d_at_vin_max,
        "vs_nom": vs_nom,
        "vs_max": vs_max,
        "di_l": di_l,
        "l_out_h": l_out,
        "ip_peak": ip_peak,
        "ip_rms_approx": ip_rms_approx,
        "is_rms_approx": is_rms_approx,
        "c_in_f": c_in,
        "vds_ideal": vds_ideal,
        "vds_with_margin": vds_with_margin,
        "diode_vrrm": vs_max,
        "d1_iavg": spec.io * d_at_vin_nom,
        "d2_iavg": spec.io * (1.0 - d_at_vin_nom),
        "fs": spec.fs,
        "vo": spec.vo,
        "io": spec.io,
        "ap_cu_mm2": ap_cu,
        "as_cu_mm2": as_cu,
        "ar_cu_mm2": ar_cu,
        "wire_p": awg_from_area_mm2(ap_cu),
        "wire_s": awg_from_area_mm2(as_cu),
        "wire_r": awg_from_area_mm2(ar_cu),
        "lm_h": lm_h,
        "im_peak": im_peak,
        "al_nh": al_nh,
        "mosfet": spec.mosfet,
        "mosfet_vdss": spec.mosfet_vdss,
        "mosfet_rds_typ": spec.mosfet_rds_typ,
        "mosfet_rds_max": spec.mosfet_rds_max,
        "mosfet_rds_hot": spec.mosfet_rds_hot,
        "mosfet_id_25c": spec.mosfet_id_25c,
        "mosfet_id_100c": spec.mosfet_id_100c,
        "mosfet_qg_nc": spec.mosfet_qg_nc,
        "p_cond": p_cond,
        "p_sw_est": p_sw_est,
        "p_gate": p_gate,
        "p_fet_est": p_fet_est,
        "vds_margin": vds_margin,
        "dr_part": spec.dr_part,
        "dr_vrrm": spec.dr_vrrm,
        "dr_if": spec.dr_if,
        "dr_vr": dr_vr,
        "dr_i_peak": dr_i_peak,
        "dr_i_avg": dr_i_avg,
        "dr_p_est": dr_p_est,
        "dr_v_margin": dr_v_margin,
        "ll_uh": spec.ll_h * 1e6,
        "v_off": v_off,
        "v_clamp": spec.v_clamp,
        "ip_rcd": ip_rcd,
        "cs_min_f": cs_min,
        "cs_pick_f": spec.rcd_cs_pick_f,
        "rs_est": rs_est,
        "rs_pick": spec.rcd_rs_pick_ohm,
        "p_rcd": p_rcd,
        "rcd_ds": spec.rcd_ds_part,
    }


def print_report(d: dict[str, float | str]) -> None:
    print("=" * 62)
    print("Single-Switch Forward + Nr  |  ETD49  |  STW20N95K5")
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
    print("--- ETD49 transformer ---")
    print(f"Core                   = {d['core_name']} (N87 แนะนำ)")
    print(
        f"Ae / Ie / Ve           = "
        f"{d['ae_mm2']:.0f} mm² / {d['ie_mm']:.0f} mm / {d['ve_mm3']:.0f} mm³"
    )
    print(f"Bmax design            = {d['b_max_t']:.2f} T")
    print(f"Np min (from Ae,B,fs)  = {d['np_min']:.1f} turns → use {d['np_turns']:.0f}")
    print(
        f"Turns Np:Ns:Nr         = "
        f"{d['np_turns']:.0f}:{d['ns_turns']:.0f}:{d['nr_turns']:.0f}"
    )
    print(f"n = Ns/Np actual       = {d['n_actual']:.4f}  (target ≥ {d['n_use']:.2f})")
    print(
        f"B @ Vin min/nom        = "
        f"{d['b_at_vin_min']*1e3:.0f} / {d['b_at_vin_nom']*1e3:.0f} mT"
    )
    print(
        f"D @ Vin min/nom/max    = "
        f"{d['d_at_vin_min']:.3f} / {d['d_at_vin_nom']:.3f} / {d['d_at_vin_max']:.3f}"
    )
    print(f"Lm (AL≈{d['al_nh']:.0f} nH/N²)   ≈ {d['lm_h']*1e3:.2f} mH")
    print(f"Im peak (@Vin nom)     ≈ {d['im_peak']:.3f} A")
    print()
    print("--- Wire (J≈4.5 A/mm², 100 kHz ใช้ Litz/หลายเส้น) ---")
    print(f"Primary Cu             ≈ {d['ap_cu_mm2']:.2f} mm²  → {d['wire_p']}")
    print(f"Secondary Cu           ≈ {d['as_cu_mm2']:.2f} mm²  → {d['wire_s']}")
    print(f"Reset Nr Cu            ≈ {d['ar_cu_mm2']:.2f} mm²  → {d['wire_r']}")
    print()
    print("--- MOSFET Q1: STW20N95K5 ---")
    print(f"Part                   = {d['mosfet']} (TO-247, MDmesh K5)")
    print(
        f"VDSS / ID              = "
        f"{d['mosfet_vdss']:.0f} V / {d['mosfet_id_25c']:.1f} A (25°C), "
        f"{d['mosfet_id_100c']:.1f} A (100°C)"
    )
    print(
        f"RDS(on) typ/max/hot    = "
        f"{d['mosfet_rds_typ']:.3f} / {d['mosfet_rds_max']:.3f} / "
        f"{d['mosfet_rds_hot']:.2f} Ω"
    )
    print(f"Qg                     ≈ {d['mosfet_qg_nc']:.0f} nC @ 10 V")
    print(f"Nr/Np                  = {d['nr_over_np']:.2f}")
    print(f"Dmax theory / use      = {d['d_max_theory']:.3f} / {d['d_max_use']:.3f}")
    print(
        f"Vds ideal / +15%       = "
        f"{d['vds_ideal']:.0f} / {d['vds_with_margin']:.0f} V"
    )
    print(
        f"Voltage margin         = "
        f"{d['vds_margin']:.2f}×  (950/{d['vds_ideal']:.0f}) — ใช้ได้ถ้ามี RCD"
    )
    print(f"Q1 Ipeak / Irms≈       = {d['ip_peak']:.2f} / {d['ip_rms_approx']:.2f} A")
    print(
        f"Pcond / Psw≈ / Pgate   ≈ "
        f"{d['p_cond']:.2f} / {d['p_sw_est']:.2f} / {d['p_gate']:.2f} W"
    )
    print(f"P_FET total (est.)     ≈ {d['p_fet_est']:.1f} W  → ติดฮีตซิงก์")
    print(f"Vs nom / max           = {d['vs_nom']:.1f} / {d['vs_max']:.1f} V")
    print()
    print("--- Output LC ---")
    print(f"ΔI_L                   = {d['di_l']:.2f} A")
    print(f"L_out                  ≈ {d['l_out_h'] * 1e6:.0f} µH  (choose 330–390 µH)")
    print("C_out                  = 470–1000 µF / ≥80 V + MLCC")
    print()
    print("--- RCD snubber (drain–source) ---")
    print(f"Assume Ll / Ip         = {d['ll_uh']:.0f} µH / {d['ip_rcd']:.1f} A")
    print(f"Voff (2·Vin) / Vclamp  = {d['v_off']:.0f} / {d['v_clamp']:.0f} V")
    print(f"Cs min (calc)          ≈ {d['cs_min_f']*1e12:.0f} pF")
    print(
        f"Cs / Rs / Ds (pick)    = "
        f"{d['cs_pick_f']*1e9:.1f} nF / {d['rs_pick']:.0f} Ω / {d['rcd_ds']}"
    )
    print(f"Rs power (≈½Ll Ip² fs) ≈ {d['p_rcd']:.1f} W  → ใช้ต้านทาน 10 W")
    print(
        f"Rs tune range         = 47–220 Ω (damping); "
        f"clamp-style ≈ {d['rs_est']:.0f} Ω"
    )
    print()
    print("--- Reset diode Dr ---")
    print(f"Recommend              = {d['dr_part']}  (alt: STTH112A)")
    print(f"VRRM / IF rating       = {d['dr_vrrm']:.0f} V / {d['dr_if']:.0f} A")
    print(
        f"VR stress / margin     = "
        f"{d['dr_vr']:.0f} V / {d['dr_v_margin']:.1f}×"
    )
    print(
        f"Ipeak / Iavg           ≈ "
        f"{d['dr_i_peak']:.2f} / {d['dr_i_avg']:.3f} A"
    )
    print(f"P_Dr (est.)            ≈ {d['dr_p_est']:.2f} W  (ไม่ต้องฮีตซิงก์)")
    print()
    print("--- Output diodes D1/D2 ---")
    print(f"D1/D2 VRRM             ≥ {d['diode_vrrm']:.0f} V  → use 200–300 V")
    print(f"D1 / D2 Iavg           ≈ {d['d1_iavg']:.2f} / {d['d2_iavg']:.2f} A")
    print(f"fs                     = {d['fs'] / 1e3:.0f} kHz")
    print("=" * 62)
    print("STW20N95K5 + UF4007(Dr): ต้องมี RCD ที่ drain — interleaved ลด leakage")


def main() -> None:
    print_report(design())


if __name__ == "__main__":
    main()
