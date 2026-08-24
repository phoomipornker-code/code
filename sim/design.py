"""Design numbers for the 240 VAC → 58 V / 5 A single-switch forward."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ForwardDesign:
    """Single-ended forward with tertiary reset winding Nr = Np."""

    vac_nom: float = 240.0
    vac_min: float = 216.0
    vac_max: float = 264.0
    vo: float = 58.0
    io: float = 5.0
    vf: float = 0.7
    fs: float = 67_000.0
    np: int = 48
    ns: int = 21
    nr: int = 48
    ae_m2: float = 211e-6  # ETD49/25/16
    al_h: float = 3500e-9  # ungapped ETD49 N87, H/turn²
    l_h: float = 520e-6
    d_max_use: float = 0.45
    cin_f: float = 220e-6
    co_f: float = 680e-6

    @property
    def n(self) -> float:
        return self.ns / self.np

    @property
    def nr_over_np(self) -> float:
        return self.nr / self.np

    @property
    def d_max_reset(self) -> float:
        return 1.0 / (1.0 + self.nr_over_np)

    @property
    def vin_min(self) -> float:
        return 300.0  # after bulk-cap ripple at 216 VAC

    @property
    def vin_nom(self) -> float:
        return 340.0  # 240 VAC * √2, rounded

    @property
    def vin_max(self) -> float:
        return 373.0  # 264 VAC * √2

    @property
    def vs_nom(self) -> float:
        return self.vin_nom * self.n

    def duty(self, vin: float) -> float:
        return (self.vo + self.vf) / (vin * self.n)

    @property
    def duty_nom(self) -> float:
        return self.duty(self.vin_nom)

    @property
    def duty_at_vmin(self) -> float:
        return self.duty(self.vin_min)

    def vds_reset(self, vin: float) -> float:
        return vin * (1.0 + self.np / self.nr)

    @property
    def vds_reset_max(self) -> float:
        return self.vds_reset(self.vin_max)

    def bmax(self, vin: float, d: float) -> float:
        return (vin * d) / (self.np * self.ae_m2 * self.fs)

    @property
    def bmax_worst(self) -> float:
        return self.bmax(self.vin_min, self.d_max_use)

    @property
    def lm(self) -> float:
        return self.al_h * (self.np**2)

    def il_ripple(self, vin: float) -> float:
        d = self.duty(vin)
        vs = vin * self.n
        return ((vs - self.vo) * d) / (self.l_h * self.fs)

    @property
    def po(self) -> float:
        return self.vo * self.io

    @property
    def r_load(self) -> float:
        return self.vo / self.io

    def as_rows(self) -> list[tuple[str, str]]:
        return [
            ("โทโพโลยี", "Single-switch forward + tertiary reset Nr"),
            ("อินพุต", f"AC {self.vac_nom:.0f} V ({self.vac_min:.0f}–{self.vac_max:.0f} V)"),
            ("บัส DC", f"{self.vin_min:.0f} / {self.vin_nom:.0f} / {self.vin_max:.0f} V"),
            ("เอาต์พุต", f"{self.vo:.0f} V / {self.io:.0f} A ({self.po:.0f} W)"),
            ("ความถี่", f"{self.fs/1e3:.0f} kHz"),
            ("หม้อแปลง", f"ETD49 N87   Np:Ns:Nr = {self.np}:{self.ns}:{self.nr}"),
            ("n = Ns/Np", f"{self.n:.4f}"),
            ("D @ Vin nom", f"{self.duty_nom:.3f}"),
            ("D @ Vin min", f"{self.duty_at_vmin:.3f}"),
            ("Dmax (Nr=Np)", f"{self.d_max_reset:.2f}  (ใช้จริง ≤ {self.d_max_use:.2f})"),
            ("Vds ตอนรีเซ็ต max", f"{self.vds_reset_max:.0f} V  (ไม่รวม leakage)"),
            ("Bmax ที่ Vin min", f"{self.bmax_worst:.3f} T"),
            ("Lm", f"{self.lm*1e3:.1f} mH"),
            ("L เอาต์พุต", f"{self.l_h*1e6:.0f} µH"),
            ("ΔI_L @ Vin nom", f"{self.il_ripple(self.vin_nom):.2f} A"),
            ("R_load", f"{self.r_load:.2f} Ω"),
        ]


def design_240vac_58v_5a() -> ForwardDesign:
    return ForwardDesign()


def main() -> None:
    d = design_240vac_58v_5a()
    print("Forward 240 VAC → 58 V / 5 A  @ 67 kHz")
    print("-" * 48)
    for k, v in d.as_rows():
        print(f"  {k:<22} {v}")
    if d.duty_at_vmin >= d.d_max_use:
        raise SystemExit("duty at Vin min exceeds Dmax — check turns")
    print("  ตรวจ D @ Vin min < 0.45: OK")


if __name__ == "__main__":
    main()
