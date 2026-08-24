"""Generate schematic + waveform SVGs into artifacts/."""

from __future__ import annotations

from pathlib import Path

from sim.design import design_240vac_58v_5a
from sim.schematic import draw_on_off, draw_power_schematic
from sim.waveforms import ideal_ccm, plot_waveforms


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    art = root / "artifacts"
    art.mkdir(parents=True, exist_ok=True)
    d = design_240vac_58v_5a()
    print("design")
    for k, v in d.as_rows():
        print(f"  {k}: {v}")
    p1 = draw_power_schematic(art / "forward_schematic.svg")
    p2 = draw_on_off(art / "forward_on_off.svg")
    wf = ideal_ccm()
    plot_waveforms(wf, art / "forward_waveforms.svg")
    print(p1)
    print(p2)
    print(art / "forward_waveforms.svg")


if __name__ == "__main__":
    main()
