"""Generate schematic + waveform SVGs into artifacts/."""

from __future__ import annotations

from pathlib import Path

from sim.bus_ripple import fast_pi_5khz, open_loop, plot_ripple, slow_pi_20ms
from sim.design import design_240vac_58v_5a
from sim.hw_110_n13 import N_DOWN, N_UP, evaluate, plot_cases
from sim.input_line import plot_line, simulate_line
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
    plot_ripple(open_loop(), slow_pi_20ms(), fast_pi_5khz(), art / "ibat_100hz.svg")
    plot_line(
        simulate_line(vac_rms=110.0, p_load=115.0),
        simulate_line(vac_rms=240.0, p_load=115.0, rs=6.5),
        art / "input_line.svg",
    )
    plot_cases(evaluate(N_UP), evaluate(N_DOWN), art / "hw_110_n13.svg")
    print(p1)
    print(p2)
    print(art / "forward_waveforms.svg")
    print(art / "ibat_100hz.svg")
    print(art / "input_line.svg")
    print(art / "hw_110_n13.svg")


if __name__ == "__main__":
    main()
