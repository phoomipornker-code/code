"""Tests for the 240 VAC forward-converter design and CCM waveforms."""

from __future__ import annotations

import unittest

import numpy as np

from pathlib import Path
from tempfile import TemporaryDirectory

from sim.design import design_240vac_58v_5a
from sim.schematic import draw_on_off, draw_power_schematic, vertical_diode_triangle
from sim.waveforms import ideal_ccm


class DesignTests(unittest.TestCase):
    def setUp(self) -> None:
        self.d = design_240vac_58v_5a()

    def test_turns_and_ratio(self) -> None:
        self.assertEqual((self.d.np, self.d.ns, self.d.nr), (48, 21, 48))
        self.assertAlmostEqual(self.d.n, 21 / 48, places=6)

    def test_reset_limits_duty_to_half(self) -> None:
        self.assertAlmostEqual(self.d.d_max_reset, 0.5, places=6)
        self.assertLess(self.d.d_max_use, self.d.d_max_reset)
        self.assertLess(self.d.duty_at_vmin, self.d.d_max_use)
        self.assertLess(self.d.duty_nom, self.d.duty_at_vmin)

    def test_output_equation(self) -> None:
        vo = self.d.vin_nom * self.d.n * self.d.duty_nom - self.d.vf
        self.assertAlmostEqual(vo, self.d.vo, places=6)

    def test_vds_reset_is_twice_vin_when_nr_equals_np(self) -> None:
        self.assertAlmostEqual(self.d.vds_reset(self.d.vin_nom), 2 * self.d.vin_nom, places=6)
        self.assertLess(self.d.vds_reset_max, 950.0)
        self.assertGreater(self.d.vds_reset_max, 700.0)

    def test_flux_density_stays_around_0p2_tesla(self) -> None:
        self.assertAlmostEqual(self.d.bmax_worst, 0.20, delta=0.02)

    def test_inductor_ripple_near_1_amp(self) -> None:
        self.assertAlmostEqual(self.d.il_ripple(self.d.vin_nom), 1.0, delta=0.15)

    def test_power(self) -> None:
        self.assertAlmostEqual(self.d.po, 290.0, places=6)


class WaveformTests(unittest.TestCase):
    def setUp(self) -> None:
        self.wf = ideal_ccm()

    def test_duty_matches_design(self) -> None:
        d = design_240vac_58v_5a()
        self.assertAlmostEqual(self.wf.d, d.duty_nom, places=6)

    def test_magnetizing_current_resets_to_zero(self) -> None:
        # last sample of each cycle should be ~0 after reset
        n = len(self.wf.i_m) // 3
        self.assertLess(abs(self.wf.i_m[n - 1]), 1e-4)
        self.assertGreater(self.wf.im_peak, 0.0)

    def test_d1_and_d2_are_complementary(self) -> None:
        both = np.logical_and(self.wf.i_d1 > 0.2, self.wf.i_d2 > 0.2)
        self.assertFalse(np.any(both))
        self.assertGreater(float(self.wf.i_d1.max()), 4.0)
        self.assertGreater(float(self.wf.i_d2.max()), 4.0)

    def test_vds_has_three_levels(self) -> None:
        levels = np.unique(np.round(self.wf.v_ds, 0))
        self.assertIn(0.0, levels)
        self.assertIn(round(self.wf.vin), levels)
        self.assertIn(round(self.wf.vds_reset), levels)

    def test_inductor_current_stays_positive_ccm(self) -> None:
        self.assertGreater(float(self.wf.i_l.min()), 3.5)


def _triangle_near(svg_text: str, x: float, tol: float = 12.0) -> list[tuple[float, float]]:
    found: list[list[tuple[float, float]]] = []
    for chunk in svg_text.split("<polygon points="):
        if chunk.startswith('"'):
            pts_s = chunk.split('"', 2)[1]
            pts = []
            for pair in pts_s.split():
                a, b = pair.split(",")
                pts.append((float(a), float(b)))
            if len(pts) == 3 and all(abs(px - x) <= tol for px, _py in pts):
                found.append(pts)
    if len(found) != 1:
        raise AssertionError(f"expected one triangle near x={x}, got {len(found)}")
    return found[0]


def _tip_is_above_base(tri: list[tuple[float, float]]) -> bool:
    ys = [round(p[1], 1) for p in tri]
    unique = set(ys)
    if len(unique) != 2:
        raise AssertionError(f"expected a triangle with a shared base y, got {ys}")
    tip_y = next(y for y in unique if ys.count(y) == 1)
    base_y = next(y for y in unique if ys.count(y) == 2)
    return tip_y < base_y


class DiodePolarityTests(unittest.TestCase):
    def test_freewheel_triangle_points_up(self) -> None:
        tri = vertical_diode_triangle(1080, 176, 470, "top")
        self.assertTrue(_tip_is_above_base(tri))
        down = vertical_diode_triangle(1080, 176, 470, "bottom")
        self.assertFalse(_tip_is_above_base(down))

    def test_schematic_d2_cathode_is_at_l_node(self) -> None:
        with TemporaryDirectory() as tmp:
            path = Path(tmp) / "fwd.svg"
            draw_power_schematic(path)
            tri = _triangle_near(path.read_text(encoding="utf-8"), 1080)
        self.assertTrue(_tip_is_above_base(tri))

    def test_on_off_d2_cathode_is_at_l_node(self) -> None:
        with TemporaryDirectory() as tmp:
            path = Path(tmp) / "onoff.svg"
            draw_on_off(path)
            text = path.read_text(encoding="utf-8")
        for x in (548.0, 1340.0):
            self.assertTrue(_tip_is_above_base(_triangle_near(text, x)))


if __name__ == "__main__":
    unittest.main()
