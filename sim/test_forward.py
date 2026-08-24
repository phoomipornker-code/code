"""Tests for the 240 VAC forward-converter design and CCM waveforms."""

from __future__ import annotations

import unittest

import numpy as np

from sim.design import design_240vac_58v_5a
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


if __name__ == "__main__":
    unittest.main()
