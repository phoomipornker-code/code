"""Predicted 240 V / 48:21 Vout and Iout."""

from __future__ import annotations

import unittest

from sim.scope_240 import IOUT, VBAT, line_scale
from sim.waveforms import ideal_ccm


class Scope240Tests(unittest.TestCase):
    def test_switching_ripple_near_one_amp(self) -> None:
        sw = ideal_ccm()
        self.assertAlmostEqual(sw.vo, 58.0, places=6)
        self.assertAlmostEqual(sw.di, 1.0, delta=0.2)
        self.assertAlmostEqual(sw.d, 0.395, delta=0.02)

    def test_vin_feedforward_holds_5a(self) -> None:
        run = line_scale(True)
        self.assertAlmostEqual(run.i_mean, IOUT, delta=0.25)
        self.assertLess(run.i_pp, 0.5)

    def test_open_loop_line_ripple_voltage_stays_stiff(self) -> None:
        run = line_scale(False)
        self.assertGreater(run.i_pp, 1.5)
        vpp = run.i_pp * 0.040
        self.assertLess(vpp, 0.5)


if __name__ == "__main__":
    unittest.main()
