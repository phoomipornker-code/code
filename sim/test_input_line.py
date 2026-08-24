"""Input-side cap-input rectifier vs the 110 V line-current scope shot."""

from __future__ import annotations

import unittest

import numpy as np

from sim.input_line import simulate_line


class InputLineTests(unittest.TestCase):
    def setUp(self) -> None:
        self.w110 = simulate_line(vac_rms=110.0, p_load=115.0)
        self.w240 = simulate_line(vac_rms=240.0, p_load=115.0, rs=6.5)

    def test_110v_matches_scope_order(self) -> None:
        # scope: 110 Vrms, 1.71 Arms, 3.84 A peak, 7.28 A pp
        self.assertAlmostEqual(self.w110.v_rms, 110.0, delta=2.0)
        self.assertAlmostEqual(self.w110.i_rms, 1.71, delta=0.25)
        self.assertAlmostEqual(self.w110.i_peak, 3.84, delta=0.4)
        self.assertGreater(self.w110.i_pp, 6.0)

    def test_current_is_peaky_pulses_not_sine(self) -> None:
        settle = self.w110.t_ms >= 40.0
        conducting = np.mean(np.abs(self.w110.iac[settle]) > 0.25)
        self.assertLess(conducting, 0.40)
        self.assertGreater(self.w110.crest, 2.0)

    def test_110v_cannot_make_58v_with_48_21_and_dmax_045(self) -> None:
        self.assertLess(self.w110.vo_max_at_dmax, 40.0)

    def test_240v_can_make_58v(self) -> None:
        self.assertGreater(self.w240.vo_max_at_dmax, 56.0)

    def test_bus_ripple_is_100hz(self) -> None:
        t = self.w110.t_ms / 1000.0
        y = self.w110.vbus - np.mean(self.w110.vbus)
        mask = t >= 0.04
        spec = np.fft.rfft(y[mask])
        freq = np.fft.rfftfreq(int(np.count_nonzero(mask)), float(t[1] - t[0]))
        spec[freq < 20] = 0
        f = float(freq[int(np.argmax(np.abs(spec)))])
        self.assertAlmostEqual(f, 100.0, delta=15.0)


if __name__ == "__main__":
    unittest.main()
