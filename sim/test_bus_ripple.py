"""Battery-current 100 Hz ripple vs current-loop / Vin feedforward."""

from __future__ import annotations

import unittest

from sim.bus_ripple import fast_pi_5khz, open_loop, slow_pi_20ms


class BusRippleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.open = open_loop()
        cls.slow = slow_pi_20ms()
        cls.fast = fast_pi_5khz()

    def test_open_loop_is_line_frequency(self) -> None:
        self.assertAlmostEqual(self.open.f_ripple_hz, 100.0, delta=15.0)
        self.assertGreater(self.open.i_pp, 1.5)

    def test_open_loop_matches_scope_order_of_magnitude(self) -> None:
        # scope: 2.12 A mean, 1.92 A pp at 5 ms/div
        self.assertGreater(self.open.i_mean, 1.2)
        self.assertLess(self.open.i_mean, 3.0)
        self.assertGreater(self.open.i_pp, 1.5)
        self.assertLess(self.open.i_pp, 5.0)

    def test_20ms_firmware_loop_cannot_reject_100hz(self) -> None:
        self.assertGreater(self.slow.i_pp, 1.5)
        self.assertAlmostEqual(self.slow.f_ripple_hz, 100.0, delta=20.0)

    def test_vin_feedforward_plus_fast_pi_kills_the_ripple(self) -> None:
        self.assertLess(self.fast.i_pp, 0.25)
        self.assertAlmostEqual(self.fast.i_mean, 2.12, delta=0.15)


if __name__ == "__main__":
    unittest.main()
