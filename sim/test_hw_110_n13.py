"""110 V + 1:1.3 switching transformer operating-point tests."""

from __future__ import annotations

import unittest

from sim.hw_110_n13 import N_DOWN, N_UP, evaluate


class Turns110Tests(unittest.TestCase):
    def test_n_1_3_can_hold_54v_at_110vac(self) -> None:
        c = evaluate(N_UP)
        self.assertLess(c.d_needed, 0.45)
        self.assertGreater(c.vo_max, 54.4)
        self.assertTrue(c.can_hold_vbat)
        self.assertGreater(c.i_mean, 1.0)

    def test_n_1_over_1_3_cannot_hold_54v_at_110vac(self) -> None:
        c = evaluate(N_DOWN)
        self.assertGreater(c.d_needed, 0.45)
        self.assertLess(c.vo_max, 54.4)
        self.assertFalse(c.can_hold_vbat)

    def test_110v_n13_ripple_is_worse_than_240v_design(self) -> None:
        from sim.bus_ripple import open_loop

        c = evaluate(N_UP)
        ref = open_loop()
        self.assertGreater(c.i_pp, ref.i_pp)
        self.assertGreater(c.i_pp, 1.9)


if __name__ == "__main__":
    unittest.main()
