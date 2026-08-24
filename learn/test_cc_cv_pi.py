"""Host checks for the CC/CV PI lesson (no extra packages)."""

from __future__ import annotations

import os
import tempfile
import unittest

from pi import PiController, clamp, run_pi
from simulate_cc_cv import (
    I_CC,
    simulate_charge,
    simulate_current_step,
    plot_charge,
    plot_p_vs_pi,
)


class TestPiMath(unittest.TestCase):
    def test_zero_error_holds(self) -> None:
        out, integ = run_pi(0.0, 8.0, 35.0, 0.02, 12.0, -20.0, 25.0)
        self.assertAlmostEqual(out, 12.0)
        self.assertAlmostEqual(integ, 12.0)

    def test_integral_grows_with_persistent_error(self) -> None:
        integ = 0.0
        last = 0.0
        for _ in range(10):
            last, integ = run_pi(1.0, 0.0, 10.0, 0.02, integ, -100.0, 100.0)
        self.assertAlmostEqual(last, 2.0)  # 10 * 10 * 1 * 0.02
        self.assertGreater(integ, 0.0)

    def test_anti_windup_freezes_integral_at_ceiling(self) -> None:
        ctrl = PiController(kp=10.0, ki=50.0, out_min=0.0, out_max=5.0)
        for _ in range(40):
            ctrl.step(2.0, 0.02)
        frozen = ctrl.integ
        self.assertEqual(ctrl.step(2.0, 0.02), 5.0)
        self.assertEqual(ctrl.integ, frozen)

    def test_anti_windup_freezes_integral_at_floor(self) -> None:
        ctrl = PiController(kp=10.0, ki=50.0, out_min=-5.0, out_max=5.0)
        for _ in range(40):
            ctrl.step(-2.0, 0.02)
        frozen = ctrl.integ
        self.assertEqual(ctrl.step(-2.0, 0.02), -5.0)
        self.assertEqual(ctrl.integ, frozen)

    def test_clamp(self) -> None:
        self.assertEqual(clamp(-1.0, 0.0, 3.0), 0.0)
        self.assertEqual(clamp(8.0, 0.0, 3.0), 3.0)
        self.assertEqual(clamp(1.5, 0.0, 3.0), 1.5)


class TestChargeCascade(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.result = simulate_charge(t_stop=180.0)

    def test_enters_cc_and_cv(self) -> None:
        self.assertIn(1, self.result.entered)
        self.assertIn(2, self.result.entered)
        self.assertIn(3, self.result.entered)

    def test_cc_holds_near_five_amps(self) -> None:
        cc = [s for s in self.result.samples if s.mode == 1 and 8.0 < s.t < 40.0]
        self.assertGreater(len(cc), 50)
        i_avg = sum(s.ibat for s in cc) / len(cc)
        self.assertGreater(i_avg, 4.4)
        self.assertLess(i_avg, 5.6)

    def test_cv_holds_voltage_and_tapers_current(self) -> None:
        cc = [s for s in self.result.samples if s.mode == 1 and 8.0 < s.t < 40.0]
        cv = [s for s in self.result.samples if s.mode == 2]
        self.assertGreater(len(cv), 50)
        v_end = cv[-1].vbat
        i_end = abs(cv[-1].ibat)
        i_cc = sum(abs(s.ibat) for s in cc) / len(cc)
        self.assertGreater(v_end, 57.20)
        self.assertLess(v_end, 57.85)
        self.assertLess(i_end, i_cc * 0.70)
        self.assertLess(i_end, 3.0)

    def test_never_exceeds_duty_rail_or_bms_ovp(self) -> None:
        for s in self.result.samples:
            self.assertLessEqual(s.duty, 460.0001)
            self.assertGreaterEqual(s.duty, 0.0)
            self.assertLess(s.vbat, 58.40)

    def test_voltage_climbs_during_cc(self) -> None:
        cc = [s for s in self.result.samples if s.mode == 1]
        self.assertGreater(cc[-1].vbat, cc[0].vbat + 0.5)


class TestPvsPi(unittest.TestCase):
    def test_integral_removes_standing_error(self) -> None:
        p_only = simulate_current_step(False)
        with_i = simulate_current_step(True)
        p_err = abs(I_CC - p_only[-1].ibat)
        i_err = abs(I_CC - with_i[-1].ibat)
        self.assertGreater(p_err, 1.0)
        self.assertLess(i_err, 0.20)
        self.assertLess(i_err, p_err)


class TestPlots(unittest.TestCase):
    def test_svg_files_are_written(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            charge = simulate_charge(t_stop=60.0)
            charge_path = os.path.join(tmp, "cc_cv_charge.svg")
            plot_charge(charge, charge_path)
            p_path = os.path.join(tmp, "p_vs_pi.svg")
            plot_p_vs_pi(simulate_current_step(False), simulate_current_step(True), p_path)
            for path in (charge_path, p_path):
                with open(path, encoding="utf-8") as fh:
                    body = fh.read()
                self.assertTrue(body.startswith("<svg"))
                self.assertIn("</svg>", body)


if __name__ == "__main__":
    unittest.main()
