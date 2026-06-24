import unittest

from charger import Battery, BatteryCharger, ChargeStatus, ChargerConfig


class BatteryChargerTests(unittest.TestCase):
    def test_bulk_charge_increases_state_of_charge(self) -> None:
        battery = Battery(
            capacity_mah=5000,
            state_of_charge=40,
            voltage=3.7,
            temperature_c=25,
        )
        charger = BatteryCharger()

        result = charger.charge_for(battery, minutes=30)

        self.assertEqual(result.status, ChargeStatus.CHARGING)
        self.assertEqual(result.current_ma, 2000)
        self.assertAlmostEqual(result.delivered_mah, 920)
        self.assertAlmostEqual(result.state_of_charge, 58.4)
        self.assertEqual(battery.state_of_charge, result.state_of_charge)

    def test_tapers_current_near_full(self) -> None:
        battery = Battery(
            capacity_mah=5000,
            state_of_charge=90,
            voltage=4.1,
            temperature_c=25,
        )
        charger = BatteryCharger()

        status, current_ma, _ = charger.calculate_current(battery)

        self.assertEqual(status, ChargeStatus.TAPERING)
        self.assertGreater(current_ma, ChargerConfig.trickle_current_ma)
        self.assertLess(current_ma, ChargerConfig.max_current_ma)

    def test_completes_at_cutoff(self) -> None:
        battery = Battery(
            capacity_mah=1000,
            state_of_charge=99,
            voltage=4.18,
            temperature_c=25,
        )
        config = ChargerConfig(max_current_ma=2000)
        charger = BatteryCharger(config)

        result = charger.charge_for(battery, minutes=60)

        self.assertEqual(result.status, ChargeStatus.COMPLETE)
        self.assertEqual(result.current_ma, 0)
        self.assertAlmostEqual(result.delivered_mah, 10)
        self.assertEqual(result.state_of_charge, 100)

    def test_pauses_when_battery_is_too_hot(self) -> None:
        battery = Battery(
            capacity_mah=5000,
            state_of_charge=50,
            voltage=3.8,
            temperature_c=60,
        )
        charger = BatteryCharger()

        result = charger.charge_for(battery, minutes=30)

        self.assertEqual(result.status, ChargeStatus.PAUSED)
        self.assertEqual(result.current_ma, 0)
        self.assertEqual(result.delivered_mah, 0)
        self.assertEqual(result.state_of_charge, 50)

    def test_rejects_invalid_battery_capacity(self) -> None:
        with self.assertRaises(ValueError):
            Battery(
                capacity_mah=0,
                state_of_charge=50,
                voltage=3.8,
                temperature_c=25,
            )


if __name__ == "__main__":
    unittest.main()
