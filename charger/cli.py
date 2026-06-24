"""Command line interface for the charger simulator."""

from __future__ import annotations

import argparse

from .core import Battery, BatteryCharger, ChargerConfig


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python3 -m charger",
        description="Simulate a safe constant-current/constant-voltage battery charger.",
    )
    parser.add_argument(
        "--capacity",
        type=float,
        required=True,
        help="Battery capacity in mAh.",
    )
    parser.add_argument(
        "--soc",
        type=float,
        required=True,
        help="Initial state of charge, 0-100.",
    )
    parser.add_argument(
        "--voltage",
        type=float,
        required=True,
        help="Initial battery voltage.",
    )
    parser.add_argument(
        "--temperature",
        type=float,
        required=True,
        help="Battery temperature in Celsius.",
    )
    parser.add_argument("--minutes", type=float, default=30, help="Minutes to charge.")
    parser.add_argument(
        "--max-current",
        type=float,
        default=ChargerConfig.max_current_ma,
        help="Maximum charging current in mA.",
    )
    parser.add_argument(
        "--trickle-current",
        type=float,
        default=ChargerConfig.trickle_current_ma,
        help="Minimum taper/trickle current in mA.",
    )
    parser.add_argument(
        "--target-voltage",
        type=float,
        default=ChargerConfig.target_voltage,
        help="Target charging voltage.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    battery = Battery(
        capacity_mah=args.capacity,
        state_of_charge=args.soc,
        voltage=args.voltage,
        temperature_c=args.temperature,
    )
    config = ChargerConfig(
        max_current_ma=args.max_current,
        trickle_current_ma=args.trickle_current,
        target_voltage=args.target_voltage,
    )
    result = BatteryCharger(config).charge_for(battery, args.minutes)

    print(f"status: {result.status.value}")
    print(f"current_ma: {result.current_ma:.2f}")
    print(f"delivered_mah: {result.delivered_mah:.2f}")
    print(f"state_of_charge: {result.state_of_charge:.2f}%")
    print(f"voltage: {result.voltage:.3f}V")
    print(f"message: {result.message}")
    return 0
