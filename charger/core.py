"""Battery charger domain model and charging controller."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class ChargeStatus(str, Enum):
    """Possible charger states."""

    CHARGING = "charging"
    TAPERING = "tapering"
    COMPLETE = "complete"
    PAUSED = "paused"


@dataclass
class Battery:
    """A rechargeable battery pack.

    Attributes:
        capacity_mah: Battery capacity in milliamp-hours.
        state_of_charge: Current charge level as a percentage from 0 to 100.
        voltage: Current pack/cell voltage in volts.
        temperature_c: Battery temperature in Celsius.
    """

    capacity_mah: float
    state_of_charge: float
    voltage: float
    temperature_c: float

    def __post_init__(self) -> None:
        if self.capacity_mah <= 0:
            raise ValueError("capacity_mah must be greater than 0")
        if not 0 <= self.state_of_charge <= 100:
            raise ValueError("state_of_charge must be between 0 and 100")
        if self.voltage <= 0:
            raise ValueError("voltage must be greater than 0")


@dataclass(frozen=True)
class ChargerConfig:
    """Configuration for a constant-current/constant-voltage charger."""

    max_current_ma: float = 2000
    trickle_current_ma: float = 200
    target_voltage: float = 4.2
    taper_start_soc: float = 80
    cutoff_soc: float = 100
    min_temperature_c: float = 0
    max_temperature_c: float = 45
    charge_efficiency: float = 0.92

    def __post_init__(self) -> None:
        if self.max_current_ma <= 0:
            raise ValueError("max_current_ma must be greater than 0")
        if self.trickle_current_ma < 0:
            raise ValueError("trickle_current_ma cannot be negative")
        if self.trickle_current_ma > self.max_current_ma:
            raise ValueError("trickle_current_ma cannot exceed max_current_ma")
        if self.target_voltage <= 0:
            raise ValueError("target_voltage must be greater than 0")
        if not 0 <= self.taper_start_soc <= 100:
            raise ValueError("taper_start_soc must be between 0 and 100")
        if not 0 < self.cutoff_soc <= 100:
            raise ValueError("cutoff_soc must be between 0 and 100")
        if self.taper_start_soc >= self.cutoff_soc:
            raise ValueError("taper_start_soc must be lower than cutoff_soc")
        if self.min_temperature_c >= self.max_temperature_c:
            raise ValueError("min_temperature_c must be lower than max_temperature_c")
        if not 0 < self.charge_efficiency <= 1:
            raise ValueError("charge_efficiency must be between 0 and 1")


@dataclass(frozen=True)
class ChargeResult:
    """Result returned after a charging step."""

    status: ChargeStatus
    current_ma: float
    delivered_mah: float
    state_of_charge: float
    voltage: float
    message: str


class BatteryCharger:
    """Simple CC/CV battery charger controller.

    The charger provides full current until either the target voltage is
    reached or the battery enters the configured taper region. Current is then
    reduced linearly until the cutoff state of charge is reached.
    """

    def __init__(self, config: ChargerConfig | None = None) -> None:
        self.config = config or ChargerConfig()

    def calculate_current(self, battery: Battery) -> tuple[ChargeStatus, float, str]:
        """Return the safe charging current for the battery."""

        config = self.config
        if battery.temperature_c < config.min_temperature_c:
            return ChargeStatus.PAUSED, 0, "Battery is too cold to charge safely."
        if battery.temperature_c > config.max_temperature_c:
            return ChargeStatus.PAUSED, 0, "Battery is too hot to charge safely."
        if battery.state_of_charge >= config.cutoff_soc:
            return ChargeStatus.COMPLETE, 0, "Battery is fully charged."

        tapering = (
            battery.state_of_charge >= config.taper_start_soc
            or battery.voltage >= config.target_voltage
        )
        if not tapering:
            return ChargeStatus.CHARGING, config.max_current_ma, "Charging at bulk current."

        taper_range = config.cutoff_soc - config.taper_start_soc
        remaining_ratio = (config.cutoff_soc - battery.state_of_charge) / taper_range
        remaining_ratio = max(0, min(1, remaining_ratio))
        current = config.trickle_current_ma + (
            config.max_current_ma - config.trickle_current_ma
        ) * remaining_ratio
        return ChargeStatus.TAPERING, current, "Charging current is tapering near full."

    def charge_for(self, battery: Battery, minutes: float) -> ChargeResult:
        """Charge the battery for the requested duration.

        The input ``battery`` is mutated to reflect the new state.
        """

        if minutes < 0:
            raise ValueError("minutes cannot be negative")

        status, current_ma, message = self.calculate_current(battery)
        if minutes == 0 or current_ma == 0:
            return ChargeResult(
                status=status,
                current_ma=current_ma,
                delivered_mah=0,
                state_of_charge=battery.state_of_charge,
                voltage=battery.voltage,
                message=message,
            )

        potential_mah = current_ma * (minutes / 60) * self.config.charge_efficiency
        percent_added = (potential_mah / battery.capacity_mah) * 100
        old_soc = battery.state_of_charge
        next_soc = min(self.config.cutoff_soc, old_soc + percent_added)
        delivered_mah = ((next_soc - old_soc) / 100) * battery.capacity_mah

        battery.state_of_charge = next_soc
        battery.voltage = self._estimate_voltage(battery)
        if battery.state_of_charge >= self.config.cutoff_soc:
            status = ChargeStatus.COMPLETE
            current_ma = 0
            message = "Battery reached the configured cutoff."

        return ChargeResult(
            status=status,
            current_ma=current_ma,
            delivered_mah=delivered_mah,
            state_of_charge=battery.state_of_charge,
            voltage=battery.voltage,
            message=message,
        )

    def _estimate_voltage(self, battery: Battery) -> float:
        soc_ratio = battery.state_of_charge / 100
        estimated_voltage = 3.0 + (self.config.target_voltage - 3.0) * soc_ratio
        voltage = min(self.config.target_voltage, max(battery.voltage, estimated_voltage))
        return round(voltage, 3)
