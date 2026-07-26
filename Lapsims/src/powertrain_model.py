"""Coupled accumulator, inverter, and EMRAX powertrain model.

The model is intentionally independent of the lap solver. The established
positive-torque path supplies the traction surface. A separate signed
generator path supports opt-in endurance regeneration without changing the
traction solution.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd


@dataclass(frozen=True)
class Curve:
    x: np.ndarray
    y: np.ndarray

    @classmethod
    def from_pairs(cls, pairs: list[list[float]]) -> "Curve":
        values = np.asarray(pairs, dtype=float)
        if values.ndim != 2 or values.shape[1] != 2:
            raise ValueError("Curve data must contain [x, y] pairs")
        if np.any(np.diff(values[:, 0]) <= 0.0):
            raise ValueError("Curve x values must be strictly increasing")
        return cls(values[:, 0], values[:, 1])

    def at(self, value: float) -> float:
        return float(np.interp(value, self.x, self.y))

    def integral(self, lower: float, upper: float) -> float:
        if upper <= lower:
            return 0.0
        previous_x = lower
        previous_y = self.at(lower)
        total = 0.0
        for point_x, point_y in zip(self.x, self.y):
            if not lower < point_x < upper:
                continue
            total += 0.5 * (previous_y + point_y) * (
                point_x - previous_x
            )
            previous_x = float(point_x)
            previous_y = float(point_y)
        upper_y = self.at(upper)
        total += 0.5 * (previous_y + upper_y) * (upper - previous_x)
        return float(total)


@dataclass(frozen=True)
class PackConfig:
    series_cells: int
    parallel_cells: int
    base_series_cells: int
    base_parallel_cells: int
    cell_capacity_ah: float
    cell_mass_kg: float
    base_cell_mass_kg: float
    pack_mass_multiplier: float
    initial_soc: float
    minimum_soc: float
    minimum_cell_voltage_v: float
    maximum_cell_discharge_current_a: float
    cell_temperature_c: float
    resistance_reference_temperature_c: float
    resistance_temperature_coefficient_per_c: float
    ocv_v: Curve
    resistance_ohm: Curve
    maximum_cell_voltage_v: float = 4.2
    maximum_cell_charge_current_a: float | None = None

    @property
    def capacity_ah(self) -> float:
        return self.parallel_cells * self.cell_capacity_ah

    @property
    def maximum_discharge_current_a(self) -> float:
        return self.parallel_cells * self.maximum_cell_discharge_current_a

    @property
    def minimum_terminal_voltage_v(self) -> float:
        return self.series_cells * self.minimum_cell_voltage_v

    @property
    def maximum_terminal_voltage_v(self) -> float:
        return self.series_cells * self.maximum_cell_voltage_v

    @property
    def maximum_charge_current_a(self) -> float:
        if self.maximum_cell_charge_current_a is None:
            return math.inf
        return self.parallel_cells * self.maximum_cell_charge_current_a

    def cell_ocv_v(self, soc: float) -> float:
        return self.ocv_v.at(float(np.clip(soc, 0.0, 1.0)))

    def open_circuit_voltage_v(self, soc: float) -> float:
        return self.series_cells * self.cell_ocv_v(soc)

    def cell_resistance_ohm(self, soc: float) -> float:
        base = self.resistance_ohm.at(float(np.clip(soc, 0.0, 1.0)))
        temperature_multiplier = (
            1.0
            + self.resistance_temperature_coefficient_per_c
            * (
                self.cell_temperature_c
                - self.resistance_reference_temperature_c
            )
        )
        return base * max(temperature_multiplier, 0.0)

    def pack_resistance_ohm(self, soc: float) -> float:
        return (
            self.series_cells
            / self.parallel_cells
            * self.cell_resistance_ohm(soc)
        )

    def chemical_energy_wh(self, lower_soc: float, upper_soc: float) -> float:
        return (
            self.capacity_ah
            * self.series_cells
            * self.ocv_v.integral(lower_soc, upper_soc)
        )

    def usable_soe(self, soc: float) -> float:
        full = self.chemical_energy_wh(self.minimum_soc, 1.0)
        remaining = self.chemical_energy_wh(
            self.minimum_soc, max(self.minimum_soc, soc)
        )
        return remaining / full if full > 0.0 else 0.0

    def with_parallel_cells(self, parallel_cells: int) -> "PackConfig":
        if parallel_cells <= 0:
            raise ValueError("parallel_cells must be positive")
        return replace(self, parallel_cells=parallel_cells)

    def vehicle_mass_delta_kg(self) -> float:
        candidate_cell_mass = (
            self.series_cells * self.parallel_cells * self.cell_mass_kg
        )
        baseline_cell_mass = (
            self.base_series_cells
            * self.base_parallel_cells
            * self.base_cell_mass_kg
        )
        return (
            candidate_cell_mass - baseline_cell_mass
        ) * self.pack_mass_multiplier


@dataclass(frozen=True)
class MotorConfig:
    peak_torque_nm: float
    peak_power_w: float
    maximum_speed_rpm: float
    torque_constant_nm_per_arms: float
    maximum_phase_current_arms: float
    phase_resistance_ohm_at_reference_c: float
    resistance_reference_temperature_c: float
    winding_temperature_c: float
    copper_temperature_coefficient_per_c: float
    ld_h: float
    lq_h: float
    pole_pairs: int
    induced_voltage_ll_rms_per_rpm: float
    iron_loss_c0_w: float
    iron_loss_c1_w_per_radps: float
    iron_loss_c2_w_per_radps2: float

    @property
    def hot_phase_resistance_ohm(self) -> float:
        return self.phase_resistance_ohm_at_reference_c * (
            1.0
            + self.copper_temperature_coefficient_per_c
            * (
                self.winding_temperature_c
                - self.resistance_reference_temperature_c
            )
        )

    def iron_loss_w(self, speed_rpm: float) -> float:
        omega = speed_rpm * 2.0 * math.pi / 60.0
        return max(
            0.0,
            self.iron_loss_c0_w
            + self.iron_loss_c1_w_per_radps * omega
            + self.iron_loss_c2_w_per_radps2 * omega**2,
        )


@dataclass(frozen=True)
class InverterConfig:
    voltage_utilization_vll_rms_per_vdc: float
    maximum_bus_current_a: float
    base_loss_w: float
    conduction_loss_fraction: float
    switching_frequency_hz: float
    equivalent_switch_time_s: float
    maximum_regen_bus_current_a: float | None = None

    @property
    def maximum_regen_bus_current_limit_a(self) -> float:
        if self.maximum_regen_bus_current_a is None:
            return self.maximum_bus_current_a
        return self.maximum_regen_bus_current_a

    def loss_w(
        self, terminal_voltage_v: float, phase_current_arms: float, ac_power_w: float
    ) -> float:
        switching = (
            3.0
            * max(terminal_voltage_v, 0.0)
            * max(phase_current_arms, 0.0)
            * self.switching_frequency_hz
            * self.equivalent_switch_time_s
        )
        return (
            self.base_loss_w
            + self.conduction_loss_fraction * abs(ac_power_w)
            + switching
        )


@dataclass(frozen=True)
class DrivetrainConfig:
    final_drive_ratio: float
    gear_ratio: float
    tire_radius_m: float
    mechanical_efficiency: float

    def motor_speed_rpm(self, vehicle_speed_mps: float) -> float:
        wheel_rad_s = max(vehicle_speed_mps, 0.0) / self.tire_radius_m
        motor_rad_s = wheel_rad_s * self.final_drive_ratio * self.gear_ratio
        return motor_rad_s * 60.0 / (2.0 * math.pi)

    def motor_torque_for_force(self, force_n: float) -> float:
        ratio = self.final_drive_ratio * self.gear_ratio
        return (
            max(force_n, 0.0)
            * self.tire_radius_m
            / (ratio * self.mechanical_efficiency)
        )

    def force_for_motor_torque(self, torque_nm: float) -> float:
        ratio = self.final_drive_ratio * self.gear_ratio
        return (
            max(torque_nm, 0.0)
            * ratio
            * self.mechanical_efficiency
            / self.tire_radius_m
        )

    def motor_torque_for_regenerative_force(self, force_n: float) -> float:
        """Return negative motor torque for braking force at the tires."""

        ratio = self.final_drive_ratio * self.gear_ratio
        return (
            -max(force_n, 0.0)
            * self.tire_radius_m
            * self.mechanical_efficiency
            / ratio
        )

    def regenerative_force_for_motor_torque(self, torque_nm: float) -> float:
        """Return tire braking force generated by negative motor torque."""

        ratio = self.final_drive_ratio * self.gear_ratio
        return (
            max(-torque_nm, 0.0)
            * ratio
            / (self.tire_radius_m * self.mechanical_efficiency)
        )


@dataclass(frozen=True)
class PowertrainConfig:
    pack: PackConfig
    motor: MotorConfig
    inverter: InverterConfig
    drivetrain: DrivetrainConfig
    terminal_power_limit_w: float

    def with_parallel_cells(self, parallel_cells: int) -> "PowertrainConfig":
        return replace(self, pack=self.pack.with_parallel_cells(parallel_cells))

    def validate(self) -> None:
        pack = self.pack
        motor = self.motor
        inverter = self.inverter
        drivetrain = self.drivetrain
        if pack.series_cells <= 0 or pack.parallel_cells <= 0:
            raise ValueError("Pack series/parallel counts must be positive")
        if (
            pack.base_series_cells <= 0
            or pack.base_parallel_cells <= 0
            or pack.cell_capacity_ah <= 0.0
            or pack.cell_mass_kg <= 0.0
            or pack.base_cell_mass_kg <= 0.0
        ):
            raise ValueError("Pack base count and cell capacity must be positive")
        if not 0.0 <= pack.minimum_soc < pack.initial_soc <= 1.0:
            raise ValueError("Pack SOC limits must satisfy 0 <= min < initial <= 1")
        if (
            pack.minimum_cell_voltage_v <= 0.0
            or pack.maximum_cell_voltage_v <= 0.0
            or pack.maximum_cell_discharge_current_a <= 0.0
        ):
            raise ValueError("Pack voltage and current limits must be positive")
        if pack.maximum_cell_voltage_v < float(np.max(pack.ocv_v.y)) - 1e-12:
            raise ValueError("Maximum cell voltage must cover the OCV curve")
        if (
            pack.maximum_cell_charge_current_a is not None
            and pack.maximum_cell_charge_current_a <= 0.0
        ):
            raise ValueError("Maximum cell charge current must be positive")
        if np.any(pack.ocv_v.y <= 0.0) or np.any(pack.resistance_ohm.y < 0.0):
            raise ValueError("Pack OCV must be positive and resistance nonnegative")
        if (
            motor.peak_torque_nm <= 0.0
            or motor.peak_power_w <= 0.0
            or motor.maximum_speed_rpm <= 0.0
            or motor.torque_constant_nm_per_arms <= 0.0
            or motor.maximum_phase_current_arms <= 0.0
            or motor.pole_pairs <= 0
        ):
            raise ValueError("Motor limits and constants must be positive")
        if (
            motor.phase_resistance_ohm_at_reference_c < 0.0
            or motor.ld_h < 0.0
            or motor.lq_h < 0.0
            or motor.induced_voltage_ll_rms_per_rpm < 0.0
        ):
            raise ValueError(
                "Motor resistance, inductance, and EMF must be nonnegative"
            )
        if (
            not 0.0 < inverter.voltage_utilization_vll_rms_per_vdc <= 1.0
            or inverter.maximum_bus_current_a <= 0.0
            or (
                inverter.maximum_regen_bus_current_a is not None
                and inverter.maximum_regen_bus_current_a <= 0.0
            )
        ):
            raise ValueError("Inverter voltage utilization/current limit is invalid")
        if (
            drivetrain.final_drive_ratio <= 0.0
            or drivetrain.gear_ratio <= 0.0
            or drivetrain.tire_radius_m <= 0.0
            or not 0.0 < drivetrain.mechanical_efficiency <= 1.0
        ):
            raise ValueError("Drivetrain ratios, radius, or efficiency are invalid")
        if self.terminal_power_limit_w <= 0.0:
            raise ValueError("Terminal power limit must be positive")


@dataclass(frozen=True)
class OperatingPoint:
    feasible: bool
    active_limiter: str
    motor_torque_nm: float
    motor_speed_rpm: float
    motor_shaft_power_w: float
    phase_current_arms: float
    id_arms: float
    iq_arms: float
    motor_voltage_required_vll_rms: float
    motor_voltage_available_vll_rms: float
    motor_copper_loss_w: float
    motor_iron_loss_w: float
    inverter_loss_w: float
    battery_ocv_v: float
    battery_terminal_voltage_v: float
    battery_current_a: float
    battery_terminal_power_w: float
    battery_resistive_loss_w: float

    def as_dict(self) -> dict[str, Any]:
        return self.__dict__.copy()


def load_powertrain_config(path: Path) -> PowertrainConfig:
    data = json.loads(path.read_text(encoding="utf-8"))
    pack_data = data["pack"]
    pack = PackConfig(
        series_cells=int(pack_data["series_cells"]),
        parallel_cells=int(pack_data["parallel_cells"]),
        base_series_cells=int(
            pack_data.get("base_series_cells", pack_data["series_cells"])
        ),
        base_parallel_cells=int(pack_data["base_parallel_cells"]),
        cell_capacity_ah=float(pack_data["cell_capacity_ah"]),
        cell_mass_kg=float(pack_data["cell_mass_kg"]),
        base_cell_mass_kg=float(
            pack_data.get("base_cell_mass_kg", pack_data["cell_mass_kg"])
        ),
        pack_mass_multiplier=float(pack_data["pack_mass_multiplier"]),
        initial_soc=float(pack_data["initial_soc"]),
        minimum_soc=float(pack_data["minimum_soc"]),
        minimum_cell_voltage_v=float(pack_data["minimum_cell_voltage_v"]),
        maximum_cell_discharge_current_a=float(
            pack_data["maximum_cell_discharge_current_a"]
        ),
        cell_temperature_c=float(pack_data["cell_temperature_c"]),
        resistance_reference_temperature_c=float(
            pack_data["resistance_reference_temperature_c"]
        ),
        resistance_temperature_coefficient_per_c=float(
            pack_data["resistance_temperature_coefficient_per_c"]
        ),
        ocv_v=Curve.from_pairs(pack_data["cell_ocv_curve"]),
        resistance_ohm=Curve.from_pairs(
            pack_data["cell_resistance_curve_ohm"]
        ),
        maximum_cell_voltage_v=float(
            pack_data.get(
                "maximum_cell_voltage_v",
                max(float(pair[1]) for pair in pack_data["cell_ocv_curve"]),
            )
        ),
        maximum_cell_charge_current_a=(
            float(pack_data["maximum_cell_charge_current_a"])
            if pack_data.get("maximum_cell_charge_current_a") is not None
            else None
        ),
    )
    motor = MotorConfig(
        **{
            key: int(value) if key == "pole_pairs" else float(value)
            for key, value in data["motor"].items()
        }
    )
    inverter = InverterConfig(
        **{key: float(value) for key, value in data["inverter"].items()}
    )
    drivetrain = DrivetrainConfig(
        **{key: float(value) for key, value in data["drivetrain"].items()}
    )
    config = PowertrainConfig(
        pack=pack,
        motor=motor,
        inverter=inverter,
        drivetrain=drivetrain,
        terminal_power_limit_w=float(data["terminal_power_limit_w"]),
    )
    config.validate()
    return config


class PowertrainModel:
    def __init__(self, config: PowertrainConfig):
        self.config = config
        self._cached_pack_soc: float | None = None
        self._cached_pack_ocv_v = 0.0
        self._cached_pack_resistance_ohm = 0.0

    def _pack_values(self, soc: float) -> tuple[float, float]:
        """Cache the pack values shared by torque-search calls at one SOC."""

        if self._cached_pack_soc != soc:
            pack = self.config.pack
            self._cached_pack_soc = soc
            self._cached_pack_ocv_v = pack.open_circuit_voltage_v(soc)
            self._cached_pack_resistance_ohm = pack.pack_resistance_ohm(soc)
        return self._cached_pack_ocv_v, self._cached_pack_resistance_ohm

    def _pack_terminal_for_power(
        self,
        soc: float,
        requested_terminal_power_w: float,
        *,
        maximum_charge_current_a: float | None = None,
        maximum_terminal_charge_power_w: float | None = None,
    ) -> tuple[bool, float, float]:
        pack = self.config.pack
        ocv, resistance = self._pack_values(soc)
        if requested_terminal_power_w == 0.0:
            return True, ocv, 0.0
        if resistance <= 1e-15:
            current = requested_terminal_power_w / ocv
            terminal_voltage = ocv
        else:
            discriminant = (
                ocv**2 - 4.0 * resistance * requested_terminal_power_w
            )
            if discriminant < 0.0:
                return False, math.nan, math.nan
            square_root = math.sqrt(discriminant)
            current = (
                2.0 * requested_terminal_power_w / (ocv + square_root)
            )
            terminal_voltage = ocv - current * resistance
        if current >= 0.0:
            maximum_current = min(
                pack.maximum_discharge_current_a,
                self.config.inverter.maximum_bus_current_a,
            )
            feasible = (
                current <= maximum_current + 1e-9
                and terminal_voltage
                >= pack.minimum_terminal_voltage_v - 1e-9
            )
        else:
            charge_current_limit = min(
                pack.maximum_charge_current_a,
                self.config.inverter.maximum_regen_bus_current_limit_a,
                (
                    maximum_charge_current_a
                    if maximum_charge_current_a is not None
                    else math.inf
                ),
            )
            charge_power_ok = (
                maximum_terminal_charge_power_w is None
                or -requested_terminal_power_w
                <= maximum_terminal_charge_power_w + 1e-9
            )
            feasible = (
                soc < 1.0 - 1e-12
                and -current <= charge_current_limit + 1e-9
                and terminal_voltage
                <= pack.maximum_terminal_voltage_v + 1e-9
                and charge_power_ok
            )
        return feasible, terminal_voltage, current

    @staticmethod
    def _field_weakening_current(
        iq_arms: float,
        id_min_arms: float,
        resistance_ohm: float,
        omega_e_rad_s: float,
        ld_h: float,
        lq_h: float,
        back_emf_phase_rms_v: float,
        voltage_limit_phase_rms_v: float,
    ) -> tuple[bool, float]:
        def voltage_squared(id_arms: float) -> float:
            vd = resistance_ohm * id_arms - omega_e_rad_s * lq_h * iq_arms
            vq = (
                resistance_ohm * iq_arms
                + back_emf_phase_rms_v
                + omega_e_rad_s * ld_h * id_arms
            )
            return vd**2 + vq**2

        limit_squared = voltage_limit_phase_rms_v**2
        if voltage_squared(0.0) <= limit_squared:
            return True, 0.0
        a = resistance_ohm**2 + (omega_e_rad_s * ld_h) ** 2
        b = 2.0 * (
            -resistance_ohm * omega_e_rad_s * lq_h * iq_arms
            + omega_e_rad_s
            * ld_h
            * (resistance_ohm * iq_arms + back_emf_phase_rms_v)
        )
        c = (
            (omega_e_rad_s * lq_h * iq_arms) ** 2
            + (resistance_ohm * iq_arms + back_emf_phase_rms_v) ** 2
            - limit_squared
        )
        if a <= 0.0:
            return False, 0.0
        discriminant = b**2 - 4.0 * a * c
        if discriminant < 0.0:
            return False, 0.0
        roots = (
            (-b + math.sqrt(discriminant)) / (2.0 * a),
            (-b - math.sqrt(discriminant)) / (2.0 * a),
        )
        candidates = [
            root for root in roots if id_min_arms - 1e-9 <= root <= 1e-9
        ]
        if not candidates:
            return False, 0.0
        selected = max(candidates)
        return voltage_squared(selected) <= limit_squared + 1e-6, selected

    def operating_point(
        self,
        motor_torque_nm: float,
        motor_speed_rpm: float,
        soc: float,
        *,
        classify_limiter: bool = False,
    ) -> OperatingPoint:
        config = self.config
        pack = config.pack
        motor = config.motor
        inverter = config.inverter
        torque = max(motor_torque_nm, 0.0)
        speed_rpm = max(motor_speed_rpm, 0.0)
        ocv, _ = self._pack_values(soc)
        omega_m = speed_rpm * 2.0 * math.pi / 60.0
        shaft_power = torque * omega_m
        iq = torque / motor.torque_constant_nm_per_arms
        structural_feasible = (
            torque <= motor.peak_torque_nm + 1e-9
            and shaft_power <= motor.peak_power_w + 1e-9
            and speed_rpm <= motor.maximum_speed_rpm + 1e-9
            and iq <= motor.maximum_phase_current_arms + 1e-9
            and soc >= pack.minimum_soc - 1e-12
        )
        id_min = -math.sqrt(
            max(motor.maximum_phase_current_arms**2 - iq**2, 0.0)
        )
        id_arms = 0.0
        terminal_voltage = ocv
        bus_current = 0.0
        terminal_power = 0.0
        copper_loss = 0.0
        iron_loss = motor.iron_loss_w(speed_rpm)
        inverter_loss = 0.0
        voltage_required_ll = 0.0
        voltage_available_ll = (
            inverter.voltage_utilization_vll_rms_per_vdc * terminal_voltage
        )
        coupled_feasible = structural_feasible

        converged = False
        for _ in range(30):
            phase_current = math.sqrt(iq**2 + id_arms**2)
            copper_loss = (
                3.0 * motor.hot_phase_resistance_ohm * phase_current**2
            )
            motor_input_power = shaft_power + copper_loss + iron_loss
            inverter_loss = inverter.loss_w(
                terminal_voltage, phase_current, motor_input_power
            )
            terminal_power = motor_input_power + inverter_loss
            pack_feasible, terminal_voltage, bus_current = (
                self._pack_terminal_for_power(soc, terminal_power)
            )
            if not pack_feasible:
                coupled_feasible = False
                break
            back_emf_phase = (
                motor.induced_voltage_ll_rms_per_rpm
                * speed_rpm
                / math.sqrt(3.0)
            )
            omega_e = motor.pole_pairs * omega_m
            voltage_available_ll = (
                inverter.voltage_utilization_vll_rms_per_vdc
                * terminal_voltage
            )
            voltage_ok, new_id = self._field_weakening_current(
                iq,
                id_min,
                motor.hot_phase_resistance_ohm,
                omega_e,
                motor.ld_h,
                motor.lq_h,
                back_emf_phase,
                voltage_available_ll / math.sqrt(3.0),
            )
            if not voltage_ok:
                coupled_feasible = False
                break
            if abs(new_id - id_arms) < 1e-7:
                id_arms = new_id
                converged = True
                break
            id_arms = new_id

        if coupled_feasible and not converged:
            coupled_feasible = False

        # Close the electrical power balance at the final field-weakening
        # current instead of returning the previous fixed-point iterate.
        if coupled_feasible:
            phase_current = math.sqrt(iq**2 + id_arms**2)
            copper_loss = (
                3.0 * motor.hot_phase_resistance_ohm * phase_current**2
            )
            motor_input_power = shaft_power + copper_loss + iron_loss
            inverter_loss = inverter.loss_w(
                terminal_voltage, phase_current, motor_input_power
            )
            terminal_power = motor_input_power + inverter_loss
            pack_feasible, terminal_voltage, bus_current = (
                self._pack_terminal_for_power(soc, terminal_power)
            )
            coupled_feasible = coupled_feasible and pack_feasible

        phase_current = math.sqrt(iq**2 + id_arms**2)
        vd = (
            motor.hot_phase_resistance_ohm * id_arms
            - motor.pole_pairs * omega_m * motor.lq_h * iq
        )
        vq = (
            motor.hot_phase_resistance_ohm * iq
            + motor.induced_voltage_ll_rms_per_rpm
            * speed_rpm
            / math.sqrt(3.0)
            + motor.pole_pairs * omega_m * motor.ld_h * id_arms
        )
        voltage_required_ll = math.sqrt(3.0) * math.sqrt(vd**2 + vq**2)
        voltage_available_ll = (
            inverter.voltage_utilization_vll_rms_per_vdc * terminal_voltage
        )
        terminal_power_from_bus = terminal_voltage * bus_current
        coupled_feasible = (
            coupled_feasible
            and phase_current <= motor.maximum_phase_current_arms + 1e-6
            and voltage_required_ll <= voltage_available_ll + 1e-5
            and terminal_power_from_bus <= config.terminal_power_limit_w + 1e-6
            and abs(terminal_power - terminal_power_from_bus)
            <= max(1e-5, 1e-10 * terminal_power)
        )
        active_limiter = "feasible"
        if classify_limiter:
            active_limiter = self._classify_limiter(
                torque,
                speed_rpm,
                shaft_power,
                phase_current,
                voltage_required_ll,
                voltage_available_ll,
                terminal_power,
                terminal_voltage,
                bus_current,
            )
        return OperatingPoint(
            feasible=coupled_feasible,
            active_limiter=active_limiter,
            motor_torque_nm=torque,
            motor_speed_rpm=speed_rpm,
            motor_shaft_power_w=shaft_power,
            phase_current_arms=phase_current,
            id_arms=id_arms,
            iq_arms=iq,
            motor_voltage_required_vll_rms=voltage_required_ll,
            motor_voltage_available_vll_rms=voltage_available_ll,
            motor_copper_loss_w=copper_loss,
            motor_iron_loss_w=iron_loss,
            inverter_loss_w=inverter_loss,
            battery_ocv_v=ocv,
            battery_terminal_voltage_v=terminal_voltage,
            battery_current_a=bus_current,
            battery_terminal_power_w=terminal_power_from_bus,
            battery_resistive_loss_w=(
                bus_current**2 * pack.pack_resistance_ohm(soc)
            ),
        )

    def regenerative_operating_point(
        self,
        motor_torque_nm: float,
        motor_speed_rpm: float,
        soc: float,
        *,
        maximum_charge_current_a: float | None = None,
        maximum_terminal_charge_power_w: float | None = None,
    ) -> OperatingPoint:
        """Return a signed generator operating point.

        Torque, shaft power, battery current, and battery terminal power are
        negative during useful regeneration. Losses remain positive. Keeping
        this path separate leaves the established positive-torque solution and
        its lookup surface numerically unchanged.
        """

        config = self.config
        pack = config.pack
        motor = config.motor
        inverter = config.inverter
        torque = -abs(motor_torque_nm)
        speed_rpm = max(motor_speed_rpm, 0.0)
        ocv, _ = self._pack_values(soc)
        omega_m = speed_rpm * 2.0 * math.pi / 60.0
        shaft_power = torque * omega_m
        iq = torque / motor.torque_constant_nm_per_arms
        structural_feasible = (
            abs(torque) <= motor.peak_torque_nm + 1e-9
            and abs(shaft_power) <= motor.peak_power_w + 1e-9
            and speed_rpm <= motor.maximum_speed_rpm + 1e-9
            and abs(iq) <= motor.maximum_phase_current_arms + 1e-9
            and pack.minimum_soc - 1e-12 <= soc <= 1.0 + 1e-12
        )
        id_min = -math.sqrt(
            max(motor.maximum_phase_current_arms**2 - iq**2, 0.0)
        )
        id_arms = 0.0
        terminal_voltage = ocv
        bus_current = 0.0
        terminal_power = 0.0
        copper_loss = 0.0
        iron_loss = motor.iron_loss_w(speed_rpm)
        inverter_loss = 0.0
        voltage_available_ll = (
            inverter.voltage_utilization_vll_rms_per_vdc * terminal_voltage
        )
        coupled_feasible = structural_feasible
        converged = False

        for _ in range(30):
            phase_current = math.sqrt(iq**2 + id_arms**2)
            copper_loss = (
                3.0 * motor.hot_phase_resistance_ohm * phase_current**2
            )
            motor_ac_power = shaft_power + copper_loss + iron_loss
            inverter_loss = inverter.loss_w(
                terminal_voltage, phase_current, motor_ac_power
            )
            terminal_power = motor_ac_power + inverter_loss
            pack_feasible, terminal_voltage, bus_current = (
                self._pack_terminal_for_power(
                    soc,
                    terminal_power,
                    maximum_charge_current_a=maximum_charge_current_a,
                    maximum_terminal_charge_power_w=(
                        maximum_terminal_charge_power_w
                    ),
                )
            )
            if not pack_feasible:
                coupled_feasible = False
                break
            back_emf_phase = (
                motor.induced_voltage_ll_rms_per_rpm
                * speed_rpm
                / math.sqrt(3.0)
            )
            omega_e = motor.pole_pairs * omega_m
            voltage_available_ll = (
                inverter.voltage_utilization_vll_rms_per_vdc
                * terminal_voltage
            )
            voltage_ok, new_id = self._field_weakening_current(
                iq,
                id_min,
                motor.hot_phase_resistance_ohm,
                omega_e,
                motor.ld_h,
                motor.lq_h,
                back_emf_phase,
                voltage_available_ll / math.sqrt(3.0),
            )
            if not voltage_ok:
                coupled_feasible = False
                break
            if abs(new_id - id_arms) < 1e-7:
                id_arms = new_id
                converged = True
                break
            id_arms = new_id

        if coupled_feasible and not converged:
            coupled_feasible = False

        if coupled_feasible:
            phase_current = math.sqrt(iq**2 + id_arms**2)
            copper_loss = (
                3.0 * motor.hot_phase_resistance_ohm * phase_current**2
            )
            motor_ac_power = shaft_power + copper_loss + iron_loss
            inverter_loss = inverter.loss_w(
                terminal_voltage, phase_current, motor_ac_power
            )
            terminal_power = motor_ac_power + inverter_loss
            pack_feasible, terminal_voltage, bus_current = (
                self._pack_terminal_for_power(
                    soc,
                    terminal_power,
                    maximum_charge_current_a=maximum_charge_current_a,
                    maximum_terminal_charge_power_w=(
                        maximum_terminal_charge_power_w
                    ),
                )
            )
            coupled_feasible = coupled_feasible and pack_feasible

        phase_current = math.sqrt(iq**2 + id_arms**2)
        vd = (
            motor.hot_phase_resistance_ohm * id_arms
            - motor.pole_pairs * omega_m * motor.lq_h * iq
        )
        vq = (
            motor.hot_phase_resistance_ohm * iq
            + motor.induced_voltage_ll_rms_per_rpm
            * speed_rpm
            / math.sqrt(3.0)
            + motor.pole_pairs * omega_m * motor.ld_h * id_arms
        )
        voltage_required_ll = math.sqrt(3.0) * math.sqrt(vd**2 + vq**2)
        voltage_available_ll = (
            inverter.voltage_utilization_vll_rms_per_vdc * terminal_voltage
        )
        terminal_power_from_bus = terminal_voltage * bus_current
        coupled_feasible = (
            coupled_feasible
            and phase_current <= motor.maximum_phase_current_arms + 1e-6
            and voltage_required_ll <= voltage_available_ll + 1e-5
            and (
                terminal_power_from_bus <= config.terminal_power_limit_w + 1e-6
            )
            and abs(terminal_power - terminal_power_from_bus)
            <= max(1e-5, 1e-10 * max(abs(terminal_power), 1.0))
        )
        return OperatingPoint(
            feasible=coupled_feasible,
            active_limiter=(
                "regenerative_feasible"
                if coupled_feasible
                else "regenerative_infeasible"
            ),
            motor_torque_nm=torque,
            motor_speed_rpm=speed_rpm,
            motor_shaft_power_w=shaft_power,
            phase_current_arms=phase_current,
            id_arms=id_arms,
            iq_arms=iq,
            motor_voltage_required_vll_rms=voltage_required_ll,
            motor_voltage_available_vll_rms=voltage_available_ll,
            motor_copper_loss_w=copper_loss,
            motor_iron_loss_w=iron_loss,
            inverter_loss_w=inverter_loss,
            battery_ocv_v=ocv,
            battery_terminal_voltage_v=terminal_voltage,
            battery_current_a=bus_current,
            battery_terminal_power_w=terminal_power_from_bus,
            battery_resistive_loss_w=(
                bus_current**2 * pack.pack_resistance_ohm(soc)
            ),
        )

    def regenerative_point_for_terminal_power(
        self,
        target_terminal_charge_power_w: float,
        motor_speed_rpm: float,
        soc: float,
        maximum_motor_torque_magnitude_nm: float,
        *,
        maximum_charge_current_a: float | None = None,
    ) -> OperatingPoint:
        """Match a positive-magnitude terminal charge target when feasible."""

        neutral = self.operating_point(0.0, motor_speed_rpm, soc)
        target = max(float(target_terminal_charge_power_w), 0.0)
        speed_rpm = max(float(motor_speed_rpm), 0.0)
        if (
            target <= 0.0
            or speed_rpm <= 1e-9
            or maximum_motor_torque_magnitude_nm <= 0.0
            or soc >= 1.0 - 1e-12
        ):
            return replace(neutral, active_limiter="regen_disabled")

        motor = self.config.motor
        omega = speed_rpm * 2.0 * math.pi / 60.0
        upper = min(
            float(maximum_motor_torque_magnitude_nm),
            motor.peak_torque_nm,
            motor.maximum_phase_current_arms
            * motor.torque_constant_nm_per_arms,
            motor.peak_power_w / omega,
        )

        def point_at(magnitude_nm: float) -> OperatingPoint:
            return self.regenerative_operating_point(
                -magnitude_nm,
                speed_rpm,
                soc,
                maximum_charge_current_a=maximum_charge_current_a,
            )

        maximum_point = point_at(upper)
        if not maximum_point.feasible:
            lower_feasible = 0.0
            upper_infeasible = upper
            for _ in range(24):
                middle = 0.5 * (lower_feasible + upper_infeasible)
                point = point_at(middle)
                if point.feasible:
                    lower_feasible = middle
                    maximum_point = point
                else:
                    upper_infeasible = middle
            upper = lower_feasible

        requested_power = -target
        if (
            not maximum_point.feasible
            or maximum_point.battery_terminal_power_w >= -1e-6
        ):
            return replace(neutral, active_limiter="regen_source_limited")
        if maximum_point.battery_terminal_power_w > requested_power:
            return replace(
                maximum_point, active_limiter="regen_source_limited"
            )

        lower = 0.0
        upper_target = upper
        lower_point = self.regenerative_operating_point(
            0.0,
            speed_rpm,
            soc,
            maximum_charge_current_a=maximum_charge_current_a,
        )
        # Eighteen bisection steps resolve a 220 N-m range below 0.001 N-m,
        # far tighter than the lap/telemetry model fidelity.
        for _ in range(18):
            middle = 0.5 * (lower + upper_target)
            point = point_at(middle)
            if not point.feasible:
                upper_target = middle
                continue
            if point.battery_terminal_power_w > requested_power:
                lower = middle
                lower_point = point
            else:
                upper_target = middle
        return replace(
            lower_point, active_limiter="regen_terminal_power_target"
        )

    def _classify_limiter(
        self,
        torque_nm: float,
        speed_rpm: float,
        shaft_power_w: float,
        phase_current_arms: float,
        required_voltage_v: float,
        available_voltage_v: float,
        terminal_power_w: float,
        terminal_voltage_v: float,
        bus_current_a: float,
    ) -> str:
        config = self.config
        pack = config.pack
        limits = {
            "motor_torque": torque_nm / config.motor.peak_torque_nm,
            "motor_speed": speed_rpm / config.motor.maximum_speed_rpm,
            "motor_peak_power": shaft_power_w / config.motor.peak_power_w,
            "phase_current": (
                phase_current_arms / config.motor.maximum_phase_current_arms
            ),
            "motor_voltage": required_voltage_v / max(available_voltage_v, 1e-12),
            "terminal_power_limit": (
                terminal_power_w / config.terminal_power_limit_w
            ),
            "bus_current": bus_current_a
            / min(
                pack.maximum_discharge_current_a,
                config.inverter.maximum_bus_current_a,
            ),
            "minimum_terminal_voltage": (
                pack.minimum_terminal_voltage_v
                / max(terminal_voltage_v, 1e-12)
            ),
        }
        active = [
            name
            for name in (
                "terminal_power_limit",
                "bus_current",
                "minimum_terminal_voltage",
                "motor_voltage",
                "phase_current",
                "motor_torque",
                "motor_peak_power",
                "motor_speed",
            )
            if limits[name] >= 0.9999
        ]
        return "+".join(active) if active else max(limits, key=limits.get)

    def maximum_available_torque(
        self, motor_speed_rpm: float, soc: float
    ) -> OperatingPoint:
        motor = self.config.motor
        if (
            motor_speed_rpm > motor.maximum_speed_rpm
            or soc < self.config.pack.minimum_soc
        ):
            return self.operating_point(0.0, motor_speed_rpm, soc)
        upper = min(
            motor.peak_torque_nm,
            motor.maximum_phase_current_arms
            * motor.torque_constant_nm_per_arms,
        )
        if motor_speed_rpm > 1e-9:
            omega = motor_speed_rpm * 2.0 * math.pi / 60.0
            upper = min(upper, motor.peak_power_w / omega)
        lower = 0.0
        # Twenty-two bisection steps resolve the torque ceiling to better than
        # 0.0001 N-m over the full motor range, far below lap-model fidelity.
        for _ in range(22):
            middle = 0.5 * (lower + upper)
            if self.operating_point(middle, motor_speed_rpm, soc).feasible:
                lower = middle
            else:
                upper = middle
        return self.operating_point(
            lower, motor_speed_rpm, soc, classify_limiter=True
        )


class MaximumTorqueSurface:
    """Fast bilinear lookup over exact coupled-model solutions."""

    def __init__(
        self,
        model: PowertrainModel,
        maximum_vehicle_speed_mps: float,
        soc_count: int = 25,
        speed_count: int = 121,
        verification_iterations: int = 12,
    ):
        pack = model.config.pack
        self.model = model
        if (
            soc_count < 2
            or speed_count < 2
            or verification_iterations < 1
        ):
            raise ValueError("Torque surface dimensions must each be at least 2")
        self.verification_iterations = verification_iterations
        self.soc = np.linspace(pack.minimum_soc, pack.initial_soc, soc_count)
        self.speed_mps = np.linspace(
            0.0, maximum_vehicle_speed_mps, speed_count
        )
        self.torque_nm = np.empty((soc_count, speed_count), dtype=float)
        for soc_index, soc in enumerate(self.soc):
            for speed_index, speed in enumerate(self.speed_mps):
                rpm = model.config.drivetrain.motor_speed_rpm(speed)
                self.torque_nm[soc_index, speed_index] = (
                    model.maximum_available_torque(rpm, soc).motor_torque_nm
                )

    def maximum_torque_nm(
        self, vehicle_speed_mps: float, soc: float, *, verify: bool = True
    ) -> float:
        soc_value = float(np.clip(soc, self.soc[0], self.soc[-1]))
        speed_value = float(
            np.clip(vehicle_speed_mps, self.speed_mps[0], self.speed_mps[-1])
        )
        soc_index = int(
            np.clip(np.searchsorted(self.soc, soc_value) - 1, 0, len(self.soc) - 2)
        )
        speed_index = int(
            np.clip(
                np.searchsorted(self.speed_mps, speed_value) - 1,
                0,
                len(self.speed_mps) - 2,
            )
        )
        soc_fraction = (
            (soc_value - self.soc[soc_index])
            / (self.soc[soc_index + 1] - self.soc[soc_index])
        )
        speed_fraction = (
            (speed_value - self.speed_mps[speed_index])
            / (
                self.speed_mps[speed_index + 1]
                - self.speed_mps[speed_index]
            )
        )
        lower_soc_torque = (
            (1.0 - speed_fraction)
            * self.torque_nm[soc_index, speed_index]
            + speed_fraction * self.torque_nm[soc_index, speed_index + 1]
        )
        upper_soc_torque = (
            (1.0 - speed_fraction)
            * self.torque_nm[soc_index + 1, speed_index]
            + speed_fraction
            * self.torque_nm[soc_index + 1, speed_index + 1]
        )
        estimate = float(
            (1.0 - soc_fraction) * lower_soc_torque
            + soc_fraction * upper_soc_torque
        )
        if not verify:
            return estimate
        rpm = self.model.config.drivetrain.motor_speed_rpm(speed_value)
        if self.model.operating_point(estimate, rpm, soc_value).feasible:
            return estimate

        # Bilinear interpolation can sit just above a curved constraint
        # boundary. First establish a tight feasible bracket; this avoids a
        # full-range solve at most track samples while still returning a point
        # explicitly checked by the coupled model.
        lower = 0.995 * estimate
        if not self.model.operating_point(lower, rpm, soc_value).feasible:
            lower = 0.0
        upper = estimate
        for _ in range(self.verification_iterations):
            middle = 0.5 * (lower + upper)
            if self.model.operating_point(middle, rpm, soc_value).feasible:
                lower = middle
            else:
                upper = middle
        return lower

    def to_frame(self) -> pd.DataFrame:
        rows = []
        drivetrain = self.model.config.drivetrain
        for soc_index, soc in enumerate(self.soc):
            for speed_index, speed in enumerate(self.speed_mps):
                motor_torque = self.torque_nm[soc_index, speed_index]
                rows.append(
                    {
                        "soc": soc,
                        "vehicle_speed_mps": speed,
                        "motor_speed_rpm": drivetrain.motor_speed_rpm(speed),
                        "maximum_motor_torque_nm": motor_torque,
                        "maximum_wheel_force_n": (
                            drivetrain.force_for_motor_torque(motor_torque)
                        ),
                    }
                )
        return pd.DataFrame(rows)
