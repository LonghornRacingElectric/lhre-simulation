"""Unit and integration checks for battery-aware endurance."""

from __future__ import annotations

import sys
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from endurance_solver import simulate_endurance  # noqa: E402
from openlap_solver import load_vehicle  # noqa: E402
from powertrain_model import PowertrainModel, load_powertrain_config  # noqa: E402


class PowertrainTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config = load_powertrain_config(
            ROOT / "inputs" / "powertrain_130s5p_p30b_provisional.json"
        )
        cls.model = PowertrainModel(cls.config)

    def test_maximum_torque_obeys_all_coupled_constraints(self) -> None:
        for soc in (1.0, 0.5, 0.1):
            for rpm in (0.0, 2000.0, 4000.0, 6000.0):
                point = self.model.maximum_available_torque(rpm, soc)
                self.assertTrue(point.feasible)
                self.assertLessEqual(
                    point.battery_terminal_power_w, 80000.0
                )
                self.assertAlmostEqual(
                    point.battery_terminal_power_w,
                    point.battery_terminal_voltage_v
                    * point.battery_current_a,
                    places=6,
                )
                self.assertLessEqual(
                    point.phase_current_arms,
                    self.config.motor.maximum_phase_current_arms + 1e-5,
                )
                self.assertLessEqual(
                    point.motor_voltage_required_vll_rms,
                    point.motor_voltage_available_vll_rms + 1e-4,
                )
                self.assertLessEqual(
                    point.battery_current_a,
                    min(
                        self.config.pack.maximum_discharge_current_a,
                        self.config.inverter.maximum_bus_current_a,
                    )
                    + 1e-5,
                )
                self.assertGreaterEqual(
                    point.battery_terminal_voltage_v,
                    self.config.pack.minimum_terminal_voltage_v - 1e-5,
                )

    def test_lower_soc_reduces_high_speed_torque(self) -> None:
        high_soc = self.model.maximum_available_torque(6000.0, 1.0)
        low_soc = self.model.maximum_available_torque(6000.0, 0.1)
        self.assertGreater(high_soc.motor_torque_nm, low_soc.motor_torque_nm)
        self.assertIn("terminal_power_limit", high_soc.active_limiter)
        self.assertIn("motor_voltage", high_soc.active_limiter)
        self.assertIn("bus_current", low_soc.active_limiter)

    def test_zero_resistance_and_lossless_chain_close_legacy_curve(self) -> None:
        lossless_pack = replace(
            self.config.pack,
            resistance_ohm=replace(
                self.config.pack.resistance_ohm,
                y=np.zeros_like(self.config.pack.resistance_ohm.y),
            ),
            maximum_cell_discharge_current_a=1e6,
        )
        lossless_motor = replace(
            self.config.motor,
            winding_temperature_c=25.0,
            iron_loss_c0_w=0.0,
            iron_loss_c1_w_per_radps=0.0,
            iron_loss_c2_w_per_radps2=0.0,
            phase_resistance_ohm_at_reference_c=0.0,
            induced_voltage_ll_rms_per_rpm=0.0,
            maximum_phase_current_arms=1e6,
        )
        lossless_inverter = replace(
            self.config.inverter,
            base_loss_w=0.0,
            conduction_loss_fraction=0.0,
            switching_frequency_hz=0.0,
            maximum_bus_current_a=1e6,
        )
        model = PowertrainModel(
            replace(
                self.config,
                pack=lossless_pack,
                motor=lossless_motor,
                inverter=lossless_inverter,
            )
        )
        low_speed = model.maximum_available_torque(1000.0, 1.0)
        self.assertAlmostEqual(low_speed.motor_torque_nm, 220.0, places=3)
        rpm_at_corner = 80000.0 / 220.0 * 60.0 / (2.0 * np.pi)
        at_corner = model.maximum_available_torque(rpm_at_corner, 1.0)
        self.assertAlmostEqual(at_corner.motor_torque_nm, 220.0, places=3)
        above_corner = model.maximum_available_torque(5000.0, 1.0)
        expected = 80000.0 / (5000.0 * 2.0 * np.pi / 60.0)
        self.assertAlmostEqual(
            above_corner.motor_torque_nm, expected, places=3
        )

    def test_pack_size_changes_all_coupled_pack_properties(self) -> None:
        four_parallel = self.config.with_parallel_cells(4).pack
        six_parallel = self.config.with_parallel_cells(6).pack
        self.assertLess(four_parallel.capacity_ah, six_parallel.capacity_ah)
        self.assertGreater(
            four_parallel.pack_resistance_ohm(1.0),
            six_parallel.pack_resistance_ohm(1.0),
        )
        self.assertLess(
            four_parallel.maximum_discharge_current_a,
            six_parallel.maximum_discharge_current_a,
        )
        self.assertLess(
            four_parallel.vehicle_mass_delta_kg(),
            six_parallel.vehicle_mass_delta_kg(),
        )

    def test_pack_mass_delta_handles_series_and_cell_mass(self) -> None:
        baseline = self.config.pack
        self.assertAlmostEqual(baseline.vehicle_mass_delta_kg(), 0.0)
        lighter = replace(
            baseline,
            series_cells=140,
            parallel_cells=3,
            cell_mass_kg=0.067,
        )
        expected = (
            140 * 3 * 0.067 - 130 * 5 * 0.047
        ) * baseline.pack_mass_multiplier
        self.assertAlmostEqual(lighter.vehicle_mass_delta_kg(), expected)

    def test_spinning_zero_torque_includes_electrical_losses(self) -> None:
        point = self.model.operating_point(0.0, 3000.0, 0.8)
        self.assertTrue(point.feasible)
        self.assertEqual(point.motor_shaft_power_w, 0.0)
        self.assertGreater(point.motor_iron_loss_w, 0.0)
        self.assertGreater(point.inverter_loss_w, 0.0)
        self.assertGreater(point.battery_current_a, 0.0)
        self.assertGreater(point.battery_terminal_power_w, 0.0)

    def test_regenerative_pack_point_is_signed_and_heats_pack(self) -> None:
        point = self.model.regenerative_point_for_terminal_power(
            8000.0,
            2500.0,
            0.8,
            100.0,
        )
        self.assertTrue(point.feasible)
        self.assertLess(point.motor_torque_nm, 0.0)
        self.assertLess(point.motor_shaft_power_w, 0.0)
        self.assertLess(point.battery_current_a, 0.0)
        self.assertLess(point.battery_terminal_power_w, 0.0)
        self.assertGreater(
            point.battery_terminal_voltage_v, point.battery_ocv_v
        )
        self.assertGreater(point.battery_resistive_loss_w, 0.0)
        self.assertAlmostEqual(
            point.battery_terminal_power_w,
            point.battery_terminal_voltage_v * point.battery_current_a,
            places=6,
        )
        chemical_power = (
            point.battery_ocv_v * point.battery_current_a
        )
        self.assertAlmostEqual(
            chemical_power,
            point.battery_terminal_power_w
            + point.battery_resistive_loss_w,
            places=6,
        )


class EnduranceIntegrationTest(unittest.TestCase):
    @staticmethod
    def _inputs() -> tuple:
        vehicle = load_vehicle(ROOT / "inputs" / "openlap_vehicle.json")
        full_track = pd.read_csv(
            ROOT / "inputs" / "michigan_openlap_track.csv"
        )
        indices = np.arange(0, len(full_track), 40)
        track = full_track.iloc[indices].copy().reset_index(drop=True)
        edges = np.concatenate(([0.0], track["distance_m"].to_numpy()))
        track["dx_m"] = np.diff(edges)
        config = load_powertrain_config(
            ROOT / "inputs" / "powertrain_130s5p_p30b_provisional.json"
        )
        return vehicle, track, config

    def test_chronological_soc_and_energy_balance(self) -> None:
        vehicle, track, config = self._inputs()
        trace, summary = simulate_endurance(
            vehicle,
            track,
            PowertrainModel(config),
            laps=2,
            surface_soc_count=5,
            surface_speed_count=15,
        )
        self.assertTrue(summary["completed"])
        self.assertTrue(
            summary["checks"]["terminal_power_never_exceeds_limit"]
        )
        self.assertTrue(
            summary["checks"][
                "terminal_power_matches_voltage_times_current"
            ]
        )
        self.assertTrue(summary["checks"]["soc_monotonic_nonincreasing"])
        self.assertLess(
            abs(summary["checks"]["pack_energy_balance_relative"]), 1e-10
        )
        self.assertLess(
            abs(summary["checks"]["powertrain_chain_balance_relative"]),
            1e-10,
        )
        self.assertLess(trace["soc"].iloc[-1], trace["soc"].iloc[0])
        required_columns = {
            "soc",
            "soe",
            "battery_ocv_v",
            "battery_terminal_voltage_v",
            "battery_current_a",
            "battery_terminal_power_w",
            "battery_terminal_power_from_vi_w",
            "available_motor_torque_nm",
            "used_motor_torque_nm",
            "phase_current_arms",
            "drivetrain_loss_w",
            "active_limiter",
        }
        self.assertTrue(required_columns.issubset(trace.columns))

    def test_regen_is_endurance_only_signed_and_time_neutral(self) -> None:
        vehicle, track, config = self._inputs()
        no_regen_trace, no_regen = simulate_endurance(
            vehicle,
            track,
            PowertrainModel(config),
            laps=2,
            surface_soc_count=5,
            surface_speed_count=15,
        )
        zero_trace, zero = simulate_endurance(
            vehicle,
            track,
            PowertrainModel(config),
            laps=2,
            surface_soc_count=5,
            surface_speed_count=15,
            regen_terminal_power_target_w=0.0,
        )
        regen_trace, regen = simulate_endurance(
            vehicle,
            track,
            PowertrainModel(config),
            laps=2,
            surface_soc_count=5,
            surface_speed_count=15,
            regen_terminal_power_target_w=10_221.0,
        )
        pd.testing.assert_frame_equal(no_regen_trace, zero_trace)
        self.assertEqual(no_regen["elapsed_time_s"], zero["elapsed_time_s"])
        self.assertLessEqual(
            regen["elapsed_time_s"], no_regen["elapsed_time_s"] + 1e-9
        )
        self.assertGreater(regen["terminal_regenerated_energy_kwh"], 0.0)
        self.assertGreater(regen["regen_pack_resistive_loss_kwh"], 0.0)
        self.assertLess(
            regen["terminal_energy_kwh"], no_regen["terminal_energy_kwh"]
        )
        self.assertTrue((regen_trace["battery_current_a"] < 0.0).any())
        self.assertTrue((regen_trace["next_soc"] > regen_trace["soc"]).any())
        self.assertTrue(regen["checks"]["soc_within_bounds"])
        self.assertTrue(regen["checks"]["regen_only_during_braking"])
        self.assertTrue(regen["checks"]["regen_target_not_exceeded"])
        self.assertLess(
            abs(regen["checks"]["pack_energy_balance_relative"]), 1e-10
        )
        self.assertLess(
            abs(regen["checks"]["powertrain_chain_balance_relative"]),
            1e-10,
        )


if __name__ == "__main__":
    unittest.main()
