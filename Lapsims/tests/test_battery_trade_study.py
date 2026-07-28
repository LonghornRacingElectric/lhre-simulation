"""Checks for battery topology policy and exact-distance aggregation."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "tools"))

from battery_trade_study import (  # noqa: E402
    MAXIMUM_PACK_VOLTAGE_V,
    MAXIMUM_USABLE_ENERGY_KWH,
    generate_candidates,
    truncate_trace_at_distance,
)
from run_battery_trade_study import (  # noqa: E402
    _expand_lower_power_bracket,
)


class BatteryCandidateTest(unittest.TestCase):
    def test_generated_candidates_obey_hard_constraints(self) -> None:
        cell = {
            "cell_id": "test",
            "manufacturer": "Test",
            "model": "Test",
            "form_factor": "18650",
            "nominal_voltage_v": 3.6,
            "maximum_voltage_v": 4.2,
            "minimum_voltage_v": 2.5,
            "capacity_ah": 2.0,
            "dcir_ohm": 0.01,
            "mass_kg": 0.01,
            "diameter_m": 0.005,
            "height_m": 0.01,
            "continuous_current_a": 10.0,
            "source_url": "https://example.invalid",
            "dcir_basis": "test",
        }
        accepted, rejected = generate_candidates([cell], [142, 143])
        self.assertTrue(accepted)
        self.assertTrue(rejected)
        self.assertTrue(
            all(
                row["maximum_pack_voltage_v"] < MAXIMUM_PACK_VOLTAGE_V
                and row["model_usable_energy_kwh"]
                <= MAXIMUM_USABLE_ENERGY_KWH
                for row in accepted
            )
        )
        self.assertTrue(
            any(
                "maximum_voltage_not_below_600v"
                in row["rejection_reasons"]
                for row in rejected
            )
        )


class ExactDistanceTest(unittest.TestCase):
    def test_partial_segment_uses_constant_acceleration_time(self) -> None:
        full_dx = 100.0
        start_speed = 10.0
        acceleration = 1.0
        end_speed = math.sqrt(start_speed**2 + 2.0 * acceleration * full_dx)
        full_dt = 2.0 * full_dx / (start_speed + end_speed)
        trace = pd.DataFrame(
            [
                {
                    "lap": 1,
                    "segment": 0,
                    "lap_distance_m": full_dx,
                    "cumulative_distance_m": full_dx,
                    "dx_m": full_dx,
                    "speed_mps": start_speed,
                    "next_speed_mps": end_speed,
                    "longitudinal_accel_mps2": acceleration,
                    "time_in_segment_s": full_dt,
                    "elapsed_time_s": full_dt,
                    "soc": 1.0,
                    "next_soc": 0.9,
                    "battery_terminal_power_w": 10000.0,
                    "battery_ocv_v": 400.0,
                    "battery_current_a": 30.0,
                    "cumulative_terminal_energy_wh": (
                        10000.0 * full_dt / 3600.0
                    ),
                    "cumulative_chemical_energy_wh": (
                        12000.0 * full_dt / 3600.0
                    ),
                }
            ]
        )
        target = 40.0
        exact = truncate_trace_at_distance(trace, target)
        expected_end_speed = math.sqrt(
            start_speed**2 + 2.0 * acceleration * target
        )
        expected_dt = 2.0 * target / (start_speed + expected_end_speed)
        expected_fraction = expected_dt / full_dt
        self.assertAlmostEqual(
            float(exact["cumulative_distance_m"].iloc[-1]), target
        )
        self.assertAlmostEqual(
            float(exact["time_in_segment_s"].iloc[-1]), expected_dt
        )
        self.assertAlmostEqual(
            float(exact["next_soc"].iloc[-1]),
            1.0 - 0.1 * expected_fraction,
        )
        self.assertAlmostEqual(
            float(exact["cumulative_terminal_energy_wh"].iloc[-1]),
            10000.0 * expected_dt / 3600.0,
        )

    def test_partial_regen_segment_preserves_signed_energy(self) -> None:
        full_dt = 10.0
        trace = pd.DataFrame(
            [
                {
                    "lap": 1,
                    "segment": 0,
                    "lap_distance_m": 100.0,
                    "cumulative_distance_m": 100.0,
                    "dx_m": 100.0,
                    "speed_mps": 10.0,
                    "next_speed_mps": 10.0,
                    "longitudinal_accel_mps2": 0.0,
                    "time_in_segment_s": full_dt,
                    "elapsed_time_s": full_dt,
                    "soc": 0.5,
                    "next_soc": 0.51,
                    "battery_terminal_power_w": -8000.0,
                    "battery_ocv_v": 400.0,
                    "battery_current_a": -20.0,
                    "cumulative_terminal_energy_wh": (
                        -8000.0 * full_dt / 3600.0
                    ),
                    "cumulative_chemical_energy_wh": (
                        -8000.0 * full_dt / 3600.0
                    ),
                }
            ]
        )
        exact = truncate_trace_at_distance(trace, 40.0)
        expected_dt = 4.0
        self.assertAlmostEqual(
            float(exact["cumulative_terminal_energy_wh"].iloc[-1]),
            -8000.0 * expected_dt / 3600.0,
        )
        self.assertAlmostEqual(
            float(exact["next_soc"].iloc[-1]), 0.504
        )


class ReserveBracketFallbackTest(unittest.TestCase):
    def test_expands_below_centered_grid_until_reserve_is_positive(self) -> None:
        cache = {
            power: {
                "completed_target_distance": True,
                "power_limit_kw": power,
                "remaining_usable_chemical_kwh": 1.0 - power / 100.0,
            }
            for power in (72.0, 77.0, 80.0)
        }
        evaluated: list[float] = []

        def evaluate(power_kw: float) -> dict:
            evaluated.append(power_kw)
            row = {
                "completed_target_distance": True,
                "power_limit_kw": power_kw,
                "remaining_usable_chemical_kwh": 1.0 - power_kw / 100.0,
            }
            cache[round(power_kw, 8)] = row
            return row

        positive, bracket = _expand_lower_power_bracket(
            evaluate,
            cache,
            target_reserve=0.5,
        )
        self.assertEqual(evaluated, [54.0, 40.5])
        self.assertIsNotNone(positive)
        self.assertEqual(bracket, (40.5, 54.0))


if __name__ == "__main__":
    unittest.main()
