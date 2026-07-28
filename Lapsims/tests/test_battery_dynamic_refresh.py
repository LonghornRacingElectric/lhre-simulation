"""Tests for endurance-only dynamic-points refreshes."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from battery_dynamic_points import (  # noqa: E402
    BASELINE_CANDIDATE_ID,
    EVENT_RULES,
)
from battery_dynamic_refresh import (  # noqa: E402
    copy_sprint_long_file,
    refresh_dynamic_summary,
    sprint_invariant_columns,
    validate_refresh,
)


def _source_summary() -> pd.DataFrame:
    rows = []
    definitions = (
        (BASELINE_CANDIDATE_ID, "molicel_p30b", 130, 5, 650, 38.1875, 261.0),
        ("candidate_b_120s4p", "candidate_b", 120, 4, 480, 30.0, 253.0),
    )
    for index, (
        candidate_id,
        cell_id,
        series,
        parallel,
        total,
        pack_mass,
        vehicle_mass,
    ) in enumerate(definitions):
        pack_nonenergy = 290.0 - index
        mass_nonenergy = 289.0 - index
        old_endurance = 250.0 - index
        old_efficiency = 90.0 - index
        row = {
            "candidate_id": candidate_id,
            "cell_id": cell_id,
            "manufacturer": "Example",
            "cell_model": cell_id,
            "series_cells": series,
            "parallel_cells": parallel,
            "total_cells": total,
            "pack_mass_kg": pack_mass,
            "vehicle_mass_kg": vehicle_mass,
            "nominal_pack_energy_kwh": 7.0 - 0.2 * index,
            "model_usable_energy_kwh": 6.8 - 0.2 * index,
            "endurance_terminal_power_limit_kw": 30.0,
            "pack_aware_acceleration_raw_time_s": 4.2 + 0.1 * index,
            "pack_aware_skidpad_raw_time_s": 4.8 + 0.01 * index,
            "pack_aware_autocross_raw_time_s": 52.0 + 0.2 * index,
            "mass_isolated_acceleration_raw_time_s": 4.1 + 0.1 * index,
            "mass_isolated_skidpad_raw_time_s": 4.8 + 0.01 * index,
            "mass_isolated_autocross_raw_time_s": 51.8 + 0.2 * index,
            "pack_aware_nonenergy_points": pack_nonenergy,
            "mass_isolated_sprint_nonenergy_points": mass_nonenergy,
            "pack_aware_minus_mass_isolated_nonenergy_points": (
                pack_nonenergy - mass_nonenergy
            ),
            "raw_endurance_time_s": 1200.0 + 10.0 * index,
            "projected_endurance_time_s": 1312.281 + 10.0 * index,
            "endurance_points": old_endurance,
            "projected_average_lap_s": 65.192 + index,
            "projected_terminal_energy_kwh_per_lap": 0.15 + 0.01 * index,
            "efficiency_factor": 0.7 - 0.05 * index,
            "efficiency_points": old_efficiency,
            "pack_aware_performance_points_excluding_efficiency": (
                pack_nonenergy + old_endurance
            ),
            "pack_aware_full_dynamic_points": (
                pack_nonenergy + old_endurance + old_efficiency
            ),
            "mass_isolated_sprint_hybrid_performance_points_excluding_efficiency": (
                mass_nonenergy + old_endurance
            ),
            "mass_isolated_sprint_hybrid_full_dynamic_points": (
                mass_nonenergy + old_endurance + old_efficiency
            ),
            "pack_aware_minus_mass_isolated_full_dynamic_points": (
                pack_nonenergy - mass_nonenergy
            ),
            "pack_aware_performance_points_excluding_efficiency_rank": float(
                index + 1
            ),
            "pack_aware_full_dynamic_points_rank": float(index + 1),
            "mass_isolated_sprint_hybrid_performance_points_excluding_efficiency_rank": float(
                index + 1
            ),
            "mass_isolated_sprint_hybrid_full_dynamic_points_rank": float(
                index + 1
            ),
        }
        rows.append(row)
    return pd.DataFrame(rows)


def _battery_results() -> pd.DataFrame:
    return pd.DataFrame(
        [
            {
                "candidate_id": BASELINE_CANDIDATE_ID,
                "cell_id": "molicel_p30b",
                "manufacturer": "Example",
                "cell_model": "molicel_p30b",
                "series_cells": 130,
                "parallel_cells": 5,
                "total_cells": 650,
                "pack_mass_kg": 38.1875,
                "vehicle_mass_kg": 261.0,
                "nominal_pack_energy_kwh": 7.0,
                "model_usable_energy_kwh": 6.8,
                "power_limit_kw": 42.0,
                "elapsed_time_s": 1180.0,
                "equivalent_average_lap_time_s": 58.0,
                "terminal_energy_kwh": 4.7,
                "completed_target_distance": True,
                "constraint_valid": True,
            },
            {
                "candidate_id": "candidate_b_120s4p",
                "cell_id": "candidate_b",
                "manufacturer": "Example",
                "cell_model": "candidate_b",
                "series_cells": 120,
                "parallel_cells": 4,
                "total_cells": 480,
                "pack_mass_kg": 30.0,
                "vehicle_mass_kg": 253.0,
                "nominal_pack_energy_kwh": 6.8,
                "model_usable_energy_kwh": 6.6,
                "power_limit_kw": 36.0,
                "elapsed_time_s": 1240.0,
                "equivalent_average_lap_time_s": 61.0,
                "terminal_energy_kwh": 4.2,
                "completed_target_distance": True,
                "constraint_valid": True,
            },
        ]
    )


class BatteryDynamicRefreshTest(unittest.TestCase):
    def test_refresh_preserves_every_sprint_invariant_column_exactly(self) -> None:
        source = _source_summary()
        battery = _battery_results()
        invariant_columns = sprint_invariant_columns(source)

        refreshed, shared = refresh_dynamic_summary(source, battery)

        pd.testing.assert_frame_equal(
            source[invariant_columns],
            refreshed[invariant_columns],
            check_exact=True,
            check_dtype=True,
        )
        self.assertEqual(len(shared), 2)
        baseline = refreshed.loc[
            refreshed["candidate_id"] == BASELINE_CANDIDATE_ID
        ].iloc[0]
        self.assertEqual(baseline["raw_endurance_time_s"], 1180.0)
        self.assertEqual(baseline["terminal_energy_kwh"], 4.7)
        self.assertEqual(baseline["endurance_terminal_power_limit_kw"], 42.0)
        self.assertAlmostEqual(
            baseline["projected_endurance_time_s"],
            EVENT_RULES["endurance"].tmin_s,
        )
        self.assertAlmostEqual(baseline["efficiency_points"], 100.0)
        self.assertAlmostEqual(
            baseline["pack_aware_full_dynamic_points"],
            baseline["pack_aware_nonenergy_points"]
            + baseline["endurance_points"]
            + baseline["efficiency_points"],
        )

        validation = validate_refresh(source, refreshed, battery)
        self.assertTrue(validation["all_checks_passed"], validation)

    def test_candidate_set_mismatch_is_rejected(self) -> None:
        battery = _battery_results().iloc[:1].copy()
        with self.assertRaisesRegex(ValueError, "candidate sets differ"):
            refresh_dynamic_summary(_source_summary(), battery)

    def test_sprint_long_csv_copy_is_byte_identical(self) -> None:
        payload = b"a,b\r\n1,2\r\n3,4\r\n"
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source.csv"
            destination = root / "nested" / "destination.csv"
            source.write_bytes(payload)

            source_hash = copy_sprint_long_file(source, destination)

            self.assertEqual(destination.read_bytes(), payload)
            self.assertEqual(len(source_hash), 64)


if __name__ == "__main__":
    unittest.main()
