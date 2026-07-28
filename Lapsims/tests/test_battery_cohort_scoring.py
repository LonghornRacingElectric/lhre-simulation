"""Tests for simulated-cohort-normalized battery scoring."""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from battery_cohort_scoring import (  # noqa: E402
    _validation_report,
    cohort_efficiency_projection,
    score_cohort,
    score_time_series,
)
from battery_dynamic_points import EVENT_RULES  # noqa: E402


class CohortScoreUnitTest(unittest.TestCase):
    def test_fastest_time_receives_event_maximum(self) -> None:
        points, rule = score_time_series(
            pd.Series([5.0, 4.0, 4.5]), EVENT_RULES["acceleration"]
        )
        self.assertEqual(rule.tmin_s, 4.0)
        self.assertAlmostEqual(points.iloc[1], 100.0)
        self.assertGreater(points.iloc[2], points.iloc[0])

    def test_skidpad_uses_squared_formula_with_cohort_anchor(self) -> None:
        points, rule = score_time_series(
            pd.Series([5.0, 4.8]), EVENT_RULES["skidpad"]
        )
        expected = rule.score(5.0)
        self.assertAlmostEqual(points.iloc[0], expected)
        self.assertAlmostEqual(points.iloc[1], 75.0)

    def test_tmax_receives_completion_points(self) -> None:
        points, rule = score_time_series(
            pd.Series([4.0, 6.0]), EVENT_RULES["acceleration"]
        )
        self.assertAlmostEqual(rule.tmax_s, 6.0)
        self.assertAlmostEqual(points.iloc[1], 4.5)

    def test_strict_efficiency_marks_energy_excess_ineligible(self) -> None:
        results, metadata = cohort_efficiency_projection(
            pd.Series([100.0, 101.0, 102.0]),
            pd.Series([6.9, 6.0, 3.0]),
        )
        self.assertFalse(results.iloc[0]["cohort_efficiency_eligible"])
        self.assertEqual(results.iloc[0]["cohort_efficiency_points"], 0.0)
        self.assertAlmostEqual(metadata["eligible_tmin_s"], 101.0)
        self.assertAlmostEqual(metadata["eligible_emin_kwh"], 3.0)

    def test_strict_efficiency_max_factor_receives_100_points(self) -> None:
        results, _ = cohort_efficiency_projection(
            pd.Series([100.0, 110.0, 120.0]),
            pd.Series([6.0, 3.0, 4.0]),
        )
        max_factor_index = results["cohort_efficiency_factor"].idxmax()
        self.assertAlmostEqual(
            results.loc[max_factor_index, "cohort_efficiency_points"], 100.0
        )

    def test_validation_accepts_scenario_specific_efficiency_anchors(self) -> None:
        rows = []
        for index in range(3):
            row = {
                "candidate_id": f"candidate_{index}",
                "cell_id": f"cell_{index}",
                "manufacturer": "Example",
                "cell_model": f"Model {index}",
                "series_cells": 120 + index * 5,
                "parallel_cells": 3,
                "total_cells": (120 + index * 5) * 3,
                "pack_mass_kg": 30.0 + index,
                "vehicle_mass_kg": 250.0 + index,
                "nominal_pack_energy_kwh": 6.0 + index * 0.2,
                "model_usable_energy_kwh": 5.8 + index * 0.2,
                "endurance_terminal_power_limit_kw": 30.0 + index,
                "raw_endurance_time_s": 1000.0 + index * 200.0,
                "terminal_energy_kwh": 5.0 + index,
                "projected_average_lap_s": 60.0 + index,
                "projected_terminal_energy_kwh_per_lap": 0.15 + 0.01 * index,
                "efficiency_factor": 0.7 - 0.1 * index,
                "efficiency_points": 90.0 - 10.0 * index,
            }
            for mode in ("pack_aware", "mass_isolated"):
                for event_index, slug in enumerate(
                    ("acceleration", "skidpad", "autocross")
                ):
                    row[f"{mode}_{slug}_raw_time_s"] = (
                        4.0 + event_index * 20.0 + index * 0.1
                    )
                    row[f"{mode}_{slug}_converged"] = True
            rows.append(row)
        source = pd.DataFrame(rows)

        summary, long, anchors = score_cohort(source)
        validation = _validation_report(source, summary, long, anchors)

        self.assertTrue(validation["all_checks_passed"], validation)
        self.assertEqual(
            anchors["efficiency"]["energy_ineligible_count"],
            1,
        )
        self.assertNotEqual(
            anchors["efficiency"]["eligible_tmin_s"],
            1244.122620288108,
        )


class CohortArtifactTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.output_dir = (
            ROOT
            / "outputs"
            / "battery_dynamic_points_cohort_normalized_20260725"
        )
        if not cls.output_dir.exists():
            raise unittest.SkipTest("Cohort-normalized artifacts not generated")
        cls.summary = pd.read_csv(
            cls.output_dir / "battery_cohort_points_summary.csv"
        )
        cls.long = pd.read_csv(
            cls.output_dir / "battery_cohort_event_results_long.csv"
        )
        cls.validation = json.loads(
            (cls.output_dir / "validation_report.json").read_text(
                encoding="utf-8"
            )
        )

    def test_all_128_candidates_are_present(self) -> None:
        self.assertEqual(len(self.summary), 128)
        self.assertEqual(self.summary["candidate_id"].nunique(), 128)

    def test_seven_timed_rows_per_candidate(self) -> None:
        self.assertEqual(len(self.long), 128 * 7)
        self.assertTrue(
            self.long.groupby("candidate_id").size().eq(7).all()
        )

    def test_every_timed_event_has_a_max_score_winner(self) -> None:
        maxima = {
            "acceleration": 100.0,
            "skidpad": 75.0,
            "autocross": 125.0,
            "endurance": 275.0,
        }
        for (_, event), group in self.long.groupby(
            ["simulation_mode_short", "event_slug"]
        ):
            self.assertAlmostEqual(group["points"].max(), maxima[event])

    def test_strict_efficiency_expected_eligibility(self) -> None:
        self.assertEqual(
            (~self.summary["cohort_efficiency_energy_eligible"]).sum(), 4
        )
        self.assertTrue(
            self.summary["cohort_efficiency_time_eligible"].all()
        )
        self.assertEqual(
            (
                self.summary["cohort_efficiency_points"].sub(100.0).abs()
                < 1e-9
            ).sum(),
            1,
        )

    def test_recommended_total_arithmetic(self) -> None:
        expected = (
            self.summary[
                "pack_aware_cohort_performance_points_excluding_efficiency"
            ]
            + self.summary["cohort_efficiency_points"]
        )
        self.assertTrue(
            (
                expected
                - self.summary[
                    "pack_aware_fully_cohort_normalized_full_dynamic_points"
                ]
            )
            .abs()
            .lt(1e-9)
            .all()
        )

    def test_recommended_winner_regression(self) -> None:
        winner = self.summary.nlargest(
            1, "pack_aware_fully_cohort_normalized_full_dynamic_points"
        ).iloc[0]
        self.assertEqual(winner["candidate_id"], "molicel_p50b_140s3p")
        self.assertAlmostEqual(
            winner[
                "pack_aware_fully_cohort_normalized_full_dynamic_points"
            ],
            589.937268805531,
        )

    def test_generated_validation_passes(self) -> None:
        self.assertTrue(self.validation["all_checks_passed"])


if __name__ == "__main__":
    unittest.main()
