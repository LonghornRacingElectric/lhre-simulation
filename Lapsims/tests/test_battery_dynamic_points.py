"""Tests for battery dynamic-event scoring and mass isolation."""

from __future__ import annotations

import json
import sys
import unittest
from dataclasses import replace
from pathlib import Path

import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from battery_dynamic_points import (  # noqa: E402
    BASELINE_CANDIDATE_ID,
    EFFICIENCY_INPUTS,
    EVENT_RULES,
    MASS_ISOLATED_MODE,
    PACK_AWARE_MODE,
    efficiency_projection,
    project_nonenergy_scores,
)
from openlap_solver import load_vehicle, solve_track  # noqa: E402


class BatteryDynamicPointsUnitTest(unittest.TestCase):
    def test_official_score_anchors_and_slow_thresholds(self) -> None:
        for rule in EVENT_RULES.values():
            self.assertAlmostEqual(
                rule.score(rule.tmin_s), rule.maximum_points, places=12
            )
            self.assertAlmostEqual(
                rule.score(rule.tmax_s), rule.completion_points, places=12
            )
            self.assertAlmostEqual(
                rule.score(rule.tmax_s * 1.1),
                rule.completion_points,
                places=12,
            )

    def test_inherited_efficiency_baseline_maps_to_winner(self) -> None:
        projected = efficiency_projection(
            candidate_average_lap_s=1.0,
            candidate_terminal_energy_kwh=1.0,
            baseline_average_lap_s=1.0,
            baseline_terminal_energy_kwh=1.0,
        )
        self.assertAlmostEqual(
            projected["projected_average_lap_s"],
            EFFICIENCY_INPUTS["winner_average_lap_s"],
            places=12,
        )
        self.assertAlmostEqual(
            projected["projected_terminal_energy_kwh_per_lap"],
            EFFICIENCY_INPUTS["winner_total_energy_kwh"]
            / EFFICIENCY_INPUTS["winner_lap_count"],
            places=12,
        )
        self.assertAlmostEqual(projected["efficiency_points"], 100.0, places=12)

    def test_sprint_projection_preserves_same_mode_time_ratio(self) -> None:
        baseline = {
            slug: {
                "raw_time_s": 10.0,
                "converged": True,
                "iterations": 1,
                "maximum_speed_mps": 1.0,
                "minimum_speed_mps": 1.0,
            }
            for slug in ("acceleration", "skidpad", "autocross")
        }
        candidate = {
            slug: {**result, "raw_time_s": 11.0}
            for slug, result in baseline.items()
        }
        projected = project_nonenergy_scores(candidate, baseline)
        for slug, result in projected.items():
            self.assertAlmostEqual(
                result["projected_time_s"],
                EVENT_RULES[slug].tmin_s * 1.1,
                places=12,
            )

    def test_added_mass_loses_acceleration_points_on_common_curve(self) -> None:
        vehicle = load_vehicle(ROOT / "inputs" / "openlap_vehicle.json")
        track = pd.read_csv(
            ROOT / "inputs" / "events" / "acceleration_openlap_track.csv"
        )
        _, light = solve_track(replace(vehicle, mass=vehicle.mass - 10.0), track, False)
        _, heavy = solve_track(replace(vehicle, mass=vehicle.mass + 10.0), track, False)
        self.assertLess(
            EVENT_RULES["acceleration"].score(heavy["lap_time_s"]),
            EVENT_RULES["acceleration"].score(light["lap_time_s"]),
        )


class BatteryDynamicPointsArtifactTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.output_dir = (
            ROOT / "outputs" / "battery_dynamic_points_20260725"
        )
        if not cls.output_dir.exists():
            raise unittest.SkipTest("Dynamic-points artifacts have not been run")
        cls.summary = pd.read_csv(
            cls.output_dir / "battery_dynamic_points_summary.csv"
        )
        cls.long = pd.read_csv(
            cls.output_dir / "battery_dynamic_event_results_long.csv"
        )
        cls.validation = json.loads(
            (cls.output_dir / "validation_report.json").read_text(
                encoding="utf-8"
            )
        )

    def test_all_128_candidates_and_event_rows_exist(self) -> None:
        self.assertEqual(len(self.summary), 128)
        self.assertEqual(self.summary["candidate_id"].nunique(), 128)
        self.assertEqual(len(self.long), 128 * 2 * 3)
        self.assertEqual(
            set(self.long["simulation_mode"]),
            {PACK_AWARE_MODE, MASS_ISOLATED_MODE},
        )

    def test_baseline_and_total_limits(self) -> None:
        baseline = self.summary.loc[
            self.summary["candidate_id"] == BASELINE_CANDIDATE_ID
        ].iloc[0]
        self.assertAlmostEqual(
            baseline["pack_aware_acceleration_raw_time_s"], 4.284205, places=4
        )
        self.assertAlmostEqual(
            baseline["pack_aware_autocross_raw_time_s"], 52.259586, places=4
        )
        self.assertAlmostEqual(
            baseline["pack_aware_skidpad_raw_time_s"], 4.800188, places=4
        )
        self.assertAlmostEqual(
            baseline["pack_aware_nonenergy_points"], 300.0, places=9
        )
        self.assertAlmostEqual(
            baseline["mass_isolated_sprint_nonenergy_points"],
            300.0,
            places=9,
        )
        self.assertTrue(
            self.summary["pack_aware_full_dynamic_points"].between(
                0.0, 675.0
            ).all()
        )

    def test_generated_validation_passes(self) -> None:
        self.assertTrue(self.validation["all_checks_passed"])


if __name__ == "__main__":
    unittest.main()
