"""Focused tests for the common-field reduced-DOF CG report builder."""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from build_reduced_dof_cg_comparison import (
    DEFAULT_SCORING_REFERENCE,
    StudySource,
    _load_scoring_reference,
    _load_source_events,
    _rescore_common_field,
    build_report_bundle,
)

HEIGHTS_IN = (8.0, 9.2, 10.4, 11.6, 12.8, 14.0)
EVENT_INPUTS = {
    "acceleration": (75.0, 3.50),
    "skidpad": (57.33406592801372, 4.50),
    "autocross": (791.0, 42.0),
    "michigan_endurance": (1069.968773, 62.0),
}


def _case_name(height_in: float) -> str:
    return f"cg_{height_in:.1f}in".replace(".", "p")


def _write_study(
    root: Path,
    model_offset_s: float,
    *,
    legacy: bool,
    native_model_key: str | None = None,
) -> None:
    root.mkdir(parents=True)
    rows: list[dict[str, object]] = []
    published_totals: list[dict[str, object]] = []
    for height_index, height_in in enumerate(HEIGHTS_IN):
        case = _case_name(height_in)
        for event_slug, (distance_m, base_time_s) in EVENT_INPUTS.items():
            event_offset = {
                "acceleration": -0.010 * height_index,
                "skidpad": 0.010 * height_index,
                "autocross": 0.080 * height_index,
                "michigan_endurance": 0.100 * height_index,
            }[event_slug]
            rows.append(
                {
                    "cg_case": case,
                    "cg_height_in": height_in,
                    "cg_height_m": height_in * 0.0254,
                    "event_slug": event_slug,
                    "event_name": event_slug,
                    "lap_time_s": base_time_s + model_offset_s + event_offset,
                    "track_length_m": distance_m,
                    "projected_points": 50.0 + height_index,
                    "converged": True,
                    "ggv_speed_cap_segments": 0,
                    "effective_front_antiroll_stiffness_fraction": (
                        0.45 + 0.01 * height_index
                    ),
                    "effective_front_roll_stiffness_fraction": (
                        0.48 + 0.005 * height_index
                    ),
                    "effective_brake_distribution_front": (0.65 + 0.02 * height_index),
                    **(
                        {"model_key": native_model_key}
                        if native_model_key is not None
                        else {}
                    ),
                }
            )
        if legacy:
            published_totals.append(
                {
                    "cg_case": case,
                    "projected_timed_event_points": 490.0 - height_index,
                }
            )
    pd.DataFrame(rows).to_csv(root / "event_results.csv", index=False)
    if legacy:
        pd.DataFrame(published_totals).to_csv(root / "case_totals.csv", index=False)


class ReducedDofComparisonBuilderTest(unittest.TestCase):
    def test_native_model_key_is_preserved_under_source_model_key(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "3dof"
            _write_study(
                root,
                0.20,
                legacy=False,
                native_model_key="dyn_py_3dof_qss",
            )

            events = _load_source_events(StudySource("3dof", root))

            self.assertEqual(set(events["model_key"]), {"3dof"})
            self.assertEqual(
                set(events["source_model_key"]),
                {"dyn_py_3dof_qss"},
            )

    def test_only_exact_fastest_simulation_ties_share_maximum(self) -> None:
        rows: list[dict[str, object]] = []
        for event_slug, (distance_m, base_time_s) in EVENT_INPUTS.items():
            for model_key, offset_s in (
                ("legacy", 0.20),
                ("3dof", 0.00),
                ("6dof", 0.00 if event_slug == "acceleration" else 0.10),
            ):
                rows.append(
                    {
                        "event_slug": event_slug,
                        "cg_case": f"{model_key}:case",
                        "lap_time_s": base_time_s + offset_s,
                        "track_length_m": distance_m,
                    }
                )
        events = pd.DataFrame(rows)
        near_tie = events[
            (events["event_slug"] == "acceleration")
            & (events["cg_case"] == "legacy:case")
        ].index[0]
        events.loc[near_tie, "lap_time_s"] = 3.500000000001

        rescored, summary = _rescore_common_field(
            events,
            _load_scoring_reference(DEFAULT_SCORING_REFERENCE),
        )

        acceleration = rescored[rescored["event_slug"] == "acceleration"]
        self.assertEqual(summary["acceleration"]["simulated_maximum_score_count"], 2)
        winners = acceleration[acceleration["common_is_event_fastest"]]
        self.assertEqual(set(winners["cg_case"]), {"3dof:case", "6dof:case"})
        near = acceleration[acceleration["cg_case"] == "legacy:case"].iloc[0]
        self.assertLess(
            near["common_projected_points"],
            near["common_maximum_points"],
        )

    def test_common_field_bundle_prefixes_and_rescores_all_cases(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            temp = Path(temporary)
            legacy = temp / "legacy"
            dof3 = temp / "3dof"
            dof6 = temp / "6dof"
            output = temp / "comparison"
            _write_study(legacy, 0.40, legacy=True)
            _write_study(
                dof3,
                0.20,
                legacy=False,
                native_model_key="dyn_py_3dof_qss",
            )
            _write_study(
                dof6,
                0.00,
                legacy=False,
                native_model_key="dyn_py_6dof_qss",
            )

            validation = build_report_bundle(
                three_dof_root=dof3,
                six_dof_root=dof6,
                legacy_root=legacy,
                output_root=output,
            )

            self.assertEqual(validation["status"], "passed")
            self.assertEqual(validation["combined_event_row_count"], 72)
            self.assertEqual(validation["combined_case_count"], 18)
            self.assertTrue(validation["tuning_plot_written"])
            for event_slug, count in validation[
                "exact_fastest_simulation_count_by_event"
            ].items():
                self.assertEqual(count, 1, event_slug)

            events = pd.read_csv(output / "combined_event_results.csv")
            totals = pd.read_csv(output / "combined_case_totals.csv")
            self.assertEqual(events["cg_case"].nunique(), 18)
            self.assertTrue(events["cg_case"].str.contains(":").all())
            self.assertEqual(
                set(events.loc[events["model_key"] == "3dof", "source_model_key"]),
                {"dyn_py_3dof_qss"},
            )
            self.assertEqual(
                set(events.loc[events["model_key"] == "6dof", "source_model_key"]),
                {"dyn_py_6dof_qss"},
            )
            self.assertEqual(len(totals), 18)
            self.assertEqual(
                totals["legacy_published_timed_event_points"].notna().sum(),
                6,
            )

            endurance = events[
                (events["model_key"] == "6dof")
                & (events["source_cg_case"] == _case_name(8.0))
                & (events["event_slug"] == "michigan_endurance")
            ].iloc[0]
            self.assertAlmostEqual(
                endurance["common_projected_competition_time_s"],
                endurance["lap_time_s"] * 22000.0 / endurance["track_length_m"],
            )

            expected_files = {
                "combined_event_results.csv",
                "combined_case_totals.csv",
                "correlations_by_model.csv",
                "correlations_by_model.json",
                "model_comparison_by_height.csv",
                "cg_points_overlay.png",
                "event_time_percent_change.png",
                "dof6_minus_3dof_delta.png",
                "tuning_setup.png",
                "validation_summary.json",
                "study_report.md",
            }
            self.assertEqual(
                expected_files,
                {path.name for path in output.iterdir()},
            )

            report = (output / "study_report.md").read_text(encoding="utf-8")
            self.assertIn("confounded by both the tire-mu change", report)
            self.assertIn("separately retuned design-outcome", report)
            self.assertIn("common-field point values are cohort-dependent", report)
            self.assertIn("## Model validity / interpretation", report)
            self.assertIn("`fitted_tire_mu_scale =\n  0.622543713`", report)
            self.assertIn("the +/-0.25 rad sideslip guard", report)
            self.assertIn("below the TIR's 100 N minimum", report)
            self.assertIn("requested 42.053998 m/s endpoint is empty", report)
            self.assertIn("## Raw simulated event times by CG height", report)
            self.assertIn("Endurance modeled lap (s)", report)
            self.assertIn("## High-to-low raw event-time change", report)
            self.assertIn("## Per-height tuned setup", report)
            self.assertIn("Fixed front brake bias (%)", report)
            self.assertIn("`dof6_minus_3dof_delta.png`", report)
            self.assertIn(
                "| 8.0000 | Reduced 6DOF | 45.0000 | 48.0000 | 65.0000 |",
                report,
            )
            summary = json.loads(
                (output / "validation_summary.json").read_text(encoding="utf-8")
            )
            self.assertEqual(summary["legacy_published_total_count"], 6)

    def test_inconsistent_tuning_across_event_rows_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            temp = Path(temporary)
            legacy = temp / "legacy"
            dof3 = temp / "3dof"
            dof6 = temp / "6dof"
            output = temp / "comparison"
            _write_study(legacy, 0.40, legacy=True)
            _write_study(dof3, 0.20, legacy=False)
            _write_study(dof6, 0.00, legacy=False)

            events = pd.read_csv(dof6 / "event_results.csv")
            events.loc[0, "effective_brake_distribution_front"] += 0.01
            events.to_csv(dof6 / "event_results.csv", index=False)

            with self.assertRaisesRegex(
                ValueError,
                "Tuning column .* is not constant",
            ):
                build_report_bundle(
                    three_dof_root=dof3,
                    six_dof_root=dof6,
                    legacy_root=legacy,
                    output_root=output,
                )

    def test_missing_completed_root_fails_before_creating_output(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            temp = Path(temporary)
            legacy = temp / "legacy"
            dof3 = temp / "3dof"
            output = temp / "comparison"
            _write_study(legacy, 0.40, legacy=True)
            _write_study(dof3, 0.20, legacy=False)

            with self.assertRaisesRegex(
                FileNotFoundError,
                "Comparison inputs are not complete yet",
            ):
                build_report_bundle(
                    three_dof_root=dof3,
                    six_dof_root=temp / "missing-6dof",
                    legacy_root=legacy,
                    output_root=output,
                )
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
