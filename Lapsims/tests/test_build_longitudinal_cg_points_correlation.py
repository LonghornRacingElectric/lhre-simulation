"""Tests for the longitudinal-CG smoke-test report builder."""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from build_longitudinal_cg_points_correlation import (
    DEFAULT_MU_SCALE,
    build_report_bundle,
)

REAR_FRACTIONS = (0.45, 0.50, 0.55)
NEW_REAR_FRACTIONS = (0.52, 0.54, 0.56, 0.58, 0.60)
EVENTS = {
    "acceleration": (75.0, (3.50, 3.50, 3.500000000001)),
    "skidpad": (57.33406592801372, (5.08, 5.00, 5.04)),
    "autocross": (791.0, (51.0, 50.0, 50.4)),
    "michigan_endurance": (1069.968773, (69.0, 68.0, 68.5)),
}


def _case_name(rear_fraction: float) -> str:
    return f"rear_{rear_fraction * 100:.0f}pct"


def _fixture_time(
    event_slug: str,
    rear_fraction: float,
    index: int,
    rear_fractions: tuple[float, ...],
) -> float:
    if rear_fractions == REAR_FRACTIONS:
        return float(EVENTS[event_slug][1][index])
    rear_percent = rear_fraction * 100.0
    formulas = {
        "acceleration": 4.02 - 0.018 * (rear_percent - 45.0),
        "skidpad": 5.00 + 0.0020 * (rear_percent - 54.0) ** 2,
        "autocross": 57.0 - 0.20 * (rear_percent - 45.0)
        + 0.012 * (rear_percent - 57.0) ** 2,
        "michigan_endurance": 65.0 - 0.25 * (rear_percent - 45.0)
        + 0.015 * (rear_percent - 57.0) ** 2,
    }
    return float(formulas[event_slug])


def _write_fixture(
    root: Path,
    *,
    rear_fractions: tuple[float, ...] = REAR_FRACTIONS,
    inconsistent_fraction: bool = False,
    tuning_name: str = "tuning.json",
) -> Path:
    root.mkdir(parents=True)
    wheelbase_m = 1.5494
    total_mass_kg = 261.07265114
    sprung_mass_kg = 230.72288408
    rows: list[dict[str, object]] = []
    tuning: list[dict[str, object]] = []
    audit_cases: list[dict[str, object]] = []
    audit_states: list[dict[str, object]] = []
    for index, rear_fraction in enumerate(rear_fractions):
        case = _case_name(rear_fraction)
        for event_slug, (track_length_m, times) in EVENTS.items():
            rows.append(
                {
                    "model_key": "dyn_py_6dof_qss",
                    "model_dof": 6,
                    "sweep_axis": "rear_static_weight_fraction",
                    "cg_case": case,
                    "cg_height_in": 11.5,
                    "rear_static_weight_fraction": rear_fraction,
                    "event_slug": event_slug,
                    "event_name": event_slug,
                    "lap_time_s": _fixture_time(
                        event_slug, rear_fraction, index, rear_fractions
                    ),
                    "track_length_m": track_length_m,
                    "converged": True,
                    "ggv_speed_cap_segments": 0,
                }
            )
        case_root = root / case
        case_root.mkdir()
        stored_rear = rear_fraction + (
            0.002 if inconsistent_fraction and index == len(rear_fractions) - 1 else 0.0
        )
        total_weight_n = total_mass_kg * 9.80665
        front_weight_n = total_weight_n * (1.0 - rear_fraction)
        rear_weight_n = total_weight_n * rear_fraction
        front_arb_fraction = 0.47 + 0.35 * rear_fraction
        metadata = {
            "model_family": "dyn_py_reduced_order_qss",
            "model_key": "dyn_py_6dof_qss",
            "model_dof": 6,
            "sweep_axis": "rear_static_weight_fraction",
            "cg_case": case,
            "cg_height_in": 11.5,
            "rear_static_weight_fraction": stored_rear,
            "front_static_weight_fraction": 1.0 - stored_rear,
            "longitudinal_cg_x_m": -rear_fraction * wheelbase_m,
            "sprung_mass_kg": sprung_mass_kg,
            "total_mass_kg": total_mass_kg,
            "tire_mu_scale": DEFAULT_MU_SCALE,
            "tire_mu_source": "test fixture",
            "front_antiroll_stiffness_fraction": front_arb_fraction,
            "front_elastic_roll_stiffness_fraction": 0.30 + 0.45 * rear_fraction,
            "effective_brake_distribution_front": 0.90 - 0.35 * rear_fraction,
            "effective_aero_balance_front": 0.5,
            "effective_cop_from_front_m": wheelbase_m / 2.0,
            "drive_distribution_front": 0.0,
            "limited_slip_differential_model": False,
            "wheel_load_min_n": 130.0 + 5.0 * index,
            "wheel_load_max_n": 1700.0 + 20.0 * index,
            "tire_valid_load_min_n": 100.0,
            "tire_valid_load_max_n": 1800.0,
            "wheel_load_outside_tire_validity": False,
            "ggv_source_sha256": f"hash-{case}",
            "ggv_ay_points": 23,
            "ggv_ay_max_g": 2.2,
            "ggv_ax_search_points": 301,
            "ggv_ax_binary_iterations": 10,
            "ggv_speed_slices": [
                0.0,
                0.1,
                *[float(value) for value in range(2, 43, 2)],
                42.053998,
            ],
            "ggv_reuse_validation": {
                "serialized_terminal_speed_mps": 42.0,
                "requested_terminal_slice_had_no_serialized_rows": True,
            },
            "vehicle": {
                "wheelbase": wheelbase_m,
                "front_static_frac": 1.0 - rear_fraction,
                "aero_balance_front": 0.5,
                "drive_distribution_front": 0.0,
                "fz_min_valid": 100.0,
                "fz_max_valid": 1800.0,
            },
            "reduced_vehicle_parameters": {
                "mass_kg": total_mass_kg,
                "sprung_mass_kg": sprung_mass_kg,
                "center_of_gravity_m": [-rear_fraction * wheelbase_m, 0.0, 0.2921],
                "static_wheel_loads_n": [
                    front_weight_n / 2.0,
                    front_weight_n / 2.0,
                    rear_weight_n / 2.0,
                    rear_weight_n / 2.0,
                ],
                "sprung_inertia_kg_m2": [
                    [14.55, 0.30, -5.37],
                    [0.30, 73.84, -0.05],
                    [-5.37, -0.05, 73.46],
                ],
                "suspension_stiffness_n_per_m": [
                    26955.5,
                    26955.5,
                    27897.3,
                    27897.3,
                ],
                "antiroll_stiffness_nm_per_rad": [
                    front_arb_fraction * 45547.94,
                    (1.0 - front_arb_fraction) * 45547.94,
                ],
                "cl_area_m2": 2.186115,
                "cd_area_m2": 1.229539,
                "peak_drive_power_w": 80000.0,
                "maximum_drive_speed_mps": 42.053998,
            },
        }
        (case_root / "case_metadata.json").write_text(
            json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
        )
        tuning.append(
            {
                "name": case,
                "rear_static_weight_fraction": rear_fraction,
                "front_antiroll_stiffness_fraction": front_arb_fraction,
                "front_elastic_roll_stiffness_fraction": 0.30
                + 0.45 * rear_fraction,
                "brake_distribution_front": 0.90 - 0.35 * rear_fraction,
                "pure_lateral_limit_g": 1.47
                - 0.20 * (rear_fraction - 0.53) ** 2,
                "weighted_pure_braking_limit_g": 1.65 + 0.20 * rear_fraction,
                "roll_optimization_at_search_bound": False,
                "brake_optimization_at_search_bound": False,
            }
        )
        audit_cases.append(
            {
                "cg_case": case,
                "model_dof": 6,
                "sweep_axis": "rear_static_weight_fraction",
                "rear_static_weight_fraction": rear_fraction,
                "sample_count": 2,
                "successful_trim_count": 1,
                "failed_trim_count": 1,
                "sampled_normal_load_min_n": 125.0 + index,
                "sampled_normal_load_max_n": 1720.0 + 10.0 * index,
                "sampled_wheel_lift_state_count": 0,
            }
        )
        audit_states.extend(
            [
                {
                    "cg_case": case,
                    "trim_success": True,
                    "valid_for_load_summary": True,
                    "event_slug": "autocross",
                    "speed_mps": 12.0,
                    "ax_mps2": 0.0,
                    "ay_mps2": 12.0,
                },
                {
                    "cg_case": case,
                    "trim_success": False,
                    "valid_for_load_summary": False,
                    "event_slug": "autocross",
                    "speed_mps": 14.0,
                    "ax_mps2": 0.0,
                    "ay_mps2": 14.0,
                },
            ]
        )
    pd.DataFrame(rows).to_csv(root / "event_results.csv", index=False)
    audit_root = root / "reduced_qss_audit"
    audit_root.mkdir()
    pd.DataFrame(audit_cases).to_csv(
        audit_root / "case_qss_audit_summary.csv", index=False
    )
    pd.DataFrame(audit_states).to_csv(
        audit_root / "sampled_trim_states.csv", index=False
    )
    (audit_root / "reduced_qss_audit.json").write_text(
        json.dumps(
            {
                "schema": "lapsims.reduced-qss-sampled-audit.v1",
                "model_dof": 6,
                "sweep_axis": "rear_static_weight_fraction",
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    tuning_path = root.parent / tuning_name
    tuning_path.write_text(
        json.dumps(
            {
                "sweep_axis": "rear_static_weight_fraction",
                "models": {"dyn_py_6dof_qss": {"cases": tuning}},
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    return tuning_path


def _write_fine_fixture(
    fine_root: Path,
    *,
    prior_study: Path,
    new_study: Path,
) -> None:
    source_events = {
        "prior": pd.read_csv(prior_study / "event_results.csv"),
        "new": pd.read_csv(new_study / "event_results.csv"),
    }
    cases = (
        ("prior", prior_study, 0.55),
        ("new", new_study, 0.56),
        ("new", new_study, 0.58),
    )
    for cohort, source_root, rear_fraction in cases:
        case_name = _case_name(rear_fraction)
        source_metadata = json.loads(
            (source_root / case_name / "case_metadata.json").read_text(
                encoding="utf-8"
            )
        )
        fingerprint = f"fine-fingerprint-{case_name}"
        source_metadata["input_fingerprint_sha256"] = fingerprint
        source_metadata["ggv_source_sha256"] = f"fine-ggv-{case_name}"
        source_metadata["ggv_speed_slices"] = [
            0.0,
            0.1,
            *[float(value) for value in range(1, 43)],
            42.053998,
        ]
        source_metadata["ggv_ax_search_points"] = 301
        source_metadata["ggv_ax_binary_iterations"] = 12
        target = fine_root / cohort / case_name
        target.mkdir(parents=True)
        (target / "case_metadata.json").write_text(
            json.dumps(source_metadata, indent=2) + "\n", encoding="utf-8"
        )
        raw_events: list[dict[str, object]] = []
        selected = source_events[cohort][
            source_events[cohort]["cg_case"] == case_name
        ]
        for row in selected.to_dict(orient="records"):
            event_slug = str(row["event_slug"])
            resolution_gain = {
                "acceleration": 0.002,
                "skidpad": 0.014,
                "autocross": 0.030,
                "michigan_endurance": 0.050,
            }[event_slug]
            row["lap_time_s"] = float(row["lap_time_s"]) - resolution_gain
            row["tire_mu_scale"] = DEFAULT_MU_SCALE
            row["ggv_source_sha256"] = f"fine-ggv-{case_name}"
            row["ggv_csv_speed_max_mps"] = 42.0
            raw_events.append(row)
        cache = {
            "schema": "lapsims.ggv-cg-case-cache.v1",
            "input_fingerprint_sha256": fingerprint,
            "case_metadata": source_metadata,
            "raw_event_results": raw_events,
        }
        (target / "raw_case_results.json").write_text(
            json.dumps(cache, indent=2) + "\n", encoding="utf-8"
        )


class LongitudinalCgPointsCorrelationTest(unittest.TestCase):
    def test_adds_strict_fine_grid_resolution_validation(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            new_study = root / "new_study"
            prior_study = root / "prior_study"
            fine_root = root / "fine_validation"
            output = root / "comparison_report"
            new_tuning = _write_fixture(
                new_study,
                rear_fractions=NEW_REAR_FRACTIONS,
                tuning_name="new_tuning.json",
            )
            prior_tuning = _write_fixture(
                prior_study,
                rear_fractions=REAR_FRACTIONS,
                tuning_name="prior_tuning.json",
            )
            _write_fine_fixture(
                fine_root, prior_study=prior_study, new_study=new_study
            )
            validation = build_report_bundle(
                study_root=new_study,
                output_root=output,
                tuning_json_path=new_tuning,
                comparison_study_root=prior_study,
                comparison_tuning_json_path=prior_tuning,
                fine_validation_root=fine_root,
                expected_rear_fractions=NEW_REAR_FRACTIONS,
                comparison_expected_rear_fractions=REAR_FRACTIONS,
            )

            fine = validation["fine_resolution_validation"]
            self.assertTrue(fine["passed"])
            self.assertEqual(fine["coarse_grid"]["nominal_speed_step_mps"], 2.0)
            self.assertEqual(fine["coarse_grid"]["ax_binary_iterations"], 10)
            self.assertEqual(fine["fine_grid"]["nominal_speed_step_mps"], 1.0)
            self.assertEqual(fine["fine_grid"]["ay_points"], 23)
            self.assertEqual(fine["fine_grid"]["ax_binary_iterations"], 12)
            self.assertEqual(fine["tire_mu_scale"], DEFAULT_MU_SCALE)
            self.assertTrue(fine["best_case_ranking_survives"])
            self.assertTrue(fine["rear_58_remains_best"])
            self.assertEqual(
                fine["requested_terminal_slice_empty_cases"],
                ["rear_55pct", "rear_56pct", "rear_58pct"],
            )
            self.assertFalse(fine["fine_reduced_qss_audit_present"])

            ranking = pd.read_csv(output / "coarse_vs_fine_case_ranking.csv")
            self.assertEqual(
                ranking.sort_values("fine_rank")["rear_static_weight_percent"]
                .round(8)
                .tolist(),
                [58.0, 56.0, 55.0],
            )
            self.assertEqual(
                len(pd.read_csv(output / "coarse_vs_fine_event_validation.csv")),
                12,
            )
            report = (output / "study_report.md").read_text(encoding="utf-8")
            self.assertIn("Fine-grid resolution validation", report)
            self.assertIn("58% rear case remains the best", report)
            self.assertIn("0.6225437130779028", report)
            self.assertNotIn("0.53602047", report)
            for artifact in (
                "coarse_vs_fine_case_ranking.csv",
                "coarse_vs_fine_event_validation.csv",
                "fine_resolution_validation.json",
                "fine_resolution_validation.png",
                "fine_validation_case_performance_and_points.csv",
                "fine_validation_event_optima.csv",
                "fine_validation_event_results.csv",
            ):
                self.assertTrue((output / artifact).is_file(), artifact)

            tampered_path = (
                fine_root / "new" / "rear_56pct" / "case_metadata.json"
            )
            tampered = json.loads(tampered_path.read_text(encoding="utf-8"))
            tampered["front_antiroll_stiffness_fraction"] += 0.01
            tampered_path.write_text(
                json.dumps(tampered, indent=2) + "\n", encoding="utf-8"
            )
            with self.assertRaisesRegex(ValueError, "controlled input"):
                build_report_bundle(
                    study_root=new_study,
                    output_root=root / "tampered_report",
                    tuning_json_path=new_tuning,
                    comparison_study_root=prior_study,
                    comparison_tuning_json_path=prior_tuning,
                    fine_validation_root=fine_root,
                    expected_rear_fractions=NEW_REAR_FRACTIONS,
                    comparison_expected_rear_fractions=REAR_FRACTIONS,
                )

    def test_builds_five_case_sweep_and_eight_case_common_field(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            new_study = root / "new_study"
            prior_study = root / "prior_study"
            output = root / "comparison_report"
            new_tuning = _write_fixture(
                new_study,
                rear_fractions=NEW_REAR_FRACTIONS,
                tuning_name="new_tuning.json",
            )
            prior_tuning = _write_fixture(
                prior_study,
                rear_fractions=REAR_FRACTIONS,
                tuning_name="prior_tuning.json",
            )
            validation = build_report_bundle(
                study_root=new_study,
                output_root=output,
                tuning_json_path=new_tuning,
                comparison_study_root=prior_study,
                comparison_tuning_json_path=prior_tuning,
                expected_rear_fractions=NEW_REAR_FRACTIONS,
                comparison_expected_rear_fractions=REAR_FRACTIONS,
            )

            self.assertEqual(validation["status"], "passed")
            self.assertEqual(validation["new_sweep_case_count"], 5)
            self.assertEqual(validation["common_field_case_count"], 8)
            self.assertEqual(validation["new_sweep_event_row_count"], 20)
            self.assertEqual(validation["common_field_event_row_count"], 32)
            self.assertTrue(
                validation["all_rear_fractions_retained_without_deduplication"]
            )
            self.assertEqual(
                validation["cross_study_validation"][
                    "rear_static_weight_percents"
                ],
                [45.0, 50.0, 52.0, 54.0, 55.0, 56.0, 58.0, 60.0],
            )
            self.assertEqual(
                validation["new_sweep_correlation"]["sample_count"], 5
            )
            self.assertEqual(
                validation["common_field_correlation"]["sample_count"], 8
            )
            self.assertFalse(
                validation["new_sweep_correlation"][
                    "quadratic_vertex_is_three_point_interpolation_only"
                ]
            )

            common = pd.read_csv(
                output / "common_8case_case_performance_and_points.csv"
            )
            self.assertEqual(len(common), 8)
            self.assertEqual(set(common["source_cohort"]), {"new_sweep", "prior_sweep"})
            self.assertEqual(
                sorted(common["rear_static_weight_percent"].round(8).tolist()),
                [45.0, 50.0, 52.0, 54.0, 55.0, 56.0, 58.0, 60.0],
            )
            new_only = pd.read_csv(
                output / "new_sweep_case_performance_and_points.csv"
            )
            self.assertEqual(len(new_only), 5)
            new_optima = pd.read_csv(output / "new_sweep_event_optima.csv")
            common_optima = pd.read_csv(output / "common_8case_event_optima.csv")
            self.assertEqual(set(new_optima["event_slug"]), set(EVENTS))
            self.assertEqual(set(common_optima["event_slug"]), set(EVENTS))
            self.assertEqual(
                common_optima.set_index("event_slug").loc[
                    "acceleration", "fastest_rear_weight_percents"
                ],
                "45,50",
            )
            deltas = pd.read_csv(
                output / "common_8case_event_points_delta_vs_50pct_rear.csv"
            )
            self.assertEqual(len(deltas), 32)
            self.assertEqual(set(deltas["event_slug"]), set(EVENTS))
            baseline = deltas[deltas["is_50pct_baseline_case"]]
            self.assertEqual(len(baseline), 4)
            self.assertTrue(
                (baseline["rear_static_weight_percent"] == 50.0).all()
            )
            self.assertTrue(
                (baseline["projected_points_delta_vs_50pct_rear"] == 0.0).all()
            )
            for event_slug in EVENTS:
                group = deltas[deltas["event_slug"] == event_slug]
                baseline_points = float(
                    group.loc[
                        group["rear_static_weight_percent"] == 50.0,
                        "common_projected_points",
                    ].iloc[0]
                )
                expected_delta = group["common_projected_points"] - baseline_points
                pd.testing.assert_series_equal(
                    group["projected_points_delta_vs_50pct_rear"].reset_index(
                        drop=True
                    ),
                    expected_delta.reset_index(drop=True),
                    check_names=False,
                )
            self.assertTrue(
                validation[
                    "event_points_delta_uses_exact_simulated_baseline_without_interpolation"
                ]
            )
            self.assertTrue(
                validation["event_points_delta_uses_final_common_scoring_field"]
            )
            report = (output / "study_report.md").read_text(encoding="utf-8")
            self.assertIn("five-case sweep and eight-case common field", report)
            self.assertIn("No case is deduplicated", report)
            self.assertIn("quadratic diagnostic only", report)
            self.assertIn("best observed", report)
            self.assertIn("52% to 60% rear", report)
            self.assertIn("P_e(r) - P_e(50%)", report)
            self.assertIn("final common eight-case", report)
            self.assertIn("not an interpolation", report)

            expected_files = {
                "common_8case_case_performance_and_points.csv",
                "common_8case_event_points_delta_vs_50pct_rear.csv",
                "common_8case_event_results.csv",
                "common_8case_event_optima.csv",
                "common_8case_event_sensitivity.csv",
                "event_points_delta_vs_50pct_rear_weight.png",
                "event_points_vs_rear_weight.png",
                "failed_qss_trim_reconstructions.csv",
                "new_sweep_case_performance_and_points.csv",
                "new_sweep_event_results.csv",
                "new_sweep_event_optima.csv",
                "new_sweep_event_sensitivity.csv",
                "raw_event_time_percent_change.png",
                "rear_weight_correlations.json",
                "rear_weight_to_points.png",
                "study_report.md",
                "tuning_vs_rear_weight.png",
                "validation_summary.json",
                "validity_and_tuning_by_case.csv",
            }
            self.assertEqual(expected_files, {path.name for path in output.iterdir()})

    def test_builds_complete_bundle_and_enforces_exact_tie_scoring(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            study = root / "study"
            output = root / "report"
            tuning = _write_fixture(study)
            validation = build_report_bundle(
                study_root=study,
                output_root=output,
                tuning_json_path=tuning,
            )

            self.assertEqual(validation["status"], "passed")
            self.assertEqual(validation["study_resolution"], "smoke")
            self.assertEqual(validation["case_count"], 3)
            self.assertEqual(validation["event_row_count"], 12)
            self.assertTrue(validation["only_exact_common_tmin_ties_receive_maximum"])
            self.assertTrue(validation["reduced_qss_audit"]["present"])
            self.assertEqual(validation["reduced_qss_audit"]["failed_trim_count"], 3)
            self.assertEqual(
                validation["design"]["rear_static_weight_percents"], [45.0, 50.0, 55.0]
            )

            events = pd.read_csv(output / "common_field_event_results.csv")
            acceleration = events[events["event_slug"] == "acceleration"].sort_values(
                "rear_static_weight_fraction"
            )
            self.assertTrue(acceleration.iloc[0]["common_is_event_fastest"])
            self.assertTrue(acceleration.iloc[1]["common_is_event_fastest"])
            self.assertFalse(acceleration.iloc[2]["common_is_event_fastest"])
            self.assertLess(
                acceleration.iloc[2]["common_projected_points"],
                acceleration.iloc[2]["common_maximum_points"],
            )
            self.assertEqual(
                validation["event_scoring"]["acceleration"][
                    "simulated_maximum_score_count"
                ],
                2,
            )

            fit = validation["rear_weight_correlation"]
            self.assertIn(
                "linear_slope_points_per_rear_weight_percentage_point", fit
            )
            self.assertTrue(
                fit["quadratic_vertex_is_three_point_interpolation_only"]
            )
            report = (output / "study_report.md").read_text(encoding="utf-8")
            self.assertIn("smoke-resolution result", report)
            self.assertIn("points per rear-weight percentage point", report)
            self.assertIn("Only simulations with a time exactly equal", report)
            self.assertIn("Per-case retuning", report)
            self.assertIn("RWD hypothesis", report)
            self.assertIn("Do not extrapolate", report)
            self.assertIn("0.296702 m = 11.681 in", report)
            self.assertIn("51.65037%", report)
            self.assertIn("not the exact nominal vehicle balance", report)
            self.assertIn("P_e(r) - P_e(50%)", report)
            deltas = pd.read_csv(output / "event_points_delta_vs_50pct_rear.csv")
            self.assertEqual(len(deltas), 12)
            baseline = deltas[deltas["is_50pct_baseline_case"]]
            self.assertEqual(len(baseline), 4)
            self.assertTrue(
                (baseline["projected_points_delta_vs_50pct_rear"] == 0.0).all()
            )

            expected_files = {
                "case_performance_and_points.csv",
                "common_field_event_results.csv",
                "event_points_delta_vs_50pct_rear.csv",
                "event_points_delta_vs_50pct_rear_weight.png",
                "event_points_vs_rear_weight.png",
                "event_sensitivity.csv",
                "failed_qss_trim_reconstructions.csv",
                "raw_event_time_percent_change.png",
                "rear_weight_correlation.json",
                "rear_weight_to_points.png",
                "study_report.md",
                "tuning_vs_rear_weight.png",
                "validation_summary.json",
                "validity_and_tuning_by_case.csv",
            }
            self.assertEqual(expected_files, {path.name for path in output.iterdir()})

    def test_rejects_disagreement_between_static_loads_and_declared_fraction(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            study = root / "study"
            tuning = _write_fixture(study, inconsistent_fraction=True)
            with self.assertRaisesRegex(ValueError, "rear static fraction sources disagree"):
                build_report_bundle(
                    study_root=study,
                    output_root=root / "report",
                    tuning_json_path=tuning,
                )


if __name__ == "__main__":
    unittest.main()
