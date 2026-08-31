"""Tests for the mixed-power longitudinal-CG report assembler."""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from build_longitudinal_cg_mixed_power_report import build_mixed_power_report

REAR_FRACTIONS = (0.45, 0.50, 0.52, 0.54, 0.55, 0.56, 0.58, 0.60)
EVENT_LENGTHS = {
    "acceleration": 75.0,
    "skidpad": 57.33406592801372,
    "autocross": 791.0,
    "michigan_endurance": 1069.968773,
}


def _case_name(rear_fraction: float) -> str:
    return f"rear_{rear_fraction * 100:.0f}pct"


def _metadata(
    case_name: str,
    rear_fraction: float,
    *,
    peak_power_w: float,
    ggv_hash: str,
) -> dict[str, object]:
    front_fraction = 1.0 - rear_fraction
    total_mass_kg = 261.07265114
    front_arb = 0.47 + 0.35 * rear_fraction
    wheelbase_m = 1.5494
    return {
        "model_family": "dyn_py_reduced_order_qss",
        "model_key": "dyn_py_6dof_qss",
        "model_dof": 6,
        "sweep_axis": "rear_static_weight_fraction",
        "input_fingerprint_sha256": f"fingerprint-{case_name}-{peak_power_w:g}",
        "ggv_source_sha256": ggv_hash,
        "source_peak_drive_power_w": 80000.0,
        "effective_drive_power_limit_w": peak_power_w,
        "cg_case": case_name,
        "cg_height_m": 0.2921,
        "cg_height_in": 11.5,
        "sprung_mass_kg": 230.72288408,
        "total_mass_kg": total_mass_kg,
        "rear_static_weight_fraction": rear_fraction,
        "front_static_weight_fraction": front_fraction,
        "front_antiroll_stiffness_fraction": front_arb,
        "front_elastic_roll_stiffness_fraction": 0.30 + 0.45 * rear_fraction,
        "effective_brake_distribution_front": 0.90 - 0.35 * rear_fraction,
        "effective_aero_balance_front": 0.5,
        "effective_cop_from_front_m": wheelbase_m / 2.0,
        "tire_mu_scale": 0.6225437130779028,
        "drive_distribution_front": 0.0,
        "limited_slip_differential_model": False,
        "ggv_ay_points": 23,
        "ggv_ax_search_points": 301,
        "ggv_ax_binary_iterations": 10,
        "vehicle": {"max_drive_power": peak_power_w},
        "reduced_parameter_overrides": {
            "source_peak_drive_power_w": 80000.0,
            "effective_peak_drive_power_w": peak_power_w,
            "effective_continuous_drive_power_w": peak_power_w,
        },
        "reduced_vehicle_parameters": {
            "mass_kg": total_mass_kg,
            "sprung_mass_kg": 230.72288408,
            "center_of_gravity_m": [-rear_fraction * wheelbase_m, 0.0, 0.2921],
            "inertia_kg_m2": [15.0, 75.0, 74.0],
            "sprung_inertia_kg_m2": [
                [14.55, 0.30, -5.37],
                [0.30, 73.84, -0.05],
                [-5.37, -0.05, 73.46],
            ],
            "corner_positions_m": [
                [0.75, 0.61, -0.29],
                [0.75, -0.61, -0.29],
                [-0.80, 0.59, -0.29],
                [-0.80, -0.59, -0.29],
            ],
            "static_wheel_loads_n": [
                total_mass_kg * 9.80665 * front_fraction / 2.0,
                total_mass_kg * 9.80665 * front_fraction / 2.0,
                total_mass_kg * 9.80665 * rear_fraction / 2.0,
                total_mass_kg * 9.80665 * rear_fraction / 2.0,
            ],
            "wheel_radius_m": [0.2032, 0.2032, 0.2032, 0.2032],
            "wheel_inertia_kg_m2": [0.31, 0.31, 0.31, 0.31],
            "unsprung_mass_kg": [7.5, 7.5, 7.7, 7.7],
            "suspension_stiffness_n_per_m": [26955.5, 26955.5, 27897.3, 27897.3],
            "suspension_damping_n_s_per_m": [1200.0, 1200.0, 1250.0, 1250.0],
            "antiroll_stiffness_nm_per_rad": [
                front_arb * 45547.94,
                (1.0 - front_arb) * 45547.94,
            ],
            "tire_vertical_stiffness_n_per_m": [90000.0] * 4,
            "tire_vertical_damping_n_s_per_m": [50.0] * 4,
            "rho_air_kg_m3": 1.225,
            "cl_area_m2": 2.186115,
            "cd_area_m2": 1.229539,
            "aero_balance_front": 0.5,
            "aero_cop_m": [wheelbase_m / 2.0, 0.0, 0.25],
            "aero_drag_application_m": [0.0, 0.0, 0.25],
            "peak_drive_power_w": peak_power_w,
            "continuous_drive_power_w": peak_power_w,
            "peak_drive_force_n": 3560.88019559902,
            "continuous_drive_force_n": 2104.1564792176,
            "maximum_drive_speed_mps": 42.0539983362,
            "drive_distribution_front": 0.0,
            "brake_distribution_front": 0.90 - 0.35 * rear_fraction,
        },
    }


def _base_time(event_slug: str, rear_percent: float) -> float:
    if event_slug == "acceleration":
        return 4.05 - 0.015 * (rear_percent - 45.0)
    if event_slug == "skidpad":
        return 5.00 + 0.0015 * (rear_percent - 51.0) ** 2
    if event_slug == "autocross":
        return 57.0 - 0.12 * (rear_percent - 45.0) + 0.01 * (rear_percent - 57.0) ** 2
    return 65.0 - 0.15 * (rear_percent - 45.0) + 0.01 * (rear_percent - 57.0) ** 2


def _write_fixture(root: Path) -> tuple[Path, Path]:
    base_root = root / "base_report"
    endurance_root = root / "endurance_32kw"
    base_root.mkdir()
    endurance_root.mkdir()
    base_rows: list[dict[str, object]] = []
    endurance_rows: list[dict[str, object]] = []
    metadata_rows: list[dict[str, object]] = []
    endurance_times = {
        45.0: 66.0,
        50.0: 64.0,
        52.0: 63.0,
        54.0: 62.4,
        55.0: 62.000000001,
        56.0: 62.0,
        58.0: 62.0,
        60.0: 62.2,
    }
    for index, rear_fraction in enumerate(REAR_FRACTIONS):
        rear_percent = 100.0 * rear_fraction
        case_name = _case_name(rear_fraction)
        source_cohort = (
            "prior_sweep" if rear_percent in {45.0, 50.0, 55.0} else "new_sweep"
        )
        base_case_root = root / "base_source" / case_name
        base_case_root.mkdir(parents=True)
        base_payload = _metadata(
            case_name,
            rear_fraction,
            peak_power_w=80000.0,
            ggv_hash=f"base-hash-{case_name}",
        )
        base_metadata_path = base_case_root / "case_metadata.json"
        base_metadata_path.write_text(
            json.dumps(base_payload, indent=2) + "\n", encoding="utf-8"
        )
        endurance_case_root = endurance_root / case_name
        endurance_case_root.mkdir()
        endurance_payload = _metadata(
            case_name,
            rear_fraction,
            peak_power_w=32000.0,
            ggv_hash=f"endurance-hash-{case_name}",
        )
        (endurance_case_root / "case_metadata.json").write_text(
            json.dumps(endurance_payload, indent=2) + "\n", encoding="utf-8"
        )
        metadata_rows.append(
            {
                "source_cohort": source_cohort,
                "source_study_root": str(root / "base_source"),
                "cg_case": case_name,
                "rear_static_weight_fraction": rear_fraction,
                "rear_static_weight_percent": rear_percent,
                "front_static_weight_fraction": 1.0 - rear_fraction,
                "longitudinal_cg_x_m": -rear_fraction * 1.5494,
                "cg_height_in": 11.5,
                "sprung_mass_kg": 230.72288408,
                "total_mass_kg": 261.07265114,
                "peak_drive_power_w": 80000.0,
                "case_metadata_path": str(base_metadata_path),
            }
        )
        for event_slug, track_length in EVENT_LENGTHS.items():
            base_rows.append(
                {
                    "model_key": "dyn_py_6dof_qss",
                    "source_cohort": source_cohort,
                    "source_study_root": str(root / "base_source"),
                    "cg_case": case_name,
                    "rear_static_weight_fraction": rear_fraction,
                    "rear_static_weight_percent": rear_percent,
                    "event_slug": event_slug,
                    "event_name": event_slug,
                    "lap_time_s": _base_time(event_slug, rear_percent),
                    "track_length_m": track_length,
                    "converged": True,
                    "all_track_solvers_converged": True,
                    "ggv_speed_cap_segments": 0,
                    "ggv_source_sha256": f"base-hash-{case_name}",
                }
            )
            endurance_rows.append(
                {
                    "model_key": "dyn_py_6dof_qss",
                    "cg_case": case_name,
                    "rear_static_weight_fraction": rear_fraction,
                    "event_slug": event_slug,
                    "event_name": event_slug,
                    "lap_time_s": (
                        endurance_times[round(rear_percent, 8)]
                        if event_slug == "michigan_endurance"
                        else 999.0 + index
                    ),
                    "track_length_m": track_length,
                    "converged": True,
                    "all_track_solvers_converged": True,
                    "ggv_speed_cap_segments": 0,
                    "ggv_source_sha256": f"endurance-hash-{case_name}",
                    "effective_drive_power_limit_w": 32000.0,
                }
            )
    pd.DataFrame(base_rows).to_csv(
        base_root / "common_8case_event_results.csv", index=False
    )
    pd.DataFrame(metadata_rows).to_csv(
        base_root / "validity_and_tuning_by_case.csv", index=False
    )
    (base_root / "validation_summary.json").write_text(
        json.dumps({"status": "passed", "common_field_case_count": 8}, indent=2) + "\n",
        encoding="utf-8",
    )
    pd.DataFrame(endurance_rows).to_csv(
        endurance_root / "event_results.csv", index=False
    )
    sweep_path = endurance_root / "sweep.json"
    sweep_definition = {
        "drive_power_limit_kw": 32.0,
        "power_limit_scope": (
            "GGV maps generated for endurance-only replacement; accepted 80 kW "
            "maps remain authoritative for acceleration, skidpad, and autocross"
        ),
    }
    sweep_path.write_text(
        json.dumps(sweep_definition, indent=2) + "\n", encoding="utf-8"
    )
    (endurance_root / "smoke_summary.json").write_text(
        json.dumps(
            {
                "sweep_json": str(sweep_path),
                "source_peak_drive_power_w": 80000.0,
                "effective_drive_power_limit_w": 32000.0,
                "drive_power_limit_source": "sweep_json",
                "sweep_definition": sweep_definition,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    return base_root, endurance_root


class LongitudinalCgMixedPowerReportTest(unittest.TestCase):
    def test_replaces_only_endurance_and_rescores_one_common_field(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            base_root, endurance_root = _write_fixture(root)
            output = root / "mixed_report"
            validation = build_mixed_power_report(
                base_report_root=base_root,
                endurance_study_root=endurance_root,
                output_root=output,
            )

            self.assertEqual(validation["status"], "passed")
            self.assertEqual(validation["case_count"], 8)
            self.assertEqual(validation["event_row_count"], 32)
            self.assertTrue(validation["only_endurance_rows_replaced"])
            self.assertTrue(validation["base_all_80kw_common_scoring_reproduced"])
            self.assertTrue(validation["all_80kw_sprint_times_and_distances_unchanged"])
            self.assertTrue(validation["all_80kw_sprint_projected_points_unchanged"])
            self.assertTrue(
                validation["all_endurance_times_and_distances_match_32kw_source"]
            )
            self.assertTrue(validation["only_exact_common_tmin_ties_receive_maximum"])
            self.assertTrue(validation["all_event_point_deltas_exactly_zero_at_50pct"])
            self.assertTrue(validation["total_points_delta_exactly_zero_at_50pct"])
            self.assertNotIn(
                "endpoint_change_points_55_minus_45", validation["correlation"]
            )
            events = pd.read_csv(output / "mixed_power_event_results.csv")
            base = pd.read_csv(base_root / "common_8case_event_results.csv")
            sprint = events[events["event_slug"] != "michigan_endurance"]
            sprint_base = base[base["event_slug"] != "michigan_endurance"]
            joined = sprint.merge(
                sprint_base[["cg_case", "event_slug", "lap_time_s"]],
                on=["cg_case", "event_slug"],
                suffixes=("_mixed", "_base"),
                validate="one_to_one",
            )
            self.assertTrue(
                (joined["lap_time_s_mixed"] == joined["lap_time_s_base"]).all()
            )
            self.assertEqual(set(sprint["mixed_power_power_limit_w"]), {80000.0})
            endurance = events[events["event_slug"] == "michigan_endurance"]
            self.assertEqual(set(endurance["mixed_power_power_limit_w"]), {32000.0})
            self.assertTrue(endurance["mixed_power_replaced_base_endurance"].all())
            winners = endurance[endurance["common_is_event_fastest"]]
            self.assertEqual(set(winners["cg_case"]), {"rear_56pct", "rear_58pct"})
            self.assertTrue(
                (
                    winners["common_projected_points"]
                    == winners["common_maximum_points"]
                ).all()
            )
            near = endurance[endurance["cg_case"] == "rear_55pct"].iloc[0]
            self.assertFalse(bool(near["common_is_event_fastest"]))
            self.assertLess(
                near["common_projected_points"], near["common_maximum_points"]
            )
            deltas = pd.read_csv(
                output / "mixed_power_event_points_delta_vs_50pct_rear.csv"
            )
            baseline = deltas[deltas["is_50pct_baseline_case"]]
            self.assertEqual(len(baseline), 4)
            self.assertTrue(
                (baseline["projected_points_delta_vs_50pct_rear"] == 0.0).all()
            )
            totals = pd.read_csv(output / "mixed_power_case_points.csv")
            total_baseline = totals[totals["rear_static_weight_percent"] == 50.0].iloc[
                0
            ]
            self.assertEqual(
                total_baseline["timed_event_points_delta_vs_50pct_rear"], 0.0
            )
            report = (output / "mixed_power_report.md").read_text(encoding="utf-8")
            self.assertIn("Only Michigan endurance is replaced", report)
            self.assertIn("explicitly ignores its", report)
            self.assertIn("P_e(r) - P_e(50%)", report)
            self.assertIn("exactly equal", report)
            self.assertIn("Mixed-field raw event times", report)
            self.assertIn("separately scored", report)
            self.assertIn("0.296702 m", report)
            self.assertIn("11.681 in", report)
            for artifact in (
                "mixed_power_case_points.csv",
                "mixed_power_endurance_replacement.csv",
                "mixed_power_event_points_delta_vs_50pct_rear.csv",
                "mixed_power_event_points_delta_vs_50pct_rear.png",
                "mixed_power_event_provenance.csv",
                "mixed_power_event_results.csv",
                "mixed_power_rear_weight_to_points.png",
                "mixed_power_report.md",
                "validation_summary.json",
            ):
                self.assertTrue((output / artifact).is_file(), artifact)

    def test_rejects_wrong_endurance_power(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            base_root, endurance_root = _write_fixture(root)
            path = endurance_root / "rear_54pct" / "case_metadata.json"
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["reduced_vehicle_parameters"]["peak_drive_power_w"] = 33000.0
            path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "33000 W, not 32000 W"):
                build_mixed_power_report(
                    base_report_root=base_root,
                    endurance_study_root=endurance_root,
                    output_root=root / "bad_report",
                )


if __name__ == "__main__":
    unittest.main()
