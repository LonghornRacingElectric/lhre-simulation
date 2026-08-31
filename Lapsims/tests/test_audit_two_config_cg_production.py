"""Unit tests for the independent two-config production acceptance gates."""

from __future__ import annotations

import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path

import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from audit_two_config_cg_production import (
    SCORE_RULES,
    independent_mixed_scoring,
    validate_design_inputs,
    validate_ggv_map,
    validate_lateral_endpoint_equality,
    validate_locked_manifest,
    validate_qss_audit,
    validate_run_contract,
)
from audit_two_config_qss_samples import (
    EXPECTED_MU_SCALE,
    EXPECTED_SPRUNG_MASS_KG,
    EXPECTED_TOTAL_MASS_KG,
)


def _sweep(power_kw: float) -> dict[str, object]:
    return {
        "sweep_axis": "configuration",
        "model_dof": 6,
        "reference_case": "config_nominal_54",
        "aero_balance_front": 0.5,
        "tire_mu_scale": EXPECTED_MU_SCALE,
        "drive_power_limit_kw": power_kw,
        "drive_model": "RWD with equal rear-wheel force split; no LSD model",
        "cases": [
            {
                "name": "config_nominal_54",
                "cg_height_in": 11.5,
                "rear_static_weight_fraction": 0.54,
                "front_static_weight_fraction": 0.46,
                "cg_x_m": -0.54 * 1.5494,
                "sprung_mass_kg": EXPECTED_SPRUNG_MASS_KG,
                "total_mass_kg": EXPECTED_TOTAL_MASS_KG,
                "front_antiroll_stiffness_fraction": 0.61,
                "brake_distribution_front": 0.70,
            },
            {
                "name": "config_plus0p25_56p5",
                "cg_height_in": 11.75,
                "rear_static_weight_fraction": 0.565,
                "front_static_weight_fraction": 0.435,
                "cg_x_m": -0.565 * 1.5494,
                "sprung_mass_kg": EXPECTED_SPRUNG_MASS_KG,
                "total_mass_kg": EXPECTED_TOTAL_MASS_KG,
                "front_antiroll_stiffness_fraction": 0.66,
                "brake_distribution_front": 0.68,
            },
        ],
    }


def _tuning_artifact() -> dict[str, object]:
    cases = []
    for name, height, rear, arb, brake in (
        ("config_nominal_54", 11.5, 0.54, 0.61, 0.70),
        ("config_plus0p25_56p5", 11.75, 0.565, 0.66, 0.68),
    ):
        cases.append(
            {
                "name": name,
                "model_dof": 6,
                "cg_height_in": height,
                "cg_height_m": height * 0.0254,
                "sprung_mass_kg": EXPECTED_SPRUNG_MASS_KG,
                "total_mass_kg": EXPECTED_TOTAL_MASS_KG,
                "rear_static_weight_fraction": rear,
                "front_static_weight_fraction": 1.0 - rear,
                "cg_x_m": -rear * 1.5494,
                "tire_mu_scale": EXPECTED_MU_SCALE,
                "front_antiroll_stiffness_fraction": arb,
                "brake_distribution_front": brake,
                "roll_optimization_at_search_bound": False,
                "brake_optimization_at_search_bound": False,
                "pure_lateral_limit_trim": {
                    "racing_feasible": True,
                    "solver_success": True,
                    "residual_norm": 1e-8,
                    "beta_rad": 0.1,
                    "steering_rad": 0.1,
                    "normal_loads_n": [300.0, 900.0, 400.0, 950.0],
                },
                "robustness_lateral_limit_g_by_speed_mps": {
                    "8.0": 1.3,
                    "11.8": 1.4,
                    "18.0": 1.45,
                    "25.0": 1.5,
                },
            }
        )
    return {
        "sweep_axis": "configuration",
        "aero_balance_front": 0.5,
        "tire": {"mu_scale": EXPECTED_MU_SCALE},
        "roll_tuning_method": {
            "endpoint_solver": "solve_lateral_limit descending interval scan",
            "cold_monotonic_binary_assumption": False,
        },
        "models": {"dyn_py_6dof_qss": {"model_dof": 6, "cases": cases}},
    }


def _run_contract() -> dict[str, object]:
    return validate_run_contract(
        {
            "schema": "lapsims.two-config-cg-production-run-contract.v1",
            "ggv": {
                "ay_points": 23,
                "ay_max_g": 2.2,
                "ax_search_points": 301,
                "ax_binary_iterations": 10,
                "top_speed_mps": 42.0539983361957,
                "speed_step_mps": 2.0,
                "qss_zero_proxy_mps": 0.1,
            },
            "sampled_qss_audit": {
                "maximum_binned_trace_samples_per_case": 36,
                "speed_bins": 6,
                "lateral_bins": 7,
                "longitudinal_bins": 4,
                "pure_lateral_speed_mps": 11.8,
                "trim_max_nfev": 220,
                "trim_tolerance": 1e-7,
            },
            "acceptance": {
                "maximum_final_speed_change_mps": 1e-6,
                "maximum_trim_residual_norm": 1e-4,
                "maximum_abs_beta_rad": 0.25,
                "maximum_abs_steering_rad": 0.5,
                "tir_valid_load_min_n": 100.0,
                "tir_valid_load_max_n": 1800.0,
                "normal_load_numerical_tolerance_n": 1e-5,
                "lateral_equality_tolerance_mps2": 1e-9,
            },
        }
    )


class DesignAndLockGateTest(unittest.TestCase):
    def test_exact_power_split_and_tuning_are_required(self) -> None:
        tuning = validate_design_inputs(
            _sweep(80.0), _sweep(32.0), _tuning_artifact()
        )
        self.assertAlmostEqual(
            tuning["config_plus0p25_56p5"]["front_antiroll_stiffness_fraction"],
            0.66,
        )
        changed = _sweep(32.0)
        changed["cases"][1]["brake_distribution_front"] = 0.69
        with self.assertRaises(ValueError):
            validate_design_inputs(_sweep(80.0), changed, _tuning_artifact())

    def test_locked_manifest_detects_path_hash_or_membership_drift(self) -> None:
        locked = {
            "lapsims:a": {"path": "C:/study/a", "sha256": "abc"},
            "bobsim:b": {"path": "C:/study/b", "sha256": "def"},
        }
        validate_locked_manifest(locked, copy.deepcopy(locked))
        changed = copy.deepcopy(locked)
        changed["lapsims:a"]["sha256"] = "changed"
        with self.assertRaisesRegex(ValueError, "drift"):
            validate_locked_manifest(locked, changed)
        with self.assertRaisesRegex(ValueError, "members"):
            validate_locked_manifest(locked, {"lapsims:a": locked["lapsims:a"]})


class MapTopologyTest(unittest.TestCase):
    def _map(self) -> pd.DataFrame:
        rows = []
        for speed in (0.0, 10.0):
            for ay in (-10.0, -5.0, 0.0, 5.0, 10.0):
                accel = 5.0 * (1.0 - abs(ay) / 12.0)
                brake = -8.0 * (1.0 - abs(ay) / 12.0)
                rows.append(
                    {
                        "speed_mps": speed,
                        "ay_mps2": ay,
                        "ax_accel_mps2": accel,
                        "ax_brake_mps2": brake,
                        "accel_feasible": 1,
                        "brake_feasible": 1,
                    }
                )
        return pd.DataFrame(rows)

    def test_map_topology_requires_symmetric_unique_feasible_branches(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "ggv.csv"
            frame = self._map()
            frame.to_csv(path, index=False)
            summary = validate_ggv_map(path, expected_speed_slices=[0.0, 10.0])
            self.assertTrue(summary["symmetric_topology"])
            self.assertEqual(summary["lateral_endpoints_mps2"]["10"], 10.0)

            asymmetric = frame.copy()
            asymmetric.loc[
                (asymmetric["speed_mps"] == 10.0)
                & (asymmetric["ay_mps2"] == 10.0),
                "ax_accel_mps2",
            ] += 0.1
            asymmetric.to_csv(path, index=False)
            with self.assertRaisesRegex(ValueError, "not laterally symmetric"):
                validate_ggv_map(path)

            duplicate = pd.concat((frame, frame.iloc[[0]]), ignore_index=True)
            duplicate.to_csv(path, index=False)
            with self.assertRaisesRegex(ValueError, "duplicate"):
                validate_ggv_map(path)

    def test_lateral_equality_uses_the_common_power_sustainable_domain(self) -> None:
        comparison = validate_lateral_endpoint_equality(
            {"0": 10.0, "10": 11.0, "20": 12.0},
            {"0": 10.0, "10": 11.0},
            case_name="config_nominal_54",
            tolerance_mps2=1e-9,
        )
        self.assertTrue(comparison["32kw_domain_is_subset_of_80kw_domain"])
        self.assertEqual(
            comparison["80kw_only_power_sustainable_speed_slices_mps"], [20.0]
        )
        with self.assertRaisesRegex(ValueError, "lateral boundary differs"):
            validate_lateral_endpoint_equality(
                {"0": 10.0},
                {"0": 9.9},
                case_name="config_nominal_54",
                tolerance_mps2=1e-9,
            )


class IndependentScoringTest(unittest.TestCase):
    def _events(self, power_w: float) -> pd.DataFrame:
        rows = []
        for case, delta in (
            ("config_nominal_54", 0.0),
            ("config_plus0p25_56p5", 0.1),
        ):
            for event, time, length in (
                ("acceleration", 4.0, 75.0),
                ("skidpad", 5.0, 57.0),
                ("autocross", 50.0, 791.0),
                ("michigan_endurance", 60.0, 1100.0),
            ):
                rows.append(
                    {
                        "cg_case": case,
                        "event_slug": event,
                        "lap_time_s": time + delta,
                        "track_length_m": length,
                        "effective_drive_power_limit_w": power_w,
                    }
                )
        return pd.DataFrame(rows)

    def test_mixed_selection_and_official_formula_are_independent(self) -> None:
        reference = {
            "events": {
                "acceleration": {
                    "simulation_time_conversion": "direct",
                    "valid_adjusted_times_s": [3.7, 4.2],
                },
                "skidpad": {
                    "simulation_time_conversion": "direct",
                    "valid_adjusted_times_s": [4.8, 5.5],
                },
                "autocross": {
                    "simulation_time_conversion": "direct",
                    "valid_adjusted_times_s": [44.0, 55.0],
                },
                "michigan_endurance": {
                    "simulation_time_conversion": "distance_scaled",
                    "official_event_distance_m": 22000.0,
                    "valid_adjusted_times_s": [1320.0, 1600.0],
                },
            }
        }
        mixed, totals = independent_mixed_scoring(
            self._events(80_000.0), self._events(32_000.0), reference
        )
        self.assertEqual(
            set(mixed.loc[mixed["event_slug"] != "michigan_endurance", "audit_source_power_kw"]),
            {80.0},
        )
        self.assertEqual(
            set(mixed.loc[mixed["event_slug"] == "michigan_endurance", "audit_source_power_kw"]),
            {32.0},
        )
        nominal_accel = mixed.loc[
            (mixed["cg_case"] == "config_nominal_54")
            & (mixed["event_slug"] == "acceleration")
        ].iloc[0]
        expected = SCORE_RULES["acceleration"].score(4.0, 3.7)
        self.assertAlmostEqual(nominal_accel["audit_projected_points"], expected)
        nominal_endurance = mixed.loc[
            (mixed["cg_case"] == "config_nominal_54")
            & (mixed["event_slug"] == "michigan_endurance")
        ].iloc[0]
        self.assertAlmostEqual(nominal_endurance["audit_competition_time_s"], 1200.0)
        self.assertAlmostEqual(nominal_endurance["audit_projected_points"], 275.0)
        self.assertEqual(len(totals), 2)


class QssAcceptanceTest(unittest.TestCase):
    def _write_qss_fixture(self, root: Path) -> Path:
        audit = root / "reduced_qss_audit"
        audit.mkdir(parents=True)
        rows = []
        for name, height, rear, arb, brake in (
            ("config_nominal_54", 11.5, 0.54, 0.61, 0.70),
            ("config_plus0p25_56p5", 11.75, 0.565, 0.66, 0.68),
        ):
            for event in (
                "acceleration",
                "skidpad",
                "autocross",
                "michigan_endurance",
                "GGV",
            ):
                rows.append(
                    {
                        "cg_case": name,
                        "sweep_axis": "configuration",
                        "model_dof": 6,
                        "event_slug": event,
                        "source_kind": (
                            "boundary_pure_lateral_11p8" if event == "GGV" else "trace_binned"
                        ),
                        "trim_success": True,
                        "trim_residual_norm": 1e-8,
                        "beta_rad": 0.1,
                        "steering_rad": 0.1,
                        "fz_fl_n": 300.0,
                        "fz_fr_n": 900.0,
                        "fz_rl_n": 350.0,
                        "fz_rr_n": 950.0,
                        "normal_load_min_n": 300.0,
                        "normal_load_max_n": 950.0,
                        "cg_height_in": height,
                        "sprung_mass_kg": EXPECTED_SPRUNG_MASS_KG,
                        "total_mass_kg": EXPECTED_TOTAL_MASS_KG,
                        "rear_static_weight_fraction": rear,
                        "front_static_weight_fraction": 1.0 - rear,
                        "front_antiroll_stiffness_fraction": arb,
                        "brake_distribution_front": brake,
                        "aero_balance_front": 0.5,
                        "tire_mu_scale": EXPECTED_MU_SCALE,
                        "tir_valid_load_min_n": 100.0,
                        "tir_valid_load_max_n": 1800.0,
                    }
                )
        sample_path = audit / "sampled_trim_states.csv"
        pd.DataFrame(rows).to_csv(sample_path, index=False)
        pd.DataFrame(
            {
                "cg_case": ["config_nominal_54", "config_plus0p25_56p5"],
                "sample_count": [5, 5],
                "successful_trim_count": [5, 5],
                "failed_trim_count": [0, 0],
                "maximum_abs_beta_rad": [0.1, 0.1],
                "maximum_abs_steering_rad": [0.1, 0.1],
                "maximum_residual_norm": [1e-8, 1e-8],
                "sampled_normal_load_min_n": [300.0, 300.0],
                "sampled_normal_load_max_n": [950.0, 950.0],
            }
        ).to_csv(audit / "case_qss_audit_summary.csv", index=False)
        (audit / "reduced_qss_audit.json").write_text(
            json.dumps(
                {
                    "schema": "lapsims.two-config-reduced-qss-sampled-audit.v1",
                    "sampling": _run_contract()["sampled_qss_audit"],
                    "case_summaries": [
                        {"cg_case": "config_nominal_54"},
                        {"cg_case": "config_plus0p25_56p5"},
                    ],
                }
            ),
            encoding="utf-8",
        )
        return sample_path

    def test_qss_acceptance_checks_event_coverage_and_tir_load_bounds(self) -> None:
        tuning = {
            "config_nominal_54": {
                "front_antiroll_stiffness_fraction": 0.61,
                "brake_distribution_front": 0.70,
            },
            "config_plus0p25_56p5": {
                "front_antiroll_stiffness_fraction": 0.66,
                "brake_distribution_front": 0.68,
            },
        }
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            sample_path = self._write_qss_fixture(root)
            result = validate_qss_audit(
                root, expected_tuning=tuning, run_contract=_run_contract()
            )
            self.assertTrue(result["all_samples_within_tir_fit_load_range"])

            frame = pd.read_csv(sample_path)
            frame.loc[0, "fz_fl_n"] = 99.0
            frame.to_csv(sample_path, index=False)
            with self.assertRaisesRegex(ValueError, "below the TIR"):
                validate_qss_audit(
                    root, expected_tuning=tuning, run_contract=_run_contract()
                )


if __name__ == "__main__":
    unittest.main()
