"""Focused tests for deterministic reduced-QSS audit sampling and summaries."""

from __future__ import annotations

import copy
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from audit_reduced_qss_sweep import (
    _longitudinal_cg_case_metadata,
    _mass_case_metadata,
    _validate_case_metadata,
    compare_resolution_outputs,
    select_trace_states,
    summarize_trim_rows,
    validate_completed_sweep_metadata,
)


class ReducedQssAuditSelectionTest(unittest.TestCase):
    def _trace_fixture(self) -> pd.DataFrame:
        count = 48
        speed = np.linspace(0.1, 40.0, count)
        ax = np.linspace(-12.0, 8.0, count)
        ay = 12.0 * np.sin(np.linspace(-np.pi, np.pi, count))
        lateral_utilization = np.clip(np.abs(ay) / 12.0, 0.0, 1.0)
        return pd.DataFrame(
            {
                "event_slug": np.where(
                    np.arange(count) % 2 == 0, "autocross", "michigan_endurance"
                ),
                "trace_row_index": np.arange(count),
                "longitudinal_accel_mps2": ax,
                "ggv_constraint_speed_mps": speed,
                "ggv_constraint_lateral_accel_mps2": ay,
                "ggv_accel_limit_mps2": np.full(count, 8.0),
                "ggv_brake_limit_mps2": np.full(count, -12.0),
                "ggv_lateral_utilization": lateral_utilization,
                "ggv_speed_domain_fraction": speed / 42.0539983361957,
            }
        )

    def test_selection_is_deterministic_and_retains_trace_extrema(self) -> None:
        frame = self._trace_fixture()
        kwargs = {
            "maximum_samples": 12,
            "speed_bins": 4,
            "lateral_bins": 5,
            "longitudinal_bins": 3,
        }
        first = select_trace_states(frame, **kwargs)
        second = select_trace_states(frame, **kwargs)
        self.assertEqual(first, second)
        reasons = {
            reason
            for state in first
            for reason in state["selection_reasons"]
        }
        self.assertIn("trace_strongest_accel", reasons)
        self.assertIn("trace_strongest_brake", reasons)
        self.assertIn("trace_highest_lateral_utilization", reasons)
        self.assertIn("trace_highest_speed", reasons)
        self.assertEqual({state["branch"] for state in first}, {"accel", "brake"})
        self.assertLessEqual(len(first), kwargs["maximum_samples"] + 4)

    def test_exact_zero_speed_trace_state_is_excluded_from_qss(self) -> None:
        frame = self._trace_fixture()
        frame.loc[0, "ggv_constraint_speed_mps"] = 0.0
        selected = select_trace_states(
            frame,
            maximum_samples=12,
            speed_bins=4,
            lateral_bins=5,
            longitudinal_bins=3,
        )
        self.assertTrue(all(state["speed_mps"] >= 0.1 for state in selected))


class ReducedQssAuditMetadataTest(unittest.TestCase):
    def _metadata(self, tire_mu_scale: object) -> dict[str, object]:
        return {
            "cg_case": "cg_11p6in",
            "model_dof": 6,
            "model_family": "dyn_py_reduced_order_qss",
            "tire_mu_scale": tire_mu_scale,
            "effective_aero_balance_front": 0.5,
            "front_antiroll_stiffness_fraction": 0.55,
        }

    def test_accepts_raw_and_scaled_positive_tire_mu_metadata(self) -> None:
        self.assertEqual(
            _validate_case_metadata(self._metadata(1.0), model_dof=6),
            1.0,
        )
        scale = 0.6225437130779028
        self.assertEqual(
            _validate_case_metadata(self._metadata(scale), model_dof=6),
            scale,
        )

    def test_rejects_missing_nonfinite_or_nonpositive_tire_mu_metadata(self) -> None:
        for invalid in (None, True, "1.0", 0.0, -0.1, float("nan"), float("inf")):
            with self.subTest(invalid=invalid), self.assertRaisesRegex(
                ValueError,
                "finite positive tire_mu_scale",
            ):
                _validate_case_metadata(self._metadata(invalid), model_dof=6)

    def _mass_metadata(self, offset_lb: float) -> dict[str, object]:
        fixed_unsprung = 30.34976706
        nominal_sprung = 230.72288408
        sprung = nominal_sprung + offset_lb * 0.45359237
        total = sprung + fixed_unsprung
        return {
            **self._metadata(0.6225437130779028),
            "sweep_axis": "sprung_mass_kg",
            "cg_case": f"mass_{offset_lb:+.0f}lb",
            "cg_height_in": 11.5,
            "sprung_mass_kg": sprung,
            "total_mass_kg": total,
            "mass_offset_lb": offset_lb,
            "vehicle": {"mass": total},
            "reduced_parameter_overrides": {
                "sprung_mass_kg": sprung,
                "total_mass_kg": total,
                "fixed_unsprung_mass_kg": fixed_unsprung,
            },
            "reduced_vehicle_parameters": {
                "mass_kg": total,
                "sprung_mass_kg": sprung,
                "unsprung_mass_kg": [
                    7.82268281,
                    7.82268281,
                    7.35220072,
                    7.35220072,
                ],
            },
        }

    def test_mass_metadata_requires_consistent_explicit_mass_identity(self) -> None:
        metadata = self._mass_metadata(50.0)
        mass = _mass_case_metadata(metadata)
        self.assertIsNotNone(mass)
        assert mass is not None
        self.assertAlmostEqual(mass["mass_offset_lb"], 50.0)
        self.assertEqual(
            _validate_case_metadata(metadata, model_dof=6),
            0.6225437130779028,
        )

        metadata["total_mass_kg"] = float(metadata["total_mass_kg"]) + 1.0
        with self.assertRaisesRegex(ValueError, "total mass"):
            _mass_case_metadata(metadata)

    def test_completed_mass_sweep_requires_five_fixed_height_offsets(self) -> None:
        metadata = [
            self._mass_metadata(offset)
            for offset in (-100.0, -50.0, 0.0, 50.0, 100.0)
        ]
        result = validate_completed_sweep_metadata(metadata)
        self.assertEqual(result["sweep_axis"], "sprung_mass_kg")
        self.assertEqual(result["mass_sweep_validation"]["case_count"], 5)
        self.assertAlmostEqual(
            result["mass_sweep_validation"]["fixed_cg_height_in"], 11.5
        )

        metadata[-1]["mass_offset_lb"] = 90.0
        with self.assertRaisesRegex(ValueError, "offsets must be exactly"):
            validate_completed_sweep_metadata(metadata)

    def _longitudinal_metadata(
        self,
        rear_fraction: float,
        *,
        reference_fraction: float = 0.5,
    ) -> dict[str, object]:
        total_mass = 261.07265114
        sprung_mass = 230.72288408
        fixed_unsprung = total_mass - sprung_mass
        wheelbase = 1.5494
        cg_x = -rear_fraction * wheelbase
        front_fraction = 1.0 - rear_fraction
        cg_z = 11.5 * 0.0254
        static_loads = [
            0.5 * front_fraction * total_mass * 9.80665,
            0.5 * front_fraction * total_mass * 9.80665,
            0.5 * rear_fraction * total_mass * 9.80665,
            0.5 * rear_fraction * total_mass * 9.80665,
        ]
        return {
            **self._metadata(0.6225437130779028),
            "sweep_axis": "rear_static_weight_fraction",
            "cg_case": f"rear_{100.0 * rear_fraction:.0f}pct",
            "cg_height_m": cg_z,
            "cg_height_in": 11.5,
            "sprung_mass_kg": sprung_mass,
            "total_mass_kg": total_mass,
            "rear_static_weight_fraction": rear_fraction,
            "front_static_weight_fraction": front_fraction,
            "is_reference_case": rear_fraction == reference_fraction,
            "drive_distribution_front": 0.0,
            "limited_slip_differential_model": False,
            "vehicle": {
                "mass": total_mass,
                "front_static_frac": front_fraction,
                "aero_balance_front": 0.5,
                "drive_distribution_front": 0.0,
            },
            "reduced_parameter_overrides": {
                "sprung_mass_kg": sprung_mass,
                "total_mass_kg": total_mass,
                "fixed_unsprung_mass_kg": fixed_unsprung,
                "rear_static_weight_fraction": rear_fraction,
                "front_static_weight_fraction": front_fraction,
                "center_of_gravity_m": [cg_x, 0.0, cg_z],
                "aero_balance_front": 0.5,
            },
            "reduced_vehicle_parameters": {
                "mass_kg": total_mass,
                "sprung_mass_kg": sprung_mass,
                "unsprung_mass_kg": [
                    7.82268281,
                    7.82268281,
                    7.35220072,
                    7.35220072,
                ],
                "center_of_gravity_m": [cg_x, 0.0, cg_z],
                "corner_positions_m": [
                    [-cg_x, 0.6, -0.296702],
                    [-cg_x, -0.6, -0.296702],
                    [-wheelbase - cg_x, 0.6, -0.296702],
                    [-wheelbase - cg_x, -0.6, -0.296702],
                ],
                "static_wheel_loads_n": static_loads,
                "aero_balance_front": 0.5,
                "aero_cop_m": [-0.5 * wheelbase - cg_x, 0.0, -cg_z],
                "drive_distribution_front": 0.0,
            },
        }

    def test_longitudinal_cg_metadata_requires_exact_synchronized_design(self) -> None:
        metadata_rows = [
            self._longitudinal_metadata(fraction)
            for fraction in (0.45, 0.50, 0.55)
        ]
        for metadata in metadata_rows:
            validated = _longitudinal_cg_case_metadata(metadata)
            self.assertIsNotNone(validated)
            self.assertEqual(
                _validate_case_metadata(metadata, model_dof=6),
                0.6225437130779028,
            )
        result = validate_completed_sweep_metadata(metadata_rows)
        self.assertEqual(result["sweep_axis"], "rear_static_weight_fraction")
        summary = result["longitudinal_cg_sweep_validation"]
        self.assertEqual(summary["rear_static_weight_fractions"], [0.45, 0.5, 0.55])
        for actual, expected in zip(
            summary["cg_x_m"], (-0.69723, -0.7747, -0.85217)
        ):
            self.assertAlmostEqual(actual, expected)
        declared_result = validate_completed_sweep_metadata(
            metadata_rows,
            sweep_definition={
                "sweep_axis": "rear_static_weight_fraction",
                "reference_case": "rear_50pct",
                "cases": [
                    {
                        "name": f"rear_{100.0 * fraction:.0f}pct",
                        "rear_static_weight_fraction": fraction,
                    }
                    for fraction in (0.45, 0.50, 0.55)
                ],
            },
        )
        self.assertEqual(
            declared_result["longitudinal_cg_sweep_validation"][
                "declared_fraction_validation_source"
            ],
            "sweep_json",
        )

    def test_longitudinal_cg_metadata_matches_declared_five_case_grid(self) -> None:
        fractions = (0.52, 0.54, 0.56, 0.58, 0.60)
        metadata_rows = [
            self._longitudinal_metadata(
                fraction,
                reference_fraction=0.52,
            )
            for fraction in fractions
        ]
        definition = {
            "sweep_axis": "rear_static_weight_fraction",
            "reference_case": "rear_52pct",
            "cases": [
                {
                    "name": f"rear_{100.0 * fraction:.0f}pct",
                    "rear_static_weight_fraction": fraction,
                }
                for fraction in fractions
            ],
        }

        result = validate_completed_sweep_metadata(
            metadata_rows,
            sweep_definition=definition,
        )
        summary = result["longitudinal_cg_sweep_validation"]
        self.assertEqual(summary["rear_static_weight_fractions"], list(fractions))
        self.assertAlmostEqual(summary["rear_static_weight_fraction_step"], 0.02)
        self.assertEqual(summary["reference_case"], "rear_52pct")
        self.assertEqual(summary["declared_fraction_validation_source"], "sweep_json")
        for actual, expected in zip(
            summary["cg_x_m"],
            (-0.805688, -0.836676, -0.867664, -0.898652, -0.92964),
        ):
            self.assertAlmostEqual(actual, expected)

        definition["cases"][-2]["rear_static_weight_fraction"] = 0.60
        definition["cases"][-1]["rear_static_weight_fraction"] = 0.58
        with self.assertRaisesRegex(ValueError, "does not match the sweep definition"):
            validate_completed_sweep_metadata(
                metadata_rows,
                sweep_definition=definition,
            )

    def test_longitudinal_cg_metadata_rejects_desynchronized_blocks(self) -> None:
        base = self._longitudinal_metadata(0.55)
        mutations = (
            ("outer static fraction", lambda row: row["vehicle"].update({"front_static_frac": 0.5})),
            (
                "nested CG x",
                lambda row: row["reduced_vehicle_parameters"][
                    "center_of_gravity_m"
                ].__setitem__(0, -0.8),
            ),
            (
                "nested static loads",
                lambda row: row["reduced_vehicle_parameters"][
                    "static_wheel_loads_n"
                ].__setitem__(0, 1.0),
            ),
        )
        for label, mutate in mutations:
            with self.subTest(label=label):
                metadata = copy.deepcopy(base)
                mutate(metadata)
                with self.assertRaises(ValueError):
                    _longitudinal_cg_case_metadata(metadata)

    def test_longitudinal_cg_audit_requires_scaled_6dof_rwd_without_lsd(self) -> None:
        mutations = (
            (
                "model",
                lambda row: row.update({"model_dof": 3}),
                {"model_dof": 3},
                "6DOF",
            ),
            (
                "mu scale",
                lambda row: row.update({"tire_mu_scale": 1.0}),
                {"model_dof": 6},
                "calibrated mu scale",
            ),
            (
                "drive",
                lambda row: row.update({"drive_distribution_front": 0.5}),
                {"model_dof": 6},
                "rear-wheel drive",
            ),
            (
                "LSD",
                lambda row: row.update({"limited_slip_differential_model": True}),
                {"model_dof": 6},
                "no LSD",
            ),
        )
        for label, mutate, kwargs, message in mutations:
            with self.subTest(label=label):
                metadata = self._longitudinal_metadata(0.5)
                mutate(metadata)
                with self.assertRaisesRegex(ValueError, message):
                    _validate_case_metadata(metadata, **kwargs)

    def test_legacy_cg_metadata_does_not_require_mass_fields(self) -> None:
        self.assertIsNone(_mass_case_metadata(self._metadata(1.0)))


class ReducedQssAuditSummaryTest(unittest.TestCase):
    def test_summary_counts_sampled_wheel_thresholds_and_ignores_failed_loads(self) -> None:
        rows = [
            {
                "trim_success": True,
                "valid_for_load_summary": True,
                "trim_residual_norm": 0.01,
                "beta_rad": 0.02,
                "steering_rad": 0.1,
                "normal_load_min_n": 80.0,
                "normal_load_max_n": 1810.0,
                "wheel_observations_below_100n": 1,
                "wheel_observations_above_1800n": 1,
                "wheel_lift": False,
                "heave_m": 0.001,
                "roll_rad": -0.02,
                "pitch_rad": 0.01,
            },
            {
                "trim_success": False,
                "valid_for_load_summary": False,
                "trim_residual_norm": 2.0,
                "beta_rad": -0.03,
                "steering_rad": -0.2,
                "normal_load_min_n": 0.0,
                "normal_load_max_n": 2500.0,
                "wheel_observations_below_100n": 2,
                "wheel_observations_above_1800n": 2,
                "wheel_lift": True,
                "heave_m": 0.003,
                "roll_rad": 0.04,
                "pitch_rad": -0.02,
            },
        ]
        summary = summarize_trim_rows(rows)
        self.assertEqual(summary["sample_count"], 2)
        self.assertEqual(summary["successful_trim_count"], 1)
        self.assertEqual(summary["failed_trim_count"], 1)
        self.assertEqual(summary["sampled_state_count_below_100n"], 1)
        self.assertEqual(summary["sampled_wheel_observations_below_100n"], 1)
        self.assertEqual(summary["sampled_wheel_lift_state_count"], 0)
        self.assertAlmostEqual(summary["sampled_normal_load_min_n"], 80.0)
        self.assertAlmostEqual(summary["sampled_normal_load_max_n"], 1810.0)
        self.assertAlmostEqual(summary["sampled_roll_rad_min"], -0.02)
        self.assertAlmostEqual(summary["sampled_roll_rad_max"], -0.02)

    def test_resolution_comparison_joins_stable_model_case_event_keys(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            primary = root / "primary"
            comparison = root / "comparison"
            primary.mkdir()
            comparison.mkdir()
            base = pd.DataFrame(
                {
                    "model_key": ["dyn_py_3dof_qss"],
                    "cg_case": ["cg_8p0in"],
                    "event_slug": ["autocross"],
                    "lap_time_s": [50.0],
                    "projected_points": [100.0],
                }
            )
            base.to_csv(primary / "event_results.csv", index=False)
            changed = base.copy()
            changed["lap_time_s"] = 50.5
            changed["projected_points"] = 99.0
            changed.to_csv(comparison / "event_results.csv", index=False)

            result = compare_resolution_outputs(primary, comparison)
            self.assertEqual(len(result), 1)
            self.assertAlmostEqual(
                result.loc[0, "primary_minus_comparison_time_s"], -0.5
            )
            self.assertAlmostEqual(
                result.loc[0, "primary_minus_comparison_points"], 1.0
            )


if __name__ == "__main__":
    unittest.main()
