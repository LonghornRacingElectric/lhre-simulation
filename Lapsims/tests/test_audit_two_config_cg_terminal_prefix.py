"""Focused tests for the read-only 32 kW terminal-prefix sidecar."""

from __future__ import annotations

import hashlib
import json
import sys
import tempfile
import unittest
from pathlib import Path

import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from audit_two_config_cg_terminal_prefix import (
    EVENTS,
    inspect_qss_strict_gates,
    validate_event_domains,
    validate_physical_speed_prefix,
    validate_serialized_map,
    validate_terminal_rpm_speed_prefix,
)


def _metadata() -> dict[str, object]:
    return {
        "effective_drive_power_limit_w": 32_000.0,
        "vehicle": {
            "max_drive_power": 32_000.0,
            "max_drive_speed": 42.0539983361957,
            "rho": 1.225,
            "cd_a": 1.2295390217039197,
        },
        "ggv_requested_speed_slices_mps": [0.0, 2.0, 34.0, 36.0],
        "ggv_speed_slices": [0.0, 0.1, 2.0, 34.0, 36.0],
        "qss_zero_speed_proxy_mps": 0.1,
    }


def _metadata_80kw() -> dict[str, object]:
    metadata = _metadata()
    metadata["effective_drive_power_limit_w"] = 80_000.0
    vehicle = dict(metadata["vehicle"])
    vehicle["max_drive_power"] = 80_000.0
    metadata["vehicle"] = vehicle
    metadata["ggv_requested_speed_slices_mps"] = [0.0, 2.0, 40.0, 42.0539983361957]
    metadata["ggv_speed_slices"] = [0.0, 0.1, 2.0, 40.0, 42.0539983361957]
    return metadata


def _map_frame(*, three_row_origin: bool = False) -> pd.DataFrame:
    rows: list[dict[str, float]] = []
    for speed in (0.0, 0.1, 2.0, 34.0):
        ay_values = (-0.35, 0.0, 0.35) if three_row_origin and speed < 1.0 else (
            -2.0,
            -1.0,
            0.0,
            1.0,
            2.0,
        )
        for ay in ay_values:
            accel = (
                2.0
                if ay == 0.0
                else (0.0 if abs(ay) == max(abs(value) for value in ay_values) else 1.0)
            )
            rows.append(
                {
                    "speed_mps": speed,
                    "ay_mps2": ay,
                    "ax_accel_mps2": accel,
                    "ax_brake_mps2": -2.0,
                    "accel_feasible": 1.0,
                    "brake_feasible": 1.0,
                }
            )
    return pd.DataFrame(rows)


class PhysicalPrefixTests(unittest.TestCase):
    def test_accepts_exact_contiguous_prefix_through_terminal(self) -> None:
        result = validate_physical_speed_prefix(_map_frame(), _metadata())
        self.assertAlmostEqual(
            result["drag_power_terminal_speed_mps"], 34.89532034807606
        )
        self.assertEqual(result["serialized_speed_slices_mps"], [0.0, 0.1, 2.0, 34.0])
        self.assertEqual(result["omitted_above_terminal_speed_slices_mps"], [36.0])
        self.assertGreater(result["last_included_drive_minus_drag_force_n"], 0.0)
        self.assertLess(result["first_omitted_drive_minus_drag_force_n"], 0.0)

    def test_rejects_an_interior_missing_slice(self) -> None:
        frame = _map_frame()
        frame = frame[frame["speed_mps"] != 2.0]
        with self.assertRaisesRegex(ValueError, "exact contiguous physical prefix"):
            validate_physical_speed_prefix(frame, _metadata())

    def test_rejects_a_serialized_row_above_terminal(self) -> None:
        frame = pd.concat(
            [_map_frame(), _map_frame().assign(speed_mps=36.0)],
            ignore_index=True,
        )
        with self.assertRaisesRegex(ValueError, "exact contiguous physical prefix"):
            validate_physical_speed_prefix(frame, _metadata())

    def test_accepts_only_one_empty_80kw_terminal_rpm_slice(self) -> None:
        frame = _map_frame()
        frame.loc[frame["speed_mps"] == 34.0, "speed_mps"] = 40.0
        result = validate_terminal_rpm_speed_prefix(frame, _metadata_80kw())
        self.assertEqual(result["terminal_kind"], "terminal_rpm")
        self.assertEqual(
            result["omitted_terminal_rpm_speed_slices_mps"],
            [42.0539983361957],
        )


class OriginTopologyTests(unittest.TestCase):
    def test_accepts_exact_three_row_zero_proxy_closure(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "ggv.csv"
            _map_frame(three_row_origin=True).to_csv(path, index=False)
            result = validate_serialized_map(path, _metadata())
        self.assertTrue(result["three_row_origin_exception_used"])
        self.assertEqual(result["three_row_origin_speeds_mps"], [0.0, 0.1])
        self.assertTrue(result["branch_masks_match_finiteness"])

    def test_rejects_three_row_origin_without_zero_ax_closure(self) -> None:
        frame = _map_frame(three_row_origin=True)
        mask = (frame["speed_mps"] < 1.0) & (frame["ay_mps2"].abs() > 0.0)
        frame.loc[mask, "ax_accel_mps2"] = 0.1
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "ggv.csv"
            frame.to_csv(path, index=False)
            with self.assertRaisesRegex(ValueError, "close both drive branches"):
                validate_serialized_map(path, _metadata())

    def test_rejects_three_row_origin_with_false_finite_branch_mask(self) -> None:
        frame = _map_frame(three_row_origin=True)
        frame.loc[
            (frame["speed_mps"] == 0.1) & (frame["ay_mps2"] > 0.0),
            "accel_feasible",
        ] = 0.0
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "ggv.csv"
            frame.to_csv(path, index=False)
            with self.assertRaisesRegex(ValueError, "disagrees with ax_accel"):
                validate_serialized_map(path, _metadata())


class EventDomainTests(unittest.TestCase):
    def _write_event_fixture(self, case_dir: Path) -> dict[str, object]:
        map_path = case_dir / "ggv.csv"
        map_path.write_text("immutable-map-fixture\n", encoding="utf-8")
        map_hash = hashlib.sha256(map_path.read_bytes()).hexdigest()
        speeds = [1.0, 20.0, 29.0]
        domains = [value / 34.0 for value in speeds]
        for event in EVENTS:
            trace_path = case_dir / f"{event}_trace.csv"
            pd.DataFrame(
                {
                    "speed_mps": speeds,
                    "ggv_constraint_speed_mps": speeds,
                    "ggv_speed_domain_fraction": domains,
                }
            ).to_csv(trace_path, index=False)
            summary = {
                "converged": True,
                "final_max_speed_change_mps": 0.0,
                "ggv_speed_cap_segments": 0,
                "ggv_source_sha256": map_hash,
                "ggv_source_csv": str(map_path.resolve()),
                "effective_drive_power_limit_w": 32_000.0,
                "ggv_csv_speed_max_mps": 34.0,
                "ggv_solver_speed_cap_mps": 34.0,
                "maximum_speed_mps": 29.0,
                "ggv_max_speed_domain_fraction": 29.0 / 34.0,
            }
            (case_dir / f"{event}_summary.json").write_text(
                json.dumps(summary), encoding="utf-8"
            )
        return {
            "path": str(map_path.resolve()),
            "sha256": map_hash,
            "physical_speed_prefix": {
                "last_included_speed_mps": 34.0,
                "drag_power_terminal_speed_mps": 34.89532034807606,
                "physical_terminal_speed_mps": 34.89532034807606,
            },
        }

    def test_accepts_events_strictly_inside_actual_domain(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            case_dir = Path(temporary)
            summary = self._write_event_fixture(case_dir)
            result = validate_event_domains(
                case_dir,
                map_summary=summary,
                maximum_final_speed_change_mps=1e-6,
            )
        self.assertEqual(len(result), 4)
        self.assertTrue(all(row["ggv_speed_cap_segments"] == 0 for row in result))

    def test_rejects_any_event_cap_contact(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            case_dir = Path(temporary)
            summary = self._write_event_fixture(case_dir)
            path = case_dir / "autocross_summary.json"
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["ggv_speed_cap_segments"] = 1
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "touched the GGV speed cap"):
                validate_event_domains(
                    case_dir,
                    map_summary=summary,
                    maximum_final_speed_change_mps=1e-6,
                )


class QssGateReportingTests(unittest.TestCase):
    def test_reports_frozen_qss_failures_without_relaxing_thresholds(self) -> None:
        columns = {
            "source_kind": "trace_binned",
            "selection_reasons": "event_coverage",
            "event_slug": "skidpad",
            "trace_row_index": 0,
            "speed_mps": 11.25,
            "ax_mps2": 0.0,
            "ay_mps2": 13.8,
            "beta_rad": 0.01,
            "steering_rad": 0.02,
            "fz_fr_n": 1000.0,
            "fz_rl_n": 350.0,
            "fz_rr_n": 1100.0,
            "tir_valid_load_min_n": 100.0,
            "tir_valid_load_max_n": 1800.0,
        }
        rows = [
            {
                **columns,
                "sample_index": 0,
                "cg_case": "config_nominal_54",
                "trim_success": True,
                "trim_residual_norm": 1e-8,
                "fz_fl_n": 100.0,
            },
            {
                **columns,
                "sample_index": 1,
                "cg_case": "config_nominal_54",
                "trim_success": True,
                "trim_residual_norm": 2e-8,
                "fz_fl_n": 101.0,
            },
            {
                **columns,
                "sample_index": 2,
                "cg_case": "config_plus0p25_56p5",
                "trim_success": True,
                "trim_residual_norm": 3e-8,
                "fz_fl_n": 99.5,
            },
            {
                **columns,
                "sample_index": 3,
                "cg_case": "config_plus0p25_56p5",
                "event_slug": "autocross",
                "trace_row_index": 260,
                "trim_success": False,
                "trim_residual_norm": 0.0014,
                "fz_fl_n": 108.0,
            },
        ]
        summary = pd.DataFrame(
            [
                {
                    "cg_case": "config_nominal_54",
                    "sample_count": 2,
                    "successful_trim_count": 2,
                    "failed_trim_count": 0,
                    "sampled_normal_load_min_n": 100.0,
                    "maximum_residual_norm": 2e-8,
                },
                {
                    "cg_case": "config_plus0p25_56p5",
                    "sample_count": 2,
                    "successful_trim_count": 1,
                    "failed_trim_count": 1,
                    "sampled_normal_load_min_n": 99.5,
                    "maximum_residual_norm": 0.0014,
                },
            ]
        )
        run_contract = {
            "acceptance": {
                "maximum_trim_residual_norm": 1e-4,
                "maximum_abs_beta_rad": 0.25,
                "maximum_abs_steering_rad": 0.5,
                "tir_valid_load_min_n": 100.0,
                "tir_valid_load_max_n": 1800.0,
                "normal_load_numerical_tolerance_n": 1e-5,
            }
        }
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            audit_dir = root / "reduced_qss_audit"
            audit_dir.mkdir()
            pd.DataFrame(rows).to_csv(
                audit_dir / "sampled_trim_states.csv", index=False
            )
            summary.to_csv(audit_dir / "case_qss_audit_summary.csv", index=False)
            (audit_dir / "reduced_qss_audit.json").write_text(
                "{}\n", encoding="utf-8"
            )
            result = inspect_qss_strict_gates(root, run_contract=run_contract)

        self.assertEqual(result["strict_status"], "failed")
        self.assertFalse(result["strict_checks"]["all_trims_successful"])
        self.assertFalse(
            result["strict_checks"]["all_residuals_within_frozen_limit"]
        )
        self.assertFalse(
            result["strict_checks"]["all_successful_loads_within_frozen_tir_range"]
        )
        self.assertEqual(result["frozen_thresholds"]["tir_valid_load_min_n"], 100.0)
        self.assertEqual(
            result["frozen_thresholds"]["normal_load_numerical_tolerance_n"],
            1e-5,
        )
        self.assertEqual(len(result["failed_trim_samples"]), 1)
        self.assertEqual(len(result["residual_limit_violations"]), 1)
        self.assertEqual(len(result["below_tir_load_samples"]), 1)
        self.assertEqual(
            result["below_tir_load_samples"][0]["minimum_load_wheel"], "FL"
        )


if __name__ == "__main__":
    unittest.main()
