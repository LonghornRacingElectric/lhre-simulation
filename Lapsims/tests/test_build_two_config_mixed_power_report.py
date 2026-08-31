from __future__ import annotations

import hashlib
import json
import sys
import tempfile
import unittest
from pathlib import Path

import pandas as pd

LAPSIMS_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = LAPSIMS_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from build_reduced_dof_cg_comparison import DEFAULT_SCORING_REFERENCE
from build_two_config_mixed_power_report import (
    CONFIG_1,
    CONFIG_2,
    EXPECTED_CONFIGS,
    EXPECTED_EVENTS,
    PROVENANCE_SCHEMA,
    build_two_config_mixed_power_report,
)

EVENT_LENGTHS = {
    "acceleration": 75.0,
    "skidpad": 57.33406592801317,
    "autocross": 791.0,
    "michigan_endurance": 1069.968773,
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _configuration(case_name: str) -> dict[str, float]:
    if case_name == CONFIG_1:
        return {
            "cg_height_in": 11.5,
            "rear_fraction": 0.54,
            "front_arb_fraction": 0.63,
            "front_brake_bias": 0.70,
        }
    return {
        "cg_height_in": 11.75,
        "rear_fraction": 0.565,
        "front_arb_fraction": 0.67,
        "front_brake_bias": 0.685,
    }


def _event_time(power_w: float, case_name: str, event_slug: str) -> float:
    selected = {
        CONFIG_1: {
            "acceleration": 4.0,
            "skidpad": 5.0,
            "autocross": 55.0,
            "michigan_endurance": 64.0,
        },
        CONFIG_2: {
            "acceleration": 3.9,
            "skidpad": 5.1,
            "autocross": 54.0,
            "michigan_endurance": 63.5,
        },
    }
    value = selected[case_name][event_slug]
    if power_w == 80_000.0 and event_slug == "michigan_endurance":
        return value + 100.0
    if power_w == 32_000.0 and event_slug != "michigan_endurance":
        return value + 100.0
    return value


def _metadata(
    *,
    case_name: str,
    power_w: float,
    map_hash: str,
) -> dict[str, object]:
    config = _configuration(case_name)
    rear = config["rear_fraction"]
    front = 1.0 - rear
    height_in = config["cg_height_in"]
    height_m = height_in * 0.0254
    total_mass = 261.07265114
    sprung_mass = 230.72288408
    front_arb = config["front_arb_fraction"]
    brake_bias = config["front_brake_bias"]
    tuning = {
        "front_antiroll_stiffness_fraction": front_arb,
        "brake_distribution_front": brake_bias,
    }
    return {
        "model_family": "dyn_py_reduced_order_qss",
        "model_key": "dyn_py_6dof_qss",
        "model_dof": 6,
        "sweep_axis": "two_configuration_comparison",
        "cg_case": case_name,
        "cg_height_m": height_m,
        "cg_height_in": height_in,
        "sprung_mass_kg": sprung_mass,
        "total_mass_kg": total_mass,
        "rear_static_weight_fraction": rear,
        "front_static_weight_fraction": front,
        "effective_lltd": 0.60,
        "effective_lltd_interpretation": "synthetic test value",
        "front_antiroll_stiffness_fraction": front_arb,
        "front_elastic_roll_stiffness_fraction": 0.61,
        "effective_brake_distribution_front": brake_bias,
        "effective_aero_balance_front": 0.5,
        "effective_cop_from_front_m": 0.7747,
        "tire_mu_scale": 0.622543713077903,
        "drive_distribution_front": 0.0,
        "limited_slip_differential_model": False,
        "case_tuning": tuning,
        "effective_drive_power_limit_w": power_w,
        "ggv_source_sha256": map_hash,
        "wheel_load_min_n": 120.0,
        "wheel_load_max_n": 1800.0,
        "wheel_load_outside_tire_validity": False,
        "tire_valid_load_min_n": 100.0,
        "tire_valid_load_max_n": 1800.0,
        "reduced_vehicle_parameters": {
            "center_of_gravity_m": [-1.5494 * rear, 0.0, height_m],
            "mass_kg": total_mass,
            "sprung_mass_kg": sprung_mass,
            "inertia_kg_m2": [[26.0, 0.0, -5.0], [0.0, 92.0, 0.0], [-5.0, 0.0, 103.0]],
            "sprung_inertia_kg_m2": [[15.0, 0.0, -5.0], [0.0, 74.0, 0.0], [-5.0, 0.0, 73.0]],
            "static_wheel_loads_n": [600.0, 600.0, 680.0, 680.0],
            "antiroll_stiffness_nm_per_rad": [30_000.0, 15_000.0],
            "brake_distribution_front": brake_bias,
        },
    }


def _write_study(
    root: Path,
    *,
    power_w: float,
    case_names: tuple[str, str] = EXPECTED_CONFIGS,
) -> None:
    root.mkdir(parents=True)
    rows: list[dict[str, object]] = []
    for case_name in case_names:
        case_dir = root / case_name
        case_dir.mkdir()
        map_path = case_dir / "ggv.csv"
        map_path.write_text(
            f"speed_mps,ay_mps2,ax_accel_mps2\n0,0,{power_w / 10000.0}\n",
            encoding="utf-8",
        )
        map_hash = _sha256(map_path)
        metadata = _metadata(
            case_name=case_name if case_name in EXPECTED_CONFIGS else CONFIG_2,
            power_w=power_w,
            map_hash=map_hash,
        )
        metadata["cg_case"] = case_name
        (case_dir / "case_metadata.json").write_text(
            json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
        )
        for event_slug in EXPECTED_EVENTS:
            source_case = case_name if case_name in EXPECTED_CONFIGS else CONFIG_2
            lap_time = _event_time(power_w, source_case, event_slug)
            event_name = {
                "acceleration": "FSAE Acceleration 75m",
                "skidpad": "FSAE Skidpad 9.125m Radius",
                "autocross": "FSAE Autocross Nebraska 2013",
                "michigan_endurance": "FSAE Michigan Endurance 2014",
            }[event_slug]
            summary = {
                "lap_time_s": lap_time,
                "track_length_m": EVENT_LENGTHS[event_slug],
                "converged": True,
            }
            (case_dir / f"{event_slug}_summary.json").write_text(
                json.dumps(summary, indent=2) + "\n", encoding="utf-8"
            )
            pd.DataFrame(
                [{"distance_m": 1.0, "speed_mps": 10.0, "elapsed_time_s": 0.1}]
            ).to_csv(case_dir / f"{event_slug}_trace.csv", index=False)
            rows.append(
                {
                    "solver": "synthetic test solver",
                    "lap_time_s": lap_time,
                    "track_length_m": EVENT_LENGTHS[event_slug],
                    "segments": 10,
                    "track_configuration": (
                        "closed"
                        if event_slug in {"skidpad", "michigan_endurance"}
                        else "open"
                    ),
                    "standing_start": event_slug in {"acceleration", "autocross"},
                    "converged": True,
                    "final_max_speed_change_mps": 0.0,
                    "ggv_source_csv": str(map_path.resolve()),
                    "ggv_source_sha256": map_hash,
                    "ggv_solver_speed_cap_mps": 42.0,
                    "ggv_max_lateral_utilization": 1.0,
                    "ggv_max_speed_domain_fraction": 0.8,
                    "ggv_speed_cap_segments": 0,
                    "model_family": "dyn_py_reduced_order_qss",
                    "model_key": "dyn_py_6dof_qss",
                    "model_dof": 6,
                    "sweep_axis": "two_configuration_comparison",
                    "cg_case": case_name,
                    "cg_height_m": metadata["cg_height_m"],
                    "cg_height_in": metadata["cg_height_in"],
                    "sprung_mass_kg": metadata["sprung_mass_kg"],
                    "total_mass_kg": metadata["total_mass_kg"],
                    "rear_static_weight_fraction": metadata[
                        "rear_static_weight_fraction"
                    ],
                    "front_static_weight_fraction": metadata[
                        "front_static_weight_fraction"
                    ],
                    "effective_lltd": metadata["effective_lltd"],
                    "effective_lltd_interpretation": metadata[
                        "effective_lltd_interpretation"
                    ],
                    "front_antiroll_stiffness_fraction": metadata[
                        "front_antiroll_stiffness_fraction"
                    ],
                    "front_elastic_roll_stiffness_fraction": metadata[
                        "front_elastic_roll_stiffness_fraction"
                    ],
                    "effective_brake_distribution_front": metadata[
                        "effective_brake_distribution_front"
                    ],
                    "effective_aero_balance_front": metadata[
                        "effective_aero_balance_front"
                    ],
                    "tire_mu_scale": metadata["tire_mu_scale"],
                    "effective_drive_power_limit_w": power_w,
                    "drive_power_limit_source": "sweep_json",
                    "event_slug": event_slug,
                    "event_name": event_name,
                    "wheel_load_min_n": metadata["wheel_load_min_n"],
                    "wheel_load_max_n": metadata["wheel_load_max_n"],
                    "wheel_load_range_method": "synthetic",
                    "wheel_load_outside_tire_validity": False,
                    # Stale columns must not survive the final report.
                    "projected_points": -999.0,
                    "common_projected_points": -998.0,
                }
            )
    pd.DataFrame(rows).to_csv(root / "event_results.csv", index=False)
    (root / "smoke_summary.json").write_text(
        json.dumps({"effective_drive_power_limit_w": power_w}, indent=2) + "\n",
        encoding="utf-8",
    )


class TwoConfigMixedPowerReportTests(unittest.TestCase):
    def test_builds_one_common_field_and_all_artifacts(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            root_80 = temporary / "study_80kw"
            root_32 = temporary / "study_32kw"
            output = temporary / "report"
            _write_study(root_80, power_w=80_000.0)
            _write_study(root_32, power_w=32_000.0)

            validation = build_two_config_mixed_power_report(
                study_80kw_root=root_80,
                study_32kw_root=root_32,
                output_root=output,
                scoring_reference_path=DEFAULT_SCORING_REFERENCE,
            )

            self.assertEqual(validation["status"], "passed")
            self.assertEqual(validation["case_count"], 2)
            self.assertEqual(validation["event_row_count"], 8)
            self.assertTrue(validation["only_exact_common_tmin_ties_receive_maximum"])
            self.assertTrue(validation["config_1_total_delta_exactly_zero"])

            events = pd.read_csv(output / "two_config_event_results.csv")
            acceleration = events[
                (events["cg_case"] == CONFIG_2)
                & (events["event_slug"] == "acceleration")
            ].iloc[0]
            endurance = events[
                (events["cg_case"] == CONFIG_2)
                & (events["event_slug"] == "michigan_endurance")
            ].iloc[0]
            self.assertEqual(acceleration["mixed_power_source"], "fresh_80kw_sprint")
            self.assertAlmostEqual(acceleration["lap_time_s"], 3.9)
            self.assertEqual(endurance["mixed_power_source"], "fresh_32kw_endurance")
            self.assertAlmostEqual(endurance["lap_time_s"], 63.5)
            self.assertNotIn("projected_points", events.columns)

            deltas = pd.read_csv(output / "two_config_event_deltas_vs_config1.csv")
            accel_delta = deltas[deltas["event_slug"] == "acceleration"].iloc[0]
            endurance_delta = deltas[
                deltas["event_slug"] == "michigan_endurance"
            ].iloc[0]
            self.assertAlmostEqual(
                accel_delta["config_2_minus_config_1_raw_time_s"], -0.1
            )
            self.assertAlmostEqual(
                endurance_delta["config_2_minus_config_1_raw_time_s"], -0.5
            )

            totals = pd.read_csv(output / "two_config_case_points.csv")
            config_1 = totals[totals["cg_case"] == CONFIG_1].iloc[0]
            self.assertEqual(config_1["timed_event_points_delta_vs_config_1"], 0.0)

            provenance = json.loads(
                (output / "two_config_provenance.json").read_text(encoding="utf-8")
            )
            self.assertEqual(provenance["schema"], PROVENANCE_SCHEMA)
            self.assertEqual(len(provenance["selected_rows"]), 8)
            for filename in (
                "two_config_mixed_power_report.md",
                "two_config_event_points_delta_vs_config1.png",
                "two_config_raw_time_delta_vs_config1.png",
                "two_config_tuning_validity.csv",
                "validation_summary.json",
            ):
                path = output / filename
                self.assertTrue(path.is_file(), filename)
                self.assertGreater(path.stat().st_size, 100, filename)

    def test_rejects_wrong_configuration_set(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            root_80 = temporary / "study_80kw"
            root_32 = temporary / "study_32kw"
            _write_study(
                root_80,
                power_w=80_000.0,
                case_names=(CONFIG_1, "unexpected_config"),
            )
            _write_study(root_32, power_w=32_000.0)
            with self.assertRaisesRegex(ValueError, "must contain exactly"):
                build_two_config_mixed_power_report(
                    study_80kw_root=root_80,
                    study_32kw_root=root_32,
                    output_root=temporary / "report",
                )

    def test_rejects_nonconverged_or_speed_capped_source(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            root_80 = temporary / "study_80kw"
            root_32 = temporary / "study_32kw"
            _write_study(root_80, power_w=80_000.0)
            _write_study(root_32, power_w=32_000.0)
            events = pd.read_csv(root_32 / "event_results.csv")
            events.loc[0, "ggv_speed_cap_segments"] = 1
            events.to_csv(root_32 / "event_results.csv", index=False)
            with self.assertRaisesRegex(ValueError, "speed-cap-limited"):
                build_two_config_mixed_power_report(
                    study_80kw_root=root_80,
                    study_32kw_root=root_32,
                    output_root=temporary / "report",
                )

    def test_rejects_80_32_controlled_metadata_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            root_80 = temporary / "study_80kw"
            root_32 = temporary / "study_32kw"
            _write_study(root_80, power_w=80_000.0)
            _write_study(root_32, power_w=32_000.0)
            metadata_path = root_32 / CONFIG_2 / "case_metadata.json"
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            metadata["effective_brake_distribution_front"] += 0.01
            metadata_path.write_text(
                json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
            )
            events_path = root_32 / "event_results.csv"
            events = pd.read_csv(events_path)
            events.loc[
                events["cg_case"] == CONFIG_2,
                "effective_brake_distribution_front",
            ] = metadata["effective_brake_distribution_front"]
            events.to_csv(events_path, index=False)
            with self.assertRaisesRegex(ValueError, "controlled metadata differs"):
                build_two_config_mixed_power_report(
                    study_80kw_root=root_80,
                    study_32kw_root=root_32,
                    output_root=temporary / "report",
                )


if __name__ == "__main__":
    unittest.main()
