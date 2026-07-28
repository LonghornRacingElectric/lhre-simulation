"""Validate a completed battery trade-study output directory."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from battery_trade_study import (  # noqa: E402
    BASELINE_PARALLEL_CELLS,
    BASELINE_SERIES_CELLS,
    MAXIMUM_PACK_VOLTAGE_V,
    MAXIMUM_USABLE_ENERGY_KWH,
    pareto_frontier_mask,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-root",
        type=Path,
        default=ROOT / "outputs" / "battery_trade_study_20260725_final",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_root = args.output_root.resolve()
    generated = pd.read_csv(output_root / "generated_pack_configurations.csv")
    results = pd.read_csv(output_root / "all_configuration_results.csv")
    ranked = pd.read_csv(output_root / "ranked_constraint_valid_results.csv")
    pareto = pd.read_csv(output_root / "pareto_frontier.csv")
    sensitivities = pd.read_csv(output_root / "sensitivity_cases.csv")
    cell_document = json.loads(
        (ROOT / "inputs" / "battery_trade_study" / "cells.json").read_text(
            encoding="utf-8"
        )
    )
    headline = json.loads(
        (output_root / "headline_summary.json").read_text(encoding="utf-8")
    )

    checks: dict[str, bool | int | float] = {}
    checks["all_generated_topologies_simulated_once"] = bool(
        len(generated) == len(results)
        and generated["candidate_id"].is_unique
        and results["candidate_id"].is_unique
        and set(generated["candidate_id"]) == set(results["candidate_id"])
    )
    checks["all_voltage_constraints_pass"] = bool(
        (generated["maximum_pack_voltage_v"] < MAXIMUM_PACK_VOLTAGE_V).all()
    )
    checks["all_energy_constraints_pass"] = bool(
        (
            generated["model_usable_energy_kwh"]
            <= MAXIMUM_USABLE_ENERGY_KWH + 1e-10
        ).all()
    )
    checks["all_packaging_constraints_pass"] = bool(
        (generated["cell_envelope_vs_baseline_pct"] <= 100.0 + 1e-9).all()
    )
    checks["all_requested_power_caps_pass"] = bool(
        (results["power_limit_kw"] <= 80.0 + 1e-9).all()
    )

    valid = results[results["constraint_valid"].fillna(False)].copy()
    required_metrics = [
        "elapsed_time_s",
        "equivalent_average_lap_time_s",
        "peak_cell_current_a",
        "rms_cell_current_a",
        "pack_resistive_heat_kwh",
        "battery_discharge_efficiency_pct",
        "remaining_usable_chemical_kwh",
    ]
    checks["valid_metrics_are_finite"] = bool(
        np.isfinite(valid[required_metrics].to_numpy(dtype=float)).all()
    )
    checks["valid_rows_end_at_exact_distance"] = bool(
        np.allclose(valid["target_distance_m"], 22000.0, atol=1e-9)
    )
    checks["valid_rows_respect_cell_current"] = bool(
        (
            valid["peak_pack_current_a"]
            <= valid["pack_current_limit_a"] + 1e-5
        ).all()
    )
    if "regen_endurance_only" in valid and valid[
        "regen_endurance_only"
    ].fillna(False).any():
        checks["regen_terminal_energy_identity_passes"] = bool(
            np.allclose(
                valid["terminal_energy_kwh"],
                valid["terminal_discharge_energy_kwh"]
                - valid["terminal_regenerated_energy_kwh"],
                rtol=0.0,
                atol=1e-9,
            )
        )
        checks["regen_energy_and_heat_are_nonnegative"] = bool(
            (valid["terminal_regenerated_energy_kwh"] >= 0.0).all()
            and (valid["regen_pack_resistive_heat_kwh"] >= 0.0).all()
            and (
                valid["regen_pack_resistive_heat_kwh"]
                <= valid["pack_resistive_heat_kwh"] + 1e-12
            ).all()
        )
        checks["regen_command_and_peak_are_respected"] = bool(
            (
                valid["peak_regen_terminal_power_kw"]
                <= valid["regen_command_power_kw"] + 1e-6
            ).all()
        )
        checks["maximum_charge_voltage_is_respected"] = bool(
            (
                valid["maximum_terminal_voltage_v"]
                <= valid["maximum_pack_voltage_v"] + 1e-6
            ).all()
        )
        checks["regen_rms_uses_policy_threshold"] = bool(
            np.allclose(
                valid["regen_active_power_threshold_kw"],
                valid["regen_active_power_threshold_kw"].iloc[0],
                rtol=0.0,
                atol=0.0,
            )
        )
        checks["regen_active_rms_is_finite"] = bool(
            np.isfinite(
                valid["regen_active_rms_terminal_power_kw"].to_numpy(
                    dtype=float
                )
            ).all()
        )
    equal_reserve = valid[valid["equal_reserve_eligible"].fillna(False)]
    checks["equal_reserve_rows_within_tolerance"] = bool(
        (equal_reserve["reserve_error_kwh"].abs() <= 0.0100001).all()
    )
    saturated = valid[valid["search_status"] == "power_cap_extra_reserve"]
    checks["saturated_rows_are_at_80kw"] = bool(
        saturated.empty
        or np.allclose(saturated["power_limit_kw"], 80.0, atol=1e-8)
    )

    baseline_id = (
        f"molicel_p30b_{BASELINE_SERIES_CELLS}s"
        f"{BASELINE_PARALLEL_CELLS}p"
    )
    baseline = results[results["candidate_id"] == baseline_id].iloc[0]
    expected_improvement = (
        (
            float(baseline["elapsed_time_s"])
            - results["elapsed_time_s"].to_numpy(dtype=float)
        )
        / float(baseline["elapsed_time_s"])
        * 100.0
    )
    checks["improvement_sign_and_formula_pass"] = bool(
        np.allclose(
            results["improvement_vs_baseline_pct"].to_numpy(dtype=float),
            expected_improvement,
            equal_nan=True,
        )
    )
    checks["baseline_is_native_mesh"] = bool(
        baseline["simulation_fidelity"] == "native_0p25m"
    )
    native_cell_ids = set(
        results[results["simulation_fidelity"] == "native_0p25m"]["cell_id"]
    )
    expected_cell_ids = {cell["cell_id"] for cell in cell_document["cells"]}
    checks["every_cell_has_native_mesh_finalist"] = bool(
        native_cell_ids == expected_cell_ids
    )
    checks["all_report_recommendations_are_native_mesh"] = bool(
        all(
            headline[key]["simulation_fidelity"] == "native_0p25m"
            for key in (
                "best_overall",
                "best_lightweight",
                "best_thermal",
                "best_value",
            )
        )
    )

    recomputed_pareto = valid[pareto_frontier_mask(valid)]
    checks["pareto_file_matches_recomputation"] = bool(
        set(pareto["candidate_id"]) == set(recomputed_pareto["candidate_id"])
    )
    checks["ranked_file_matches_valid_rows"] = bool(
        set(ranked["candidate_id"]) == set(valid["candidate_id"])
        and ranked["elapsed_time_s"].is_monotonic_increasing
    )
    checks["six_local_sensitivity_cases_present"] = bool(
        len(sensitivities) == 6
        and sensitivities["sensitivity_case"].nunique() == 6
        and np.isfinite(
            sensitivities["elapsed_time_s"].to_numpy(dtype=float)
        ).all()
    )
    checks["valid_configuration_count"] = int(len(valid))
    checks["equal_reserve_count"] = int(len(equal_reserve))
    checks["power_cap_saturated_count"] = int(len(saturated))
    checks["native_refinement_count"] = int(
        (results["simulation_fidelity"] == "native_0p25m").sum()
    )
    checks["maximum_absolute_equal_reserve_error_kwh"] = (
        float(equal_reserve["reserve_error_kwh"].abs().max())
        if not equal_reserve.empty
        else math.nan
    )
    failed = [
        name
        for name, value in checks.items()
        if isinstance(value, bool) and not value
    ]
    report = {
        "passed": not failed,
        "failed_checks": failed,
        "checks": checks,
    }
    (output_root / "validation_report.json").write_text(
        json.dumps(report, indent=2),
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2))
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
