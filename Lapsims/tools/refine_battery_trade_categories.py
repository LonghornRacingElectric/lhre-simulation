"""Native-mesh refinement for report category recommendations."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "tools"))

from run_battery_trade_study import (  # noqa: E402
    _add_comparisons,
    _best_rows,
    _build_payload,
    _centered_refine_grid,
    _run_parallel,
    _save_plots,
    _write_report,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-root",
        type=Path,
        default=ROOT / "outputs" / "battery_trade_study_20260725_final",
    )
    parser.add_argument("--workers", type=int, default=3)
    parser.add_argument(
        "--report-only",
        action="store_true",
        help="Regenerate derived files and the report without new simulations.",
    )
    return parser.parse_args()


def main() -> None:
    cli = parse_args()
    output_root = cli.output_root.resolve()
    results = pd.read_csv(output_root / "all_configuration_results.csv")
    generated = pd.read_csv(
        output_root / "generated_pack_configurations.csv"
    )
    rejected = pd.read_csv(
        output_root / "rejected_pack_configurations.csv"
    )
    cell_path = ROOT / "inputs" / "battery_trade_study" / "cells.json"
    cell_document = json.loads(cell_path.read_text(encoding="utf-8"))
    cells_by_id = {
        cell["cell_id"]: cell for cell in cell_document["cells"]
    }
    category_keys = ("best_lightweight", "best_thermal", "best_value")
    if cli.report_only:
        target_ids: set[str] = set()
    else:
        existing_best = _best_rows(results)
        target_ids = {
            str(existing_best[key]["candidate_id"]) for key in category_keys
        }
        valid = results[results["constraint_valid"]].copy()
        if not valid.empty:
            best_time = float(valid["elapsed_time_s"].min())
            target_ids.update(
                valid[valid["elapsed_time_s"] <= best_time * 1.01]
                .nsmallest(3, "pack_mass_kg")["candidate_id"]
                .astype(str)
            )
            target_ids.update(
                valid[valid["elapsed_time_s"] <= best_time * 1.02]
                .nsmallest(6, "pack_resistive_heat_kwh")["candidate_id"]
                .astype(str)
            )
        target_ids = {
            candidate_id
            for candidate_id in target_ids
            if str(
                results.loc[
                    results["candidate_id"] == candidate_id,
                    "simulation_fidelity",
                ].iloc[0]
            )
            != "native_0p25m"
        }
    settings = json.loads((output_root / "study_inputs.json").read_text())
    args = SimpleNamespace(
        vehicle=ROOT / "inputs" / "openlap_vehicle.json",
        track=ROOT / "inputs" / "michigan_openlap_track.csv",
        powertrain=(
            ROOT / "inputs" / "powertrain_130s5p_p30b_provisional.json"
        ),
        distance_km=float(settings["distance_km"]),
        reserve_kwh=float(settings["reserve_target_kwh"]),
        reserve_tolerance_kwh=float(settings["reserve_tolerance_kwh"]),
        power_cap_kw=float(settings["power_cap_kw"]),
        search_iterations=7,
    )
    payloads = []
    for candidate_id in sorted(target_ids):
        candidate = generated[
            generated["candidate_id"] == candidate_id
        ].iloc[0].to_dict()
        coarse = results[
            results["candidate_id"] == candidate_id
        ].iloc[0].to_dict()
        payloads.append(
            _build_payload(
                args,
                candidate,
                cells_by_id[str(candidate["cell_id"])],
                downsample_factor=1,
                surface_soc_count=9,
                surface_speed_count=41,
                surface_verification_iterations=12,
                fidelity="native_0p25m",
                search_grid_kw=_centered_refine_grid(
                    float(coarse["power_limit_kw"]), args.power_cap_kw
                ),
            )
        )
    if payloads:
        refined_rows = _run_parallel(payloads, cli.workers)
        refined = pd.DataFrame(refined_rows)
        refined.to_csv(
            output_root / "category_refined_results.csv", index=False
        )
        for _, row in refined.iterrows():
            mask = results["candidate_id"] == row["candidate_id"]
            for column, value in row.items():
                results.loc[mask, column] = value
    else:
        refined = pd.DataFrame()
        print("All category recommendations are already native-mesh results.")

    results, baseline = _add_comparisons(results)
    results = results.sort_values(
        ["constraint_valid", "elapsed_time_s"],
        ascending=[False, True],
        na_position="last",
    )
    results.to_csv(
        output_root / "all_configuration_results.csv", index=False
    )
    valid = results[results["constraint_valid"]].copy()
    valid.to_csv(
        output_root / "ranked_constraint_valid_results.csv", index=False
    )
    valid[valid["pareto_frontier"]].to_csv(
        output_root / "pareto_frontier.csv", index=False
    )
    best_rows = _best_rows(results)
    sensitivity_summary = json.loads(
        (output_root / "sensitivity_summary.json").read_text()
    )
    local = sensitivity_summary["local_finite_difference"]
    global_fit = sensitivity_summary["cross_configuration_linear_fit"]
    _save_plots(results, output_root)
    report_path = _write_report(
        output_root,
        results,
        baseline,
        best_rows,
        global_fit,
        local,
        cell_document,
        len(generated),
        len(rejected),
        int(settings["screening_downsample_factor"]),
    )
    headline = {
        "baseline": baseline,
        **best_rows,
        "local_sensitivity": local,
        "global_sensitivity": global_fit,
        "constraint_valid_count": int(len(valid)),
        "equal_reserve_count": int(
            results["equal_reserve_eligible"].sum()
        ),
        "pareto_count": int(results["pareto_frontier"].sum()),
        "report": str(report_path),
    }
    (output_root / "headline_summary.json").write_text(
        json.dumps(headline, indent=2, default=str),
        encoding="utf-8",
    )
    if not refined.empty:
        print(
            refined[
                [
                    "candidate_id",
                    "elapsed_time_s",
                    "power_limit_kw",
                    "remaining_usable_chemical_kwh",
                ]
            ].to_string(index=False)
        )
    print("Updated category recommendations:")
    for key in category_keys:
        row = best_rows[key]
        print(
            f"{key}: {row['candidate_id']} "
            f"({row['simulation_fidelity']}, {row['elapsed_time_s']:.3f} s)"
        )


if __name__ == "__main__":
    main()
