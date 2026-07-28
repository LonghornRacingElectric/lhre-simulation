"""Rerun selected battery candidates on the native mesh and refresh outputs."""

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

from battery_trade_study import (  # noqa: E402
    fit_linear_sensitivities,
)
from run_battery_trade_study import (  # noqa: E402
    _add_comparisons,
    _best_rows,
    _build_payload,
    _calculate_local_sensitivities,
    _centered_refine_grid,
    _local_sensitivity_payloads,
    _run_parallel,
    _save_plots,
    _write_report,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Native-mesh rerun for explicit candidate IDs, followed by a "
            "consistent refresh of rankings, sensitivities, plots, and report."
        )
    )
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument(
        "--candidate-id",
        action="append",
        required=True,
        help="Candidate ID to rerun; may be supplied multiple times.",
    )
    parser.add_argument("--workers", type=int, default=2)
    return parser.parse_args()


def _replace_rows(
    destination: pd.DataFrame,
    replacements: pd.DataFrame,
    *,
    append_missing: bool = False,
) -> pd.DataFrame:
    result = destination.copy()
    for _, row in replacements.iterrows():
        mask = result["candidate_id"] == row["candidate_id"]
        if not mask.any():
            if not append_missing:
                raise ValueError(
                    f"Unknown candidate_id {row['candidate_id']!r}"
                )
            result = pd.concat(
                [result, pd.DataFrame([row]).reindex(columns=result.columns)],
                ignore_index=True,
            )
            continue
        for column, value in row.items():
            result.loc[mask, column] = value
    return result


def main() -> None:
    cli = parse_args()
    output_root = cli.output_root.resolve()
    settings = json.loads(
        (output_root / "study_inputs.json").read_text(encoding="utf-8")
    )
    regen_settings = settings.get(
        "regeneration",
        {
            "enabled": False,
            "frozen_command_power_kw": 0.0,
            "active_rms_target_kw": 0.0,
            "active_power_threshold_kw": 1.0,
        },
    )
    results = pd.read_csv(output_root / "all_configuration_results.csv")
    screening = pd.read_csv(output_root / "screening_results.csv")
    generated = pd.read_csv(
        output_root / "generated_pack_configurations.csv"
    )
    rejected = pd.read_csv(
        output_root / "rejected_pack_configurations.csv"
    )
    cell_path = Path(
        settings.get(
            "source_cell_file",
            ROOT / "inputs" / "battery_trade_study" / "cells.json",
        )
    )
    cell_document = json.loads(cell_path.read_text(encoding="utf-8"))
    cells_by_id = {
        str(cell["cell_id"]): cell for cell in cell_document["cells"]
    }
    args = SimpleNamespace(
        vehicle=Path(
            settings.get(
                "source_vehicle_file",
                ROOT / "inputs" / "openlap_vehicle.json",
            )
        ),
        track=Path(
            settings.get(
                "source_track_file",
                ROOT / "inputs" / "michigan_openlap_track.csv",
            )
        ),
        powertrain=Path(
            settings.get(
                "source_powertrain_file",
                ROOT
                / "inputs"
                / "powertrain_130s5p_p30b_provisional.json",
            )
        ),
        distance_km=float(settings["distance_km"]),
        reserve_kwh=float(settings["reserve_target_kwh"]),
        reserve_tolerance_kwh=float(settings["reserve_tolerance_kwh"]),
        power_cap_kw=float(settings["power_cap_kw"]),
        search_iterations=7,
        search_downsample=int(settings["screening_downsample_factor"]),
        search_soc_count=int(settings["screening_surface"][0]),
        search_speed_count=int(settings["screening_surface"][1]),
        regen_command_power_kw_resolved=float(
            regen_settings.get("frozen_command_power_kw", 0.0)
        ),
        regen_active_rms_target_kw_resolved=float(
            regen_settings.get("active_rms_target_kw", 0.0)
        ),
        regen_active_power_threshold_kw_resolved=float(
            regen_settings.get("active_power_threshold_kw", 1.0)
        ),
    )

    requested_ids = list(dict.fromkeys(cli.candidate_id))
    missing = sorted(set(requested_ids) - set(generated["candidate_id"]))
    if missing:
        raise ValueError(f"Unknown candidate IDs: {missing}")

    payloads: list[dict] = []
    for candidate_id in requested_ids:
        candidate = generated[
            generated["candidate_id"] == candidate_id
        ].iloc[0].to_dict()
        coarse = screening[
            screening["candidate_id"] == candidate_id
        ].iloc[0].to_dict()
        payloads.append(
            _build_payload(
                args,
                candidate,
                cells_by_id[str(candidate["cell_id"])],
                downsample_factor=1,
                surface_soc_count=int(settings["refinement_surface"][0]),
                surface_speed_count=int(settings["refinement_surface"][1]),
                surface_verification_iterations=12,
                fidelity="native_0p25m",
                search_grid_kw=_centered_refine_grid(
                    float(coarse["power_limit_kw"]),
                    args.power_cap_kw,
                ),
            )
        )

    targeted = pd.DataFrame(_run_parallel(payloads, cli.workers))
    targeted.to_csv(
        output_root / "targeted_refinement_results.csv",
        index=False,
    )
    if not bool(targeted["constraint_valid"].all()):
        failed = targeted.loc[
            ~targeted["constraint_valid"],
            ["candidate_id", "search_status", "reserve_error_kwh"],
        ]
        raise RuntimeError(
            "Targeted refinement still contains invalid rows:\n"
            + failed.to_string(index=False)
        )

    results = _replace_rows(results, targeted)
    refined_path = output_root / "refined_results.csv"
    if refined_path.exists():
        refined = pd.read_csv(refined_path)
        refined = _replace_rows(
            refined,
            targeted,
            append_missing=True,
        )
        refined.to_csv(refined_path, index=False)

    results, baseline = _add_comparisons(results)
    results = results.sort_values(
        ["constraint_valid", "elapsed_time_s"],
        ascending=[False, True],
        na_position="last",
    )
    results.to_csv(
        output_root / "all_configuration_results.csv",
        index=False,
    )
    valid = results[results["constraint_valid"]].copy()
    valid.to_csv(
        output_root / "ranked_constraint_valid_results.csv",
        index=False,
    )
    valid[valid["pareto_frontier"]].to_csv(
        output_root / "pareto_frontier.csv",
        index=False,
    )

    best_rows = _best_rows(results)
    global_sensitivity = fit_linear_sensitivities(
        screening[screening["constraint_valid"]]
    )
    sensitivity_payloads = _local_sensitivity_payloads(
        args,
        best_rows["best_overall"],
        cells_by_id,
    )
    sensitivity_results = _run_parallel(
        sensitivity_payloads,
        min(cli.workers, len(sensitivity_payloads)),
    )
    for result, payload in zip(
        sensitivity_results,
        sensitivity_payloads,
    ):
        result["sensitivity_case"] = payload["sensitivity_case"]
    sensitivity_frame = pd.DataFrame(sensitivity_results)
    sensitivity_frame.to_csv(
        output_root / "sensitivity_cases.csv",
        index=False,
    )
    local_sensitivity = _calculate_local_sensitivities(
        best_rows["best_overall"],
        sensitivity_frame,
    )
    (output_root / "sensitivity_summary.json").write_text(
        json.dumps(
            {
                "local_finite_difference": local_sensitivity,
                "cross_configuration_linear_fit": global_sensitivity,
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    _save_plots(results, output_root)
    report_path = _write_report(
        output_root,
        results,
        baseline,
        best_rows,
        global_sensitivity,
        local_sensitivity,
        cell_document,
        len(generated),
        len(rejected),
        int(settings["screening_downsample_factor"]),
        regen_settings,
    )
    headline = {
        "baseline": baseline,
        **best_rows,
        "local_sensitivity": local_sensitivity,
        "global_sensitivity": global_sensitivity,
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
    print(
        targeted[
            [
                "candidate_id",
                "elapsed_time_s",
                "power_limit_kw",
                "remaining_usable_chemical_kwh",
                "search_status",
            ]
        ].to_string(index=False),
        flush=True,
    )
    print(
        f"Updated {len(targeted)} candidates; "
        f"{len(valid)}/{len(results)} rows are constraint-valid.",
        flush=True,
    )


if __name__ == "__main__":
    main()
