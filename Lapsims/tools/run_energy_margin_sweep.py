"""Tune one terminal-power limit per pack size for a fixed energy margin.

The endurance solver uses a repeating Michigan lap. This driver evaluates the
state at an exact cumulative distance, rather than rounding the request to an
integer number of laps.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from endurance_solver import simulate_endurance  # noqa: E402
from openlap_solver import load_vehicle  # noqa: E402
from powertrain_model import (  # noqa: E402
    MaximumTorqueSurface,
    PowertrainModel,
    load_powertrain_config,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--vehicle",
        type=Path,
        default=ROOT / "inputs" / "openlap_vehicle.json",
    )
    parser.add_argument(
        "--track",
        type=Path,
        default=ROOT / "inputs" / "michigan_openlap_track.csv",
    )
    parser.add_argument(
        "--powertrain",
        type=Path,
        default=ROOT / "inputs" / "powertrain_130s5p_p30b_provisional.json",
    )
    parser.add_argument("--parallel-counts", type=int, nargs="+", default=[3, 4, 5, 6, 7])
    parser.add_argument("--distance-km", type=float, default=22.0)
    parser.add_argument("--margin-kwh", type=float, default=0.5)
    parser.add_argument("--baseline-parallel-count", type=int, default=5)
    parser.add_argument("--search-downsample", type=int, default=8)
    parser.add_argument("--search-iterations", type=int, default=8)
    parser.add_argument("--surface-soc-count", type=int, default=25)
    parser.add_argument("--surface-speed-count", type=int, default=121)
    parser.add_argument("--workers", type=int, default=5)
    parser.add_argument(
        "--reuse-coarse",
        action="store_true",
        help="Reuse already bracketed rows in coarse_search_results.csv.",
    )
    parser.add_argument(
        "--refine-existing",
        action="store_true",
        help="Refine non-saturated rows already in the final summary.",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=ROOT / "outputs" / "energy_margin_22km_20260723",
    )
    return parser.parse_args()


def downsample_track(track: pd.DataFrame, factor: int) -> pd.DataFrame:
    if factor <= 1:
        return track.copy()
    rows: list[dict[str, float]] = []
    cumulative_distance = 0.0
    for start in range(0, len(track), factor):
        group = track.iloc[start : start + factor]
        dx = group["dx_m"].to_numpy(dtype=float)
        group_dx = float(dx.sum())
        cumulative_distance += group_dx
        rows.append(
            {
                "distance_m": cumulative_distance,
                "dx_m": group_dx,
                "curvature_1pm": float(
                    np.average(
                        group["curvature_1pm"].to_numpy(dtype=float),
                        weights=dx,
                    )
                ),
            }
        )
    return pd.DataFrame(rows)


def state_at_distance(
    trace: pd.DataFrame,
    target_distance_m: float,
    pack: Any,
) -> dict[str, float] | None:
    if trace.empty or float(trace["cumulative_distance_m"].iloc[-1]) < target_distance_m:
        return None
    distances = trace["cumulative_distance_m"].to_numpy(dtype=float)
    index = int(np.searchsorted(distances, target_distance_m, side="left"))
    row = trace.iloc[index]
    end_distance = float(row["cumulative_distance_m"])
    start_distance = end_distance - float(row["dx_m"])
    fraction = float(
        np.clip(
            (target_distance_m - start_distance)
            / max(end_distance - start_distance, 1e-12),
            0.0,
            1.0,
        )
    )
    end_time = float(row["elapsed_time_s"])
    start_time = end_time - float(row["time_in_segment_s"])
    start_soc = float(row["soc"])
    end_soc = float(row["next_soc"])
    end_terminal_wh = float(row["cumulative_terminal_energy_wh"])
    segment_terminal_wh = (
        float(row["battery_terminal_power_w"])
        * float(row["time_in_segment_s"])
        / 3600.0
    )
    start_terminal_wh = end_terminal_wh - segment_terminal_wh
    end_chemical_wh = float(row["cumulative_chemical_energy_wh"])
    segment_chemical_wh = (
        float(row["battery_ocv_v"])
        * float(row["battery_current_a"])
        * float(row["time_in_segment_s"])
        / 3600.0
    )
    start_chemical_wh = end_chemical_wh - segment_chemical_wh
    soc = start_soc + fraction * (end_soc - start_soc)
    return {
        "elapsed_time_s": start_time + fraction * (end_time - start_time),
        "soc": soc,
        "terminal_energy_kwh": (
            start_terminal_wh
            + fraction * (end_terminal_wh - start_terminal_wh)
        )
        / 1000.0,
        "chemical_energy_kwh": (
            start_chemical_wh
            + fraction * (end_chemical_wh - start_chemical_wh)
        )
        / 1000.0,
        "remaining_usable_chemical_kwh": pack.chemical_energy_wh(
            pack.minimum_soc, soc
        )
        / 1000.0,
        "minimum_terminal_voltage_v": float(
            trace.iloc[: index + 1]["battery_terminal_voltage_v"].min()
        ),
        "maximum_terminal_power_kw": float(
            trace.iloc[: index + 1]["battery_terminal_power_w"].max()
        )
        / 1000.0,
    }


def evaluate_case(
    *,
    vehicle_path: str,
    track_path: str,
    powertrain_path: str,
    parallel_count: int,
    power_limit_kw: float,
    target_distance_m: float,
    downsample_factor: int,
    surface_soc_count: int,
    surface_speed_count: int,
    include_trace: bool = False,
) -> dict[str, Any]:
    vehicle = load_vehicle(Path(vehicle_path))
    track = downsample_track(pd.read_csv(track_path), downsample_factor)
    base_config = load_powertrain_config(Path(powertrain_path))
    config = replace(
        base_config.with_parallel_cells(parallel_count),
        terminal_power_limit_w=power_limit_kw * 1000.0,
    )
    model = PowertrainModel(config)
    surface = MaximumTorqueSurface(
        model,
        maximum_vehicle_speed_mps=vehicle.v_max,
        soc_count=surface_soc_count,
        speed_count=surface_speed_count,
    )
    lap_length_m = float(track["dx_m"].sum())
    laps = int(math.ceil(target_distance_m / lap_length_m))
    trace, summary = simulate_endurance(
        vehicle,
        track,
        model,
        laps,
        initial_speed_mps=0.0,
        torque_surface=surface,
    )
    state = state_at_distance(trace, target_distance_m, config.pack)
    result: dict[str, Any] = {
        "parallel_count": parallel_count,
        "power_limit_kw": power_limit_kw,
        "completed_target_distance": state is not None,
        "vehicle_mass_kg": summary["vehicle_mass_kg"],
        "nominal_pack_energy_kwh": (
            config.pack.series_cells
            * config.pack.parallel_cells
            * config.pack.cell_capacity_ah
            * 3.6
            / 1000.0
        ),
        "model_full_charge_energy_kwh": config.pack.chemical_energy_wh(0.0, 1.0)
        / 1000.0,
        "model_usable_energy_kwh": config.pack.chemical_energy_wh(
            config.pack.minimum_soc, 1.0
        )
        / 1000.0,
        "lap_length_m": lap_length_m,
        "target_distance_m": target_distance_m,
    }
    if state is None:
        result.update(
            {
                "elapsed_time_s": math.nan,
                "soc": config.pack.minimum_soc,
                "terminal_energy_kwh": summary["terminal_energy_kwh"],
                "chemical_energy_kwh": summary["chemical_energy_kwh"],
                "remaining_usable_chemical_kwh": 0.0,
                "minimum_terminal_voltage_v": summary[
                    "minimum_terminal_voltage_v"
                ],
                "maximum_terminal_power_kw": (
                    summary["maximum_terminal_power_w"] / 1000.0
                ),
                "distance_reached_m": summary["distance_m"],
            }
        )
    else:
        result.update(state)
        result["distance_reached_m"] = target_distance_m
    if include_trace:
        result["_trace"] = trace
        result["_surface"] = surface.to_frame()
        result["_summary"] = summary
    return result


def search_one(payload: dict[str, Any]) -> dict[str, Any]:
    target_margin = float(payload["target_margin_kwh"])
    iterations = int(payload["search_iterations"])
    evaluation_args = {
        key: payload[key]
        for key in (
            "vehicle_path",
            "track_path",
            "powertrain_path",
            "parallel_count",
            "target_distance_m",
            "downsample_factor",
            "surface_soc_count",
            "surface_speed_count",
        )
    }
    cache: dict[float, dict[str, Any]] = {}

    def evaluate(limit_kw: float) -> dict[str, Any]:
        key = round(limit_kw, 9)
        if key not in cache:
            cache[key] = evaluate_case(
                **evaluation_args,
                power_limit_kw=limit_kw,
            )
        return cache[key]

    if "initial_low_kw" in payload and "initial_high_kw" in payload:
        grid = [
            float(payload["initial_low_kw"]),
            float(payload["initial_high_kw"]),
        ]
    else:
        grid = [
            4.0,
            8.0,
            12.0,
            16.0,
            20.0,
            25.0,
            30.0,
            40.0,
            60.0,
            80.0,
            100.0,
            120.0,
            160.0,
        ]
    values = [evaluate(grid[0])]
    bracket: tuple[float, float] | None = None
    for limit in grid[1:]:
        values.append(evaluate(limit))
        lower, upper = values[-2:]
        lower_error = (
            lower["remaining_usable_chemical_kwh"] - target_margin
        )
        upper_error = (
            upper["remaining_usable_chemical_kwh"] - target_margin
        )
        if lower_error >= 0.0 and upper_error <= 0.0:
            bracket = (lower["power_limit_kw"], upper["power_limit_kw"])
            break
    if bracket is None:
        best = min(
            values,
            key=lambda row: abs(
                row["remaining_usable_chemical_kwh"] - target_margin
            ),
        )
        best["search_status"] = "target_not_bracketed"
        best["search_evaluations"] = len(cache)
        return best

    low, high = bracket
    for _ in range(iterations):
        middle = 0.5 * (low + high)
        result = evaluate(middle)
        error = result["remaining_usable_chemical_kwh"] - target_margin
        if error > 0.0:
            low = middle
        else:
            high = middle
    best = min(
        (evaluate(low), evaluate(high)),
        key=lambda row: abs(
            row["remaining_usable_chemical_kwh"] - target_margin
        ),
    )
    best["search_status"] = "bracketed"
    best["search_evaluations"] = len(cache)
    return best


def run_parallel(function: Any, payloads: list[dict[str, Any]], workers: int) -> list[Any]:
    with ProcessPoolExecutor(max_workers=min(workers, len(payloads))) as executor:
        return list(executor.map(function, payloads))


def full_evaluate(payload: dict[str, Any]) -> dict[str, Any]:
    return evaluate_case(**payload)


def refine_existing(args: argparse.Namespace, output_root: Path) -> None:
    summary_path = output_root / "energy_margin_sweep_summary.csv"
    frame = pd.read_csv(summary_path)
    rows = frame.to_dict(orient="records")
    common = {
        "vehicle_path": str(args.vehicle.resolve()),
        "track_path": str(args.track.resolve()),
        "powertrain_path": str(args.powertrain.resolve()),
        "target_distance_m": args.distance_km * 1000.0,
        "surface_soc_count": args.surface_soc_count,
        "surface_speed_count": args.surface_speed_count,
    }
    correction_payloads: list[dict[str, Any]] = []
    for row in rows:
        error = float(row["remaining_usable_chemical_kwh"]) - args.margin_kwh
        limit = float(row["power_limit_kw"])
        saturated = float(row["maximum_terminal_power_kw"]) < 0.98 * limit
        if abs(error) <= 0.005 or saturated:
            continue
        delta = max(0.25, 0.02 * limit)
        lower = evaluate_case(
            **common,
            parallel_count=int(row["parallel_count"]),
            power_limit_kw=max(0.5, limit - delta),
            downsample_factor=args.search_downsample,
            include_trace=False,
        )
        upper = evaluate_case(
            **common,
            parallel_count=int(row["parallel_count"]),
            power_limit_kw=limit + delta,
            downsample_factor=args.search_downsample,
            include_trace=False,
        )
        slope = (
            float(upper["remaining_usable_chemical_kwh"])
            - float(lower["remaining_usable_chemical_kwh"])
        ) / (2.0 * delta)
        corrected_limit = limit - error / slope if abs(slope) > 1e-8 else limit
        correction_payloads.append(
            {
                **common,
                "parallel_count": int(row["parallel_count"]),
                "power_limit_kw": max(0.5, corrected_limit),
                "downsample_factor": 1,
                "include_trace": True,
            }
        )
    if not correction_payloads:
        print(frame.to_string(index=False))
        return
    replacements = run_parallel(
        full_evaluate, correction_payloads, args.workers
    )
    replacements_by_count: dict[int, dict[str, Any]] = {}
    for result in replacements:
        count = int(result["parallel_count"])
        trace = result.pop("_trace")
        surface = result.pop("_surface")
        summary = result.pop("_summary")
        trace.to_csv(output_root / f"130s{count}p_trace.csv", index=False)
        surface.to_csv(
            output_root / f"130s{count}p_maximum_torque_surface.csv",
            index=False,
        )
        (output_root / f"130s{count}p_solver_summary.json").write_text(
            json.dumps(summary, indent=2), encoding="utf-8"
        )
        replacements_by_count[count] = result
    merged = [
        replacements_by_count.get(int(row["parallel_count"]), row)
        for row in rows
    ]
    baseline = next(
        row
        for row in merged
        if int(row["parallel_count"]) == args.baseline_parallel_count
    )
    baseline_time = float(baseline["elapsed_time_s"])
    equivalent_laps = args.distance_km * 1000.0 / float(baseline["lap_length_m"])
    for row in merged:
        row["equivalent_average_lap_time_s"] = (
            float(row["elapsed_time_s"]) / equivalent_laps
        )
        row["time_change_vs_baseline_pct"] = (
            float(row["elapsed_time_s"]) / baseline_time - 1.0
        ) * 100.0
        row["margin_error_kwh"] = (
            float(row["remaining_usable_chemical_kwh"]) - args.margin_kwh
        )
    refined = pd.DataFrame(merged).sort_values("parallel_count")
    refined.to_csv(summary_path, index=False)
    print(refined.to_string(index=False))


def main() -> None:
    args = parse_args()
    if args.distance_km <= 0.0 or args.margin_kwh <= 0.0:
        raise ValueError("Distance and margin must be positive")
    output_root = args.output_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    if args.refine_existing:
        refine_existing(args, output_root)
        return
    common = {
        "vehicle_path": str(args.vehicle.resolve()),
        "track_path": str(args.track.resolve()),
        "powertrain_path": str(args.powertrain.resolve()),
        "target_distance_m": args.distance_km * 1000.0,
        "surface_soc_count": args.surface_soc_count,
        "surface_speed_count": args.surface_speed_count,
    }
    search_payloads = [
        {
            **common,
            "parallel_count": count,
            "downsample_factor": args.search_downsample,
            "target_margin_kwh": args.margin_kwh,
            "search_iterations": args.search_iterations,
        }
        for count in args.parallel_counts
    ]
    reused_results: list[dict[str, Any]] = []
    if args.reuse_coarse and (output_root / "coarse_search_results.csv").exists():
        previous = pd.read_csv(output_root / "coarse_search_results.csv")
        requested = set(args.parallel_counts)
        previous = previous[
            previous["parallel_count"].isin(requested)
            & (previous["search_status"] == "bracketed")
        ]
        reused_results = previous.to_dict(orient="records")
        reused_counts = {
            int(result["parallel_count"]) for result in reused_results
        }
        search_payloads = [
            payload
            for payload in search_payloads
            if int(payload["parallel_count"]) not in reused_counts
        ]
        prior_by_count = {
            int(row["parallel_count"]): row
            for row in pd.read_csv(
                output_root / "coarse_search_results.csv"
            ).to_dict(orient="records")
        }
        for payload in search_payloads:
            prior = prior_by_count.get(int(payload["parallel_count"]))
            if prior and prior["search_status"] == "target_not_bracketed":
                payload["initial_low_kw"] = float(prior["power_limit_kw"])
                payload["initial_high_kw"] = 160.0
    new_search_results = (
        run_parallel(search_one, search_payloads, args.workers)
        if search_payloads
        else []
    )
    search_results = sorted(
        reused_results + new_search_results,
        key=lambda result: int(result["parallel_count"]),
    )
    pd.DataFrame(search_results).to_csv(
        output_root / "coarse_search_results.csv", index=False
    )

    full_payloads = [
        {
            **common,
            "parallel_count": result["parallel_count"],
            "power_limit_kw": result["power_limit_kw"],
            "downsample_factor": 1,
            "include_trace": False,
        }
        for result in search_results
    ]
    full_results = run_parallel(full_evaluate, full_payloads, args.workers)

    # Correct the small downsampling bias with a secant slope measured around
    # each coarse optimum, then perform one final full-resolution pass.
    correction_payloads: list[dict[str, Any]] = []
    for search_result, full_result in zip(search_results, full_results):
        limit = float(search_result["power_limit_kw"])
        delta = max(0.25, 0.02 * limit)
        probe_payload = {
            **common,
            "parallel_count": int(search_result["parallel_count"]),
            "power_limit_kw": max(0.5, limit - delta),
            "downsample_factor": args.search_downsample,
            "include_trace": False,
        }
        coarse_probe = evaluate_case(**probe_payload)
        slope = (
            float(search_result["remaining_usable_chemical_kwh"])
            - float(coarse_probe["remaining_usable_chemical_kwh"])
        ) / (limit - float(probe_payload["power_limit_kw"]))
        full_error = (
            float(full_result["remaining_usable_chemical_kwh"])
            - args.margin_kwh
        )
        corrected_limit = (
            limit - full_error / slope if abs(slope) > 1e-8 else limit
        )
        correction_payloads.append(
            {
                **common,
                "parallel_count": int(search_result["parallel_count"]),
                "power_limit_kw": max(0.5, corrected_limit),
                "downsample_factor": 1,
                "include_trace": True,
            }
        )

    final_results_with_artifacts = run_parallel(
        full_evaluate, correction_payloads, args.workers
    )
    final_rows: list[dict[str, Any]] = []
    for result in final_results_with_artifacts:
        count = int(result["parallel_count"])
        trace = result.pop("_trace")
        surface = result.pop("_surface")
        summary = result.pop("_summary")
        trace.to_csv(output_root / f"130s{count}p_trace.csv", index=False)
        surface.to_csv(
            output_root / f"130s{count}p_maximum_torque_surface.csv",
            index=False,
        )
        (output_root / f"130s{count}p_solver_summary.json").write_text(
            json.dumps(summary, indent=2), encoding="utf-8"
        )
        final_rows.append(result)

    baseline = next(
        row
        for row in final_rows
        if int(row["parallel_count"]) == args.baseline_parallel_count
    )
    baseline_time = float(baseline["elapsed_time_s"])
    lap_length_m = float(baseline["lap_length_m"])
    equivalent_laps = args.distance_km * 1000.0 / lap_length_m
    for row in final_rows:
        row["equivalent_average_lap_time_s"] = (
            float(row["elapsed_time_s"]) / equivalent_laps
        )
        row["time_change_vs_baseline_pct"] = (
            float(row["elapsed_time_s"]) / baseline_time - 1.0
        ) * 100.0
        row["margin_error_kwh"] = (
            float(row["remaining_usable_chemical_kwh"]) - args.margin_kwh
        )
    final_frame = pd.DataFrame(final_rows).sort_values("parallel_count")
    final_frame.to_csv(output_root / "energy_margin_sweep_summary.csv", index=False)
    (output_root / "run_metadata.json").write_text(
        json.dumps(
            {
                "target_distance_km": args.distance_km,
                "target_remaining_usable_chemical_energy_kwh": args.margin_kwh,
                "baseline_parallel_count": args.baseline_parallel_count,
                "baseline_nominal_pack_energy_kwh": baseline[
                    "nominal_pack_energy_kwh"
                ],
                "equivalent_laps": equivalent_laps,
                "method": (
                    "Power limit tuned against remaining model chemical energy "
                    "above 5% minimum SOC at exactly 22 km. Final results use "
                    "the full 0.25 m track mesh."
                ),
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    print(final_frame.to_string(index=False))


if __name__ == "__main__":
    main()
