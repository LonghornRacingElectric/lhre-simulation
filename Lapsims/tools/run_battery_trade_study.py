"""Run the 22 km OpenLAP battery-cell and pack-architecture trade study."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from typing import Any, Callable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from battery_trade_study import (  # noqa: E402
    BASELINE_CELL_ENVELOPE_M3,
    BASELINE_PARALLEL_CELLS,
    BASELINE_SERIES_CELLS,
    CELL_SPECIFIC_HEAT_J_PER_KGK,
    MAXIMUM_PACK_VOLTAGE_V,
    MAXIMUM_USABLE_ENERGY_KWH,
    aggregate_exact_distance,
    build_powertrain_config,
    downsample_track,
    fit_linear_sensitivities,
    generate_candidates,
    pareto_frontier_mask,
)
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
        "--cells",
        type=Path,
        default=ROOT / "inputs" / "battery_trade_study" / "cells.json",
    )
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
    parser.add_argument(
        "--output-root",
        type=Path,
        default=ROOT / "outputs" / "battery_trade_study_20260725",
    )
    parser.add_argument("--distance-km", type=float, default=22.0)
    parser.add_argument("--reserve-kwh", type=float, default=0.5)
    parser.add_argument("--reserve-tolerance-kwh", type=float, default=0.01)
    parser.add_argument("--power-cap-kw", type=float, default=80.0)
    parser.add_argument("--series-min", type=int, default=110)
    parser.add_argument("--series-max", type=int, default=140)
    parser.add_argument("--series-step", type=int, default=5)
    parser.add_argument("--search-downsample", type=int, default=8)
    parser.add_argument("--search-soc-count", type=int, default=7)
    parser.add_argument("--search-speed-count", type=int, default=25)
    parser.add_argument("--search-iterations", type=int, default=7)
    parser.add_argument("--refine-soc-count", type=int, default=9)
    parser.add_argument("--refine-speed-count", type=int, default=41)
    parser.add_argument(
        "--workers",
        type=int,
        default=max(1, min(12, (os.cpu_count() or 4) - 1)),
    )
    parser.add_argument("--no-refine", action="store_true")
    parser.add_argument(
        "--cell-id",
        action="append",
        help="Limit the run to one or more cell IDs; useful for smoke checks.",
    )
    parser.add_argument(
        "--series-count",
        type=int,
        action="append",
        help="Override the generated series-count set.",
    )
    parser.add_argument(
        "--regen-policy",
        type=Path,
        help=(
            "Measured endurance-regen policy JSON. Omit to preserve the "
            "mechanical-braking-only baseline."
        ),
    )
    parser.add_argument(
        "--regen-command-power-kw",
        type=float,
        help=(
            "Frozen instantaneous terminal charge command calibrated from "
            "the policy's active RMS; required when --regen-policy is used."
        ),
    )
    return parser.parse_args()


def _base_result(
    candidate: dict[str, Any],
    cell: dict[str, Any],
    config: Any,
    vehicle_mass_kg: float,
    power_limit_kw: float,
    fidelity: str,
    regen_command_power_kw: float,
    regen_active_rms_target_kw: float,
    regen_active_power_threshold_kw: float,
) -> dict[str, Any]:
    return {
        **candidate,
        "cell_nominal_voltage_v": float(cell["nominal_voltage_v"]),
        "cell_maximum_voltage_v": float(cell["maximum_voltage_v"]),
        "cell_minimum_voltage_v": float(cell["minimum_voltage_v"]),
        "cell_pulse_current_a": cell.get("pulse_current_a"),
        "cell_pulse_duration_s": cell.get("pulse_duration_s"),
        "cell_document_status": cell["document_status"],
        "power_limit_kw": float(power_limit_kw),
        "vehicle_mass_kg": vehicle_mass_kg,
        "pack_resistance_initial_mohm": (
            config.pack.pack_resistance_ohm(config.pack.initial_soc) * 1000.0
        ),
        "simulation_fidelity": fidelity,
        "regen_endurance_only": bool(regen_command_power_kw > 0.0),
        "regen_command_power_kw": float(regen_command_power_kw),
        "regen_active_rms_target_kw": float(regen_active_rms_target_kw),
        "regen_active_power_threshold_kw": float(
            regen_active_power_threshold_kw
        ),
    }


def _evaluate(
    *,
    candidate: dict[str, Any],
    cell: dict[str, Any],
    vehicle: Any,
    track: pd.DataFrame,
    base_config: Any,
    power_limit_kw: float,
    target_distance_m: float,
    surface_soc_count: int,
    surface_speed_count: int,
    surface_verification_iterations: int,
    fidelity: str,
    regen_command_power_kw: float = 0.0,
    regen_active_rms_target_kw: float = 0.0,
    regen_active_power_threshold_kw: float = 1.0,
) -> dict[str, Any]:
    config = build_powertrain_config(
        base_config,
        cell,
        int(candidate["series_cells"]),
        int(candidate["parallel_cells"]),
        power_limit_kw,
    )
    vehicle_mass = vehicle.mass + config.pack.vehicle_mass_delta_kg()
    row = _base_result(
        candidate,
        cell,
        config,
        vehicle_mass,
        power_limit_kw,
        fidelity,
        regen_command_power_kw,
        regen_active_rms_target_kw,
        regen_active_power_threshold_kw,
    )
    model = PowertrainModel(config)
    surface = MaximumTorqueSurface(
        model,
        maximum_vehicle_speed_mps=vehicle.v_max,
        soc_count=surface_soc_count,
        speed_count=surface_speed_count,
        verification_iterations=surface_verification_iterations,
    )
    lap_length = float(track["dx_m"].sum())
    requested_laps = int(math.ceil(target_distance_m / lap_length))
    try:
        trace, summary = simulate_endurance(
            vehicle,
            track,
            model,
            requested_laps,
            initial_speed_mps=0.0,
            torque_surface=surface,
            regen_terminal_power_target_w=regen_command_power_kw * 1000.0,
            regen_active_power_threshold_w=(
                regen_active_power_threshold_kw * 1000.0
            ),
        )
    except (RuntimeError, ValueError) as exc:
        return {
            **row,
            "completed_target_distance": False,
            "distance_reached_m": 0.0,
            "elapsed_time_s": math.nan,
            "remaining_usable_chemical_kwh": 0.0,
            "evaluation_error": str(exc),
        }
    if float(trace["cumulative_distance_m"].iloc[-1]) < target_distance_m - 1e-9:
        return {
            **row,
            "completed_target_distance": False,
            "distance_reached_m": float(
                trace["cumulative_distance_m"].iloc[-1]
            ),
            "elapsed_time_s": math.nan,
            "remaining_usable_chemical_kwh": 0.0,
            "evaluation_error": summary["stop_reason"],
        }
    _, metrics = aggregate_exact_distance(
        trace,
        target_distance_m,
        lap_length,
        config,
        regen_active_power_threshold_kw=regen_active_power_threshold_kw,
    )
    return {
        **row,
        **metrics,
        "distance_reached_m": target_distance_m,
        "maximum_achieved_terminal_power_kw": metrics[
            "peak_battery_terminal_power_kw"
        ],
        "pack_current_limit_respected": (
            metrics["peak_pack_current_a"]
            <= config.pack.maximum_discharge_current_a + 1e-5
        ),
        "power_limit_respected": (
            metrics["peak_battery_terminal_power_kw"]
            <= power_limit_kw + 1e-5
        ),
        "regen_active_rms_error_kw": (
            metrics["regen_active_rms_terminal_power_kw"]
            - regen_active_rms_target_kw
        ),
        "solver_energy_balance_relative": summary["checks"][
            "pack_energy_balance_relative"
        ],
    }


def _expand_lower_power_bracket(
    evaluate: Callable[[float], dict[str, Any]],
    cache: dict[float, dict[str, Any]],
    target_reserve: float,
) -> tuple[dict[str, Any] | None, tuple[float, float] | None]:
    """Find a lower-power reserve bracket after a centered grid misses it."""

    completed_negative = sorted(
        (
            row
            for row in cache.values()
            if row.get("completed_target_distance", False)
            and float(row["remaining_usable_chemical_kwh"])
            < target_reserve
        ),
        key=lambda row: float(row["power_limit_kw"]),
    )
    if not completed_negative:
        return None, None

    high = float(completed_negative[0]["power_limit_kw"])
    probe = max(1.0, 0.75 * high)
    while probe < high - 1e-9:
        result = evaluate(probe)
        if result.get("completed_target_distance", False):
            error = (
                float(result["remaining_usable_chemical_kwh"])
                - target_reserve
            )
            if error >= 0.0:
                return result, (float(result["power_limit_kw"]), high)
            high = float(result["power_limit_kw"])
        if probe <= 1.0 + 1e-9:
            break
        probe = max(1.0, 0.75 * probe)
    return None, None


def search_candidate(payload: dict[str, Any]) -> dict[str, Any]:
    candidate = payload["candidate"]
    cell = payload["cell"]
    vehicle = load_vehicle(Path(payload["vehicle_path"]))
    raw_track = pd.read_csv(payload["track_path"])
    track = downsample_track(raw_track, int(payload["downsample_factor"]))
    base_config = load_powertrain_config(Path(payload["powertrain_path"]))
    target_distance = float(payload["target_distance_m"])
    target_reserve = float(payload["target_reserve_kwh"])
    tolerance = float(payload["reserve_tolerance_kwh"])
    cap = float(payload["power_cap_kw"])
    fidelity = str(payload["fidelity"])
    cache: dict[float, dict[str, Any]] = {}

    def evaluate(power_kw: float) -> dict[str, Any]:
        limited = float(np.clip(power_kw, 1.0, cap))
        key = round(limited, 8)
        if key not in cache:
            cache[key] = _evaluate(
                candidate=candidate,
                cell=cell,
                vehicle=vehicle,
                track=track,
                base_config=base_config,
                power_limit_kw=limited,
                target_distance_m=target_distance,
                surface_soc_count=int(payload["surface_soc_count"]),
                surface_speed_count=int(payload["surface_speed_count"]),
                surface_verification_iterations=int(
                    payload["surface_verification_iterations"]
                ),
                fidelity=fidelity,
                regen_command_power_kw=float(
                    payload.get("regen_command_power_kw", 0.0)
                ),
                regen_active_rms_target_kw=float(
                    payload.get("regen_active_rms_target_kw", 0.0)
                ),
                regen_active_power_threshold_kw=float(
                    payload.get("regen_active_power_threshold_kw", 1.0)
                ),
            )
        return cache[key]

    requested_grid = payload.get(
        "search_grid_kw", [5.0, 8.0, 12.0, 18.0, 27.0, 40.0, 60.0, cap]
    )
    grid = sorted(
        {
            round(float(np.clip(value, 1.0, cap)), 8)
            for value in requested_grid
        }
    )
    if cap not in grid:
        grid.append(cap)
    positive: dict[str, Any] | None = None
    bracket: tuple[float, float] | None = None
    for power in grid:
        result = evaluate(power)
        if not result.get("completed_target_distance", False):
            if positive is not None:
                bracket = (float(positive["power_limit_kw"]), power)
                break
            continue
        error = float(result["remaining_usable_chemical_kwh"]) - target_reserve
        if error >= 0.0:
            positive = result
        elif positive is not None:
            bracket = (float(positive["power_limit_kw"]), power)
            break

    # A coarse-mesh result can sit at the power cap while a native-mesh
    # refinement consumes enough additional energy that every point in the
    # centered refinement grid falls below the reserve target.  In that case
    # expand downward geometrically until a reserve-positive point is found.
    # Without this fallback, an otherwise feasible high-energy pack can be
    # mislabeled ``target_not_bracketed`` solely because the refinement grid
    # never sampled a sufficiently low power.
    if bracket is None and positive is None:
        positive, bracket = _expand_lower_power_bracket(
            evaluate,
            cache,
            target_reserve,
        )

    if bracket is not None:
        low, high = bracket
        for _ in range(int(payload["search_iterations"])):
            middle = 0.5 * (low + high)
            result = evaluate(middle)
            if not result.get("completed_target_distance", False):
                high = middle
                continue
            error = (
                float(result["remaining_usable_chemical_kwh"])
                - target_reserve
            )
            if abs(error) <= tolerance / 2.0:
                break
            if error >= 0.0:
                low = middle
            else:
                high = middle
        completed = [
            row
            for row in cache.values()
            if row.get("completed_target_distance", False)
        ]
        best = min(
            completed,
            key=lambda row: abs(
                float(row["remaining_usable_chemical_kwh"])
                - target_reserve
            ),
        )
        reserve_error = (
            float(best["remaining_usable_chemical_kwh"]) - target_reserve
        )
        status = (
            "converged"
            if abs(reserve_error) <= tolerance
            else "target_not_converged"
        )
    else:
        completed = [
            row
            for row in cache.values()
            if row.get("completed_target_distance", False)
        ]
        if not completed:
            best = evaluate(cap)
            reserve_error = -target_reserve
            status = "cannot_complete"
        else:
            best = min(
                completed,
                key=lambda row: abs(
                    float(row["remaining_usable_chemical_kwh"])
                    - target_reserve
                ),
            )
            cap_result = evaluate(cap)
            cap_reserve = (
                float(cap_result["remaining_usable_chemical_kwh"])
                if cap_result.get("completed_target_distance", False)
                else -math.inf
            )
            reserve_error = (
                float(best["remaining_usable_chemical_kwh"])
                - target_reserve
            )
            if cap_reserve > target_reserve + tolerance:
                best = cap_result
                reserve_error = cap_reserve - target_reserve
                status = "power_cap_extra_reserve"
            else:
                status = "target_not_bracketed"

    constraint_valid = bool(
        best.get("completed_target_distance", False)
        and float(best.get("remaining_usable_chemical_kwh", 0.0))
        >= target_reserve - tolerance
        and float(best["power_limit_kw"]) <= cap + 1e-9
        and status in {"converged", "power_cap_extra_reserve"}
    )
    equal_reserve = bool(
        best.get("completed_target_distance", False)
        and abs(reserve_error) <= tolerance
    )
    return {
        **best,
        "search_status": status,
        "search_evaluations": len(cache),
        "reserve_target_kwh": target_reserve,
        "reserve_error_kwh": reserve_error,
        "constraint_valid": constraint_valid,
        "equal_reserve_eligible": equal_reserve,
    }


def _run_parallel(
    payloads: list[dict[str, Any]], workers: int
) -> list[dict[str, Any]]:
    if not payloads:
        return []
    with ProcessPoolExecutor(max_workers=min(workers, len(payloads))) as pool:
        return list(pool.map(search_candidate, payloads))


def _build_payload(
    args: argparse.Namespace,
    candidate: dict[str, Any],
    cell: dict[str, Any],
    *,
    downsample_factor: int,
    surface_soc_count: int,
    surface_speed_count: int,
    surface_verification_iterations: int,
    fidelity: str,
    search_grid_kw: list[float] | None = None,
) -> dict[str, Any]:
    payload = {
        "candidate": candidate,
        "cell": cell,
        "vehicle_path": str(args.vehicle.resolve()),
        "track_path": str(args.track.resolve()),
        "powertrain_path": str(args.powertrain.resolve()),
        "target_distance_m": args.distance_km * 1000.0,
        "target_reserve_kwh": args.reserve_kwh,
        "reserve_tolerance_kwh": args.reserve_tolerance_kwh,
        "power_cap_kw": args.power_cap_kw,
        "downsample_factor": downsample_factor,
        "surface_soc_count": surface_soc_count,
        "surface_speed_count": surface_speed_count,
        "surface_verification_iterations": surface_verification_iterations,
        "search_iterations": args.search_iterations,
        "fidelity": fidelity,
        "regen_command_power_kw": float(
            getattr(args, "regen_command_power_kw_resolved", 0.0)
        ),
        "regen_active_rms_target_kw": float(
            getattr(args, "regen_active_rms_target_kw_resolved", 0.0)
        ),
        "regen_active_power_threshold_kw": float(
            getattr(args, "regen_active_power_threshold_kw_resolved", 1.0)
        ),
    }
    if search_grid_kw is not None:
        payload["search_grid_kw"] = search_grid_kw
    return payload


def _select_refinement_ids(frame: pd.DataFrame) -> set[str]:
    valid = frame[frame["constraint_valid"]].copy()
    selected: set[str] = {
        f"molicel_p30b_{BASELINE_SERIES_CELLS}s"
        f"{BASELINE_PARALLEL_CELLS}p"
    }
    if valid.empty:
        return selected
    for _, group in valid.groupby("cell_id"):
        selected.add(str(group.sort_values("elapsed_time_s").iloc[0]["candidate_id"]))
    pareto = valid[pareto_frontier_mask(valid)]
    selected.update(pareto.nsmallest(8, "elapsed_time_s")["candidate_id"])
    return selected


def _centered_refine_grid(power_kw: float, cap_kw: float) -> list[float]:
    return sorted(
        {
            max(2.0, power_kw - 8.0),
            max(2.0, power_kw - 3.0),
            power_kw,
            min(cap_kw, power_kw + 3.0),
            min(cap_kw, power_kw + 8.0),
            cap_kw,
        }
    )


def _add_comparisons(frame: pd.DataFrame) -> tuple[pd.DataFrame, dict[str, Any]]:
    output = frame.copy()
    baseline_id = (
        f"molicel_p30b_{BASELINE_SERIES_CELLS}s"
        f"{BASELINE_PARALLEL_CELLS}p"
    )
    baseline_rows = output[
        (output["candidate_id"] == baseline_id)
        & output["completed_target_distance"].fillna(False)
    ]
    if baseline_rows.empty:
        raise RuntimeError("The 130s5p P30B baseline did not complete 22 km")
    baseline = baseline_rows.sort_values(
        ["simulation_fidelity", "elapsed_time_s"]
    ).iloc[-1]
    baseline_time = float(baseline["elapsed_time_s"])
    output["time_difference_vs_baseline_s"] = (
        output["elapsed_time_s"] - baseline_time
    )
    output["improvement_vs_baseline_pct"] = (
        (baseline_time - output["elapsed_time_s"]) / baseline_time * 100.0
    )
    output["performance_per_pack_kg"] = (
        1000.0 / (output["elapsed_time_s"] * output["pack_mass_kg"])
    )
    output["overall_rank"] = np.nan
    valid_indices = output[output["constraint_valid"]].sort_values(
        "elapsed_time_s"
    ).index
    output.loc[valid_indices, "overall_rank"] = np.arange(
        1, len(valid_indices) + 1
    )
    output["cell_rank"] = np.nan
    for _, group in output[output["constraint_valid"]].groupby("cell_id"):
        indices = group.sort_values("elapsed_time_s").index
        output.loc[indices, "cell_rank"] = np.arange(1, len(indices) + 1)
    output["pareto_frontier"] = False
    valid = output[output["constraint_valid"]]
    if not valid.empty:
        output.loc[valid.index, "pareto_frontier"] = pareto_frontier_mask(valid)
    return output, baseline.to_dict()


def _best_rows(frame: pd.DataFrame) -> dict[str, dict[str, Any]]:
    valid = frame[frame["constraint_valid"]].sort_values("elapsed_time_s")
    if valid.empty:
        return {}
    best_time = float(valid.iloc[0]["elapsed_time_s"])
    near_one_pct = valid[
        valid["elapsed_time_s"] <= best_time * 1.01
    ].sort_values("pack_mass_kg")
    near_two_pct = valid[
        valid["elapsed_time_s"] <= best_time * 1.02
    ].sort_values("pack_resistive_heat_kwh")
    return {
        "best_overall": valid.iloc[0].to_dict(),
        "best_lightweight": near_one_pct.iloc[0].to_dict(),
        "best_thermal": near_two_pct.iloc[0].to_dict(),
        "best_value": valid.sort_values(
            "performance_per_pack_kg", ascending=False
        ).iloc[0].to_dict(),
    }


def _local_sensitivity_payloads(
    args: argparse.Namespace,
    best: dict[str, Any],
    cells_by_id: dict[str, dict[str, Any]],
) -> list[dict[str, Any]]:
    base_cell = cells_by_id[str(best["cell_id"])]
    candidate = {
        key: best[key]
        for key in (
            "candidate_id",
            "cell_id",
            "manufacturer",
            "cell_model",
            "form_factor",
            "series_cells",
            "parallel_cells",
            "total_cells",
            "maximum_pack_voltage_v",
            "nominal_pack_voltage_v",
            "nominal_pack_energy_kwh",
            "model_usable_energy_kwh",
            "cell_envelope_l",
            "cell_envelope_vs_baseline_pct",
            "cell_mass_kg",
            "cell_capacity_ah",
            "cell_dcir_mohm",
            "cell_continuous_current_a",
            "pack_current_limit_a",
            "pack_mass_kg",
            "source_url",
            "dcir_basis",
        )
    }
    variants: list[tuple[str, dict[str, Any], dict[str, Any]]] = []
    for name, scale in (("capacity_minus_5pct", 0.95), ("capacity_plus_5pct", 1.05)):
        cell = {**base_cell, "capacity_ah": float(base_cell["capacity_ah"]) * scale}
        changed = {
            **candidate,
            "candidate_id": f"{candidate['candidate_id']}__{name}",
            "cell_capacity_ah": cell["capacity_ah"],
            "nominal_pack_energy_kwh": float(
                candidate["nominal_pack_energy_kwh"]
            )
            * scale,
            "model_usable_energy_kwh": float(
                candidate["model_usable_energy_kwh"]
            )
            * scale,
        }
        variants.append((name, cell, changed))
    for name, scale in (("dcir_minus_20pct", 0.8), ("dcir_plus_20pct", 1.2)):
        cell = {**base_cell, "dcir_ohm": float(base_cell["dcir_ohm"]) * scale}
        changed = {
            **candidate,
            "candidate_id": f"{candidate['candidate_id']}__{name}",
            "cell_dcir_mohm": float(candidate["cell_dcir_mohm"]) * scale,
            "dcir_basis": f"Sensitivity case: {scale:.2f} x base DCIR",
        }
        variants.append((name, cell, changed))
    total_cells = int(candidate["total_cells"])
    for name, delta_pack_mass in (("mass_minus_5kg", -5.0), ("mass_plus_5kg", 5.0)):
        delta_cell_mass = delta_pack_mass / (total_cells * 1.25)
        cell = {
            **base_cell,
            "mass_kg": float(base_cell["mass_kg"]) + delta_cell_mass,
        }
        changed = {
            **candidate,
            "candidate_id": f"{candidate['candidate_id']}__{name}",
            "cell_mass_kg": cell["mass_kg"],
            "pack_mass_kg": float(candidate["pack_mass_kg"]) + delta_pack_mass,
        }
        variants.append((name, cell, changed))
    payloads = []
    base_power = float(best["power_limit_kw"])
    for case_name, cell, changed_candidate in variants:
        payload = _build_payload(
            args,
            changed_candidate,
            cell,
            downsample_factor=args.search_downsample,
            surface_soc_count=args.search_soc_count,
            surface_speed_count=args.search_speed_count,
            surface_verification_iterations=2,
            fidelity=f"screening_sensitivity_factor{args.search_downsample}",
            search_grid_kw=_centered_refine_grid(base_power, args.power_cap_kw),
        )
        payload["sensitivity_case"] = case_name
        payloads.append(payload)
    return payloads


def _calculate_local_sensitivities(
    best: dict[str, Any], cases: pd.DataFrame
) -> dict[str, float]:
    case_map = {
        str(row["sensitivity_case"]): row
        for _, row in cases.iterrows()
    }

    def centered(
        lower_name: str, upper_name: str, x_column: str
    ) -> float:
        lower = case_map[lower_name]
        upper = case_map[upper_name]
        return (
            float(upper["elapsed_time_s"]) - float(lower["elapsed_time_s"])
        ) / (float(upper[x_column]) - float(lower[x_column]))

    return {
        "reference_candidate_id": str(best["candidate_id"]),
        "seconds_per_usable_kwh_local": centered(
            "capacity_minus_5pct",
            "capacity_plus_5pct",
            "model_usable_energy_kwh",
        ),
        "seconds_per_pack_kg_local": centered(
            "mass_minus_5kg", "mass_plus_5kg", "pack_mass_kg"
        ),
        "seconds_per_pack_mohm_local": centered(
            "dcir_minus_20pct",
            "dcir_plus_20pct",
            "pack_resistance_initial_mohm",
        ),
        "seconds_per_cell_dcir_mohm_local": centered(
            "dcir_minus_20pct",
            "dcir_plus_20pct",
            "cell_dcir_mohm",
        ),
    }


def _save_plots(frame: pd.DataFrame, output_root: Path) -> list[Path]:
    plot_root = output_root / "plots"
    plot_root.mkdir(parents=True, exist_ok=True)
    valid = frame[frame["constraint_valid"]].copy()
    paths: list[Path] = []
    if valid.empty:
        return paths
    plots = [
        ("pack_mass_kg", "Pack mass (kg)", "time_vs_pack_mass.png"),
        (
            "model_usable_energy_kwh",
            "Modeled usable energy (kWh)",
            "time_vs_usable_energy.png",
        ),
        (
            "pack_resistance_initial_mohm",
            "Initial pack resistance (mOhm)",
            "time_vs_pack_resistance.png",
        ),
        (
            "pack_resistive_heat_kwh",
            "Pack resistive heat (kWh)",
            "time_vs_pack_heat.png",
        ),
    ]
    for x_column, x_label, filename in plots:
        figure, axis = plt.subplots(figsize=(8.0, 5.0))
        for cell_id, group in valid.groupby("cell_id"):
            axis.scatter(
                group[x_column],
                group["elapsed_time_s"] / 60.0,
                s=28,
                alpha=0.78,
                label=cell_id,
            )
        pareto = valid[valid["pareto_frontier"]]
        if not pareto.empty:
            axis.scatter(
                pareto[x_column],
                pareto["elapsed_time_s"] / 60.0,
                s=90,
                facecolors="none",
                edgecolors="black",
                linewidths=1.3,
                label="Pareto",
            )
        axis.set_xlabel(x_label)
        axis.set_ylabel("22 km endurance time (min)")
        axis.grid(True, alpha=0.25)
        axis.legend(fontsize=7, ncol=2)
        figure.tight_layout()
        path = plot_root / filename
        figure.savefig(path, dpi=180)
        plt.close(figure)
        paths.append(path)
    return paths


def _format_configuration(row: dict[str, Any]) -> str:
    return (
        f"{row['manufacturer']} {row['cell_model']} "
        f"{int(row['series_cells'])}s{int(row['parallel_cells'])}p"
    )


def _write_report(
    output_root: Path,
    final: pd.DataFrame,
    baseline: dict[str, Any],
    best_rows: dict[str, dict[str, Any]],
    global_sensitivity: dict[str, float],
    local_sensitivity: dict[str, float],
    cell_document: dict[str, Any],
    screening_count: int,
    rejected_count: int,
    screening_downsample_factor: int,
    regen_settings: dict[str, Any],
) -> Path:
    best = best_rows["best_overall"]
    lines = [
        "# OpenLAP Battery Cell and Pack Trade Study",
        "",
        f"Run date: 2026-07-25<br>",
        f"Event distance: 22.000 km<br>",
        f"Vehicle/track: fixed LHRe OpenLAP Michigan endurance model<br>",
        (
            f"Configurations simulated: {screening_count} feasible topologies "
            "within the stated study grid<br>"
        ),
        f"Topologies rejected before simulation: {rejected_count}",
        "",
        "## Executive result",
        "",
        (
            f"The fastest constraint-valid result is **{_format_configuration(best)}** "
            f"at **{best['elapsed_time_s']:.2f} s** ({best['elapsed_time_s']/60.0:.2f} min), "
            f"using a **{best['power_limit_kw']:.2f} kW** terminal-power ceiling and "
            f"finishing with **{best['remaining_usable_chemical_kwh']:.3f} kWh** "
            f"of modeled usable chemical energy."
        ),
        (
            f"Against the team 7.2 kWh-class 130s5p P30B baseline "
            f"({baseline['elapsed_time_s']:.2f} s), this is "
            f"**{best['improvement_vs_baseline_pct']:.3f}% faster** "
            f"({-best['time_difference_vs_baseline_s']:.2f} s saved)."
        ),
        (
            f"The requested 7.2 kWh baseline is instantiated from the actual "
            f"130s5p P30B cell data as **"
            f"{baseline['nominal_pack_energy_kwh']:.3f} kWh nominal** and "
            f"**{baseline['model_usable_energy_kwh']:.3f} kWh modeled usable** "
            "from 100% to the 5% SOC floor."
        ),
    ]
    if regen_settings["enabled"]:
        lines.extend(
            [
                "",
                (
                    "Regeneration is enabled in endurance only. The fixed "
                    f"{regen_settings['frozen_command_power_kw']:.3f} kW "
                    "terminal command was calibrated on the native P30B "
                    "baseline to the measured "
                    f"{regen_settings['active_rms_target_kw']:.3f} kW "
                    "active-period RMS and is held constant across packs."
                ),
            ]
        )
    if "DRAFT" in str(best["cell_document_status"]).upper():
        lines.extend(
            [
                "",
                (
                    "**Decision caution:** the raw pace winner is based on "
                    "Tenpower's V0.1 draft specification and an ACIR-derived "
                    "DCIR estimate. "
                    "Do not freeze the design around it until pulse resistance, "
                    "capacity, and thermal behavior are reproduced on the team "
                    "fixture."
                ),
            ]
        )
    if str(best["cell_id"]) == "tenpower_60xg":
        reliance = final[
            (final["cell_id"] == "reliance_rs50")
            & (final["simulation_fidelity"] == "native_0p25m")
            & final["constraint_valid"]
        ].sort_values("elapsed_time_s").iloc[0].to_dict()
        published_dcir = final[
            (final["cell_id"] == "molicel_p50b")
            & (final["simulation_fidelity"] == "native_0p25m")
            & final["constraint_valid"]
        ].sort_values("elapsed_time_s").iloc[0].to_dict()
        lines.extend(
            [
                "",
                (
                    f"The winner is **{best['nominal_pack_energy_kwh']:.3f} kWh "
                    "nominal** but **"
                    f"{best['model_usable_energy_kwh']:.3f} kWh modeled usable** "
                    "from 100% to the 5% SOC floor. It passes this study's literal "
                    "8.0 kWh *usable-energy* constraint; if Battery intended an "
                    "8.0 kWh nominal/nameplate cap, exclude it and use "
                    f"**{_format_configuration(reliance)}** as the fastest "
                    f"remaining native result ({reliance['elapsed_time_s']:.2f} s)."
                ),
                (
                    f"The winner's **{best['adiabatic_cell_temperature_rise_c']:.1f} "
                    "degC** adiabatic rise would imply about "
                    f"**{25.0 + best['adiabatic_cell_temperature_rise_c']:.0f} "
                    "degC** from a 25 degC start, above the draft specification's "
                    "80 degC cell-surface discharge range. This is an uncooled "
                    "upper bound, not a cooled peak prediction, but the model has "
                    "no thermal feedback or derating; cooling validation is "
                    "mandatory and the ranking can change."
                ),
                (
                    f"The better-documented, non-draft A0 Reliance result is only "
                    f"{reliance['elapsed_time_s'] - best['elapsed_time_s']:.2f} s "
                    "slower and produces "
                    f"{best['pack_resistive_heat_kwh'] - reliance['pack_resistive_heat_kwh']:.4f} "
                    "kWh less modeled heat. Its DCIR is still ACIR-derived. "
                    f"The fastest candidate with manufacturer-published DCIR is "
                    f"**{_format_configuration(published_dcir)}** at "
                    f"**{published_dcir['elapsed_time_s']:.2f} s**."
                ),
            ]
        )
    lines.extend(
        [
        "",
        "## Fastest-configuration metrics",
        "",
        "| Metric | Result |",
        "|---|---:|",
        (
            f"| Cell count | {int(best['total_cells'])} "
            f"({int(best['series_cells'])}s{int(best['parallel_cells'])}p) |"
        ),
        f"| Maximum charged voltage | {best['maximum_pack_voltage_v']:.1f} V |",
        (
            f"| Nominal / modeled usable energy | "
            f"{best['nominal_pack_energy_kwh']:.3f} / "
            f"{best['model_usable_energy_kwh']:.3f} kWh |"
        ),
        (
            f"| Pack / vehicle mass | {best['pack_mass_kg']:.3f} / "
            f"{best['vehicle_mass_kg']:.3f} kg |"
        ),
        (
            f"| Total endurance / equivalent average lap | "
            f"{best['elapsed_time_s']:.3f} / "
            f"{best['equivalent_average_lap_time_s']:.3f} s |"
        ),
        (
            f"| Fastest / slowest completed lap | "
            f"{best['fastest_completed_lap_time_s']:.3f} / "
            f"{best['slowest_completed_lap_time_s']:.3f} s |"
        ),
        (
            f"| Terminal power, average / peak | "
            f"{best['average_battery_terminal_power_kw']:.3f} / "
            f"{best['peak_battery_terminal_power_kw']:.3f} kW |"
        ),
        (
            f"| Motor-shaft power, average / peak | "
            f"{best['average_motor_shaft_power_kw']:.3f} / "
            f"{best['peak_motor_shaft_power_kw']:.3f} kW |"
        ),
        (
            f"| Cell current, RMS / peak | {best['rms_cell_current_a']:.3f} / "
            f"{best['peak_cell_current_a']:.3f} A |"
        ),
        f"| Peak pack current | {best['peak_pack_current_a']:.3f} A |",
        (
            f"| Terminal / chemical / mechanical energy | "
            f"{best['terminal_energy_kwh']:.3f} / "
            f"{best['chemical_energy_kwh']:.3f} / "
            f"{best['mechanical_energy_kwh']:.3f} kWh |"
        ),
        (
            f"| Gross discharge / recovered / net terminal energy | "
            f"{best['terminal_discharge_energy_kwh']:.3f} / "
            f"{best['terminal_regenerated_energy_kwh']:.3f} / "
            f"{best['terminal_energy_kwh']:.3f} kWh |"
        ),
        (
            f"| Regen active / whole-event RMS | "
            f"{best['regen_active_rms_terminal_power_kw']:.3f} / "
            f"{best['regen_whole_event_rms_terminal_power_kw']:.3f} kW |"
        ),
        (
            f"| Total / regen pack resistive heat | "
            f"{best['pack_resistive_heat_kwh']:.4f} / "
            f"{best['regen_pack_resistive_heat_kwh']:.4f} kWh |"
        ),
        f"| Pack resistive heat | {best['pack_resistive_heat_kwh']:.4f} kWh |",
        (
            f"| Battery discharge efficiency | "
            f"{best['battery_discharge_efficiency_pct']:.3f}% |"
        ),
        (
            f"| Adiabatic cell temperature rise | "
            f"{best['adiabatic_cell_temperature_rise_c']:.1f} degC |"
        ),
        (
            f"| Remaining modeled usable energy | "
            f"{best['remaining_usable_chemical_kwh']:.3f} kWh |"
        ),
        "",
        "## Recommended configurations",
        "",
        "| Category | Configuration | Time (s) | Pack mass (kg) | Heat (kWh) | Reserve (kWh) |",
        "|---|---|---:|---:|---:|---:|",
        ]
    )
    labels = {
        "best_overall": "Best overall",
        "best_lightweight": "Best lightweight within 1% of fastest",
        "best_thermal": "Lowest heat within 2% of fastest",
        "best_value": "Best simulated performance per pack kg",
    }
    for key, label in labels.items():
        row = best_rows[key]
        lines.append(
            f"| {label} | {_format_configuration(row)} | "
            f"{row['elapsed_time_s']:.2f} | {row['pack_mass_kg']:.2f} | "
            f"{row['pack_resistive_heat_kwh']:.4f} | "
            f"{row['remaining_usable_chemical_kwh']:.3f} |"
        )
    lines.extend(
        [
            "",
            "## Sensitivities",
            "",
            (
                "The most decision-useful local sensitivities below come from "
                "centered reruns around the best configuration while retuning "
                "power to the same reserve target:"
            ),
            "",
            (
                f"- Usable energy: **{local_sensitivity['seconds_per_usable_kwh_local']:.2f} "
                "s/kWh**."
            ),
            (
                f"- Pack mass: **{local_sensitivity['seconds_per_pack_kg_local']:.3f} "
                "s/kg**."
            ),
            (
                f"- Cell DCIR: **{local_sensitivity['seconds_per_cell_dcir_mohm_local']:.3f} "
                "s per mOhm/cell** "
                f"(**{local_sensitivity['seconds_per_pack_mohm_local']:.3f} "
                "s/mOhm** on an initial pack-resistance basis)."
            ),
            (
                "- These are centered derivative probes with power retuned to the "
                "same reserve target, not additional selectable pack designs. In "
                "particular, the +5% capacity probe exceeds the study's 8.0 kWh "
                "usable-energy cap."
            ),
            "",
            (
                f"A cross-configuration linear fit gives R² = "
                f"{global_sensitivity.get('r_squared', math.nan):.3f} over "
                f"{int(global_sensitivity.get('sample_count', 0))} constraint-valid "
                "screening cases. Because topology variables are correlated, the "
                "local finite differences should drive design decisions."
            ),
            "",
            "## Engineering observations",
            "",
            (
                "- The terminal-power limit is capped at 80 kW. A configuration "
                "that still has more than 0.5 kWh at 80 kW is labeled "
                "`power_cap_extra_reserve`; it is physically valid but not an "
                "exact equal-reserve comparison."
            ),
            (
                "- Pack current is limited by each cell's conservative published "
                "continuous rating and the unchanged 250 A inverter bus-current "
                "limit. Published pulse values are reported but not used."
            ),
            (
                "- The thermal result is an adiabatic cell-temperature-rise upper "
                "bound from I²R heat and an assumed 1000 J/kg-K cell heat capacity. "
                "It is not a cooling-loop or peak-cell-temperature prediction."
            ),
            (
                f"- Screening uses every {screening_downsample_factor}th native "
                "track element and every accepted topology runs the full 22 km. "
                "Per-cell winners, the baseline, and leading Pareto cases are "
                "rerun on the native 0.25 m mesh."
            ),
            (
                "- The Pareto sheet is therefore a screening-level design-space "
                "frontier with native results substituted where available. Use "
                "it to select hardware-test candidates, not as a sub-second "
                "final ranking of every non-dominated topology."
            ),
            (
                "- `Best value` means the largest reciprocal endurance-time-per-"
                "pack-mass metric in this study. It is not a purchase-cost "
                "ranking because comparable cell pricing was not available, and "
                "it can favor a very light pack with materially slower pace."
            ),
            "",
            "## Constraints and pack generation",
            "",
            f"- Maximum charged voltage: strictly below {MAXIMUM_PACK_VOLTAGE_V:.0f} V.",
            f"- Maximum modeled usable chemical energy: {MAXIMUM_USABLE_ENERGY_KWH:.1f} kWh.",
            (
                "- Series counts: 110–140 in 5-cell module increments. Parallel "
                "counts begin at the first topology providing 9 Ah and stop at "
                "6p. These are explicit study-grid bounds, not an exhaustive "
                "enumeration of every possible accumulator architecture."
            ),
            (
                f"- Packaging screen: cylindrical cell envelope no larger than "
                f"the 130s5p P30B baseline ({BASELINE_CELL_ENVELOPE_M3*1000.0:.3f} L). "
                "This is a topology screen, not enclosure CAD validation."
            ),
            "",
            "## Cell-data assumptions and uncertainty",
            "",
            (
                "- The live Notion shortlist supplied eight fully identifiable "
                "cell models. The ambiguous `Tenpower 18650 4000mAh` entry is "
                "excluded until Battery supplies an exact model/datasheet."
            ),
            (
                "- Molicel P30B/P50B use published room-temperature DCIR. Cells "
                "with ACIR only use DCIR = 2.047 × ACIR, derived from the two "
                "Molicel ACIR/DCIR ratios. Treat those resistance rankings as "
                "provisional and validate on the team's pulse fixture."
            ),
            (
                "- One common normalized NMC OCV-vs-SOC shape and one common "
                "SOC resistance multiplier are used because comparable maps are "
                "not published. Ambient/cell resistance is held at 25 °C."
            ),
            (
                "- Pack mass uses 1.25 × total cell mass, matching the existing "
                "team model. Fixed non-cell pack mass and cooling strategy are "
                "otherwise identical."
            ),
            (
                "- Endurance regeneration is enabled from the measured "
                "terminal-RMS policy; acceleration, skidpad, and autocross "
                "remain regeneration-free. No driver-change stop, thermal "
                "derating, or SOC-dependent motor map beyond the coupled "
                "pack-voltage/current and EMRAX field-weakening model."
                if regen_settings["enabled"]
                else (
                    "- No regeneration, driver-change stop, thermal derating, "
                    "or SOC-dependent motor map beyond the coupled pack-"
                    "voltage/current and EMRAX field-weakening model."
                )
            ),
            "",
            "## Source register",
            "",
            "| Cell | Resistance basis | Datasheet |",
            "|---|---|---|",
        ]
    )
    for cell in cell_document["cells"]:
        lines.append(
            f"| {cell['manufacturer']} {cell['model']} | "
            f"{cell['dcir_basis']} | [source]({cell['source_url']}) |"
        )
    lines.extend(
        [
            "",
            "## Output guide",
            "",
            "- `all_configuration_results.csv`: all simulated accepted topologies.",
            "- `ranked_constraint_valid_results.csv`: completed packs with at least the reserve target.",
            "- `pareto_frontier.csv`: non-dominated time/mass/heat results.",
            "- `sensitivity_cases.csv`: centered local sensitivity reruns.",
            "- `workbooks/`: one four-sheet workbook per identifiable cell.",
            "- `plots/`: consolidated comparison plots.",
        ]
    )
    path = output_root / "battery_trade_study_report.md"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def _resolve_regen_settings(args: argparse.Namespace) -> dict[str, Any]:
    if args.regen_policy is None:
        if args.regen_command_power_kw is not None:
            raise ValueError(
                "--regen-command-power-kw requires --regen-policy"
            )
        args.regen_command_power_kw_resolved = 0.0
        args.regen_active_rms_target_kw_resolved = 0.0
        args.regen_active_power_threshold_kw_resolved = 1.0
        return {
            "enabled": False,
            "scope": "mechanical braking only",
        }

    policy_path = args.regen_policy.resolve()
    policy = json.loads(policy_path.read_text(encoding="utf-8"))
    if args.regen_command_power_kw is None:
        raise ValueError(
            "--regen-command-power-kw is required with --regen-policy"
        )
    command_kw = float(args.regen_command_power_kw)
    target_kw = float(policy["chosen_target"]["target_kw"])
    active_threshold_kw = float(
        policy["chosen_target"]["active_threshold_magnitude_kw"]
    )
    measured_peak_kw = float(
        policy["measured_support"]["below_negative_one_kw"][
            "peak_regen_magnitude_kw"
        ]
    )
    if command_kw <= 0.0:
        raise ValueError("Regen command power must be positive")
    if command_kw > measured_peak_kw + 1e-12:
        raise ValueError(
            "Regen command power may not exceed the measured telemetry peak"
        )
    digest = hashlib.sha256(policy_path.read_bytes()).hexdigest()
    args.regen_command_power_kw_resolved = command_kw
    args.regen_active_rms_target_kw_resolved = target_kw
    args.regen_active_power_threshold_kw_resolved = active_threshold_kw
    return {
        "enabled": True,
        "scope": "endurance only",
        "policy_path": str(policy_path),
        "policy_sha256": digest,
        "policy_id": policy["policy_id"],
        "active_rms_target_kw": target_kw,
        "active_selector": policy["chosen_target"]["active_selector"],
        "active_power_threshold_kw": active_threshold_kw,
        "measured_peak_kw": measured_peak_kw,
        "frozen_command_power_kw": command_kw,
        "command_calibration": (
            "Frozen on the native Michigan 22 km P30B 130s5p baseline at "
            "26.078125 kW discharge ceiling. Linear interpolation between "
            "9.800 and 10.075 kW command points set 10.059 kW for the "
            "8.574267 kW active RMS target using the same 1 kW selector."
        ),
        "candidate_policy": (
            "Hold the calibrated command fixed for every pack; report each "
            "pack's achieved RMS after voltage, SOC, inverter, motor, tire, "
            "and braking-source limits."
        ),
    }


def main() -> None:
    args = parse_args()
    if args.distance_km <= 0.0 or args.reserve_kwh <= 0.0:
        raise ValueError("Distance and reserve must be positive")
    if args.power_cap_kw > 80.0 + 1e-9:
        raise ValueError("This study intentionally caps terminal power at 80 kW")
    regen_settings = _resolve_regen_settings(args)
    output_root = args.output_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    document = json.loads(args.cells.read_text(encoding="utf-8"))
    cells = document["cells"]
    if args.cell_id:
        requested = set(args.cell_id)
        cells = [cell for cell in cells if cell["cell_id"] in requested]
        missing = requested - {cell["cell_id"] for cell in cells}
        if missing:
            raise ValueError(f"Unknown cell IDs: {sorted(missing)}")
    cells_by_id = {cell["cell_id"]: cell for cell in cells}
    if args.series_count:
        series_counts = sorted(set(args.series_count))
    else:
        series_counts = list(
            range(args.series_min, args.series_max + 1, args.series_step)
        )
    candidates, rejected = generate_candidates(cells, series_counts)
    candidates_frame = pd.DataFrame(candidates)
    rejected_frame = pd.DataFrame(rejected)
    candidates_frame.to_csv(
        output_root / "generated_pack_configurations.csv", index=False
    )
    rejected_frame.to_csv(
        output_root / "rejected_pack_configurations.csv", index=False
    )
    (output_root / "study_inputs.json").write_text(
        json.dumps(
            {
                "distance_km": args.distance_km,
                "reserve_target_kwh": args.reserve_kwh,
                "reserve_tolerance_kwh": args.reserve_tolerance_kwh,
                "power_cap_kw": args.power_cap_kw,
                "series_counts": series_counts,
                "screening_downsample_factor": args.search_downsample,
                "screening_surface": [
                    args.search_soc_count,
                    args.search_speed_count,
                ],
                "refinement_surface": [
                    args.refine_soc_count,
                    args.refine_speed_count,
                ],
                "specific_heat_j_per_kgk": CELL_SPECIFIC_HEAT_J_PER_KGK,
                "candidate_count": len(candidates),
                "rejected_count": len(rejected),
                "source_cell_file": str(args.cells.resolve()),
                "source_vehicle_file": str(args.vehicle.resolve()),
                "source_track_file": str(args.track.resolve()),
                "source_powertrain_file": str(args.powertrain.resolve()),
                "regeneration": regen_settings,
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    print(
        f"Generated {len(candidates)} accepted and {len(rejected)} rejected "
        f"topologies across {len(cells)} cells.",
        flush=True,
    )
    screening_payloads = [
        _build_payload(
            args,
            candidate,
            cells_by_id[str(candidate["cell_id"])],
            downsample_factor=args.search_downsample,
            surface_soc_count=args.search_soc_count,
            surface_speed_count=args.search_speed_count,
            surface_verification_iterations=2,
            fidelity=f"screening_factor{args.search_downsample}",
        )
        for candidate in candidates
    ]
    screening_results = _run_parallel(screening_payloads, args.workers)
    screening = pd.DataFrame(screening_results)
    screening.to_csv(output_root / "screening_results.csv", index=False)
    print(
        f"Screening complete: {int(screening['constraint_valid'].sum())} "
        f"constraint-valid results.",
        flush=True,
    )

    final = screening.copy()
    if not args.no_refine:
        refinement_ids = _select_refinement_ids(screening)
        refinement_payloads = []
        for candidate_id in sorted(refinement_ids):
            matching = screening[screening["candidate_id"] == candidate_id]
            if matching.empty:
                continue
            coarse = matching.iloc[0].to_dict()
            candidate = next(
                item for item in candidates if item["candidate_id"] == candidate_id
            )
            refinement_payloads.append(
                _build_payload(
                    args,
                    candidate,
                    cells_by_id[str(candidate["cell_id"])],
                    downsample_factor=1,
                    surface_soc_count=args.refine_soc_count,
                    surface_speed_count=args.refine_speed_count,
                    surface_verification_iterations=12,
                    fidelity="native_0p25m",
                    search_grid_kw=_centered_refine_grid(
                        float(coarse["power_limit_kw"]), args.power_cap_kw
                    ),
                )
            )
        refined_rows = _run_parallel(refinement_payloads, args.workers)
        refined = pd.DataFrame(refined_rows)
        refined.to_csv(output_root / "refined_results.csv", index=False)
        for _, row in refined.iterrows():
            mask = final["candidate_id"] == row["candidate_id"]
            for column, value in row.items():
                final.loc[mask, column] = value
        print(
            f"Native-mesh refinement complete for {len(refined)} cases.",
            flush=True,
        )

    final, baseline = _add_comparisons(final)
    final = final.sort_values(
        ["constraint_valid", "elapsed_time_s"],
        ascending=[False, True],
        na_position="last",
    )
    final.to_csv(output_root / "all_configuration_results.csv", index=False)
    valid = final[final["constraint_valid"]].copy()
    valid.to_csv(
        output_root / "ranked_constraint_valid_results.csv", index=False
    )
    pareto = valid[valid["pareto_frontier"]].copy()
    pareto.to_csv(output_root / "pareto_frontier.csv", index=False)
    best_rows = _best_rows(final)
    if not best_rows:
        raise RuntimeError("No constraint-valid configuration completed")
    global_sensitivity = fit_linear_sensitivities(
        screening[screening["constraint_valid"]]
    )

    sensitivity_payloads = _local_sensitivity_payloads(
        args, best_rows["best_overall"], cells_by_id
    )
    sensitivity_results = _run_parallel(sensitivity_payloads, args.workers)
    for result, payload in zip(sensitivity_results, sensitivity_payloads):
        result["sensitivity_case"] = payload["sensitivity_case"]
    sensitivity_frame = pd.DataFrame(sensitivity_results)
    sensitivity_frame.to_csv(
        output_root / "sensitivity_cases.csv", index=False
    )
    local_sensitivity = _calculate_local_sensitivities(
        best_rows["best_overall"], sensitivity_frame
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

    _save_plots(final, output_root)
    report_path = _write_report(
        output_root,
        final,
        baseline,
        best_rows,
        global_sensitivity,
        local_sensitivity,
        document,
        len(candidates),
        len(rejected),
        args.search_downsample,
        regen_settings,
    )
    summary = {
        "baseline": baseline,
        **best_rows,
        "local_sensitivity": local_sensitivity,
        "global_sensitivity": global_sensitivity,
        "constraint_valid_count": int(len(valid)),
        "equal_reserve_count": int(final["equal_reserve_eligible"].sum()),
        "pareto_count": int(len(pareto)),
        "report": str(report_path),
    }
    (output_root / "headline_summary.json").write_text(
        json.dumps(summary, indent=2, default=str),
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, default=str), flush=True)


if __name__ == "__main__":
    main()
