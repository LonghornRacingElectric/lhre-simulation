"""Plot the endurance-event winner's battery-terminal power versus time."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from battery_trade_study import (  # noqa: E402
    aggregate_exact_distance,
    build_powertrain_config,
)
from endurance_solver import simulate_endurance  # noqa: E402
from openlap_solver import load_vehicle  # noqa: E402
from powertrain_model import (  # noqa: E402
    MaximumTorqueSurface,
    PowertrainModel,
    load_powertrain_config,
)


DEFAULT_CANDIDATE_ID = "tenpower_60xg_125s3p"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Reproduce the 22 km endurance-event winner and plot its "
            "battery-terminal power trace."
        )
    )
    parser.add_argument("--candidate-id", default=DEFAULT_CANDIDATE_ID)
    parser.add_argument(
        "--source-results",
        type=Path,
        default=(
            ROOT
            / "outputs"
            / "battery_trade_study_20260725_final"
            / "all_configuration_results.csv"
        ),
    )
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
        "--output-dir",
        type=Path,
        default=(
            ROOT
            / "outputs"
            / "battery_dynamic_points_cohort_normalized_20260725"
            / "winning_endurance_power_trace"
        ),
    )
    parser.add_argument("--smoothing-window-s", type=float, default=5.0)
    parser.add_argument("--surface-soc-count", type=int, default=9)
    parser.add_argument("--surface-speed-count", type=int, default=41)
    parser.add_argument("--surface-verification-iterations", type=int, default=12)
    return parser.parse_args()


def _load_candidate(
    results_path: Path,
    cells_path: Path,
    candidate_id: str,
) -> tuple[pd.Series, dict[str, Any]]:
    results = pd.read_csv(results_path)
    matches = results[results["candidate_id"] == candidate_id]
    if len(matches) != 1:
        raise ValueError(
            f"Expected exactly one result for {candidate_id!r}; found "
            f"{len(matches)}"
        )
    source_row = matches.iloc[0]

    document = json.loads(cells_path.read_text(encoding="utf-8"))
    cell_matches = [
        cell
        for cell in document["cells"]
        if cell["cell_id"] == source_row["cell_id"]
    ]
    if len(cell_matches) != 1:
        raise ValueError(
            f"Expected exactly one cell specification for "
            f"{source_row['cell_id']!r}; found {len(cell_matches)}"
        )
    return source_row, cell_matches[0]


def _moving_average_from_energy(
    trace: pd.DataFrame,
    window_s: float,
    sample_period_s: float = 0.5,
) -> tuple[np.ndarray, np.ndarray]:
    """Return a centered time-weighted power average from cumulative energy."""

    if window_s <= 0.0 or sample_period_s <= 0.0:
        raise ValueError("Smoothing window and sample period must be positive")
    time_edges = np.concatenate(
        ([0.0], trace["elapsed_time_s"].to_numpy(dtype=float))
    )
    energy_edges_wh = np.concatenate(
        (
            [0.0],
            trace["cumulative_terminal_energy_wh"].to_numpy(dtype=float),
        )
    )
    duration_s = float(time_edges[-1])
    sample_count = int(math.ceil(duration_s / sample_period_s)) + 1
    time_s = np.linspace(0.0, duration_s, sample_count)
    half_window = 0.5 * window_s
    start_s = np.maximum(0.0, time_s - half_window)
    end_s = np.minimum(duration_s, time_s + half_window)
    start_energy_wh = np.interp(start_s, time_edges, energy_edges_wh)
    end_energy_wh = np.interp(end_s, time_edges, energy_edges_wh)
    averaged_power_kw = (
        (end_energy_wh - start_energy_wh)
        * 3.6
        / np.maximum(end_s - start_s, 1e-12)
    )
    return time_s, averaged_power_kw


def _validation_checks(
    source_row: pd.Series,
    exact_trace: pd.DataFrame,
    metrics: dict[str, Any],
    terminal_power_limit_kw: float,
) -> dict[str, bool]:
    expected = {
        "elapsed_time_s": float(source_row["elapsed_time_s"]),
        "terminal_energy_kwh": float(source_row["terminal_energy_kwh"]),
        "peak_battery_terminal_power_kw": float(
            source_row["peak_battery_terminal_power_kw"]
        ),
        "final_soc": float(source_row["final_soc"]),
    }
    reproduced = {
        "elapsed_time_s": float(metrics["elapsed_time_s"]),
        "terminal_energy_kwh": float(metrics["terminal_energy_kwh"]),
        "peak_battery_terminal_power_kw": float(
            metrics["peak_battery_terminal_power_kw"]
        ),
        "final_soc": float(metrics["final_soc"]),
    }
    checks = {
        f"{name}_matches_source": math.isclose(
            reproduced[name],
            expected[name],
            rel_tol=1e-9,
            abs_tol=1e-7,
        )
        for name in expected
    }
    checks.update(
        {
            "trace_ends_at_22_km": math.isclose(
                float(exact_trace["cumulative_distance_m"].iloc[-1]),
                22_000.0,
                rel_tol=0.0,
                abs_tol=1e-8,
            ),
            "terminal_power_matches_voltage_times_current": bool(
                np.allclose(
                    exact_trace["battery_terminal_power_w"],
                    exact_trace["battery_terminal_power_from_vi_w"],
                    rtol=0.0,
                    atol=1e-5,
                )
            ),
            "terminal_power_respects_limit": bool(
                float(exact_trace["battery_terminal_power_w"].max())
                <= terminal_power_limit_kw * 1000.0 + 1e-4
            ),
            "energy_integral_matches_metric": math.isclose(
                float(
                    np.sum(
                        exact_trace["battery_terminal_power_w"].to_numpy(
                            dtype=float
                        )
                        * exact_trace["time_in_segment_s"].to_numpy(
                            dtype=float
                        )
                    )
                    / 3.6e6
                ),
                float(metrics["terminal_energy_kwh"]),
                rel_tol=1e-12,
                abs_tol=1e-10,
            ),
        }
    )
    return checks


def _plot(
    exact_trace: pd.DataFrame,
    metrics: dict[str, Any],
    source_row: pd.Series,
    smoothing_window_s: float,
    output_path: Path,
) -> None:
    elapsed_s = exact_trace["elapsed_time_s"].to_numpy(dtype=float)
    terminal_power_kw = (
        exact_trace["battery_terminal_power_w"].to_numpy(dtype=float) / 1000.0
    )
    smooth_time_s, smooth_power_kw = _moving_average_from_energy(
        exact_trace, smoothing_window_s
    )
    lap_end_times = (
        exact_trace.groupby("lap", sort=True)["elapsed_time_s"]
        .max()
        .to_numpy(dtype=float)
    )
    average_power_kw = float(metrics["average_battery_terminal_power_kw"])
    peak_power_kw = float(metrics["peak_battery_terminal_power_kw"])
    power_limit_kw = float(source_row["power_limit_kw"])
    duration_s = float(metrics["elapsed_time_s"])
    terminal_energy_kwh = float(metrics["terminal_energy_kwh"])

    fig, ax = plt.subplots(figsize=(13.5, 7.2))
    time_edges_s = np.concatenate(([0.0], elapsed_s))
    ax.stairs(
        terminal_power_kw,
        time_edges_s,
        baseline=None,
        color="#75AADB",
        alpha=0.40,
        linewidth=0.42,
        label="Raw battery-terminal power",
        rasterized=True,
    )
    ax.fill_between(
        smooth_time_s,
        smooth_power_kw,
        color="#F4A261",
        alpha=0.18,
        linewidth=0.0,
    )
    ax.plot(
        smooth_time_s,
        smooth_power_kw,
        color="#D55E00",
        linewidth=1.35,
        label=f"{smoothing_window_s:g} s time-weighted average",
    )
    ax.axhline(
        average_power_kw,
        color="#0072B2",
        linestyle="--",
        linewidth=1.45,
        label=f"Event average: {average_power_kw:.2f} kW",
    )
    ax.axhline(
        power_limit_kw,
        color="#222222",
        linestyle=":",
        linewidth=1.5,
        label=f"Sustainable terminal cap: {power_limit_kw:.3f} kW",
    )
    for index, lap_end_s in enumerate(lap_end_times[:-1]):
        ax.axvline(
            lap_end_s,
            color="#777777",
            alpha=0.16,
            linewidth=0.65,
            label="Lap boundaries" if index == 0 else None,
        )

    ax.set_title(
        "Endurance event winner — battery-terminal power versus time\n"
        "Tenpower 60XG 125s3p | 22.0 km Michigan endurance simulation",
        fontsize=15,
        pad=12,
    )
    ax.set_xlabel("Elapsed time (s)")
    ax.set_ylabel("Battery-terminal power (kW)")
    ax.set_xlim(0.0, duration_s)
    ax.set_ylim(0.0, max(power_limit_kw, peak_power_kw) * 1.12)
    ax.grid(True, color="#B0B0B0", alpha=0.24, linewidth=0.7)
    ax.legend(loc="upper right", frameon=True, framealpha=0.95)
    ax.text(
        0.012,
        0.975,
        (
            f"Elapsed: {duration_s:.3f} s ({duration_s / 60.0:.2f} min)\n"
            f"Net terminal energy: {terminal_energy_kwh:.3f} kWh\n"
            f"Peak terminal power: {peak_power_kw:.3f} kW\n"
            f"Final SOC: {float(metrics['final_soc']):.3f}\n"
            "No regenerative braking modeled"
        ),
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=10,
        bbox={
            "boxstyle": "round,pad=0.45",
            "facecolor": "white",
            "edgecolor": "#BBBBBB",
            "alpha": 0.92,
        },
    )
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    paths = [
        args.source_results,
        args.cells,
        args.vehicle,
        args.track,
        args.powertrain,
    ]
    for path in paths:
        if not path.is_file():
            raise FileNotFoundError(path)
    if args.smoothing_window_s <= 0.0:
        raise ValueError("--smoothing-window-s must be positive")

    source_row, cell = _load_candidate(
        args.source_results, args.cells, args.candidate_id
    )
    if source_row["simulation_fidelity"] != "native_0p25m":
        raise ValueError(
            "The winning source result is not the expected native-mesh run"
        )

    vehicle = load_vehicle(args.vehicle)
    track = pd.read_csv(args.track)
    base_config = load_powertrain_config(args.powertrain)
    terminal_power_limit_kw = float(source_row["power_limit_kw"])
    config = build_powertrain_config(
        base_config,
        cell,
        int(source_row["series_cells"]),
        int(source_row["parallel_cells"]),
        terminal_power_limit_kw,
    )
    model = PowertrainModel(config)
    surface = MaximumTorqueSurface(
        model,
        maximum_vehicle_speed_mps=vehicle.v_max,
        soc_count=args.surface_soc_count,
        speed_count=args.surface_speed_count,
        verification_iterations=args.surface_verification_iterations,
    )
    target_distance_m = float(source_row["target_distance_m"])
    lap_length_m = float(track["dx_m"].sum())
    requested_laps = int(math.ceil(target_distance_m / lap_length_m))
    raw_trace, solver_summary = simulate_endurance(
        vehicle,
        track,
        model,
        requested_laps,
        initial_speed_mps=0.0,
        torque_surface=surface,
    )
    exact_trace, metrics = aggregate_exact_distance(
        raw_trace,
        target_distance_m,
        lap_length_m,
        config,
    )
    checks = _validation_checks(
        source_row,
        exact_trace,
        metrics,
        terminal_power_limit_kw,
    )
    if not all(checks.values()):
        failed = [name for name, passed in checks.items() if not passed]
        raise RuntimeError(f"Winning trace validation failed: {failed}")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    trace_path = args.output_dir / f"{args.candidate_id}_power_trace.csv"
    plot_path = (
        args.output_dir
        / f"{args.candidate_id}_battery_terminal_power_vs_time.png"
    )
    summary_path = args.output_dir / f"{args.candidate_id}_power_trace_summary.json"
    trace_columns = [
        "lap",
        "segment",
        "elapsed_time_s",
        "time_in_segment_s",
        "cumulative_distance_m",
        "speed_mps",
        "next_speed_mps",
        "soc",
        "next_soc",
        "battery_terminal_voltage_v",
        "battery_current_a",
        "battery_terminal_power_w",
        "motor_shaft_power_w",
        "active_limiter",
        "cumulative_terminal_energy_wh",
    ]
    trace_output = exact_trace[trace_columns].copy()
    trace_output.insert(
        2,
        "segment_start_time_s",
        trace_output["elapsed_time_s"] - trace_output["time_in_segment_s"],
    )
    trace_output["cumulative_distance_km"] = (
        trace_output["cumulative_distance_m"] / 1000.0
    )
    trace_output["battery_terminal_power_kw"] = (
        trace_output["battery_terminal_power_w"] / 1000.0
    )
    trace_output["motor_shaft_power_kw"] = (
        trace_output["motor_shaft_power_w"] / 1000.0
    )
    trace_output["cumulative_terminal_energy_kwh"] = (
        trace_output["cumulative_terminal_energy_wh"] / 1000.0
    )
    trace_output.to_csv(trace_path, index=False)
    _plot(
        exact_trace,
        metrics,
        source_row,
        args.smoothing_window_s,
        plot_path,
    )

    output_summary = {
        "candidate_id": args.candidate_id,
        "configuration": {
            "cell": f"{source_row['manufacturer']} {source_row['cell_model']}",
            "series_cells": int(source_row["series_cells"]),
            "parallel_cells": int(source_row["parallel_cells"]),
            "pack_mass_kg": float(source_row["pack_mass_kg"]),
            "vehicle_mass_kg": float(source_row["vehicle_mass_kg"]),
        },
        "trace_definition": (
            "Battery-terminal power = terminal voltage times bus current. "
            "Positive values are energy leaving the accumulator; regeneration "
            "is not modeled."
        ),
        "simulation": {
            "track": str(args.track.resolve()),
            "target_distance_km": target_distance_m / 1000.0,
            "native_track_spacing_m": float(track["dx_m"].median()),
            "requested_laps": requested_laps,
            "completed_full_laps_before_target": int(
                metrics["completed_full_laps_before_target"]
            ),
            "elapsed_time_s": float(metrics["elapsed_time_s"]),
            "terminal_power_limit_kw": terminal_power_limit_kw,
            "average_battery_terminal_power_kw": float(
                metrics["average_battery_terminal_power_kw"]
            ),
            "peak_battery_terminal_power_kw": float(
                metrics["peak_battery_terminal_power_kw"]
            ),
            "terminal_energy_kwh": float(metrics["terminal_energy_kwh"]),
            "final_soc": float(metrics["final_soc"]),
            "remaining_usable_chemical_kwh": float(
                metrics["remaining_usable_chemical_kwh"]
            ),
            "trace_rows": int(len(exact_trace)),
            "solver_stop_reason": solver_summary["stop_reason"],
        },
        "validation": {
            "all_checks_passed": all(checks.values()),
            "checks": checks,
        },
        "artifacts": {
            "plot": str(plot_path.resolve()),
            "trace_csv": str(trace_path.resolve()),
        },
    }
    summary_path.write_text(
        json.dumps(output_summary, indent=2),
        encoding="utf-8",
    )
    print(json.dumps(output_summary, indent=2))


if __name__ == "__main__":
    main()
