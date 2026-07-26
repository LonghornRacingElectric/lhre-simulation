"""Run battery-aware endurance and optional parallel-count/mass sweeps."""

from __future__ import annotations

import argparse
import json
from dataclasses import replace
from pathlib import Path

import pandas as pd

from endurance_solver import simulate_endurance
from openlap_solver import load_vehicle
from powertrain_model import (
    MaximumTorqueSurface,
    PowertrainModel,
    load_powertrain_config,
)


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--vehicle",
        type=Path,
        default=root / "inputs" / "openlap_vehicle.json",
    )
    parser.add_argument(
        "--track",
        type=Path,
        default=root / "inputs" / "michigan_openlap_track.csv",
    )
    parser.add_argument(
        "--powertrain",
        type=Path,
        default=root / "inputs" / "powertrain_130s5p_p30b_provisional.json",
    )
    parser.add_argument("--laps", type=int, default=21)
    parser.add_argument(
        "--terminal-power-limit-kw",
        type=float,
        default=None,
        help="Override the pack-terminal propulsion power limit from the input.",
    )
    parser.add_argument(
        "--parallel-counts",
        type=int,
        nargs="+",
        default=[4, 5, 6, 7],
        help="One case per count; mass, capacity, R, and current all change.",
    )
    parser.add_argument("--initial-speed-mps", type=float, default=0.0)
    parser.add_argument(
        "--output-root",
        type=Path,
        default=root / "outputs" / "endurance",
    )
    parser.add_argument("--surface-soc-count", type=int, default=25)
    parser.add_argument("--surface-speed-count", type=int, default=121)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_root = args.output_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    vehicle = load_vehicle(args.vehicle.resolve())
    track = pd.read_csv(args.track.resolve())
    base_config = load_powertrain_config(args.powertrain.resolve())
    if args.terminal_power_limit_kw is not None:
        if args.terminal_power_limit_kw <= 0.0:
            raise ValueError("Terminal power limit must be positive")
        base_config = replace(
            base_config,
            terminal_power_limit_w=args.terminal_power_limit_kw * 1000.0,
        )
    sweep_rows = []

    for parallel_count in args.parallel_counts:
        config = base_config.with_parallel_cells(parallel_count)
        model = PowertrainModel(config)
        torque_surface = MaximumTorqueSurface(
            model,
            maximum_vehicle_speed_mps=vehicle.v_max,
            soc_count=args.surface_soc_count,
            speed_count=args.surface_speed_count,
        )
        slug = f"{config.pack.series_cells}s{parallel_count}p"
        torque_surface.to_frame().to_csv(
            output_root / f"{slug}_maximum_torque_surface.csv", index=False
        )
        trace, summary = simulate_endurance(
            vehicle,
            track,
            model,
            args.laps,
            initial_speed_mps=args.initial_speed_mps,
            surface_soc_count=args.surface_soc_count,
            surface_speed_count=args.surface_speed_count,
            torque_surface=torque_surface,
        )
        trace.to_csv(output_root / f"{slug}_trace.csv", index=False)
        (output_root / f"{slug}_summary.json").write_text(
            json.dumps(summary, indent=2), encoding="utf-8"
        )
        sweep_rows.append(
            {
                "case": slug,
                "completed": summary["completed"],
                "vehicle_mass_kg": summary["vehicle_mass_kg"],
                "capacity_ah": summary["pack_capacity_ah"],
                "pack_resistance_ohm": summary[
                    "pack_resistance_at_initial_soc_ohm"
                ],
                "elapsed_time_s": summary["elapsed_time_s"],
                "final_soc": summary["final_soc"],
                "terminal_energy_kwh": summary["terminal_energy_kwh"],
                "maximum_terminal_power_w": summary[
                    "maximum_terminal_power_w"
                ],
                "completed_laps": summary["completed_laps"],
                "distance_m": summary["distance_m"],
                "minimum_terminal_voltage_v": summary[
                    "minimum_terminal_voltage_v"
                ],
            }
        )
        print(json.dumps({"case": slug, **sweep_rows[-1]}, indent=2))

    pd.DataFrame(sweep_rows).to_csv(
        output_root / "pack_size_sweep_summary.csv", index=False
    )


if __name__ == "__main__":
    main()
