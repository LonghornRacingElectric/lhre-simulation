"""CLI for the battery-configuration dynamic-points study."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from battery_dynamic_points import run_study  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run all accepted battery configurations through acceleration, "
            "skidpad, autocross, endurance, and inherited efficiency scoring."
        )
    )
    parser.add_argument(
        "--candidates",
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
        "--output-dir",
        type=Path,
        default=ROOT / "outputs" / "battery_dynamic_points_20260725",
    )
    parser.add_argument("--curve-points", type=int, default=121)
    parser.add_argument("--progress-every", type=int, default=10)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result = run_study(
        root=ROOT,
        candidate_csv=args.candidates,
        cells_json=args.cells,
        output_dir=args.output_dir,
        curve_points=args.curve_points,
        progress_every=args.progress_every,
    )
    print(json.dumps(result, indent=2, default=str))
    if not result["validation"]["all_checks_passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
