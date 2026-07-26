"""Refresh dynamic points from new endurance results without rerunning sprints."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from battery_dynamic_refresh import run_refresh  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Reuse existing acceleration/skidpad/autocross outputs exactly, "
            "then refresh endurance, net-energy efficiency, totals, and ranks."
        )
    )
    parser.add_argument(
        "--sprint-source-dir",
        type=Path,
        default=ROOT / "outputs" / "battery_dynamic_points_20260725",
    )
    parser.add_argument(
        "--battery-results",
        type=Path,
        default=(
            ROOT
            / "outputs"
            / "battery_trade_study_regen_endurance_only_20260725"
            / "all_configuration_results.csv"
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=(
            ROOT
            / "outputs"
            / "battery_dynamic_points_regen_endurance_only_20260725"
        ),
    )
    parser.add_argument(
        "--scenario-name",
        default="regen_endurance_only",
    )
    parser.add_argument(
        "--skip-plots",
        action="store_true",
        help="Skip plot generation; useful for smoke tests.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Allow replacement of files in an existing output directory.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result = run_refresh(
        sprint_source_dir=args.sprint_source_dir,
        battery_results_csv=args.battery_results,
        output_dir=args.output_dir,
        scenario_name=args.scenario_name,
        write_plots=not args.skip_plots,
        overwrite=args.overwrite,
    )
    print(json.dumps(result, indent=2, default=str))
    if not result["validation"]["all_checks_passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
