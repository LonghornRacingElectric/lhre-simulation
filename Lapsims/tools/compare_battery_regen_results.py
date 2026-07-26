"""Compare versioned no-regen and regen battery-study result artifacts."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from battery_regen_comparison import run_comparison  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Join prior no-regen and new regen all_configuration_results.csv "
            "files by candidate_id and emit deltas, ranks, plots, and a report."
        )
    )
    parser.add_argument(
        "--prior-results",
        type=Path,
        default=(
            ROOT
            / "outputs"
            / "battery_trade_study_20260725_final"
            / "all_configuration_results.csv"
        ),
    )
    parser.add_argument(
        "--regen-results",
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
        default=ROOT / "outputs" / "battery_regen_comparison_20260725",
    )
    parser.add_argument(
        "--scenario-name",
        default="regen_endurance_only_vs_no_regen",
    )
    parser.add_argument(
        "--skip-plots",
        action="store_true",
        help="Skip PNG plots; useful for CI or quick validation.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Allow replacement of files in an existing output directory.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result = run_comparison(
        prior_results_csv=args.prior_results,
        regen_results_csv=args.regen_results,
        output_dir=args.output_dir,
        scenario_name=args.scenario_name,
        write_plot_files=not args.skip_plots,
        overwrite=args.overwrite,
    )
    print(json.dumps(result, indent=2, default=str))
    if not result["validation"]["all_checks_passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
