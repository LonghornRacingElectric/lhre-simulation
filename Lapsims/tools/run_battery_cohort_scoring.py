"""CLI for simulated-cohort-normalized battery dynamic scoring."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from battery_cohort_scoring import run_cohort_scoring  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Re-score the completed battery event sweep with the fastest "
            "simulated configuration defining maximum points in each event."
        )
    )
    parser.add_argument(
        "--source-summary",
        type=Path,
        default=(
            ROOT
            / "outputs"
            / "battery_dynamic_points_20260725"
            / "battery_dynamic_points_summary.csv"
        ),
    )
    parser.add_argument(
        "--battery-results",
        type=Path,
        default=(
            ROOT
            / "outputs"
            / "battery_trade_study_20260725_final"
            / "all_configuration_results.csv"
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=(
            ROOT
            / "outputs"
            / "battery_dynamic_points_cohort_normalized_20260725"
        ),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result = run_cohort_scoring(
        source_summary_csv=args.source_summary,
        battery_results_csv=args.battery_results,
        output_dir=args.output_dir,
    )
    print(json.dumps(result, indent=2))
    if not result["validation"]["all_checks_passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
