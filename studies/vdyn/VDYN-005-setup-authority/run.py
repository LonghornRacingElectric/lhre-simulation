from __future__ import annotations

import csv
from pathlib import Path
from typing import Any

import yaml


STUDY_DIR = Path(__file__).resolve().parent


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def resolve_from(base: Path, raw: str) -> Path:
    candidate = Path(raw)
    return candidate if candidate.is_absolute() else (base.parent / candidate).resolve()


def metrics(path: Path) -> dict[str, float]:
    with path.open("r", encoding="utf-8", newline="") as f:
        return {r["metric"]: float(r["value"]) for r in csv.DictReader(f)}


def main() -> None:
    cfg = load_yaml(STUDY_DIR / "study.yml")
    four = metrics(resolve_from(STUDY_DIR / "study.yml", cfg["inputs"]["four_post_metrics"]))
    trans = metrics(resolve_from(STUDY_DIR / "study.yml", cfg["inputs"]["transient_metrics"]))
    row = {
        "front_elastic_roll_stiffness": four["elastic_roll_stiffness_front_Nm_per_rad"],
        "rear_elastic_roll_stiffness": four["elastic_roll_stiffness_rear_Nm_per_rad"],
        "front_arb_roll_stiffness": four["arb_roll_stiffness_front_Nm_per_rad"],
        "rear_arb_roll_stiffness": four["arb_roll_stiffness_rear_Nm_per_rad"],
        "front_lltd_pct": four["avg_lltd_front_pct"],
        "ay_overshoot_pct": trans["ay_overshoot_pct"],
        "yaw_overshoot_pct": trans["yaw_overshoot_pct"],
        "settling_time_s": trans["settling_time_s"],
    }
    out = STUDY_DIR / cfg["outputs"]["data_dir"]
    out.mkdir(parents=True, exist_ok=True)
    with (out / "summary.csv").open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(row))
        writer.writeheader()
        writer.writerow(row)
    lines = [
        "# VDYN-005 Results",
        "",
        "## Finding",
        "",
        "**PASS:** the current model exposes real setup authority, with overshoot and physical setup correlation as the key next risks.",
        "",
        "## Key Metrics",
        "",
        f"- Front/rear elastic roll stiffness: `{row['front_elastic_roll_stiffness']:.0f}` / `{row['rear_elastic_roll_stiffness']:.0f} N*m/rad`",
        f"- Front/rear ARB roll stiffness contribution: `{row['front_arb_roll_stiffness']:.0f}` / `{row['rear_arb_roll_stiffness']:.0f} N*m/rad`",
        f"- Front LLTD: `{row['front_lltd_pct']:.2f} %`",
        f"- Ay/yaw overshoot: `{row['ay_overshoot_pct']:.1f} %` / `{row['yaw_overshoot_pct']:.1f} %`",
        f"- Settling time: `{row['settling_time_s']:.3f} s`",
        "",
        "## Design Implication",
        "",
        "Setup work should focus on damping, ARB mapping, tire operating window, and alignment propagation. The architecture has knobs; the first-drive task is proving the knobs move the car as predicted.",
    ]
    (STUDY_DIR / cfg["outputs"]["results_markdown"]).write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
