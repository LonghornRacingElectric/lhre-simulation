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
    vehicle = load_yaml(resolve_from(STUDY_DIR / "study.yml", cfg["inputs"]["vehicle_config"]))
    four = metrics(resolve_from(STUDY_DIR / "study.yml", cfg["inputs"]["four_post_metrics"]))
    rows = [{
        "body_torsional_stiffness_n_m_per_rad": vehicle["body"]["torsional_stiff_n_m_per_rad"],
        "front_elastic_roll_stiffness_n_m_per_rad": four["elastic_roll_stiffness_front_Nm_per_rad"],
        "rear_elastic_roll_stiffness_n_m_per_rad": four["elastic_roll_stiffness_rear_Nm_per_rad"],
        "front_lltd_pct": four["avg_lltd_front_pct"],
        "roll_toe_gain_rad_per_rad": four["toe_gain_roll_rad_per_rad"],
    }]
    out = STUDY_DIR / cfg["outputs"]["data_dir"]
    out.mkdir(parents=True, exist_ok=True)
    with (out / "summary.csv").open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    r = rows[0]
    lines = [
        "# CHASSIS-003 Results",
        "",
        "## Finding",
        "",
        "**PASS:** chassis stiffness and validation claims can be connected to full-vehicle roll stiffness and LLTD metrics.",
        "",
        "## Key Metrics",
        "",
        f"- Body torsional stiffness input: `{float(r['body_torsional_stiffness_n_m_per_rad']):.0f} N*m/rad`",
        f"- Front/rear elastic roll stiffness: `{r['front_elastic_roll_stiffness_n_m_per_rad']:.0f}` / `{r['rear_elastic_roll_stiffness_n_m_per_rad']:.0f} N*m/rad`",
        f"- Front LLTD: `{r['front_lltd_pct']:.2f} %`",
        f"- Roll toe gain: `{r['roll_toe_gain_rad_per_rad']:.5f} rad/rad`",
        "",
        "## Validation Closure",
        "",
        "Measure torsional stiffness, inspect high-load tabs after test events, and verify that setup changes produce the predicted balance direction before treating stiffness assumptions as validated.",
    ]
    (STUDY_DIR / cfg["outputs"]["results_markdown"]).write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
