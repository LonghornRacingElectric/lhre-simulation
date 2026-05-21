from __future__ import annotations

import csv
import math
import os
from pathlib import Path
from typing import Any

os.environ.setdefault("MPLCONFIGDIR", str(Path(__file__).resolve().parent / ".matplotlib-cache"))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import yaml


STUDY_DIR = Path(__file__).resolve().parent


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def resolve_from(base: Path, raw: str) -> Path:
    candidate = Path(raw)
    return candidate if candidate.is_absolute() else (base.parent / candidate).resolve()


def read_rows(path: Path) -> list[dict[str, float]]:
    with path.open("r", encoding="utf-8", newline="") as f:
        rows: list[dict[str, float]] = []
        for row in csv.DictReader(f):
            rows.append({k: float(v) if v and v.lower() != "nan" else math.nan for k, v in row.items()})
        return rows


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    cfg = load_yaml(STUDY_DIR / "study.yml")
    env = [r for r in read_rows(resolve_from(STUDY_DIR / "study.yml", cfg["inputs"]["baseline_envelope"])) if abs(r["speed_mps"] - 15.0) < 1e-9]
    metrics = {r["speed_mps"]: r for r in read_rows(resolve_from(STUDY_DIR / "study.yml", cfg["inputs"]["baseline_metrics"]))}
    m15 = metrics[15.0]
    max_lat = m15["max_lateral_g"]
    max_acc = m15["max_accel_g"]
    max_brake = m15["max_brake_g"]
    targets = [0.0, 0.25 * max_lat, 0.50 * max_lat, 0.75 * max_lat, 0.90 * max_lat]
    rows: list[dict[str, Any]] = []
    for ay_target in targets:
        candidates = sorted(env, key=lambda r: abs(abs(r["ay_g"]) - ay_target))
        row = candidates[0]
        ax_acc = row["ax_accel_g"]
        ax_brake = abs(row["ax_brake_g"])
        rows.append(
            {
                "ay_g": abs(row["ay_g"]),
                "ay_fraction_of_peak": abs(row["ay_g"]) / max_lat,
                "accel_available_g": ax_acc,
                "accel_fraction_of_zero_ay": ax_acc / max_acc,
                "brake_available_g": ax_brake,
                "brake_fraction_of_zero_ay": ax_brake / max_brake,
            }
        )
    outputs = STUDY_DIR / cfg["outputs"]["data_dir"]
    plots = STUDY_DIR / cfg["outputs"]["figures_dir"]
    write_csv(outputs / "combined_slip_budget.csv", rows)
    plots.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(7.2, 4.8))
    ax.plot([r["ay_fraction_of_peak"] for r in rows], [r["accel_fraction_of_zero_ay"] for r in rows], marker="o", label="Accel remaining")
    ax.plot([r["ay_fraction_of_peak"] for r in rows], [r["brake_fraction_of_zero_ay"] for r in rows], marker="o", label="Brake remaining")
    ax.set_xlabel("Lateral demand fraction of peak [-]")
    ax.set_ylabel("Longitudinal capability fraction remaining [-]")
    ax.set_title("VDYN-010 Combined Slip Budget at 15 m/s")
    ax.grid(True, alpha=0.28)
    ax.legend()
    fig.tight_layout()
    fig.savefig(plots / "combined_slip_budget.png", dpi=220)
    plt.close(fig)
    high = rows[-1]
    lines = [
        "# VDYN-010 Results",
        "",
        "## Finding",
        "",
        "**PASS:** the baseline GGV envelope has been converted into a combined-slip budget for driver-facing explanation.",
        "",
        "## Key Metrics",
        "",
        f"- At `{high['ay_fraction_of_peak']:.0%}` of peak lateral demand, available acceleration is `{high['accel_fraction_of_zero_ay']:.0%}` of zero-ay acceleration.",
        f"- At `{high['ay_fraction_of_peak']:.0%}` of peak lateral demand, available braking is `{high['brake_fraction_of_zero_ay']:.0%}` of zero-ay braking.",
        "",
        "![Combined slip budget](plots/combined_slip_budget.png)",
        "",
        "## Design Implication",
        "",
        "Power, brake, and cornering claims must be discussed as tire-budget tradeoffs. The contact patch is the shared currency.",
    ]
    (STUDY_DIR / cfg["outputs"]["results_markdown"]).write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
