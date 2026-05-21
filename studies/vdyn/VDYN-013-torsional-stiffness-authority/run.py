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
NM_PER_DEG_TO_NM_PER_RAD = 180.0 / math.pi


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def resolve_from(base: Path, raw: str) -> Path:
    candidate = Path(raw)
    return candidate if candidate.is_absolute() else (base.parent / candidate).resolve()


def metrics(path: Path) -> dict[str, float]:
    with path.open("r", encoding="utf-8", newline="") as f:
        return {r["metric"]: float(r["value"]) for r in csv.DictReader(f)}


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    cfg = load_yaml(STUDY_DIR / "study.yml")
    vehicle = load_yaml(resolve_from(STUDY_DIR / "study.yml", cfg["inputs"]["vehicle_config"]))
    four = metrics(resolve_from(STUDY_DIR / "study.yml", cfg["inputs"]["four_post_metrics"]))
    suspension_roll = four["elastic_roll_stiffness_front_Nm_per_rad"] + four["elastic_roll_stiffness_rear_Nm_per_rad"]
    body_input = float(vehicle["body"]["torsional_stiff_n_m_per_rad"])
    body_input_nm_per_deg = body_input / NM_PER_DEG_TO_NM_PER_RAD
    body_sweep_nm_per_deg = sorted({500, 1000, 1500, 2500, 4000, 6000, body_input_nm_per_deg})
    rows = []
    for body_deg in body_sweep_nm_per_deg:
        body = body_deg * NM_PER_DEG_TO_NM_PER_RAD
        effective = 1.0 / (1.0 / suspension_roll + 1.0 / body)
        rows.append(
            {
                "body_stiffness_nm_per_deg": body_deg,
                "body_stiffness_nm_per_rad": body,
                "suspension_roll_stiffness_nm_per_rad": suspension_roll,
                "effective_roll_stiffness_nm_per_rad": effective,
                "setup_authority_fraction": effective / suspension_roll,
            }
        )
    outputs = STUDY_DIR / "outputs"
    plots = STUDY_DIR / "plots"
    write_csv(outputs / "torsional_stiffness_sweep.csv", rows)
    fig, ax = plt.subplots(figsize=(7.4, 4.8))
    ax.plot([r["body_stiffness_nm_per_deg"] for r in rows], [100.0 * r["setup_authority_fraction"] for r in rows], marker="o")
    ax.set_xlabel("Body torsional stiffness [Nm/deg]")
    ax.set_ylabel("Effective roll authority [% of suspension-only]")
    ax.set_title("VDYN-013 Torsional Stiffness Authority")
    ax.grid(True, alpha=0.28)
    fig.tight_layout()
    plots.mkdir(parents=True, exist_ok=True)
    fig.savefig(plots / "torsional_stiffness_authority.png", dpi=220)
    plt.close(fig)
    current = min(rows, key=lambda r: abs(r["body_stiffness_nm_per_rad"] - body_input))
    low = min(rows, key=lambda r: abs(r["body_stiffness_nm_per_deg"] - 1500.0))
    lines = [
        "# VDYN-013 Results",
        "",
        "## Finding",
        "",
        "**PASS:** torsional stiffness has been translated into setup-authority language.",
        "",
        "## Key Metrics",
        "",
        f"- Suspension elastic roll stiffness total: `{suspension_roll:.0f} N*m/rad`",
        f"- Vehicle YAML body torsional stiffness: `{body_input:.0f} N*m/rad` (`{body_input_nm_per_deg:.0f} Nm/deg`)",
        "- The plot axis is converted to `Nm/deg`; the source input remains `N*m/rad`.",
        f"- Current effective roll authority: `{100.0 * current['setup_authority_fraction']:.1f} %` of suspension-only roll stiffness",
        f"- At `1500 Nm/deg`, effective roll authority would be `{100.0 * low['setup_authority_fraction']:.1f} %`",
        "",
        "![Torsional stiffness authority](plots/torsional_stiffness_authority.png)",
        "",
        "## Design Implication",
        "",
        "Torsional stiffness should be validated because it determines whether spring and ARB changes actually reach the contact patches as modeled.",
    ]
    (STUDY_DIR / "RESULTS.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
