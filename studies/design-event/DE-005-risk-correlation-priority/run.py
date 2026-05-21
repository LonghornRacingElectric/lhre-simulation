from __future__ import annotations

import csv
import os
from pathlib import Path
from typing import Any

os.environ.setdefault("MPLCONFIGDIR", str(Path(__file__).resolve().parent / ".matplotlib-cache"))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import yaml


STUDY_DIR = Path(__file__).resolve().parent


RISKS: list[dict[str, Any]] = [
    {
        "variable": "Tire peak mu and load sensitivity",
        "severity": 5,
        "uncertainty": 5,
        "detectability": 3,
        "evidence": "VDYN-006, VDYN-011, VDYN-015",
        "correlation_action": "GGV capture, pressure/temp sweep, tire surface temperature",
    },
    {
        "variable": "Cornering stiffness",
        "severity": 5,
        "uncertainty": 4,
        "detectability": 3,
        "evidence": "VDYN-008, VDYN-016",
        "correlation_action": "Step steer, steady-state radius, slip-angle inference",
    },
    {
        "variable": "Relaxation length / force build-up",
        "severity": 4,
        "uncertainty": 4,
        "detectability": 3,
        "evidence": "VDYN-009, VDYN-016",
        "correlation_action": "Step steer rise/phase, sine steer, speed sweep",
    },
    {
        "variable": "Delivered drive force / power limit",
        "severity": 5,
        "uncertainty": 3,
        "detectability": 2,
        "evidence": "VDYN-002, VDYN-011, VDYN-015",
        "correlation_action": "Torque request/delivery, pack power/current, acceleration repeats",
    },
    {
        "variable": "CG height and mass distribution",
        "severity": 4,
        "uncertainty": 3,
        "detectability": 2,
        "evidence": "VDYN-001, VDYN-011",
        "correlation_action": "Corner weights, CG estimate, mass audit",
    },
    {
        "variable": "Aero downforce scale and platform sensitivity",
        "severity": 4,
        "uncertainty": 4,
        "detectability": 4,
        "evidence": "AERO-002, VDYN-012, VDYN-015",
        "correlation_action": "Coastdown, aero-on/off, ride-height-vs-speed",
    },
    {
        "variable": "Aero drag scale",
        "severity": 3,
        "uncertainty": 3,
        "detectability": 3,
        "evidence": "AERO-003, VDYN-012, VDYN-015",
        "correlation_action": "Coastdown, power-vs-speed residual",
    },
    {
        "variable": "Chassis torsional stiffness",
        "severity": 4,
        "uncertainty": 3,
        "detectability": 3,
        "evidence": "CHASSIS-003, VDYN-013, VDYN-016",
        "correlation_action": "Torsional fixture, setup response check",
    },
    {
        "variable": "Static alignment and toe repeatability",
        "severity": 3,
        "uncertainty": 3,
        "detectability": 2,
        "evidence": "VDYN-014",
        "correlation_action": "Alignment sheet before/after runs, tire temperature spread",
    },
    {
        "variable": "Frame-side hardpoint manufacturing tolerance",
        "severity": 4,
        "uncertainty": 3,
        "detectability": 3,
        "evidence": "VDYN-017",
        "correlation_action": "Built-frame CMM/fixture audit, update vehicle.yml to measured hardpoints",
    },
    {
        "variable": "DAQ channel availability and calibration",
        "severity": 5,
        "uncertainty": 3,
        "detectability": 4,
        "evidence": "DE-004",
        "correlation_action": "Channel-rate audit, calibration records, run-review dashboard",
    },
]


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def ranked_rows() -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for risk in RISKS:
        row = dict(risk)
        row["rpn"] = int(row["severity"]) * int(row["uncertainty"]) * int(row["detectability"])
        rows.append(row)
    return sorted(rows, key=lambda row: row["rpn"], reverse=True)


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot_risk(rows: list[dict[str, Any]], path: Path) -> None:
    top = rows[:8]
    fig, ax = plt.subplots(figsize=(8.8, 5.0))
    labels = [row["variable"] for row in top]
    values = [row["rpn"] for row in top]
    ax.barh(labels, values, color="#9a5143")
    ax.set_xlabel("Risk priority number")
    ax.set_title("Correlation Priority From Sensitivity x Uncertainty")
    ax.invert_yaxis()
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=220)
    plt.close(fig)


def main() -> None:
    cfg = load_yaml(STUDY_DIR / "study.yml")
    outputs = STUDY_DIR / cfg["outputs"]["data_dir"]
    plots = STUDY_DIR / cfg["outputs"]["figures_dir"]
    rows = ranked_rows()
    write_csv(outputs / "risk_correlation_priority.csv", rows)
    plot_risk(rows, plots / "risk_priority.png")

    top = rows[0]
    lines = [
        "# DE-005 Results",
        "",
        "## Finding",
        "",
        "**PASS:** the package now ranks the assumptions most likely to change the design conclusion and assigns each one to a correlation action.",
        "",
        "![Risk priority](plots/risk_priority.png)",
        "",
        "## Summary",
        "",
        f"- Risk variables ranked: `{len(rows)}`",
        f"- Highest current priority: `{top['variable']}` with RPN `{top['rpn']}`",
        "- Highest-priority cluster: tire mu/load sensitivity, aero scale/platform, DAQ availability, cornering stiffness, relaxation response",
        "",
        "## Design Implication",
        "",
        "The team should correlate the tire and DAQ spine first, then close aero and chassis stiffness. Those variables have enough sensitivity and uncertainty to move the vehicle-level conclusions.",
    ]
    (STUDY_DIR / cfg["outputs"]["results_markdown"]).write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
