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
SUBSYSTEMS = ["VDYN", "Tire", "Aero", "Chassis", "PTN", "DI", "LV/DAQ"]


INTERFACES: list[dict[str, Any]] = [
    {
        "from": "Tire",
        "to": "VDYN",
        "criticality": 5,
        "variables": "mu, cornering stiffness, relaxation scale, combined slip, pressure, temperature",
        "evidence": "VDYN-006/007/008/009/010",
        "validation": "Tire pressure/temp, step steer, slip-angle inference, GG envelope",
    },
    {
        "from": "Aero",
        "to": "VDYN",
        "criticality": 5,
        "variables": "downforce, drag, aero balance, ride-height reference, speed scaling",
        "evidence": "AERO-001/002/003, VDYN-012",
        "validation": "Coastdown, aero-on/off, ride-height-vs-speed, weather normalization",
    },
    {
        "from": "Chassis",
        "to": "VDYN",
        "criticality": 5,
        "variables": "hardpoints, motion ratios, roll stiffness, torsional stiffness, compliance",
        "evidence": "CHASSIS-001/003, VDYN-005/013/014",
        "validation": "Alignment audit, torsional fixture, setup sheet, compliance checks",
    },
    {
        "from": "PTN",
        "to": "VDYN",
        "criticality": 4,
        "variables": "delivered drive force, torque limits, regen blending, pack power, thermal derate",
        "evidence": "VDYN-002/011, AERO-003",
        "validation": "Torque request/delivery, pack current/temp, dyno/HIL, acceleration traces",
    },
    {
        "from": "DI",
        "to": "VDYN",
        "criticality": 4,
        "variables": "steering input, brake input, driver confidence, control reach, pedal feel",
        "evidence": "VDYN-003/005/014",
        "validation": "Driver comment form linked to run ID, step steer, braking repeats",
    },
    {
        "from": "LV/DAQ",
        "to": "VDYN",
        "criticality": 5,
        "variables": "speed, ax, ay, yaw, steering, wheel speed, brake pressure, torque, ride height",
        "evidence": "DE-004 and report correlation plans",
        "validation": "Channel-rate audit, calibration sheets, synchronized run logs",
    },
    {
        "from": "Aero",
        "to": "Chassis",
        "criticality": 4,
        "variables": "mount loads, pitch moment, ride-height sensitivity, floor clearance",
        "evidence": "AERO-001/002, CHASSIS-002",
        "validation": "Aero mount inspection, ride-height sweep, load-path FEA/fixture",
    },
    {
        "from": "Chassis",
        "to": "PTN",
        "criticality": 3,
        "variables": "pack/motor mounts, torque reaction, service access, cooling package space",
        "evidence": "CHASSIS-001/002",
        "validation": "Mount inspection, torque reaction case, service check",
    },
    {
        "from": "LV/DAQ",
        "to": "PTN",
        "criticality": 4,
        "variables": "pack voltage/current/temp, inverter/motor temps, torque command/delivery",
        "evidence": "DE-004",
        "validation": "Dyno/HIL, endurance log review, thermal dashboard",
    },
]


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def matrix(rows: list[dict[str, Any]]) -> list[list[float]]:
    idx = {name: i for i, name in enumerate(SUBSYSTEMS)}
    mat = [[0.0 for _ in SUBSYSTEMS] for _ in SUBSYSTEMS]
    for row in rows:
        i = idx[row["from"]]
        j = idx[row["to"]]
        mat[i][j] = float(row["criticality"])
        mat[j][i] = max(mat[j][i], float(row["criticality"]) * 0.8)
    return mat


def plot_matrix(rows: list[dict[str, Any]], path: Path) -> None:
    mat = matrix(rows)
    fig, ax = plt.subplots(figsize=(6.3, 5.6))
    im = ax.imshow(mat, cmap="viridis", vmin=0, vmax=5)
    ax.set_xticks(range(len(SUBSYSTEMS)), SUBSYSTEMS, rotation=35, ha="right")
    ax.set_yticks(range(len(SUBSYSTEMS)), SUBSYSTEMS)
    ax.set_title("Subsystem Interface Criticality")
    for i, row in enumerate(mat):
        for j, val in enumerate(row):
            if val:
                ax.text(j, i, f"{val:.0f}", ha="center", va="center", color="white")
    fig.colorbar(im, ax=ax, label="Criticality (0-5)")
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=220)
    plt.close(fig)


def main() -> None:
    cfg = load_yaml(STUDY_DIR / "study.yml")
    outputs = STUDY_DIR / cfg["outputs"]["data_dir"]
    plots = STUDY_DIR / cfg["outputs"]["figures_dir"]
    write_csv(outputs / "interface_control_matrix.csv", INTERFACES)
    plot_matrix(INTERFACES, plots / "interface_criticality_matrix.png")

    high = [row for row in INTERFACES if int(row["criticality"]) >= 5]
    lines = [
        "# DE-003 Results",
        "",
        "## Finding",
        "",
        "**PASS:** the high-risk vehicle behavior is concentrated in measurable subsystem interfaces with named evidence and validation paths.",
        "",
        "![Interface criticality matrix](plots/interface_criticality_matrix.png)",
        "",
        "## Summary",
        "",
        f"- Interfaces controlled: `{len(INTERFACES)}`",
        f"- Critical interfaces rated 5/5: `{len(high)}`",
        "- Highest-priority interfaces: tire-VDYN, aero-VDYN, chassis-VDYN, LV/DAQ-VDYN",
        "",
        "## Design Implication",
        "",
        "The vehicle should be reviewed as a system of exchange variables. Tire force, aero platform, chassis stiffness, delivered torque, driver inputs, and DAQ channels are the control points that make the report package defensible.",
    ]
    (STUDY_DIR / cfg["outputs"]["results_markdown"]).write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
