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


CROSSWALK: list[dict[str, Any]] = [
    {
        "category": "Overall Vehicle",
        "points": 25,
        "coverage": "strong",
        "primary_reports": "Index, VDYN, Aero, Chassis",
        "story": "The vehicle is justified from goals to source model, envelope, platform, load paths, and validation closure.",
        "evidence": "VDYN-001/002, AERO-001/002/003, CHASSIS-001/002/003",
        "next_validation": "Built-car audit: mass, corner weights, wheelbase, track, ride heights, alignment.",
    },
    {
        "category": "Vehicle Dynamics",
        "points": 25,
        "coverage": "strong",
        "primary_reports": "VDYN",
        "story": "Contact-patch behavior is modeled from source vehicle audit through envelope, full-model response, tires, and setup authority.",
        "evidence": "VDYN-001/002/003/004/005",
        "next_validation": "GG capture, step/sine steer, steady-state balance, tire pressure/temperature logging.",
    },
    {
        "category": "Aerodynamics",
        "points": 15,
        "coverage": "strong",
        "primary_reports": "Aero, VDYN",
        "story": "Aero is defended as a platform-sensitive vehicle system, not an isolated downforce number.",
        "evidence": "AERO-001/002/003",
        "next_validation": "Coastdown, aero-on/off, ride-height-vs-speed, pitch-moment-to-load-split conversion.",
    },
    {
        "category": "Powertrain",
        "points": 30,
        "coverage": "interface",
        "primary_reports": "VDYN, Aero",
        "story": "This package defines powertrain vehicle-level interfaces: drive-force requirement, drag power, acceleration envelope, brake/regen correlation hooks.",
        "evidence": "VDYN-002, AERO-003",
        "next_validation": "Owner artifact needed: pack energy/thermal/current model, torque delivery, endurance energy, dyno/HIL, regen/brake strategy.",
    },
    {
        "category": "Chassis",
        "points": 30,
        "coverage": "strong",
        "primary_reports": "Chassis, VDYN, Aero",
        "story": "Chassis is defended as the structure preserving modeled contact-patch behavior through hardpoints, loads, stiffness, and validation.",
        "evidence": "CHASSIS-001/002/003",
        "next_validation": "FEA/fixture substantiation, torsional stiffness test, tab/link/upright inspection.",
    },
    {
        "category": "Driver Interface",
        "points": 15,
        "coverage": "interface",
        "primary_reports": "VDYN",
        "story": "This package defines driver-confidence metrics: steering/yaw/ay response, overshoot, braking envelope, and driver-comment correlation.",
        "evidence": "VDYN-003/004/005",
        "next_validation": "Owner artifact needed: ergonomics, controls, brake feel, visibility, egress, run-linked driver feedback form.",
    },
    {
        "category": "Low Voltage/Data Acquisition",
        "points": 10,
        "coverage": "interface",
        "primary_reports": "Index, VDYN, Aero",
        "story": "DAQ is framed as the correlation system for every model claim: source audit, GG, aero, tire, setup, and powertrain interfaces.",
        "evidence": "Correlation sections in VDYN/Aero/Chassis reports",
        "next_validation": "Owner artifact needed: channel list, rates, filtering, dashboards, run review workflow.",
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


def plot_coverage(rows: list[dict[str, Any]], path: Path) -> None:
    color_for = {"strong": "#2f7d59", "interface": "#c6842f", "gap": "#b94a48"}
    fig, ax = plt.subplots(figsize=(8.2, 4.8))
    labels = [row["category"] for row in rows]
    points = [float(row["points"]) for row in rows]
    colors = [color_for[row["coverage"]] for row in rows]
    ax.bar(labels, points, color=colors)
    ax.set_ylabel("Score-sheet points")
    ax.set_title("Design Event Rubric Coverage By Current Package")
    ax.tick_params(axis="x", rotation=30)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=220)
    plt.close(fig)


def main() -> None:
    cfg = load_yaml(STUDY_DIR / "study.yml")
    outputs = STUDY_DIR / cfg["outputs"]["data_dir"]
    plots = STUDY_DIR / cfg["outputs"]["figures_dir"]
    write_csv(outputs / "rubric_crosswalk.csv", CROSSWALK)
    plot_coverage(CROSSWALK, plots / "rubric_coverage.png")

    strong_points = sum(int(row["points"]) for row in CROSSWALK if row["coverage"] == "strong")
    interface_points = sum(int(row["points"]) for row in CROSSWALK if row["coverage"] == "interface")
    lines = [
        "# DE-001 Results",
        "",
        "## Finding",
        "",
        "**PASS:** every 2026 EV Design score-sheet category is mapped to a story, evidence source, and next validation action.",
        "",
        "The current simulation package strongly supports `95` points directly across Overall Vehicle, Vehicle Dynamics, Aerodynamics, and Chassis. It also provides interface evidence for `55` additional points across Powertrain, Driver Interface, and LV/DAQ, but those categories still need owner artifacts and test data before final design judging.",
        "",
        "![Rubric coverage](plots/rubric_coverage.png)",
        "",
        "## Coverage Summary",
        "",
        f"- Strong direct coverage: `{strong_points}` points",
        f"- Interface coverage needing owner artifacts: `{interface_points}` points",
        "- Unmapped categories: `0`",
        "",
        "## Design Implication",
        "",
        "The top-level design story should not pretend simulation alone completes every category. It should use VDYN, Aero, and Chassis as the integrated spine, then explicitly hand powertrain, driver interface, and LV/DAQ their validation requests.",
    ]
    (STUDY_DIR / cfg["outputs"]["results_markdown"]).write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
