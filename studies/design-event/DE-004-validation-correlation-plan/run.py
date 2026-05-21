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


PLAN: list[dict[str, Any]] = [
    {
        "claim": "Source vehicle represents built car",
        "test": "Built-car audit",
        "required_channels": "scale mass, corner weights, ride heights, wheelbase, track, alignment",
        "pass_fail_metric": "Model geometry/mass inputs updated or confirmed before dynamic correlation",
        "model_update": "vehicle.yml mass, CG, hardpoints, aero reference heights",
        "priority": 5,
    },
    {
        "claim": "Baseline GGV envelope is credible",
        "test": "GGV capture",
        "required_channels": "speed, ax, ay, yaw rate, wheel speeds, steering, brake pressure, torque request",
        "pass_fail_metric": "Peak and shape mismatch explained by tire state, torque limit, or surface condition",
        "model_update": "tire scale, drive force, brake force, drag/downforce",
        "priority": 5,
    },
    {
        "claim": "Transient response is mild and quick",
        "test": "Step steer and sine steer",
        "required_channels": "steering angle, speed, yaw rate, ay, damper/ride-height optional, setup state",
        "pass_fail_metric": "Rise time, overshoot, and gain trend match or define setup change",
        "model_update": "cornering stiffness, relaxation scale, damping, LLTD",
        "priority": 5,
    },
    {
        "claim": "Tire operating window dominates first setup",
        "test": "Pressure/temperature and repeatability sweep",
        "required_channels": "hot/cold pressure, tire surface temp, speed, ax, ay, steering, driver comments",
        "pass_fail_metric": "Pressure/temp targets produce repeatable balance and force utilization",
        "model_update": "mu scale, cornering stiffness, relaxation scale, camber/toe settings",
        "priority": 5,
    },
    {
        "claim": "Aero map improves vehicle envelope without unacceptable drag/platform penalty",
        "test": "Coastdown and aero-on/off comparison",
        "required_channels": "speed, ax, ride heights, weather, pack power, setup state",
        "pass_fail_metric": "Drag and downforce trend match map within actionable bounds",
        "model_update": "CdA/ClA scale, balance, platform map, drag power",
        "priority": 4,
    },
    {
        "claim": "Chassis stiffness preserves setup authority",
        "test": "Torsional fixture and post-run inspection",
        "required_channels": "fixture torque, angular deflection, setup sheet, inspection log",
        "pass_fail_metric": "Measured stiffness keeps suspension authority above target or triggers stiffness fix",
        "model_update": "body torsional stiffness, compliance assumptions, load path margins",
        "priority": 4,
    },
    {
        "claim": "Powertrain delivery matches vehicle acceleration need",
        "test": "Acceleration repeat and endurance-energy review",
        "required_channels": "torque request/delivery, pack voltage/current/temp, motor/inverter temp, speed, ax",
        "pass_fail_metric": "Delivered force and thermal behavior match acceleration/endurance model",
        "model_update": "drive force cap, power cap, derate model, regen/brake split",
        "priority": 4,
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


def plot_priorities(rows: list[dict[str, Any]], path: Path) -> None:
    fig, ax = plt.subplots(figsize=(8.6, 4.8))
    labels = [row["test"] for row in rows]
    values = [row["priority"] for row in rows]
    ax.barh(labels, values, color="#406a8f")
    ax.set_xlim(0, 5)
    ax.set_xlabel("Correlation priority (0-5)")
    ax.set_title("First-Drive Validation Priority")
    ax.invert_yaxis()
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=220)
    plt.close(fig)


def main() -> None:
    cfg = load_yaml(STUDY_DIR / "study.yml")
    outputs = STUDY_DIR / cfg["outputs"]["data_dir"]
    plots = STUDY_DIR / cfg["outputs"]["figures_dir"]
    write_csv(outputs / "validation_correlation_plan.csv", PLAN)
    plot_priorities(PLAN, plots / "validation_priority.png")

    max_priority = sum(1 for row in PLAN if int(row["priority"]) == 5)
    lines = [
        "# DE-004 Results",
        "",
        "## Finding",
        "",
        "**PASS:** every major report claim now has a physical validation test, channel list, pass/fail logic, and model update action.",
        "",
        "![Validation priority](plots/validation_priority.png)",
        "",
        "## Summary",
        "",
        f"- Validation tests defined: `{len(PLAN)}`",
        f"- First-priority tests rated 5/5: `{max_priority}`",
        "- Required data spine: source audit, GGV, step/sine steer, tire pressure/temp, coastdown/aero-on-off, torsional fixture, powertrain delivery logs",
        "",
        "## Design Implication",
        "",
        "The next phase is not more unbounded simulation. It is a closed-loop validation plan where each first-drive test updates one or more admitted model assumptions.",
    ]
    (STUDY_DIR / cfg["outputs"]["results_markdown"]).write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
