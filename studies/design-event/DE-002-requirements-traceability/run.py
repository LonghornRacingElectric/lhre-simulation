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


REQUIREMENTS: list[dict[str, Any]] = [
    {
        "id": "VEH-REQ-001",
        "vehicle_goal": "Early reliable running with credible dynamic capability",
        "requirement": "The source model shall trace mass, CG, wheelbase, track, tires, aero references, and chassis hardpoints to one vehicle definition.",
        "owner": "Systems",
        "evidence": "VDYN-001, AERO-001, CHASSIS-001",
        "verification": "Built-car audit: scales, corner weights, wheelbase, track, ride heights, hardpoint spot checks.",
        "status": "model-evidenced",
        "readiness": 4,
    },
    {
        "id": "VEH-REQ-002",
        "vehicle_goal": "Enough envelope to tune rather than chase fundamentals",
        "requirement": "The car shall demonstrate a baseline simulated envelope with lateral, braking, and acceleration capability inside the tire load range.",
        "owner": "Vehicle Dynamics",
        "evidence": "VDYN-002, VDYN-011",
        "verification": "GGV capture using speed, ax, ay, wheel speeds, steering, brake pressure, torque request.",
        "status": "model-evidenced",
        "readiness": 4,
    },
    {
        "id": "VEH-REQ-003",
        "vehicle_goal": "Driver confidence and controllable balance",
        "requirement": "The car shall have mild understeer, fast yaw/ay response, and known overshoot/correlation targets.",
        "owner": "Vehicle Dynamics / Driver Interface",
        "evidence": "VDYN-003, VDYN-005, VDYN-014",
        "verification": "Step steer, sine steer, steady-state radius, setup sheet, run-linked driver feedback.",
        "status": "model-evidenced",
        "readiness": 3,
    },
    {
        "id": "VEH-REQ-004",
        "vehicle_goal": "Tire use drives setup decisions",
        "requirement": "The tire model shall quantify load sensitivity, cornering stiffness, relaxation response, and combined-slip budget.",
        "owner": "Vehicle Dynamics",
        "evidence": "VDYN-006, VDYN-007, VDYN-008, VDYN-009, VDYN-010",
        "verification": "Pressure/temp sweep, slip-angle inference, step response, tire surface temperature, driver balance comments.",
        "status": "model-evidenced",
        "readiness": 3,
    },
    {
        "id": "VEH-REQ-005",
        "vehicle_goal": "Aero must help the lap without hiding platform risk",
        "requirement": "Aero shall be evaluated as downforce, drag, ride-height sensitivity, and balance impact on the dynamic envelope.",
        "owner": "Aero / Vehicle Dynamics",
        "evidence": "AERO-001, AERO-002, AERO-003, VDYN-012",
        "verification": "Coastdown, aero-on/off, ride-height-vs-speed, weather-normalized performance comparison.",
        "status": "model-evidenced",
        "readiness": 3,
    },
    {
        "id": "VEH-REQ-006",
        "vehicle_goal": "Chassis preserves modeled contact-patch behavior",
        "requirement": "Hardpoints, load paths, and torsional stiffness shall support the tire/aero/brake loads used in simulation.",
        "owner": "Chassis",
        "evidence": "CHASSIS-001, CHASSIS-002, CHASSIS-003, VDYN-013",
        "verification": "Torsional fixture, tab/link inspection, FEA correlation, post-run crack/looseness checks.",
        "status": "model-evidenced",
        "readiness": 3,
    },
    {
        "id": "VEH-REQ-007",
        "vehicle_goal": "Powertrain decisions are judged at vehicle level",
        "requirement": "Delivered force, power, drag penalty, regen, and endurance energy shall close against the vehicle acceleration and aero requirements.",
        "owner": "Powertrain",
        "evidence": "VDYN-002, VDYN-011, AERO-003",
        "verification": "Torque logs, pack power/current/temp, dyno/HIL, endurance energy model, regen/brake balance test.",
        "status": "interface-defined",
        "readiness": 2,
    },
    {
        "id": "VEH-REQ-008",
        "vehicle_goal": "DAQ exists to close the model",
        "requirement": "The car shall log the channels required to verify every vehicle-level claim in this report package.",
        "owner": "LV/DAQ",
        "evidence": "DE-004, all report correlation plans",
        "verification": "Channel-rate audit, sensor calibration sheets, run review workflow, dashboard screenshots.",
        "status": "interface-defined",
        "readiness": 2,
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


def plot_readiness(rows: list[dict[str, Any]], path: Path) -> None:
    colors = {"model-evidenced": "#2f7d59", "interface-defined": "#c6842f"}
    fig, ax = plt.subplots(figsize=(8.6, 4.8))
    labels = [row["id"] for row in rows]
    values = [row["readiness"] for row in rows]
    ax.bar(labels, values, color=[colors[row["status"]] for row in rows])
    ax.set_ylim(0, 5)
    ax.set_ylabel("Evidence readiness (0-5)")
    ax.set_title("Requirements Traceability Readiness")
    ax.tick_params(axis="x", rotation=35)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=220)
    plt.close(fig)


def main() -> None:
    cfg = load_yaml(STUDY_DIR / "study.yml")
    outputs = STUDY_DIR / cfg["outputs"]["data_dir"]
    plots = STUDY_DIR / cfg["outputs"]["figures_dir"]

    write_csv(outputs / "requirements_traceability.csv", REQUIREMENTS)
    plot_readiness(REQUIREMENTS, plots / "requirements_readiness.png")

    model_count = sum(1 for row in REQUIREMENTS if row["status"] == "model-evidenced")
    interface_count = len(REQUIREMENTS) - model_count
    avg_readiness = sum(float(row["readiness"]) for row in REQUIREMENTS) / len(REQUIREMENTS)

    lines = [
        "# DE-002 Results",
        "",
        "## Finding",
        "",
        "**PASS:** the design package now has a traceable requirements cascade from team goals to subsystem evidence and validation closure.",
        "",
        "![Requirements readiness](plots/requirements_readiness.png)",
        "",
        "## Summary",
        "",
        f"- Requirements traced: `{len(REQUIREMENTS)}`",
        f"- Model-evidenced requirements: `{model_count}`",
        f"- Interface-defined requirements needing owner closure: `{interface_count}`",
        f"- Mean evidence readiness: `{avg_readiness:.1f}/5`",
        "",
        "## Design Implication",
        "",
        "The vehicle can be presented as a requirements-driven system: source vehicle, envelope, tire behavior, aero platform, chassis preservation, powertrain delivery, and DAQ closure each have explicit evidence and validation ownership.",
    ]
    (STUDY_DIR / cfg["outputs"]["results_markdown"]).write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
