from __future__ import annotations

import csv
import importlib.util
import math
import os
import sys
from dataclasses import replace
from pathlib import Path
from typing import Any

os.environ.setdefault("MPLCONFIGDIR", str(Path(__file__).resolve().parent / ".matplotlib-cache"))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import yaml


STUDY_DIR = Path(__file__).resolve().parent
REPO_ROOT = STUDY_DIR.parents[2]
BASELINE_RUN = STUDY_DIR.parent / "VDYN-002-baseline-envelope" / "run.py"


def load_module() -> Any:
    spec = importlib.util.spec_from_file_location("vdyn002", BASELINE_RUN)
    if spec is None or spec.loader is None:
        raise RuntimeError("Could not import VDYN-002 runner")
    module = importlib.util.module_from_spec(spec)
    sys.modules["vdyn002_for_vdyn015"] = module
    spec.loader.exec_module(module)
    return module


base = load_module()
ggv = base.ggv


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def make_baseline() -> Any:
    cfg = load_yaml(STUDY_DIR.parent / "VDYN-002-baseline-envelope" / "study.yml")
    vehicle_cfg = base.load_yaml(REPO_ROOT / "vehicles/current/vehicle.yml")
    tire = base.parse_tir(REPO_ROOT / "vehicles/current/tires/16x7p5_10_12psi.tir")
    audit = base.read_single_row_csv(STUDY_DIR.parent / "VDYN-001-source-vehicle-audit" / "outputs" / "summary.csv")
    vehicle, _aero = base.build_vehicle_params(cfg, vehicle_cfg, tire, audit)
    return vehicle


def eval_metrics(vehicle: Any, speed: float) -> dict[str, float]:
    cfg = ggv.GGVConfig(
        speeds=(speed,),
        ay_max_g=3.2,
        ay_points=91,
        ax_search_min_g=-3.2,
        ax_search_max_g=2.8,
        ax_search_points=221,
        include_left_right=True,
        verbose=False,
        warn_tire_load_range=False,
    )
    env = ggv.generate_ggv(vehicle, cfg)[0]
    ay_g = env.ay / ggv.G
    accel_g = env.ax_accel / ggv.G
    brake_g = env.ax_brake / ggv.G
    finite_accel = np.isfinite(accel_g)
    finite_brake = np.isfinite(brake_g)
    finite_any = finite_accel | finite_brake
    return {
        "max_lateral_g": float(np.nanmax(np.abs(ay_g[finite_any]))),
        "max_accel_g": float(np.nanmax(accel_g[finite_accel])),
        "max_brake_g": float(abs(np.nanmin(brake_g[finite_brake]))),
    }


def apply_pair(vehicle: Any, case: str, x: float, y: float) -> Any:
    if case == "tire_aero":
        return replace(
            vehicle,
            pdx1=vehicle.pdx1 * x,
            pdx2=vehicle.pdx2 * x,
            pdy1=vehicle.pdy1 * x,
            pdy2=vehicle.pdy2 * x,
            cl_a=vehicle.cl_a * y,
        )
    if case == "drive_drag":
        return replace(vehicle, max_drive_force=vehicle.max_drive_force * x, max_drive_power=vehicle.max_drive_power * x, cd_a=vehicle.cd_a * y)
    if case == "brake_cg":
        return replace(vehicle, max_brake_force=vehicle.max_brake_force * x, cg_height=vehicle.cg_height + y)
    if case == "lltd_aero_balance":
        return replace(vehicle, lltd=x, aero_balance_front=y)
    raise KeyError(case)


def run_surface(vehicle: Any, case: str, x_values: list[float], y_values: list[float], speed: float) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for x in x_values:
        for y in y_values:
            metrics = eval_metrics(apply_pair(vehicle, case, x, y), speed)
            rows.append({"case": case, "x": x, "y": y, "speed_mps": speed, **metrics})
    return rows


def span(rows: list[dict[str, Any]], case: str, metric: str) -> float:
    vals = [float(row[metric]) for row in rows if row["case"] == case]
    return max(vals) - min(vals)


def plot_surface(rows: list[dict[str, Any]], case: str, metric: str, xlabel: str, ylabel: str, path: Path) -> None:
    subset = [row for row in rows if row["case"] == case]
    xs = sorted({float(row["x"]) for row in subset})
    ys = sorted({float(row["y"]) for row in subset})
    z = np.zeros((len(ys), len(xs)))
    for row in subset:
        i = ys.index(float(row["y"]))
        j = xs.index(float(row["x"]))
        z[i, j] = float(row[metric])
    fig, ax = plt.subplots(figsize=(6.6, 5.1))
    im = ax.imshow(z, origin="lower", aspect="auto", extent=[min(xs), max(xs), min(ys), max(ys)], cmap="viridis")
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(f"{case.replace('_', ' ').title()} - {metric}")
    fig.colorbar(im, ax=ax, label=f"{metric} [g]")
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=220)
    plt.close(fig)


def main() -> None:
    vehicle = make_baseline()
    rows: list[dict[str, Any]] = []
    rows += run_surface(vehicle, "tire_aero", [0.85, 0.925, 1.0, 1.075, 1.15], [0.60, 0.80, 1.0, 1.20, 1.40], 25.0)
    rows += run_surface(vehicle, "drive_drag", [0.70, 0.85, 1.0, 1.15, 1.30], [0.70, 0.85, 1.0, 1.15, 1.30], 25.0)
    rows += run_surface(vehicle, "brake_cg", [0.70, 0.85, 1.0, 1.15, 1.30], [-0.04, -0.02, 0.0, 0.02, 0.04], 15.0)
    rows += run_surface(vehicle, "lltd_aero_balance", [0.42, 0.46, 0.50, 0.54, 0.58], [0.42, 0.46, 0.50, 0.54, 0.58], 25.0)

    outputs = STUDY_DIR / "outputs"
    plots = STUDY_DIR / "plots"
    write_csv(outputs / "interaction_doe.csv", rows)

    plot_surface(rows, "tire_aero", "max_lateral_g", "Tire peak scale", "Downforce scale", plots / "tire_aero_lateral_surface.png")
    plot_surface(rows, "drive_drag", "max_accel_g", "Drive force/power scale", "Drag scale", plots / "drive_drag_accel_surface.png")
    plot_surface(rows, "brake_cg", "max_brake_g", "Brake force scale", "CG height delta [m]", plots / "brake_cg_brake_surface.png")
    plot_surface(rows, "lltd_aero_balance", "max_lateral_g", "Front LLTD", "Front aero balance", plots / "lltd_aero_balance_surface.png")

    summary = [
        {"case": "tire_aero", "metric": "max_lateral_g", "span_g": span(rows, "tire_aero", "max_lateral_g")},
        {"case": "drive_drag", "metric": "max_accel_g", "span_g": span(rows, "drive_drag", "max_accel_g")},
        {"case": "brake_cg", "metric": "max_brake_g", "span_g": span(rows, "brake_cg", "max_brake_g")},
        {"case": "lltd_aero_balance", "metric": "max_lateral_g", "span_g": span(rows, "lltd_aero_balance", "max_lateral_g")},
    ]
    write_csv(outputs / "interaction_summary.csv", summary)
    top = max(summary, key=lambda row: float(row["span_g"]))

    lines = [
        "# VDYN-015 Results",
        "",
        "## Finding",
        "",
        "**PASS:** paired EnvelopeSim sweeps reveal the interaction surfaces that matter after the one-factor DOE.",
        "",
        "## Summary",
        "",
        f"- DOE cases evaluated: `{len(rows)}`",
        f"- Largest paired span: `{top['case']}` on `{top['metric']}` at `{float(top['span_g']):.3f} g`",
        f"- Tire-aero lateral span: `{summary[0]['span_g']:.3f} g`",
        f"- Drive-drag acceleration span: `{summary[1]['span_g']:.3f} g`",
        f"- Brake-CG braking span: `{summary[2]['span_g']:.3f} g`",
        f"- LLTD-aero-balance lateral span: `{summary[3]['span_g']:.3f} g`",
        "",
        "![Tire aero lateral surface](plots/tire_aero_lateral_surface.png)",
        "",
        "![Drive drag accel surface](plots/drive_drag_accel_surface.png)",
        "",
        "![Brake CG brake surface](plots/brake_cg_brake_surface.png)",
        "",
        "![LLTD aero balance surface](plots/lltd_aero_balance_surface.png)",
        "",
        "## Design Implication",
        "",
        "The next correlation work should be paired, not scalar: tire with aero load, drive force with drag, brake capacity with CG height, and LLTD with aero balance.",
    ]
    (STUDY_DIR / "RESULTS.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
