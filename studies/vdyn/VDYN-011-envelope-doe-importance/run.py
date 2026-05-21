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
import yaml


STUDY_DIR = Path(__file__).resolve().parent
BASELINE_RUN = STUDY_DIR.parent / "VDYN-002-baseline-envelope" / "run.py"


def load_module() -> Any:
    spec = importlib.util.spec_from_file_location("vdyn002", BASELINE_RUN)
    if spec is None or spec.loader is None:
        raise RuntimeError("Could not import VDYN-002 runner")
    module = importlib.util.module_from_spec(spec)
    sys.modules["vdyn002"] = module
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


def metrics(vehicle: Any) -> dict[str, float]:
    cfg = ggv.GGVConfig(
        speeds=(15.0,),
        ay_max_g=3.2,
        ay_points=121,
        ax_search_min_g=-3.2,
        ax_search_max_g=2.8,
        ax_search_points=301,
        include_left_right=True,
        verbose=False,
        warn_tire_load_range=False,
    )
    env = ggv.generate_ggv(vehicle, cfg)[0]
    ay_g = env.ay / ggv.G
    accel_g = env.ax_accel / ggv.G
    brake_g = env.ax_brake / ggv.G
    return {
        "max_lateral_g": float(max(abs(v) for v, a, b in zip(ay_g, accel_g, brake_g) if math.isfinite(a) or math.isfinite(b))),
        "max_accel_g": float(max(v for v in accel_g if math.isfinite(v))),
        "max_brake_g": float(abs(min(v for v in brake_g if math.isfinite(v)))),
    }


def make_baseline() -> Any:
    cfg = load_yaml(STUDY_DIR.parent / "VDYN-002-baseline-envelope" / "study.yml")
    vehicle_cfg = base.load_yaml((STUDY_DIR.parents[2] / "vehicles/current/vehicle.yml").resolve())
    tire = base.parse_tir((STUDY_DIR.parents[2] / "vehicles/current/tires/16x7p5_10_12psi.tir").resolve())
    audit = base.read_single_row_csv(STUDY_DIR.parent / "VDYN-001-source-vehicle-audit" / "outputs" / "summary.csv")
    vehicle, _aero = base.build_vehicle_params(cfg, vehicle_cfg, tire, audit)
    return vehicle


def apply_case(vehicle: Any, parameter: str, direction: int) -> Any:
    if parameter == "mass_kg":
        return replace(vehicle, mass=vehicle.mass + direction * 10.0)
    if parameter == "cg_height_m":
        return replace(vehicle, cg_height=vehicle.cg_height + direction * 0.03)
    if parameter == "front_static_frac":
        return replace(vehicle, front_static_frac=min(0.60, max(0.40, vehicle.front_static_frac + direction * 0.03)))
    if parameter == "front_lltd":
        return replace(vehicle, lltd=min(0.65, max(0.35, vehicle.lltd + direction * 0.05)))
    if parameter == "track_width_m":
        return replace(vehicle, track_front=vehicle.track_front + direction * 0.05, track_rear=vehicle.track_rear + direction * 0.05)
    if parameter == "wheelbase_m":
        return replace(vehicle, wheelbase=vehicle.wheelbase + direction * 0.05)
    if parameter == "tire_peak_mu":
        scale = 1.0 + direction * 0.10
        return replace(vehicle, pdx1=vehicle.pdx1 * scale, pdx2=vehicle.pdx2 * scale, pdy1=vehicle.pdy1 * scale, pdy2=vehicle.pdy2 * scale)
    if parameter == "downforce":
        return replace(vehicle, cl_a=vehicle.cl_a * (1.0 + direction * 0.25))
    if parameter == "drag":
        return replace(vehicle, cd_a=vehicle.cd_a * (1.0 + direction * 0.25))
    if parameter == "aero_balance_front":
        return replace(vehicle, aero_balance_front=min(0.65, max(0.35, vehicle.aero_balance_front + direction * 0.05)))
    if parameter == "drive_power":
        return replace(vehicle, max_drive_power=vehicle.max_drive_power * (1.0 + direction * 0.20))
    if parameter == "drive_force":
        return replace(vehicle, max_drive_force=vehicle.max_drive_force * (1.0 + direction * 0.20))
    if parameter == "brake_force":
        return replace(vehicle, max_brake_force=vehicle.max_brake_force * (1.0 + direction * 0.20))
    raise KeyError(parameter)


def plot_tornado(rows: list[dict[str, Any]], metric: str, path: Path) -> None:
    spans = [r for r in rows if r["direction"] == "span"]
    spans = sorted(spans, key=lambda r: abs(float(r[metric])), reverse=True)
    fig, ax = plt.subplots(figsize=(8.0, 5.4))
    ax.barh([r["parameter"] for r in spans], [float(r[metric]) for r in spans], color="#4c78a8")
    ax.invert_yaxis()
    ax.set_xlabel(f"Span in {metric} [g]")
    ax.set_title(f"VDYN-011 DOE Importance: {metric}")
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=220)
    plt.close(fig)


def main() -> None:
    vehicle = make_baseline()
    baseline = metrics(vehicle)
    params = [
        "mass_kg", "cg_height_m", "front_static_frac", "front_lltd", "track_width_m", "wheelbase_m",
        "tire_peak_mu", "downforce", "drag", "aero_balance_front", "drive_power", "drive_force", "brake_force",
    ]
    rows: list[dict[str, Any]] = [{"parameter": "baseline", "direction": "baseline", **baseline}]
    for p in params:
        minus = metrics(apply_case(vehicle, p, -1))
        plus = metrics(apply_case(vehicle, p, 1))
        rows.append({"parameter": p, "direction": "minus", **minus})
        rows.append({"parameter": p, "direction": "plus", **plus})
        rows.append({
            "parameter": p,
            "direction": "span",
            "max_lateral_g": plus["max_lateral_g"] - minus["max_lateral_g"],
            "max_accel_g": plus["max_accel_g"] - minus["max_accel_g"],
            "max_brake_g": plus["max_brake_g"] - minus["max_brake_g"],
        })
    outputs = STUDY_DIR / "outputs"
    plots = STUDY_DIR / "plots"
    write_csv(outputs / "doe_results.csv", rows)
    plot_tornado(rows, "max_lateral_g", plots / "lateral_importance.png")
    plot_tornado(rows, "max_accel_g", plots / "accel_importance.png")
    plot_tornado(rows, "max_brake_g", plots / "brake_importance.png")
    spans = [r for r in rows if r["direction"] == "span"]
    top_lat = max(spans, key=lambda r: abs(float(r["max_lateral_g"])))
    top_acc = max(spans, key=lambda r: abs(float(r["max_accel_g"])))
    top_brake = max(spans, key=lambda r: abs(float(r["max_brake_g"])))
    lines = [
        "# VDYN-011 Results",
        "",
        "## Finding",
        "",
        "**PASS:** the EnvelopeSim DOE ranks the current high-leverage vehicle-level variables.",
        "",
        "## Top Sensitivities",
        "",
        f"- Lateral envelope top span: `{top_lat['parameter']}` at `{float(top_lat['max_lateral_g']):+.3f} g`",
        f"- Acceleration envelope top span: `{top_acc['parameter']}` at `{float(top_acc['max_accel_g']):+.3f} g`",
        f"- Braking envelope top span: `{top_brake['parameter']}` at `{float(top_brake['max_brake_g']):+.3f} g`",
        "",
        "![Lateral importance](plots/lateral_importance.png)",
        "",
        "![Acceleration importance](plots/accel_importance.png)",
        "",
        "![Braking importance](plots/brake_importance.png)",
        "",
        "## Design Implication",
        "",
        "Validation and design effort should follow this ranking: the highest-span variables deserve the most careful correlation before final setup decisions.",
    ]
    (STUDY_DIR / "RESULTS.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
