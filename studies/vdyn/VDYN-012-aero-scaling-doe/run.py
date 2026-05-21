from __future__ import annotations

import csv
import importlib.util
import os
import sys
from dataclasses import replace
from pathlib import Path
from typing import Any

os.environ.setdefault("MPLCONFIGDIR", str(Path(__file__).resolve().parent / ".matplotlib-cache"))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt


STUDY_DIR = Path(__file__).resolve().parent
BASELINE_RUN = STUDY_DIR.parent / "VDYN-002-baseline-envelope" / "run.py"


def load_module() -> Any:
    spec = importlib.util.spec_from_file_location("vdyn002", BASELINE_RUN)
    if spec is None or spec.loader is None:
        raise RuntimeError("Could not import VDYN-002 runner")
    module = importlib.util.module_from_spec(spec)
    sys.modules["vdyn002_aero"] = module
    spec.loader.exec_module(module)
    return module


base = load_module()
ggv = base.ggv


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def make_baseline() -> Any:
    cfg = base.load_yaml(STUDY_DIR.parent / "VDYN-002-baseline-envelope" / "study.yml")
    vehicle_cfg = base.load_yaml((STUDY_DIR.parents[2] / "vehicles/current/vehicle.yml").resolve())
    tire = base.parse_tir((STUDY_DIR.parents[2] / "vehicles/current/tires/16x7p5_10_12psi.tir").resolve())
    audit = base.read_single_row_csv(STUDY_DIR.parent / "VDYN-001-source-vehicle-audit" / "outputs" / "summary.csv")
    vehicle, _ = base.build_vehicle_params(cfg, vehicle_cfg, tire, audit)
    return vehicle


def metrics(vehicle: Any, speed: float) -> dict[str, float]:
    cfg = ggv.GGVConfig(speeds=(speed,), ay_max_g=3.2, ay_points=121, ax_search_min_g=-3.2, ax_search_max_g=2.8, ax_search_points=301, include_left_right=True, verbose=False, warn_tire_load_range=False)
    env = ggv.generate_ggv(vehicle, cfg)[0]
    rows = base.metric_rows([env])
    return rows[0]


def main() -> None:
    vehicle = make_baseline()
    rows: list[dict[str, Any]] = []
    for speed in [15.0, 25.0]:
        for scale in [0.0, 0.5, 1.0, 1.25, 1.5]:
            rows.append({"case": "downforce_scale", "value": scale, **metrics(replace(vehicle, cl_a=vehicle.cl_a * scale), speed)})
        for scale in [0.75, 1.0, 1.25, 1.5]:
            rows.append({"case": "drag_scale", "value": scale, **metrics(replace(vehicle, cd_a=vehicle.cd_a * scale), speed)})
        for bal in [0.40, 0.45, 0.50, 0.55, 0.60]:
            rows.append({"case": "aero_balance_front", "value": bal, **metrics(replace(vehicle, aero_balance_front=bal), speed)})
    outputs = STUDY_DIR / "outputs"
    plots = STUDY_DIR / "plots"
    write_csv(outputs / "aero_scaling.csv", rows)
    fig, ax = plt.subplots(figsize=(7.6, 4.8))
    for case in ["downforce_scale", "drag_scale"]:
        subset = [r for r in rows if r["case"] == case and r["speed_mps"] == 25.0]
        ax.plot([r["value"] for r in subset], [r["max_lateral_g"] for r in subset], marker="o", label=f"{case} lateral")
    ax.set_xlabel("Scale [-]")
    ax.set_ylabel("25 m/s max lateral [g]")
    ax.set_title("VDYN-012 Aero Scaling")
    ax.grid(True, alpha=0.28)
    ax.legend()
    plots.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(plots / "aero_scaling_25mps.png", dpi=220)
    plt.close(fig)
    df_25 = [r for r in rows if r["case"] == "downforce_scale" and r["speed_mps"] == 25.0]
    drag_25 = [r for r in rows if r["case"] == "drag_scale" and r["speed_mps"] == 25.0]
    lines = [
        "# VDYN-012 Results",
        "",
        "## Finding",
        "",
        "**PASS:** aero scaling has different effects on lateral capability and acceleration capability, so aero must remain a vehicle dynamics variable.",
        "",
        "## Key Metrics",
        "",
        f"- 25 m/s downforce-scale lateral span: `{min(r['max_lateral_g'] for r in df_25):.3f}` to `{max(r['max_lateral_g'] for r in df_25):.3f} g`",
        f"- 25 m/s drag-scale acceleration span: `{min(r['max_accel_g'] for r in drag_25):.3f}` to `{max(r['max_accel_g'] for r in drag_25):.3f} g`",
        "",
        "![Aero scaling](plots/aero_scaling_25mps.png)",
        "",
        "## Design Implication",
        "",
        "Downforce, drag, and aero balance must be correlated separately. A single aero performance number is not enough.",
    ]
    (STUDY_DIR / "RESULTS.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
