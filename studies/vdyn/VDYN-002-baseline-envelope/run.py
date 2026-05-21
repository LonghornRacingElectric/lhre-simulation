from __future__ import annotations

import csv
import math
import os
import re
import sys
from pathlib import Path
from typing import Any

os.environ.setdefault(
    "MPLCONFIGDIR",
    str(Path(__file__).resolve().parent / ".matplotlib-cache"),
)

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import yaml


REPO_ROOT = Path(__file__).resolve().parents[3]
STUDY_DIR = Path(__file__).resolve().parent
BOBSIM_ROOT = REPO_ROOT / "BobSim"

sys.path.insert(0, str(BOBSIM_ROOT))

from _2_EnvelopeSim.GGV import ggv_generation as ggv  # noqa: E402


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise TypeError(f"Expected YAML mapping: {path}")
    return data


def resolve_from(base: Path, raw: str) -> Path:
    candidate = Path(raw)
    if candidate.is_absolute():
        return candidate
    return (base.parent / candidate).resolve()


def read_single_row_csv(path: Path) -> dict[str, str]:
    with path.open("r", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))
    if len(rows) != 1:
        raise ValueError(f"Expected exactly one row in {path}")
    return rows[0]


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def parse_tir(path: Path) -> dict[str, float]:
    values: dict[str, float] = {}
    pattern = re.compile(r"^([A-Za-z0-9_]+)\s*=\s*([-+0-9.Ee]+)")
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("$", 1)[0].strip()
        match = pattern.match(line)
        if not match:
            continue
        key, raw_value = match.groups()
        try:
            values[key.upper()] = float(raw_value)
        except ValueError:
            continue
    return values


def bilinear_interpolate(
    x_grid: list[float],
    y_grid: list[float],
    table: list[list[float]],
    x: float,
    y: float,
) -> float:
    x_arr = np.asarray(x_grid, dtype=float)
    y_arr = np.asarray(y_grid, dtype=float)
    z = np.asarray(table, dtype=float)
    x_clamped = float(np.clip(x, x_arr.min(), x_arr.max()))
    y_clamped = float(np.clip(y, y_arr.min(), y_arr.max()))

    i_hi = int(np.searchsorted(x_arr, x_clamped, side="right"))
    j_hi = int(np.searchsorted(y_arr, y_clamped, side="right"))
    i0 = max(0, min(i_hi - 1, len(x_arr) - 1))
    i1 = max(0, min(i_hi, len(x_arr) - 1))
    j0 = max(0, min(j_hi - 1, len(y_arr) - 1))
    j1 = max(0, min(j_hi, len(y_arr) - 1))
    x0, x1 = x_arr[i0], x_arr[i1]
    y0, y1 = y_arr[j0], y_arr[j1]
    if x0 == x1 and y0 == y1:
        return float(z[i0, j0])
    if x0 == x1:
        return float(np.interp(y_clamped, [y0, y1], [z[i0, j0], z[i0, j1]]))
    if y0 == y1:
        return float(np.interp(x_clamped, [x0, x1], [z[i0, j0], z[i1, j0]]))
    tx = (x_clamped - x0) / (x1 - x0)
    ty = (y_clamped - y0) / (y1 - y0)
    return float(
        (1.0 - tx) * (1.0 - ty) * z[i0, j0]
        + tx * (1.0 - ty) * z[i1, j0]
        + (1.0 - tx) * ty * z[i0, j1]
        + tx * ty * z[i1, j1]
    )


def baseline_ride_heights(vehicle_cfg: dict[str, Any]) -> tuple[float, float]:
    aero = vehicle_cfg["aero"]
    return (
        float(aero["front_ride_height_grid_m"][0]),
        float(aero["rear_ride_height_grid_m"][0]),
    )


def build_vehicle_params(
    cfg: dict[str, Any],
    vehicle_cfg: dict[str, Any],
    tire: dict[str, float],
    audit: dict[str, str],
) -> tuple[ggv.VehicleParams, dict[str, float]]:
    model = cfg["model_assumptions"]
    aero = vehicle_cfg["aero"]
    front_rh, rear_rh = baseline_ride_heights(vehicle_cfg)
    downforce = bilinear_interpolate(
        aero["front_ride_height_grid_m"],
        aero["rear_ride_height_grid_m"],
        aero["downforce_table_n"],
        front_rh,
        rear_rh,
    )
    drag = bilinear_interpolate(
        aero["front_ride_height_grid_m"],
        aero["rear_ride_height_grid_m"],
        aero["drag_table_n"],
        front_rh,
        rear_rh,
    )
    cl_a, cd_a = ggv.force_to_aero_area(
        downforce,
        drag,
        float(aero["reference_speed_m_per_s"]),
    )

    vehicle = ggv.VehicleParams(
        mass=float(audit["total_mass_kg"]),
        wheelbase=float(audit["wheelbase_m"]),
        track_front=float(audit["front_track_m"]),
        track_rear=float(audit["rear_track_m"]),
        cg_height=float(audit["cg_z_m"]),
        front_static_frac=float(audit["front_static_load_fraction"]),
        lltd=float(model["front_lltd_fraction"]),
        cl_a=cl_a,
        cd_a=cd_a,
        aero_balance_front=float(model["aero_balance_front_fraction"]),
        max_drive_power=float(model["max_drive_power_w"]),
        max_drive_force=float(model["max_drive_force_n"]),
        max_brake_force=float(model["max_brake_force_n"]),
        drive_distribution_front=float(model["drive_distribution_front"]),
        brake_distribution_front=float(model["brake_distribution_front"]),
        fz_ref=tire["FNOMIN"],
        fz_min_valid=tire["FZMIN"],
        fz_max_valid=tire["FZMAX"],
        pdx1=tire["PDX1"],
        pdx2=tire["PDX2"],
        pdy1=tire["PDY1"],
        pdy2=tire["PDY2"],
    )
    aero_summary = {
        "baseline_front_ride_height_m": front_rh,
        "baseline_rear_ride_height_m": rear_rh,
        "baseline_downforce_n": downforce,
        "baseline_drag_n": drag,
        "cl_a_m2": cl_a,
        "cd_a_m2": cd_a,
    }
    return vehicle, aero_summary


def envelope_rows(envelopes: list[ggv.GGVEnvelope]) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    for env in envelopes:
        for ay, ax_accel, ax_brake in zip(env.ay, env.ax_accel, env.ax_brake):
            rows.append(
                {
                    "speed_mps": float(env.speed),
                    "ay_g": float(ay / ggv.G),
                    "ax_accel_g": float(ax_accel / ggv.G) if np.isfinite(ax_accel) else math.nan,
                    "ax_brake_g": float(ax_brake / ggv.G) if np.isfinite(ax_brake) else math.nan,
                }
            )
    return rows


def metric_rows(envelopes: list[ggv.GGVEnvelope]) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    for env in envelopes:
        ay_g = env.ay / ggv.G
        ax_accel_g = env.ax_accel / ggv.G
        ax_brake_g = env.ax_brake / ggv.G
        finite_accel = np.isfinite(ax_accel_g)
        finite_brake = np.isfinite(ax_brake_g)
        finite_any = finite_accel | finite_brake
        rows.append(
            {
                "speed_mps": float(env.speed),
                "max_lateral_g": float(np.nanmax(np.abs(ay_g[finite_any]))),
                "max_accel_g": float(np.nanmax(ax_accel_g[finite_accel])),
                "max_brake_g": float(abs(np.nanmin(ax_brake_g[finite_brake]))),
            }
        )
    return rows


def plot_ggv(envelopes: list[ggv.GGVEnvelope], output_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(8.5, 5.8))
    for env in envelopes:
        ay_g = env.ay / ggv.G
        accel = env.ax_accel / ggv.G
        brake = env.ax_brake / ggv.G
        ax.plot(ay_g[np.isfinite(accel)], accel[np.isfinite(accel)], label=f"{env.speed:.0f} m/s accel")
        ax.plot(ay_g[np.isfinite(brake)], brake[np.isfinite(brake)], linestyle="--", label=f"{env.speed:.0f} m/s brake")
    ax.axhline(0.0, color="#555555", linewidth=0.8)
    ax.axvline(0.0, color="#555555", linewidth=0.8)
    ax.set_xlabel("Lateral acceleration [g]")
    ax.set_ylabel("Longitudinal acceleration [g]")
    ax.set_title("VDYN-002 Baseline GGV Envelope")
    ax.grid(True, alpha=0.28)
    ax.legend(fontsize=8, ncol=2)
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def plot_metrics(rows: list[dict[str, float]], output_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(7.5, 4.8))
    speeds = [row["speed_mps"] for row in rows]
    ax.plot(speeds, [row["max_lateral_g"] for row in rows], marker="o", label="Max lateral")
    ax.plot(speeds, [row["max_accel_g"] for row in rows], marker="o", label="Max accel")
    ax.plot(speeds, [row["max_brake_g"] for row in rows], marker="o", label="Max brake")
    ax.set_xlabel("Speed [m/s]")
    ax.set_ylabel("Capability [g]")
    ax.set_title("Baseline Capability vs Speed")
    ax.grid(True, alpha=0.28)
    ax.legend()
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def tire_load_margin_rows(vehicle: ggv.VehicleParams, metrics: list[dict[str, float]]) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    for row in metrics:
        speed = float(row["speed_mps"])
        cases = [
            ("zero_ay_accel", float(row["max_accel_g"]) * ggv.G, 0.0),
            ("zero_ay_brake", -float(row["max_brake_g"]) * ggv.G, 0.0),
            ("max_lateral", 0.0, float(row["max_lateral_g"]) * ggv.G),
        ]
        for case, ax, ay in cases:
            loads = ggv.wheel_loads(vehicle, speed=speed, ax=ax, ay=ay)
            rows.append(
                {
                    "speed_mps": speed,
                    "case": case,
                    "min_fz_n": float(np.min(loads)),
                    "max_fz_n": float(np.max(loads)),
                    "fz_min_valid_n": float(vehicle.fz_min_valid),
                    "fz_max_valid_n": float(vehicle.fz_max_valid),
                    "inside_range": float(np.min(loads) >= vehicle.fz_min_valid and np.max(loads) <= vehicle.fz_max_valid),
                }
            )
    return rows


def write_results(
    cfg: dict[str, Any],
    metrics: list[dict[str, float]],
    load_rows: list[dict[str, float]],
    aero: dict[str, float],
    output_path: Path,
) -> None:
    m15 = min(metrics, key=lambda row: abs(row["speed_mps"] - 15.0))
    inside_count = sum(int(row["inside_range"]) for row in load_rows)
    status = "PASS" if inside_count == len(load_rows) else "CHECK"
    lines = [
        "# VDYN-002 Results",
        "",
        "## Decision Question",
        "",
        cfg["study"]["decision_question"],
        "",
        "## Finding",
        "",
        f"**{status}:** the baseline envelope is coherent enough to proceed to setup and response studies.",
        "",
        "This study is a baseline capability gate only. It does not claim an optimized tire, aero, damping, ARB, or alignment setup.",
        "",
        "## Baseline Inputs",
        "",
        f"- Baseline aero map point: front RH `{aero['baseline_front_ride_height_m']:.5f} m`, rear RH `{aero['baseline_rear_ride_height_m']:.5f} m`",
        f"- Baseline downforce/drag at 15 m/s: `{aero['baseline_downforce_n']:.1f} N` / `{aero['baseline_drag_n']:.1f} N`",
        f"- Assumed front LLTD: `{100.0 * float(cfg['model_assumptions']['front_lltd_fraction']):.2f} %`",
        f"- Assumed front aero balance: `{100.0 * float(cfg['model_assumptions']['aero_balance_front_fraction']):.1f} %`",
        f"- Drive power / force cap: `{float(cfg['model_assumptions']['max_drive_power_w']) / 1000.0:.1f} kW` / `{float(cfg['model_assumptions']['max_drive_force_n']):.0f} N`",
        f"- Brake force cap: `{float(cfg['model_assumptions']['max_brake_force_n']):.0f} N`",
        "",
        "## Representative 15 m/s Metrics",
        "",
        f"- Max lateral: `{m15['max_lateral_g']:.3f} g`",
        f"- Max acceleration: `{m15['max_accel_g']:.3f} g`",
        f"- Max braking: `{m15['max_brake_g']:.3f} g`",
        "",
        "![Baseline GGV](plots/baseline_ggv.png)",
        "",
        "![Capability metrics](plots/capability_metrics.png)",
        "",
        "## Tire Load Range Check",
        "",
        f"- Checked representative zero-ay acceleration, zero-ay braking, and max-lateral load cases at each speed: `{inside_count}/{len(load_rows)}` are inside the tire file vertical-load range.",
        "",
        "## Design Implication",
        "",
        "If this baseline holds after correlation, the next question is not whether the architecture can produce a usable envelope. The next question is which setup and tire operating-window choices make that envelope accessible and confidence-inspiring for the driver.",
        "",
        "## Correlation Closure",
        "",
        "Compare logged GG traces from skidpad, brake, acceleration, and early endurance running against this envelope. If measured performance falls short, update tire, mass, aero, brake, or power assumptions before promoting setup sensitivity results.",
    ]
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    cfg = load_yaml(STUDY_DIR / "study.yml")
    vehicle_path = resolve_from(STUDY_DIR / "study.yml", cfg["inputs"]["vehicle_config"])
    tire_path = resolve_from(STUDY_DIR / "study.yml", cfg["inputs"]["tire_file"])
    audit_path = STUDY_DIR.parent / "VDYN-001-source-vehicle-audit" / "outputs" / "summary.csv"
    vehicle_cfg = load_yaml(vehicle_path)
    tire = parse_tir(tire_path)
    audit = read_single_row_csv(audit_path)
    vehicle, aero_summary = build_vehicle_params(cfg, vehicle_cfg, tire, audit)
    config = ggv.GGVConfig(
        speeds=tuple(float(v) for v in cfg["model_assumptions"]["speeds_mps"]),
        ay_max_g=3.2,
        ay_points=241,
        ax_search_min_g=-3.2,
        ax_search_max_g=2.8,
        ax_search_points=601,
        include_left_right=True,
        verbose=False,
        warn_tire_load_range=False,
    )
    envelopes = ggv.generate_ggv(vehicle, config)
    rows = envelope_rows(envelopes)
    metrics = metric_rows(envelopes)
    load_rows = tire_load_margin_rows(vehicle, metrics)
    outputs = STUDY_DIR / cfg["outputs"]["data_dir"]
    plots = STUDY_DIR / cfg["outputs"]["figures_dir"]
    write_csv(outputs / "ggv_envelope.csv", rows)
    write_csv(outputs / "capability_metrics.csv", metrics)
    write_csv(outputs / "tire_load_range_check.csv", load_rows)
    write_csv(outputs / "aero_baseline.csv", [aero_summary])
    plot_ggv(envelopes, plots / "baseline_ggv.png")
    plot_metrics(metrics, plots / "capability_metrics.png")
    write_results(cfg, metrics, load_rows, aero_summary, STUDY_DIR / cfg["outputs"]["results_markdown"])


if __name__ == "__main__":
    main()
