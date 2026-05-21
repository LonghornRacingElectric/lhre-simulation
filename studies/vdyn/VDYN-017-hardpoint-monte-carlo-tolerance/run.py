from __future__ import annotations

import csv
import math
import os
import time
from copy import deepcopy
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
INBOARD_KEYS = ["upper_fore_i_m", "upper_aft_i_m", "lower_fore_i_m", "lower_aft_i_m"]


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def line_intersection_yz(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, p4: np.ndarray) -> np.ndarray:
    # Points are [y, z]. Returns the front-view instant center.
    d1 = p2 - p1
    d2 = p4 - p3
    a = np.array([[d1[0], -d2[0]], [d1[1], -d2[1]]], dtype=float)
    b = p3 - p1
    det = float(np.linalg.det(a))
    if abs(det) < 1e-10:
        return np.array([math.nan, math.nan])
    t = np.linalg.solve(a, b)[0]
    return p1 + t * d1


def axle_geometry(vehicle: dict[str, Any], axle: str) -> dict[str, float]:
    s = vehicle[axle]["suspension"]
    upper_i = 0.5 * (np.array(s["upper_fore_i_m"], dtype=float) + np.array(s["upper_aft_i_m"], dtype=float))
    lower_i = 0.5 * (np.array(s["lower_fore_i_m"], dtype=float) + np.array(s["lower_aft_i_m"], dtype=float))
    upper_o = np.array(s["upper_o_m"], dtype=float)
    lower_o = np.array(s["lower_o_m"], dtype=float)
    wc = np.array(s["wheel_center_m"], dtype=float)
    radius = float(vehicle[axle]["wheel"]["radius_m"])

    ui_yz = upper_i[[1, 2]]
    li_yz = lower_i[[1, 2]]
    uo_yz = upper_o[[1, 2]]
    lo_yz = lower_o[[1, 2]]
    wc_yz = wc[[1, 2]]

    ic = line_intersection_yz(ui_yz, uo_yz, li_yz, lo_yz)
    contact = np.array([wc_yz[0], wc_yz[1] - radius])
    if np.any(~np.isfinite(ic)) or abs(ic[0] - contact[0]) < 1e-10:
        roll_center_z = math.nan
    else:
        t = -contact[0] / (ic[0] - contact[0])
        roll_center_z = float(contact[1] + t * (ic[1] - contact[1]))

    swing_arm = ic[0] - wc_yz[0] if np.all(np.isfinite(ic)) else math.nan
    camber_gain_rad_per_m = float(1.0 / swing_arm) if math.isfinite(swing_arm) and abs(swing_arm) > 1e-6 else math.nan
    camber_static_rad = float(math.atan2(upper_o[2] - lower_o[2], upper_o[1] - lower_o[1]) - math.pi / 2.0)

    lower_avg = 0.5 * (np.array(s["lower_fore_i_m"], dtype=float) + np.array(s["lower_aft_i_m"], dtype=float))
    upper_span = float(np.linalg.norm(np.array(s["upper_fore_i_m"], dtype=float) - np.array(s["upper_aft_i_m"], dtype=float)))
    lower_span = float(np.linalg.norm(np.array(s["lower_fore_i_m"], dtype=float) - np.array(s["lower_aft_i_m"], dtype=float)))

    return {
        f"{axle}_roll_center_z_m": roll_center_z,
        f"{axle}_camber_gain_rad_per_m": camber_gain_rad_per_m,
        f"{axle}_static_camber_rad": camber_static_rad,
        f"{axle}_lower_inboard_ref_x_m": float(lower_avg[0]),
        f"{axle}_lower_inboard_ref_y_m": float(lower_avg[1]),
        f"{axle}_lower_inboard_ref_z_m": float(lower_avg[2]),
        f"{axle}_upper_inboard_span_m": upper_span,
        f"{axle}_lower_inboard_span_m": lower_span,
    }


def vehicle_geometry(vehicle: dict[str, Any]) -> dict[str, float]:
    out = {}
    out.update(axle_geometry(vehicle, "front"))
    out.update(axle_geometry(vehicle, "rear"))
    return out


def perturb_vehicle(base_vehicle: dict[str, Any], rng: np.random.Generator, sigma_m: float) -> dict[str, Any]:
    vehicle = deepcopy(base_vehicle)
    for axle in ["front", "rear"]:
        for key in INBOARD_KEYS:
            point = np.array(vehicle[axle]["suspension"][key], dtype=float)
            vehicle[axle]["suspension"][key] = (point + rng.normal(0.0, sigma_m, size=3)).tolist()
    return vehicle


def geometry_deltas(base: dict[str, float], sample: dict[str, float]) -> dict[str, float]:
    out: dict[str, float] = {}
    for axle in ["front", "rear"]:
        out[f"{axle}_camber_gain_delta_pct"] = (
            100.0
            * (sample[f"{axle}_camber_gain_rad_per_m"] - base[f"{axle}_camber_gain_rad_per_m"])
            / max(abs(base[f"{axle}_camber_gain_rad_per_m"]), 1e-9)
        )
        out[f"{axle}_roll_center_delta_mm"] = 1000.0 * (
            sample[f"{axle}_roll_center_z_m"] - base[f"{axle}_roll_center_z_m"]
        )
        for axis in ["x", "y", "z"]:
            out[f"{axle}_aero_ref_{axis}_delta_mm"] = 1000.0 * (
                sample[f"{axle}_lower_inboard_ref_{axis}_m"] - base[f"{axle}_lower_inboard_ref_{axis}_m"]
            )
        out[f"{axle}_upper_inboard_span_delta_mm"] = 1000.0 * (
            sample[f"{axle}_upper_inboard_span_m"] - base[f"{axle}_upper_inboard_span_m"]
        )
        out[f"{axle}_lower_inboard_span_delta_mm"] = 1000.0 * (
            sample[f"{axle}_lower_inboard_span_m"] - base[f"{axle}_lower_inboard_span_m"]
        )
    return out


def percentile_abs(rows: list[dict[str, Any]], key: str, pct: float) -> float:
    return float(np.percentile(np.abs([float(row[key]) for row in rows]), pct))


def mean(rows: list[dict[str, Any]], key: str) -> float:
    return float(np.mean([float(row[key]) for row in rows]))


def std(rows: list[dict[str, Any]], key: str) -> float:
    return float(np.std([float(row[key]) for row in rows], ddof=1))


def summarize(rows: list[dict[str, Any]], sigma_mm: float) -> dict[str, Any]:
    thresholds = {
        "p95_aero_ref_z_delta_mm": 1.5,
        "p95_aero_ref_y_delta_mm": 1.5,
        "p95_roll_center_delta_mm": 6.0,
        "p95_camber_gain_delta_pct": 10.0,
        "p95_inboard_span_delta_mm": 3.0,
    }
    p95_aero_ref = max(percentile_abs(rows, "front_aero_ref_z_delta_mm", 95), percentile_abs(rows, "rear_aero_ref_z_delta_mm", 95))
    p95_aero_ref_y = max(percentile_abs(rows, "front_aero_ref_y_delta_mm", 95), percentile_abs(rows, "rear_aero_ref_y_delta_mm", 95))
    p95_rc = max(percentile_abs(rows, "front_roll_center_delta_mm", 95), percentile_abs(rows, "rear_roll_center_delta_mm", 95))
    p95_camber_gain = max(
        percentile_abs(rows, "front_camber_gain_delta_pct", 95),
        percentile_abs(rows, "rear_camber_gain_delta_pct", 95),
    )
    p95_span = max(
        percentile_abs(rows, "front_upper_inboard_span_delta_mm", 95),
        percentile_abs(rows, "front_lower_inboard_span_delta_mm", 95),
        percentile_abs(rows, "rear_upper_inboard_span_delta_mm", 95),
        percentile_abs(rows, "rear_lower_inboard_span_delta_mm", 95),
    )
    passes = (
        p95_aero_ref <= thresholds["p95_aero_ref_z_delta_mm"]
        and p95_aero_ref_y <= thresholds["p95_aero_ref_y_delta_mm"]
        and p95_rc <= thresholds["p95_roll_center_delta_mm"]
        and p95_camber_gain <= thresholds["p95_camber_gain_delta_pct"]
        and p95_span <= thresholds["p95_inboard_span_delta_mm"]
    )
    return {
        "tolerance_sigma_mm": sigma_mm,
        "approx_two_sigma_band_mm": 2.0 * sigma_mm,
        "samples": len(rows),
        "mean_front_aero_ref_z_delta_mm": mean(rows, "front_aero_ref_z_delta_mm"),
        "std_front_aero_ref_z_delta_mm": std(rows, "front_aero_ref_z_delta_mm"),
        "mean_rear_aero_ref_z_delta_mm": mean(rows, "rear_aero_ref_z_delta_mm"),
        "std_rear_aero_ref_z_delta_mm": std(rows, "rear_aero_ref_z_delta_mm"),
        "p95_aero_ref_z_delta_mm": p95_aero_ref,
        "p95_aero_ref_y_delta_mm": p95_aero_ref_y,
        "p95_roll_center_delta_mm": p95_rc,
        "mean_front_roll_center_delta_mm": mean(rows, "front_roll_center_delta_mm"),
        "std_front_roll_center_delta_mm": std(rows, "front_roll_center_delta_mm"),
        "mean_rear_roll_center_delta_mm": mean(rows, "rear_roll_center_delta_mm"),
        "std_rear_roll_center_delta_mm": std(rows, "rear_roll_center_delta_mm"),
        "p95_front_camber_gain_delta_pct": percentile_abs(rows, "front_camber_gain_delta_pct", 95),
        "p95_rear_camber_gain_delta_pct": percentile_abs(rows, "rear_camber_gain_delta_pct", 95),
        "p95_inboard_span_delta_mm": p95_span,
        "passes_geometry_thresholds": passes,
    }


def plot_tolerance(summary: list[dict[str, Any]], path: Path) -> None:
    fig, ax1 = plt.subplots(figsize=(8.2, 4.8))
    sigmas = [row["tolerance_sigma_mm"] for row in summary]
    ax1.plot(sigmas, [row["p95_aero_ref_z_delta_mm"] for row in summary], marker="o", label="Aero ref z delta [mm]")
    ax1.plot(sigmas, [row["p95_roll_center_delta_mm"] for row in summary], marker="o", label="Roll-center delta [mm]")
    ax1.plot(sigmas, [row["p95_front_camber_gain_delta_pct"] for row in summary], marker="o", label="Front camber-gain delta [%]")
    ax1.plot(sigmas, [row["p95_rear_camber_gain_delta_pct"] for row in summary], marker="o", label="Rear camber-gain delta [%]")
    ax1.plot(sigmas, [row["p95_inboard_span_delta_mm"] for row in summary], marker="o", label="Inboard span delta [mm]")
    ax1.set_xlabel("Independent hardpoint coordinate sigma [mm]")
    ax1.set_ylabel("95th percentile absolute diagnostic [mm or %]")
    ax1.set_title("Hardpoint Tolerance Monte Carlo Summary")
    ax1.grid(True, alpha=0.28)
    ax1.legend()
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=220)
    plt.close(fig)


def plot_hist(rows: list[dict[str, Any]], key: str, label: str, path: Path) -> None:
    fig, ax = plt.subplots(figsize=(7.2, 4.5))
    ax.hist([float(row[key]) for row in rows], bins=60, color="#4c78a8", alpha=0.82)
    ax.set_xlabel(label)
    ax.set_ylabel("Count")
    ax.set_title(f"Monte Carlo Distribution: {label}")
    ax.grid(True, alpha=0.22)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=220)
    plt.close(fig)


def main() -> None:
    start = time.perf_counter()
    cfg = load_yaml(STUDY_DIR / "study.yml")
    vehicle = load_yaml(REPO_ROOT / "vehicles/current/vehicle.yml")
    base_geom = vehicle_geometry(vehicle)
    rng = np.random.default_rng(int(cfg["swept_variables"]["random_seed"]))
    samples_per_tol = int(cfg["swept_variables"]["samples_per_tolerance"])
    sigma_values = [float(v) for v in cfg["swept_variables"]["tolerance_sigma_mm"]]

    all_rows: list[dict[str, Any]] = []
    summary_rows: list[dict[str, Any]] = []
    rows_by_sigma: dict[float, list[dict[str, Any]]] = {}
    sample_id = 0
    for sigma_mm in sigma_values:
        rows: list[dict[str, Any]] = []
        for _ in range(samples_per_tol):
            perturbed = perturb_vehicle(vehicle, rng, sigma_mm / 1000.0)
            geom = vehicle_geometry(perturbed)
            deltas = geometry_deltas(base_geom, geom)
            row = {"sample_id": sample_id, "tolerance_sigma_mm": sigma_mm, **geom, **deltas}
            rows.append(row)
            all_rows.append(row)
            sample_id += 1
        rows_by_sigma[sigma_mm] = rows
        summary_rows.append(summarize(rows, sigma_mm))

    outputs = STUDY_DIR / "outputs"
    plots = STUDY_DIR / "plots"
    write_csv(outputs / "hardpoint_monte_carlo_samples.csv", all_rows)
    write_csv(outputs / "hardpoint_monte_carlo_summary.csv", summary_rows)
    write_csv(outputs / "baseline_geometry.csv", [base_geom])

    plot_tolerance(summary_rows, plots / "tolerance_summary.png")
    chosen = max([row for row in summary_rows if row["passes_geometry_thresholds"]], key=lambda row: row["tolerance_sigma_mm"], default=summary_rows[0])
    chosen_sigma = float(chosen["tolerance_sigma_mm"])
    plot_hist(rows_by_sigma[chosen_sigma], "front_aero_ref_z_delta_mm", f"Front aero-reference z delta at {chosen_sigma:.1f} mm sigma [mm]", plots / "front_aero_ref_z_delta_hist.png")
    plot_hist(rows_by_sigma[chosen_sigma], "front_roll_center_delta_mm", f"Front roll-center delta at {chosen_sigma:.1f} mm sigma [mm]", plots / "front_roll_center_delta_hist.png")
    plot_hist(rows_by_sigma[chosen_sigma], "front_camber_gain_delta_pct", f"Front camber-gain delta at {chosen_sigma:.1f} mm sigma [%]", plots / "front_camber_gain_delta_hist.png")

    runtime_s = time.perf_counter() - start
    provenance = {
        "engine": "study_geometry_monte_carlo",
        "compiled_models": 0,
        "simulated_cases": len(all_rows),
        "runtime_s": runtime_s,
        "notes": "No StandardSim or EnvelopeSim variants were compiled or run by this study.",
    }
    write_csv(outputs / "run_provenance.csv", [provenance])

    status = "PASS" if any(row["passes_geometry_thresholds"] for row in summary_rows) else "CHECK"
    lines = [
        "# VDYN-017 Results",
        "",
        "## Finding",
        "",
        f"**{status}:** inboard hardpoint Monte Carlo variation has been translated into geometry and aero-reference tolerance bands.",
        "",
        "This is the study's own geometry Monte Carlo. It is not an EnvelopeSim output and it is not a StandardSim output. Dynamic-response claims require measured hardpoints to be pushed back into the source vehicle model and rerun through the appropriate simulator.",
        "",
        "## Run Provenance",
        "",
        "- Engine: `study_geometry_monte_carlo`",
        "- Compiled models: `0`",
        f"- Simulated geometry cases: `{len(all_rows)}`",
        f"- Runtime: `{runtime_s:.2f} s`",
        "",
        "## Key Metrics",
        "",
        f"- Samples per tolerance level: `{samples_per_tol}`",
        f"- Tolerance levels swept: `{', '.join(f'{v:.1f}' for v in sigma_values)} mm` combined machined-plus-welded coordinate sigma",
        f"- Largest passing tolerance by geometry thresholds: `{chosen_sigma:.1f} mm` sigma (`+/-{2.0 * chosen_sigma:.1f} mm` approximate two-sigma band)",
        f"- At `{chosen_sigma:.1f} mm` sigma, p95 aero reference z delta: `{float(chosen['p95_aero_ref_z_delta_mm']):.2f} mm`",
        f"- At `{chosen_sigma:.1f} mm` sigma, front aero-reference z mean/std: `{float(chosen['mean_front_aero_ref_z_delta_mm']):+.3f}` / `{float(chosen['std_front_aero_ref_z_delta_mm']):.3f} mm`",
        f"- At `{chosen_sigma:.1f} mm` sigma, rear aero-reference z mean/std: `{float(chosen['mean_rear_aero_ref_z_delta_mm']):+.3f}` / `{float(chosen['std_rear_aero_ref_z_delta_mm']):.3f} mm`",
        f"- At `{chosen_sigma:.1f} mm` sigma, p95 roll-center delta: `{float(chosen['p95_roll_center_delta_mm']):.2f} mm`",
        f"- At `{chosen_sigma:.1f} mm` sigma, front roll-center mean/std: `{float(chosen['mean_front_roll_center_delta_mm']):+.3f}` / `{float(chosen['std_front_roll_center_delta_mm']):.3f} mm`",
        f"- At `{chosen_sigma:.1f} mm` sigma, rear roll-center mean/std: `{float(chosen['mean_rear_roll_center_delta_mm']):+.3f}` / `{float(chosen['std_rear_roll_center_delta_mm']):.3f} mm`",
        f"- At `{chosen_sigma:.1f} mm` sigma, p95 front/rear camber-gain delta: `{float(chosen['p95_front_camber_gain_delta_pct']):.2f} %` / `{float(chosen['p95_rear_camber_gain_delta_pct']):.2f} %`",
        f"- At `{chosen_sigma:.1f} mm` sigma, p95 inboard span delta: `{float(chosen['p95_inboard_span_delta_mm']):.2f} mm`",
        "",
        "![Tolerance summary](plots/tolerance_summary.png)",
        "",
        "![Front aero-reference z delta histogram](plots/front_aero_ref_z_delta_hist.png)",
        "",
        "![Front roll-center diagnostic histogram](plots/front_roll_center_delta_hist.png)",
        "",
        "![Front camber-gain delta histogram](plots/front_camber_gain_delta_hist.png)",
        "",
        "## Design Implication",
        "",
        "The chassis tolerance target should be expressed statistically: hold combined machined-plus-welded frame-side suspension hardpoint coordinate variation near the largest passing geometry sigma, measure the built frame, update `vehicle.yml` with the measured hardpoints, then rerun EnvelopeSim or compile and rerun StandardSim variants before claiming a dynamic-response effect.",
    ]
    (STUDY_DIR / "RESULTS.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
