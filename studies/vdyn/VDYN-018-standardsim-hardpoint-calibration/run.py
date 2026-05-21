from __future__ import annotations

import argparse
import csv
import math
import os
import shutil
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
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
TRANSIENT_METRICS = [
    "ay_rise_time_s",
    "yaw_rise_time_s",
    "settling_time_s",
    "ay_overshoot_pct",
    "yaw_overshoot_pct",
    "ay_gain_dc",
    "yaw_gain_dc",
]


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise TypeError(f"Expected mapping in {path}")
    return data


def write_yaml(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    fieldnames: list[str] = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def read_single_row(path: Path) -> dict[str, float]:
    with path.open("r", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))
    if len(rows) != 1:
        raise ValueError(f"Expected one row in {path}")
    return {k: float(v) for k, v in rows[0].items()}


def read_metrics_csv(path: Path) -> dict[str, float]:
    with path.open("r", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))
    out: dict[str, float] = {}
    for row in rows:
        try:
            out[row["metric"]] = float(row["value"])
        except (KeyError, ValueError):
            continue
    return out


def line_intersection_yz(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, p4: np.ndarray) -> np.ndarray:
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
    lower_avg = 0.5 * (np.array(s["lower_fore_i_m"], dtype=float) + np.array(s["lower_aft_i_m"], dtype=float))

    return {
        f"{axle}_roll_center_z_m": roll_center_z,
        f"{axle}_camber_gain_rad_per_m": camber_gain_rad_per_m,
        f"{axle}_aero_ref_z_m": float(lower_avg[2]),
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
        out[f"{axle}_roll_center_delta_mm"] = 1000.0 * (
            sample[f"{axle}_roll_center_z_m"] - base[f"{axle}_roll_center_z_m"]
        )
        out[f"{axle}_camber_gain_delta_pct"] = (
            100.0
            * (sample[f"{axle}_camber_gain_rad_per_m"] - base[f"{axle}_camber_gain_rad_per_m"])
            / max(abs(base[f"{axle}_camber_gain_rad_per_m"]), 1e-9)
        )
        out[f"{axle}_aero_ref_z_delta_mm"] = 1000.0 * (
            sample[f"{axle}_aero_ref_z_m"] - base[f"{axle}_aero_ref_z_m"]
        )
    out["max_roll_center_delta_mm"] = max(
        abs(out["front_roll_center_delta_mm"]),
        abs(out["rear_roll_center_delta_mm"]),
    )
    out["max_camber_gain_delta_pct"] = max(
        abs(out["front_camber_gain_delta_pct"]),
        abs(out["rear_camber_gain_delta_pct"]),
    )
    out["max_aero_ref_z_delta_mm"] = max(
        abs(out["front_aero_ref_z_delta_mm"]),
        abs(out["rear_aero_ref_z_delta_mm"]),
    )
    return out


def simple_prediction(deltas: dict[str, float], baseline: dict[str, float]) -> dict[str, float]:
    front_cg = deltas["front_camber_gain_delta_pct"] / 100.0
    rear_cg = deltas["rear_camber_gain_delta_pct"] / 100.0
    front_rc = deltas["front_roll_center_delta_mm"]
    rear_rc = deltas["rear_roll_center_delta_mm"]
    camber_mag = abs(front_cg) + abs(rear_cg)
    rc_split = front_rc - rear_rc
    return {
        "pred_ay_rise_time_delta_pct": 100.0 * (0.075 * camber_mag + 0.0008 * abs(rc_split)),
        "pred_yaw_rise_time_delta_pct": 100.0 * (0.065 * camber_mag + 0.0007 * abs(rc_split)),
        "pred_settling_time_delta_pct": 100.0 * (0.040 * camber_mag + 0.0005 * abs(rc_split)),
        "pred_ay_overshoot_delta_pct_pt": 0.16 * abs(deltas["front_camber_gain_delta_pct"] - deltas["rear_camber_gain_delta_pct"]) + 0.08 * abs(rc_split),
        "pred_yaw_overshoot_delta_pct_pt": 0.13 * abs(deltas["front_camber_gain_delta_pct"] - deltas["rear_camber_gain_delta_pct"]) + 0.06 * abs(rc_split),
        "pred_ay_gain_delta_pct": -0.020 * deltas["max_camber_gain_delta_pct"] - 0.010 * abs(rc_split),
        "pred_yaw_gain_delta_pct": -0.018 * deltas["max_camber_gain_delta_pct"] - 0.012 * abs(rc_split),
    }


def generate_design(base_vehicle: dict[str, Any], baseline: dict[str, float], cases: int, sigma_mm: float, seed: int) -> list[dict[str, Any]]:
    base_geom = vehicle_geometry(base_vehicle)
    rows: list[dict[str, Any]] = []
    rows.append({"case_id": "case_000_baseline", "is_baseline": True, "tolerance_sigma_mm": 0.0, **base_geom, "_vehicle": base_vehicle})
    rng = np.random.default_rng(seed)
    for idx in range(1, cases):
        vehicle = perturb_vehicle(base_vehicle, rng, sigma_mm / 1000.0)
        geom = vehicle_geometry(vehicle)
        deltas = geometry_deltas(base_geom, geom)
        pred = simple_prediction(deltas, baseline)
        rows.append(
            {
                "case_id": f"case_{idx:03d}",
                "is_baseline": False,
                "tolerance_sigma_mm": sigma_mm,
                **geom,
                **deltas,
                **pred,
                "_vehicle": vehicle,
            }
        )
    rows[0].update(geometry_deltas(base_geom, base_geom))
    rows[0].update(simple_prediction(rows[0], baseline))
    return rows


def copy_bobsim(src: Path, dst: Path) -> None:
    ignore = shutil.ignore_patterns(
        ".git",
        "__pycache__",
        ".pytest_cache",
        ".mypy_cache",
        ".ruff_cache",
        "Build",
        "results",
        "*.pyc",
    )
    shutil.copytree(src, dst, ignore=ignore)


def configure_transient_eval(path: Path) -> None:
    cfg_path = path / "_3_StandardSim" / "TransientEval" / "transient_eval_config.yml"
    cfg = load_yaml(cfg_path)
    cfg.setdefault("execution", {})
    cfg["execution"].update({"parallel": False, "max_workers": 1, "cleanup": True, "stream_logs": False})
    cfg.setdefault("test", {})
    cfg["test"].update(
        {
            "testVel": [15.0],
            "run_step": True,
            "run_continuous_sine": False,
            "directions": ["left"],
            "steerStep_deg": [5.0],
        }
    )
    cfg.setdefault("report", {})
    cfg["report"]["enabled"] = False
    write_yaml(cfg_path, cfg)


def run_case_worker(payload: dict[str, Any]) -> dict[str, Any]:
    start = time.perf_counter()
    case_id = payload["case_id"]
    work_root = Path(payload["work_root"])
    case_root = work_root / case_id
    bobsim_copy = case_root / "BobSim"
    result: dict[str, Any] = {
        "case_id": case_id,
        "compile_status": "not_started",
        "eval_status": "not_started",
        "compiled_model": 0,
        "python": payload["python"],
    }
    try:
        case_root.mkdir(parents=True, exist_ok=True)
        exe_path = bobsim_copy / "_3_StandardSim" / "Build" / "VehicleSim" / "BobLib.Standards.VehicleSim"
        if payload["reuse_work"] and exe_path.exists():
            result["compile_status"] = "reused"
            result["compiled_model"] = 1
            result["compile_runtime_s"] = 0.0
            result["compile_log_tail"] = "Reused existing compiled VehicleSim executable."
        else:
            if bobsim_copy.exists():
                shutil.rmtree(bobsim_copy)
            copy_bobsim(Path(payload["bobsim_src"]), bobsim_copy)
            write_yaml(bobsim_copy / "vehicle.yml", payload["vehicle"])

            env = os.environ.copy()
            env["MPLBACKEND"] = "Agg"
            env["MPLCONFIGDIR"] = str((case_root / ".matplotlib-cache").resolve())
            env["PYTHONPATH"] = str(bobsim_copy.resolve()) + os.pathsep + env.get("PYTHONPATH", "")
            make_cmd = [
                "make",
                "build-records",
                "build-vehicle-sim",
                f"PYTHON={payload['python']}",
            ]
            compile_start = time.perf_counter()
            compile_run = subprocess.run(
                make_cmd,
                cwd=bobsim_copy,
                env=env,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=float(payload["compile_timeout_s"]),
            )
            result["compile_runtime_s"] = time.perf_counter() - compile_start
            result["compile_log_tail"] = compile_run.stdout[-3000:]
            if compile_run.returncode != 0:
                result["compile_status"] = "failed"
                result["runtime_s"] = time.perf_counter() - start
                return result
            result["compile_status"] = "passed"
            result["compiled_model"] = 1

        env = os.environ.copy()
        env["MPLBACKEND"] = "Agg"
        env["MPLCONFIGDIR"] = str((case_root / ".matplotlib-cache").resolve())
        env["PYTHONPATH"] = str(bobsim_copy.resolve()) + os.pathsep + env.get("PYTHONPATH", "")
        configure_transient_eval(bobsim_copy)
        eval_start = time.perf_counter()
        eval_run = subprocess.run(
            [payload["python"], "-m", "_3_StandardSim.TransientEval.transient_eval_sim"],
            cwd=bobsim_copy,
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=float(payload["eval_timeout_s"]),
        )
        result["eval_runtime_s"] = time.perf_counter() - eval_start
        result["eval_log_tail"] = eval_run.stdout[-3000:]
        if eval_run.returncode != 0:
            result["eval_status"] = "failed"
            result["runtime_s"] = time.perf_counter() - start
            return result
        metrics_path = bobsim_copy / "_3_StandardSim" / "results" / "transient_eval_report_metrics.csv"
        metrics = read_metrics_csv(metrics_path)
        result.update({metric: metrics.get(metric, math.nan) for metric in TRANSIENT_METRICS})
        result["eval_status"] = "passed"
        if not payload["keep_work"]:
            shutil.rmtree(case_root, ignore_errors=True)
    except subprocess.TimeoutExpired as exc:
        result["compile_status" if result["compiled_model"] == 0 else "eval_status"] = "timeout"
        result["error"] = str(exc)
    except Exception as exc:  # pragma: no cover - worker safety
        result["error"] = repr(exc)
    result["runtime_s"] = time.perf_counter() - start
    return result


def actual_deltas(rows: list[dict[str, Any]], baseline_metrics: dict[str, float]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for row in rows:
        merged = dict(row)
        for metric in TRANSIENT_METRICS:
            if metric not in row or not math.isfinite(float(row.get(metric, math.nan))):
                continue
            base = baseline_metrics.get(metric)
            if base is None:
                continue
            if abs(base) > 1e-12:
                merged[f"{metric}_delta_pct"] = 100.0 * (float(row[metric]) - base) / abs(base)
            merged[f"{metric}_delta"] = float(row[metric]) - base
        out.append(merged)
    return out


def calibration_summary(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    pairs = [
        ("ay_rise_time_s_delta_pct", "pred_ay_rise_time_delta_pct"),
        ("yaw_rise_time_s_delta_pct", "pred_yaw_rise_time_delta_pct"),
        ("settling_time_s_delta_pct", "pred_settling_time_delta_pct"),
        ("ay_overshoot_pct_delta", "pred_ay_overshoot_delta_pct_pt"),
        ("yaw_overshoot_pct_delta", "pred_yaw_overshoot_delta_pct_pt"),
        ("ay_gain_dc_delta_pct", "pred_ay_gain_delta_pct"),
        ("yaw_gain_dc_delta_pct", "pred_yaw_gain_delta_pct"),
    ]
    summary: list[dict[str, Any]] = []
    for actual_key, pred_key in pairs:
        valid = [
            row
            for row in rows
            if actual_key in row
            and pred_key in row
            and math.isfinite(float(row[actual_key]))
            and math.isfinite(float(row[pred_key]))
            and not bool(row.get("is_baseline", False))
        ]
        if not valid:
            continue
        actual = np.array([float(row[actual_key]) for row in valid])
        pred = np.array([float(row[pred_key]) for row in valid])
        err = pred - actual
        corr = float(np.corrcoef(actual, pred)[0, 1]) if len(valid) > 1 and np.std(actual) > 0 and np.std(pred) > 0 else math.nan
        summary.append(
            {
                "actual_metric": actual_key,
                "prediction_metric": pred_key,
                "cases": len(valid),
                "mean_abs_error": float(np.mean(np.abs(err))),
                "p95_abs_error": float(np.percentile(np.abs(err), 95)),
                "correlation": corr,
            }
        )
    return summary


def plot_calibration(rows: list[dict[str, Any]], actual_key: str, pred_key: str, path: Path) -> None:
    valid = [
        row
        for row in rows
        if actual_key in row
        and pred_key in row
        and math.isfinite(float(row[actual_key]))
        and math.isfinite(float(row[pred_key]))
        and not bool(row.get("is_baseline", False))
    ]
    if not valid:
        return
    actual = np.array([float(row[actual_key]) for row in valid])
    pred = np.array([float(row[pred_key]) for row in valid])
    lo = float(min(actual.min(), pred.min()))
    hi = float(max(actual.max(), pred.max()))
    fig, ax = plt.subplots(figsize=(5.8, 5.0))
    ax.scatter(actual, pred, color="#4c78a8", alpha=0.85)
    ax.plot([lo, hi], [lo, hi], color="#444444", linestyle="--", linewidth=1.1)
    ax.set_xlabel(f"Compiled StandardSim {actual_key}")
    ax.set_ylabel(f"Simplified prediction {pred_key}")
    ax.set_title("Hardpoint Calibration Check")
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=220)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-standardsim", action="store_true", help="Compile and run the selected StandardSim truth set.")
    parser.add_argument("--cases", type=int, default=None)
    parser.add_argument("--sigma-mm", type=float, default=None)
    parser.add_argument("--max-workers", type=int, default=None)
    parser.add_argument("--keep-work", action="store_true")
    parser.add_argument("--reuse-work", action="store_true")
    parser.add_argument("--compile-timeout-s", type=float, default=1800.0)
    parser.add_argument("--eval-timeout-s", type=float, default=900.0)
    args = parser.parse_args()

    start = time.perf_counter()
    cfg = load_yaml(STUDY_DIR / "study.yml")
    vehicle = load_yaml(REPO_ROOT / "vehicles/current/vehicle.yml")
    baseline = read_single_row(STUDY_DIR.parent / "VDYN-003-standardsim-baseline" / "outputs" / "summary.csv")
    baseline.setdefault("ay_gain_dc", baseline.get("ay_dc_gain_mps2_per_rad", 33.73))
    baseline.setdefault("yaw_gain_dc", baseline.get("yaw_dc_gain_radps_per_rad", 2.228))

    cases = args.cases or int(cfg["swept_variables"]["calibration_cases"])
    sigma_mm = args.sigma_mm or float(cfg["swept_variables"]["tolerance_sigma_mm"])
    max_workers = args.max_workers or int(cfg["swept_variables"]["max_parallel_builds"])
    seed = int(cfg["swept_variables"]["random_seed"])

    rows = generate_design(vehicle, baseline, cases, sigma_mm, seed)
    outputs = STUDY_DIR / "outputs"
    plots = STUDY_DIR / "plots"
    design_rows = [{k: v for k, v in row.items() if k != "_vehicle"} for row in rows]
    write_csv(outputs / "standardsim_calibration_design.csv", design_rows)

    run_rows: list[dict[str, Any]] = []
    if args.run_standardsim:
        work_root = outputs / "standardsim_truth_work"
        work_root.mkdir(parents=True, exist_ok=True)
        python = sys.executable
        payloads = [
            {
                "case_id": row["case_id"],
                "vehicle": row["_vehicle"],
                "bobsim_src": str((REPO_ROOT / "BobSim").resolve()),
                "work_root": str(work_root.resolve()),
                "python": python,
                "keep_work": args.keep_work,
                "reuse_work": args.reuse_work,
                "compile_timeout_s": args.compile_timeout_s,
                "eval_timeout_s": args.eval_timeout_s,
            }
            for row in rows
        ]
        with ProcessPoolExecutor(max_workers=max_workers) as pool:
            futures = [pool.submit(run_case_worker, payload) for payload in payloads]
            for future in as_completed(futures):
                run_rows.append(future.result())
        run_rows.sort(key=lambda row: row["case_id"])
        write_csv(outputs / "standardsim_truth_metrics.csv", run_rows)
    else:
        run_rows = [
            {
                "case_id": row["case_id"],
                "compile_status": "not_run",
                "eval_status": "not_run",
                "compiled_model": 0,
            }
            for row in rows
        ]
        write_csv(outputs / "standardsim_truth_metrics.csv", run_rows)

    by_case = {row["case_id"]: row for row in run_rows}
    merged_rows = []
    for row in design_rows:
        merged_rows.append({**row, **by_case.get(row["case_id"], {})})
    merged_rows = actual_deltas(merged_rows, baseline)
    write_csv(outputs / "standardsim_calibration_comparison.csv", merged_rows)
    summary = calibration_summary(merged_rows)
    if summary:
        write_csv(outputs / "calibration_error_summary.csv", summary)
        plot_calibration(
            merged_rows,
            "ay_rise_time_s_delta_pct",
            "pred_ay_rise_time_delta_pct",
            plots / "ay_rise_calibration.png",
        )
        plot_calibration(
            merged_rows,
            "ay_overshoot_pct_delta",
            "pred_ay_overshoot_delta_pct_pt",
            plots / "ay_overshoot_calibration.png",
        )

    compiled_models = sum(int(row.get("compiled_model", 0)) for row in run_rows)
    eval_passed = sum(1 for row in run_rows if row.get("eval_status") == "passed")
    compile_runtime_total = sum(float(row.get("compile_runtime_s", 0.0) or 0.0) for row in run_rows)
    eval_runtime_total = sum(float(row.get("eval_runtime_s", 0.0) or 0.0) for row in run_rows)
    runtime_s = time.perf_counter() - start
    provenance = {
        "engine": "StandardSim" if args.run_standardsim else "standardsim_calibration_design_only",
        "compiled_models": compiled_models,
        "simulated_cases": eval_passed if args.run_standardsim else 0,
        "requested_cases": cases,
        "max_parallel_builds": max_workers,
        "runtime_s": runtime_s,
        "notes": "Compiled VehicleSim variants and ran TransientEval." if args.run_standardsim else "Design generated only; rerun with --run-standardsim to compile variants.",
    }
    write_csv(outputs / "run_provenance.csv", [provenance])

    if args.run_standardsim and eval_passed == cases and cases >= 2:
        status = "PASS"
    elif args.run_standardsim and eval_passed > 0:
        status = "SMOKE"
    elif args.run_standardsim:
        status = "PARTIAL"
    else:
        status = "PLANNED"
    lines = [
        "# VDYN-018 Results",
        "",
        "## Finding",
        "",
        f"**{status}:** hardpoint calibration subset is defined"
        + (" and has been run through compiled StandardSim variants." if args.run_standardsim else "; compiled StandardSim execution has not been requested yet."),
        "",
        "## Run Provenance",
        "",
        f"- Engine: `{provenance['engine']}`",
        f"- Requested vehicle configurations: `{cases}`",
        f"- Compiled StandardSim models: `{compiled_models}`",
        f"- Successful TransientEval cases: `{eval_passed}`",
        f"- Max parallel builds: `{max_workers}`",
        f"- Aggregate compile runtime: `{compile_runtime_total:.2f} s`",
        f"- Aggregate eval runtime: `{eval_runtime_total:.2f} s`",
        f"- Runtime: `{runtime_s:.2f} s`",
        "",
        "## Key Metrics",
        "",
        f"- Calibration sigma: `{sigma_mm:.2f} mm`",
        f"- Design cases written: `{len(design_rows)}`",
    ]
    if summary:
        best = min(summary, key=lambda row: float(row["mean_abs_error"]))
        worst = max(summary, key=lambda row: float(row["mean_abs_error"]))
        lines.extend(
            [
                f"- Best simplified metric: `{best['prediction_metric']}` mean abs error `{float(best['mean_abs_error']):.3f}`",
                f"- Worst simplified metric: `{worst['prediction_metric']}` mean abs error `{float(worst['mean_abs_error']):.3f}`",
                "",
                "![Ay rise calibration](plots/ay_rise_calibration.png)",
                "",
                "![Ay overshoot calibration](plots/ay_overshoot_calibration.png)",
            ]
        )
    lines.extend(
        [
            "",
            "## Design Implication",
            "",
            "Use this compiled subset as the gate before claiming a 25000-case hardpoint response Monte Carlo. If the simplified mapping is weak, the large Monte Carlo may still support geometry/aero-reference tolerance, but not StandardSim response risk.",
        ]
    )
    (STUDY_DIR / "RESULTS.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
