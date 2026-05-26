#!/usr/bin/env python3
"""Run DS-008: Round 8 LC0 raw tire-data analysis."""

from __future__ import annotations

import argparse
import csv
import io
import importlib.util
import math
import os
import re
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from zipfile import ZipFile


REPO_ROOT = Path(__file__).resolve().parents[2]
STUDY_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = STUDY_DIR / "outputs"
PLOT_DIR = STUDY_DIR / "plots"
REPORT_PATH = REPO_ROOT / "reports" / "DS-008-round8-lc0-analysis.md"
PLOT_PREFIX_TOKEN = "__DS008_PLOT_PREFIX__"

RUN_GUIDE = REPO_ROOT / "RunGuide_Round8.pdf"
CORNERING_ARCHIVE = REPO_ROOT / "RunData_Cornering_Matlab_SI_10inch_Round8.zip"
ROUND8_TIRE_DIR = REPO_ROOT / "vehicles" / "current" / "tires" / "round_8_fabricated_longitudinal_um3"
ROUND8_TIRE_MANIFEST = ROUND8_TIRE_DIR / "manifest.csv"
DS006_RUN = REPO_ROOT / "studies" / "DS-006-integrated-tire-design" / "run.py"
DS006_INTEGRATED_RESULTS = REPO_ROOT / "studies" / "DS-006-integrated-tire-design" / "outputs" / "integrated_results.csv"
CURRENT_TIRE = REPO_ROOT / "vehicles" / "current" / "tires" / "16x7p5_10_12psi.tir"

KPA_PER_PSI = 6.894757293168361
PRESSURE_TOL_KPA = 2.35
DEGRADATION_PRESSURE_PSI = 12.0
NOMINAL_TEST_SPEED_KPH_MIN = 34.0
NOMINAL_TEST_SPEED_KPH_MAX = 47.0
STANDARD_STABLE_MAX_AY_MPS2 = 8.0
STANDARD_CASE_TIMEOUT_S = 75.0
R20_COMPARISON_PRESSURE_PSI = 12.0
R20_HOOSIER_MODELS = {"43070", "43075", "43100", "43164"}
PRESSURE_SENSITIVITY_WINDOWS = {
    "final_run_8psi",
    "initial_run_10psi",
    "final_run_12psi",
    "initial_run_14psi",
}

MPLCONFIGDIR = Path("/tmp/lhre-sim-matplotlib")
MPLCONFIGDIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(MPLCONFIGDIR))


def require_dependencies() -> tuple[Any, Any, Any]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
        import scipy.io
    except ModuleNotFoundError as exc:
        missing = exc.name or "a required package"
        raise SystemExit(
            f"Missing Python dependency: {missing}\n"
            "Run with the study environment, for example:\n"
            "  /tmp/lhre-sim-venv/bin/python studies/DS-008-round8-lc0-analysis/run.py"
        ) from exc
    return np, plt, scipy.io


np, plt, scipy_io = require_dependencies()


def load_module(name: str, path: Path) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def load_ds006_module() -> Any:
    module = load_module("ds006_helpers_for_ds008", DS006_RUN)
    module.WORK_DIR = STUDY_DIR / "work"
    return module


@dataclass(frozen=True)
class TireSpec:
    compound: str
    model: str
    size: str
    rim_width_in: float
    transient_run: int
    initial_run: int
    final_run: int

    @property
    def setup_id(self) -> str:
        size_slug = self.size.replace(".", "p").replace("-", "_").replace("x", "x")
        return f"{self.compound}_{self.model}_{size_slug}_{self.rim_width_in:g}in"

    @property
    def label(self) -> str:
        return f"Hoosier {self.model} {self.size} {self.compound} {self.rim_width_in:g}in"


ROUND8_SPECS = (
    TireSpec("LC0", "43075", "16x7.5-10", 8.0, 14, 15, 16),
    TireSpec("LC0", "43075", "16x7.5-10", 7.0, 17, 18, 19),
    TireSpec("LC0", "43070", "16x6.0-10", 6.0, 20, 21, 22),
    TireSpec("LC0", "43070", "16x6.0-10", 7.0, 23, 24, 25),
)


def finite_float(value: Any) -> float:
    try:
        output = float(value)
    except (TypeError, ValueError):
        return math.nan
    return output if math.isfinite(output) else math.nan


def format_float(value: Any, digits: int = 3) -> str:
    value_f = finite_float(value)
    if not math.isfinite(value_f):
        return "n/a"
    return f"{value_f:.{digits}f}"


def format_pct(value: Any, digits: int = 1, *, signed: bool = True) -> str:
    value_f = finite_float(value)
    if not math.isfinite(value_f):
        return "n/a"
    sign = "+" if signed else ""
    return f"{value_f:{sign}.{digits}f}%"


def format_pct_span(values: list[Any], digits: int = 1, *, signed: bool = True) -> str:
    finite_values = sorted(finite_float(value) for value in values if math.isfinite(finite_float(value)))
    if not finite_values:
        return "n/a"
    if abs(finite_values[0] - finite_values[-1]) <= 1e-9:
        return format_pct(finite_values[0], digits, signed=signed)
    return f"{format_pct(finite_values[0], digits, signed=signed)} to {format_pct(finite_values[-1], digits, signed=signed)}"


def format_signed_float(value: Any, digits: int = 4) -> str:
    value_f = finite_float(value)
    if not math.isfinite(value_f):
        return "n/a"
    return f"{value_f:+.{digits}f}"


def format_signed_float_span(values: list[Any], digits: int = 4) -> str:
    finite_values = sorted(finite_float(value) for value in values if math.isfinite(finite_float(value)))
    if not finite_values:
        return "n/a"
    if abs(finite_values[0] - finite_values[-1]) <= 1e-12:
        return format_signed_float(finite_values[0], digits)
    return f"{format_signed_float(finite_values[0], digits)} to {format_signed_float(finite_values[-1], digits)}"


def safe_percent_delta(final: Any, initial: Any) -> float:
    final_f = finite_float(final)
    initial_f = finite_float(initial)
    if not math.isfinite(final_f) or not math.isfinite(initial_f) or abs(initial_f) <= 1e-12:
        return math.nan
    return 100.0 * (final_f - initial_f) / abs(initial_f)


def table_line(values: list[Any]) -> str:
    return "| " + " | ".join(str(value) for value in values) + " |"


def markdown_image(caption: str, relative_plot_path: str, *, prefix: str = PLOT_PREFIX_TOKEN) -> str:
    return f"![{caption}]({prefix}/{relative_plot_path})"


def write_csv(path: Path, rows: list[dict[str, Any]], fieldnames: list[str] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if fieldnames is None:
        seen: set[str] = set()
        fieldnames = []
        for row in rows:
            for key in row:
                if key not in seen:
                    seen.add(key)
                    fieldnames.append(key)
        if not fieldnames:
            fieldnames = ["empty"]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def slugify(value: str) -> str:
    value = re.sub(r"[^A-Za-z0-9]+", "_", value).strip("_")
    return value or "value"


def model_for_size(size: str) -> str:
    if "7p5" in size or "7.5" in size:
        return "43075"
    if "16x6" in size:
        return "43070"
    return "unknown"


def normalized_tire_size(size: str) -> str:
    return str(size).replace("x7p5", "x7.5").replace("x6_10", "x6.0-10").replace("_10", "-10")


def rim_width_from_label(value: str) -> float:
    return finite_float(str(value).replace("in", ""))


def load_mat_run(run: int, cache: dict[int, dict[str, Any]]) -> dict[str, Any]:
    if run in cache:
        return cache[run]
    member = f"B1965run{run}.mat"
    with ZipFile(CORNERING_ARCHIVE) as archive:
        if member not in archive.namelist():
            cache[run] = {}
            return cache[run]
        mat = scipy_io.loadmat(
            io.BytesIO(archive.read(member)),
            squeeze_me=True,
            struct_as_record=False,
        )
    output: dict[str, Any] = {}
    for key, value in mat.items():
        if key.startswith("__") or key == "channel":
            continue
        if isinstance(value, str):
            output[key] = value
            continue
        arr = np.asarray(value)
        if arr.ndim == 1 and arr.size > 10 and arr.dtype.kind in "biufc":
            output[key] = arr.astype(float, copy=False).ravel()
    cache[run] = output
    return output


def pressure_window_mask(data: dict[str, Any], pressure_psi: float) -> Any:
    pressure = np.asarray(data["P"], dtype=float)
    return np.abs(pressure - pressure_psi * KPA_PER_PSI) <= PRESSURE_TOL_KPA


def finite_array_mask(*arrays: Any) -> Any:
    mask = np.ones_like(np.asarray(arrays[0], dtype=float), dtype=bool)
    for array in arrays:
        mask &= np.isfinite(np.asarray(array, dtype=float))
    return mask


def nominal_speed_mask(data: dict[str, Any]) -> Any:
    speed = np.asarray(data["V"], dtype=float)
    return (speed >= NOMINAL_TEST_SPEED_KPH_MIN) & (speed <= NOMINAL_TEST_SPEED_KPH_MAX)


def robust_abs_peak_mu(force: Any, normal_load: Any, mask: Any) -> float:
    idx = np.flatnonzero(mask)
    if idx.size < 120:
        return math.nan
    mu = np.abs(np.asarray(force, dtype=float)[idx] / np.asarray(normal_load, dtype=float)[idx])
    mu = mu[np.isfinite(mu)]
    return float(np.nanpercentile(mu, 95.0)) if mu.size else math.nan


def linear_abs_slope(x: Any, y: Any, mask: Any) -> float:
    idx = np.flatnonzero(mask)
    if idx.size < 60:
        return math.nan
    x_i = np.asarray(x, dtype=float)[idx]
    y_i = np.asarray(y, dtype=float)[idx]
    valid = np.isfinite(x_i) & np.isfinite(y_i)
    if np.count_nonzero(valid) < 60:
        return math.nan
    try:
        slope = np.polyfit(x_i[valid], y_i[valid], 1)[0]
    except (np.linalg.LinAlgError, ValueError):
        return math.nan
    return abs(float(slope))


def tread_arrays(data: dict[str, Any]) -> tuple[Any, Any]:
    tsti = np.asarray(data["TSTI"], dtype=float)
    tstc = np.asarray(data["TSTC"], dtype=float)
    tsto = np.asarray(data["TSTO"], dtype=float)
    stack = np.column_stack([tsti, tstc, tsto])
    tread = np.nanmean(stack, axis=1)
    spread = np.nanmax(stack, axis=1) - np.nanmin(stack, axis=1)
    return tread, spread


def lateral_metrics(data: dict[str, Any], pressure_psi: float) -> dict[str, Any]:
    required = ("P", "FY", "FZ", "SA", "V", "TSTI", "TSTC", "TSTO", "RST", "AMBTMP")
    if not all(key in data for key in required):
        return {"status": "missing_channels", "samples": 0}

    fz = -np.asarray(data["FZ"], dtype=float)
    fy = np.asarray(data["FY"], dtype=float)
    sa = np.deg2rad(np.asarray(data["SA"], dtype=float))
    mask = (
        pressure_window_mask(data, pressure_psi)
        & nominal_speed_mask(data)
        & finite_array_mask(fz, fy, sa)
        & (fz > 120.0)
        & (fz < 2200.0)
        & (np.abs(sa) <= np.deg2rad(15.0))
    )
    samples = int(np.count_nonzero(mask))
    if samples < 600:
        return {"status": "insufficient_data", "samples": samples}

    fy_norm = fy / np.maximum(fz, 1e-9)
    small_slip = mask & (np.abs(sa) <= np.deg2rad(3.0)) & (np.abs(fy_norm) <= 2.5)
    tread, spread = tread_arrays(data)
    return {
        "status": "ok",
        "samples": samples,
        "peak_mu_y_p95": robust_abs_peak_mu(fy, fz, mask),
        "ky_norm_per_rad": linear_abs_slope(sa, fy_norm, small_slip),
        "tread_mean_c": float(np.nanmean(tread[mask])),
        "tread_peak_c": float(np.nanmax(tread[mask])),
        "surface_spread_mean_c": float(np.nanmean(spread[mask])),
        "inner_minus_outer_c": float(np.nanmean(np.asarray(data["TSTI"], dtype=float)[mask] - np.asarray(data["TSTO"], dtype=float)[mask])),
        "rim_temp_mean_c": float(np.nanmean(np.asarray(data["RST"], dtype=float)[mask])),
        "ambient_temp_mean_c": float(np.nanmean(np.asarray(data["AMBTMP"], dtype=float)[mask])),
    }


def edge_mean(values: Any, order: Any, *, first: bool) -> float:
    if order.size == 0:
        return math.nan
    count = max(8, int(math.ceil(0.05 * order.size)))
    idx = order[:count] if first else order[-count:]
    return float(np.nanmean(np.asarray(values, dtype=float)[idx])) if idx.size else math.nan


def transient_temperature_rows(cache: dict[int, dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for tire in ROUND8_SPECS:
        data = load_mat_run(tire.transient_run, cache)
        if not all(key in data for key in ("P", "ET", "TSTI", "TSTC", "TSTO", "RST", "AMBTMP")):
            continue
        et = np.asarray(data["ET"], dtype=float)
        tsti = np.asarray(data["TSTI"], dtype=float)
        tstc = np.asarray(data["TSTC"], dtype=float)
        tsto = np.asarray(data["TSTO"], dtype=float)
        tread, spread = tread_arrays(data)
        finite_base = np.isfinite(et) & np.isfinite(tread) & np.isfinite(tsti) & np.isfinite(tstc) & np.isfinite(tsto)
        for pressure_psi in (10.0, 12.0, 14.0):
            mask = pressure_window_mask(data, pressure_psi) & finite_base
            if np.count_nonzero(mask) < 120:
                continue
            idx = np.flatnonzero(mask)
            order = idx[np.argsort(et[idx])]
            start = edge_mean(tread, order, first=True)
            end = edge_mean(tread, order, first=False)
            rows.append(
                {
                    "compound": tire.compound,
                    "model": tire.model,
                    "size": tire.size,
                    "rim_width_in": tire.rim_width_in,
                    "setup_id": tire.setup_id,
                    "label": tire.label,
                    "transient_run": tire.transient_run,
                    "pressure_psi": pressure_psi,
                    "samples": int(idx.size),
                    "elapsed_time_min_s": float(np.nanmin(et[idx])),
                    "elapsed_time_max_s": float(np.nanmax(et[idx])),
                    "tread_mean_c": float(np.nanmean(tread[idx])),
                    "tread_peak_c": float(np.nanmax(tread[idx])),
                    "tread_start_c": start,
                    "tread_end_c": end,
                    "tread_rise_c": end - start if math.isfinite(start) and math.isfinite(end) else math.nan,
                    "inner_mean_c": float(np.nanmean(tsti[idx])),
                    "center_mean_c": float(np.nanmean(tstc[idx])),
                    "outer_mean_c": float(np.nanmean(tsto[idx])),
                    "inner_minus_outer_c": float(np.nanmean(tsti[idx] - tsto[idx])),
                    "surface_spread_mean_c": float(np.nanmean(spread[idx])),
                    "rim_temp_mean_c": float(np.nanmean(np.asarray(data["RST"], dtype=float)[idx])),
                    "ambient_temp_mean_c": float(np.nanmean(np.asarray(data["AMBTMP"], dtype=float)[idx])),
                }
            )
    return rows


def degradation_rows(cache: dict[int, dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for tire in ROUND8_SPECS:
        initial = lateral_metrics(load_mat_run(tire.initial_run, cache), DEGRADATION_PRESSURE_PSI)
        final = lateral_metrics(load_mat_run(tire.final_run, cache), DEGRADATION_PRESSURE_PSI)
        row = {
            "compound": tire.compound,
            "model": tire.model,
            "size": tire.size,
            "rim_width_in": tire.rim_width_in,
            "setup_id": tire.setup_id,
            "label": tire.label,
            "initial_run": tire.initial_run,
            "final_run": tire.final_run,
            "pressure_psi": DEGRADATION_PRESSURE_PSI,
            "status": "ok" if initial.get("status") == "ok" and final.get("status") == "ok" else "partial",
            "initial_samples": initial.get("samples", 0),
            "final_samples": final.get("samples", 0),
            "initial_peak_mu_y_p95": initial.get("peak_mu_y_p95", math.nan),
            "final_peak_mu_y_p95": final.get("peak_mu_y_p95", math.nan),
            "peak_mu_y_delta_pct": safe_percent_delta(final.get("peak_mu_y_p95"), initial.get("peak_mu_y_p95")),
            "initial_ky_norm_per_rad": initial.get("ky_norm_per_rad", math.nan),
            "final_ky_norm_per_rad": final.get("ky_norm_per_rad", math.nan),
            "ky_delta_pct": safe_percent_delta(final.get("ky_norm_per_rad"), initial.get("ky_norm_per_rad")),
            "initial_tread_mean_c": initial.get("tread_mean_c", math.nan),
            "final_tread_mean_c": final.get("tread_mean_c", math.nan),
            "tread_delta_c": finite_float(final.get("tread_mean_c")) - finite_float(initial.get("tread_mean_c")),
            "initial_inner_minus_outer_c": initial.get("inner_minus_outer_c", math.nan),
            "final_inner_minus_outer_c": final.get("inner_minus_outer_c", math.nan),
        }
        rows.append(row)
    return rows


def pressure_rows(cache: dict[int, dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    pressure_windows = (
        (8.0, "final_run_8psi", "final_run"),
        (10.0, "initial_run_10psi", "initial_run"),
        (12.0, "initial_run_12psi", "initial_run"),
        (12.0, "final_run_12psi", "final_run"),
        (14.0, "initial_run_14psi", "initial_run"),
    )
    for tire in ROUND8_SPECS:
        for pressure_psi, window, run_attr in pressure_windows:
            run = getattr(tire, run_attr)
            metrics = lateral_metrics(load_mat_run(run, cache), pressure_psi)
            row = {
                "compound": tire.compound,
                "model": tire.model,
                "size": tire.size,
                "rim_width_in": tire.rim_width_in,
                "setup_id": tire.setup_id,
                "label": tire.label,
                "run": run,
                "pressure_psi": pressure_psi,
                "window": window,
                **metrics,
            }
            rows.append(row)
    return rows


def r20_pressure_source_rows() -> list[dict[str, str]]:
    rows = []
    for row in read_csv_rows(DS006_INTEGRATED_RESULTS):
        if row.get("source") != "round9_fitted_full_um14":
            continue
        if row.get("brand") != "Hoosier" or row.get("model") not in R20_HOOSIER_MODELS:
            continue
        if "_r20_" not in row.get("candidate_id", "").lower():
            continue
        rows.append(row)
    return rows


def r20_comparison_source_rows() -> list[dict[str, str]]:
    return [
        row
        for row in r20_pressure_source_rows()
        if abs(finite_float(row.get("pressure_psi")) - R20_COMPARISON_PRESSURE_PSI) <= 1e-9
    ]


def r20_comparison_rows(deg_rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    lookup = {
        (row["model"], row["tire_size"], float(row["rim_width_in"])): row
        for row in r20_comparison_source_rows()
    }
    rows: list[dict[str, Any]] = []
    for lc0 in [row for row in deg_rows if row["compound"] == "LC0"]:
        key = (str(lc0["model"]), str(lc0["size"]), float(lc0["rim_width_in"]))
        r20 = lookup.get(key)
        if not r20:
            continue
        rows.append(
            {
                "model": lc0["model"],
                "size": lc0["size"],
                "rim_width_in": lc0["rim_width_in"],
                "setup": f"{lc0['model']} {lc0['size']} {float(lc0['rim_width_in']):g}in",
                "r20_candidate_id": r20.get("candidate_id", ""),
                "r20_label": r20.get("label", ""),
                "r20_pressure_psi": finite_float(r20.get("pressure_psi")),
                "lc0_final_peak_mu_y_p95": lc0["final_peak_mu_y_p95"],
                "r20_final_peak_mu_y_p95": finite_float(r20.get("degradation_corner_final_peak_mu_y")),
                "lc0_vs_r20_peak_mu_y_delta_pct": safe_percent_delta(
                    lc0["final_peak_mu_y_p95"],
                    r20.get("degradation_corner_final_peak_mu_y"),
                ),
                "lc0_final_ky_norm_per_rad": lc0["final_ky_norm_per_rad"],
                "r20_final_ky_norm_per_rad": finite_float(r20.get("degradation_corner_final_ky_norm_per_rad")),
                "lc0_vs_r20_ky_delta_pct": safe_percent_delta(
                    lc0["final_ky_norm_per_rad"],
                    r20.get("degradation_corner_final_ky_norm_per_rad"),
                ),
                "lc0_peak_mu_y_degradation_pct": lc0["peak_mu_y_delta_pct"],
                "r20_peak_mu_y_degradation_pct": finite_float(r20.get("degradation_corner_peak_mu_y_delta_pct")),
                "r20_integrated_design_score": finite_float(r20.get("integrated_design_score")),
                "r20_envelope_score": finite_float(r20.get("envelope_score")),
                "r20_standardsim_score": finite_float(r20.get("standardsim_score")),
                "r20_standardsim_quality_status": r20.get("standardsim_quality_status", ""),
            }
        )
    return sorted(rows, key=lambda row: (str(row["model"]), str(row["size"]), float(row["rim_width_in"])))


def linear_pressure_slope(points: list[tuple[float, float]]) -> dict[str, Any]:
    clean = sorted(
        [(finite_float(pressure), finite_float(mu)) for pressure, mu in points],
        key=lambda item: item[0],
    )
    clean = [(pressure, mu) for pressure, mu in clean if math.isfinite(pressure) and math.isfinite(mu)]
    if len(clean) < 2:
        return {"point_count": len(clean), "dmu_dpressure_per_psi": math.nan}
    pressures = np.asarray([point[0] for point in clean], dtype=float)
    mus = np.asarray([point[1] for point in clean], dtype=float)
    if np.unique(pressures).size < 2:
        return {"point_count": len(clean), "dmu_dpressure_per_psi": math.nan}
    try:
        slope, intercept = np.polyfit(pressures, mus, 1)
    except (np.linalg.LinAlgError, ValueError):
        slope = intercept = math.nan
    p_min = float(np.nanmin(pressures))
    p_max = float(np.nanmax(pressures))
    mu_min_pressure = float(mus[np.nanargmin(pressures)])
    mu_max_pressure = float(mus[np.nanargmax(pressures)])
    endpoint_slope = (mu_max_pressure - mu_min_pressure) / (p_max - p_min) if abs(p_max - p_min) > 1e-12 else math.nan
    mu_at_12 = next((mu for pressure, mu in clean if abs(pressure - 12.0) <= 1e-9), math.nan)
    pct_per_psi_at_12 = 100.0 * slope / abs(mu_at_12) if math.isfinite(slope) and math.isfinite(mu_at_12) and abs(mu_at_12) > 1e-12 else math.nan
    if math.isfinite(slope) and math.isfinite(intercept):
        predicted = slope * pressures + intercept
        ss_res = float(np.nansum((mus - predicted) ** 2))
        ss_tot = float(np.nansum((mus - float(np.nanmean(mus))) ** 2))
        r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else math.nan
    else:
        r2 = math.nan
    return {
        "point_count": len(clean),
        "pressure_min_psi": p_min,
        "pressure_max_psi": p_max,
        "mu_at_min_pressure": mu_min_pressure,
        "mu_at_max_pressure": mu_max_pressure,
        "mu_at_12psi": mu_at_12,
        "dmu_dpressure_per_psi": float(slope) if math.isfinite(finite_float(slope)) else math.nan,
        "endpoint_dmu_dpressure_per_psi": endpoint_slope,
        "pct_dmu_dpressure_per_psi_at_12psi": pct_per_psi_at_12,
        "mu_delta_8_to_14": mu_max_pressure - mu_min_pressure,
        "mu_delta_8_to_14_pct": safe_percent_delta(mu_max_pressure, mu_min_pressure),
        "linear_fit_r2": r2,
        "pressure_mu_points": ";".join(f"{pressure:g}:{mu:.6g}" for pressure, mu in clean),
    }


def pressure_mu_sensitivity_rows(
    pressure_summary: list[dict[str, Any]],
    comparison_rows: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    matched_keys = {
        (str(row["model"]), str(row["size"]), float(row["rim_width_in"]))
        for row in comparison_rows
    }
    rows: list[dict[str, Any]] = []

    lc0_groups: dict[tuple[str, str, float], list[dict[str, Any]]] = {}
    for row in pressure_summary:
        key = (str(row.get("model")), str(row.get("size")), finite_float(row.get("rim_width_in")))
        if key not in matched_keys:
            continue
        if row.get("compound") != "LC0" or row.get("status") != "ok":
            continue
        if row.get("window") not in PRESSURE_SENSITIVITY_WINDOWS:
            continue
        lc0_groups.setdefault(key, []).append(row)

    for key, group_rows in lc0_groups.items():
        sample = group_rows[0]
        metrics = linear_pressure_slope(
            [(finite_float(row.get("pressure_psi")), finite_float(row.get("peak_mu_y_p95"))) for row in group_rows]
        )
        rows.append(
            {
                "source": "LC0_raw_peak_mu_y_p95",
                "compound": "LC0",
                "model": key[0],
                "size": key[1],
                "rim_width_in": key[2],
                "setup": f"{key[0]} {key[1]} {key[2]:g}in",
                "label": sample.get("label", ""),
                "mu_metric": "observed_peak_mu_y_p95",
                "pressure_basis": "8psi final, 10psi initial, 12psi final, 14psi initial",
                **metrics,
            }
        )

    r20_groups: dict[tuple[str, str, float], list[dict[str, str]]] = {}
    for row in r20_pressure_source_rows():
        key = (str(row.get("model")), str(row.get("tire_size")), finite_float(row.get("rim_width_in")))
        if key not in matched_keys:
            continue
        r20_groups.setdefault(key, []).append(row)

    for key, group_rows in r20_groups.items():
        sample = group_rows[0]
        metrics = linear_pressure_slope(
            [(finite_float(row.get("pressure_psi")), finite_float(row.get("mu_y"))) for row in group_rows]
        )
        rows.append(
            {
                "source": "R20_fitted_abs_PDY1",
                "compound": "R20",
                "model": key[0],
                "size": key[1],
                "rim_width_in": key[2],
                "setup": f"{key[0]} {key[1]} {key[2]:g}in",
                "label": sample.get("label", ""),
                "mu_metric": "fitted_abs_PDY1_mu_y",
                "pressure_basis": "Round 9 UM14 8,10,12,14 psi fitted tire files",
                **metrics,
            }
        )

    return sorted(rows, key=lambda row: (row["model"], float(row["rim_width_in"]), row["compound"]))


def lc0_score_rows(deg_rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    lc0_rows = [row for row in deg_rows if row["compound"] == "LC0"]

    def norm(metric: str, *, higher_is_better: bool = True) -> dict[str, float]:
        values = [finite_float(row.get(metric)) for row in lc0_rows]
        values = [value for value in values if math.isfinite(value)]
        if not values:
            return {row["setup_id"]: math.nan for row in lc0_rows}
        lo = min(values)
        hi = max(values)
        scores: dict[str, float] = {}
        for row in lc0_rows:
            value = finite_float(row.get(metric))
            if not math.isfinite(value):
                score = math.nan
            elif abs(hi - lo) <= 1e-12:
                score = 1.0
            elif higher_is_better:
                score = (value - lo) / (hi - lo)
            else:
                score = (hi - value) / (hi - lo)
            scores[row["setup_id"]] = score
        return scores

    peak_scores = norm("final_peak_mu_y_p95")
    ky_scores = norm("final_ky_norm_per_rad")
    degradation_loss_scores = norm("peak_mu_y_delta_pct")
    output = []
    for row in lc0_rows:
        parts = [
            (peak_scores[row["setup_id"]], 0.45),
            (ky_scores[row["setup_id"]], 0.30),
            (degradation_loss_scores[row["setup_id"]], 0.25),
        ]
        weight_sum = sum(weight for score, weight in parts if math.isfinite(score))
        score = (
            sum(score_part * weight for score_part, weight in parts if math.isfinite(score_part)) / weight_sum
            if weight_sum > 0.0
            else math.nan
        )
        scored = dict(row)
        scored["lc0_raw_cornering_score"] = score
        output.append(scored)
    ranked = sorted(output, key=lambda row: finite_float(row["lc0_raw_cornering_score"]), reverse=True)
    for rank, row in enumerate(ranked, start=1):
        row["lc0_raw_rank"] = rank
    return ranked


def discover_lc0_vehicle_candidates(ds006: Any) -> list[Any]:
    manifest_rows = [
        row
        for row in read_csv_rows(ROUND8_TIRE_MANIFEST)
        if row.get("written") == "1"
        and row.get("compound") == "LC0"
        and row.get("generated_tir")
    ]
    candidates = [
        ds006.TireCandidate(
            candidate_id="current_hybrid_reference",
            label="current hybrid reference",
            path=CURRENT_TIRE,
            source="current_reference",
            brand="Current",
            model="Hybrid",
            tire_size="16x7.5-10",
            rim_width_in=7.0,
            pressure_psi=12.0,
            longitudinal_combined_source="hybrid_reference",
            lateral_relaxation_source="hybrid_reference",
            is_reference=True,
            notes="Existing vehicle target/reference tire.",
        )
    ]
    for row in sorted(manifest_rows, key=lambda item: item["generated_tir"]):
        path = ROUND8_TIRE_DIR / row["generated_tir"]
        size = normalized_tire_size(row.get("tire_size", ""))
        model = model_for_size(row.get("tire_size", ""))
        rim_width = rim_width_from_label(row.get("rim", "nan"))
        pressure = finite_float(row.get("pressure_psi"))
        label = f"Hoosier LC0 {model} {size} {rim_width:g}in {pressure:g} psi"
        candidates.append(
            ds006.TireCandidate(
                candidate_id=f"Round_8_{slugify(path.stem)}",
                label=label,
                path=path,
                source="round8_lc0_fabricated_longitudinal_um3",
                brand="Hoosier",
                model=f"LC0 {model}",
                tire_size=size,
                rim_width_in=rim_width,
                pressure_psi=pressure,
                longitudinal_combined_source="fabricated_longitudinal_no_combined",
                lateral_relaxation_source="not_available_round8_lc0_um3",
                is_reference=False,
                notes=(
                    "Round 8 LC0 PAC2002 USE_MODE=3 tire. Pure longitudinal terms are fabricated "
                    "from the reference tire and combined-slip longitudinal terms are zeroed."
                ),
                manifest=row,
            )
        )
    return candidates


def setup_key_for_row(row: dict[str, Any]) -> tuple[str, str, float]:
    return (str(row["model"]).replace("LC0 ", ""), str(row["size"]), float(row["rim_width_in"]))


def setup_key_for_candidate(candidate: Any) -> tuple[str, str, float]:
    return (
        str(candidate.model).replace("LC0 ", ""),
        str(candidate.tire_size),
        float(candidate.rim_width_in),
    )


def vehicle_degradation_rows(
    candidates: list[Any],
    deg_rows: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    lookup = {
        setup_key_for_row(row): row
        for row in deg_rows
        if row["compound"] == "LC0"
    }
    rows: list[dict[str, Any]] = []
    for candidate in candidates:
        if candidate.is_reference:
            continue
        raw = lookup.get(setup_key_for_candidate(candidate))
        if not raw:
            continue
        rows.append(
            {
                "candidate_id": candidate.candidate_id,
                "label": candidate.label,
                "degradation_status": "ok",
                "degradation_pressure_psi": DEGRADATION_PRESSURE_PSI,
                "degradation_corner_status": raw["status"],
                "degradation_corner_initial_runs": raw["initial_run"],
                "degradation_corner_final_runs": raw["final_run"],
                "degradation_corner_samples_initial": raw["initial_samples"],
                "degradation_corner_samples_final": raw["final_samples"],
                "degradation_corner_initial_peak_mu_y": raw["initial_peak_mu_y_p95"],
                "degradation_corner_final_peak_mu_y": raw["final_peak_mu_y_p95"],
                "degradation_corner_peak_mu_y_delta_pct": raw["peak_mu_y_delta_pct"],
                "degradation_corner_initial_ky_norm_per_rad": raw["initial_ky_norm_per_rad"],
                "degradation_corner_final_ky_norm_per_rad": raw["final_ky_norm_per_rad"],
                "degradation_corner_ky_delta_pct": raw["ky_delta_pct"],
                "degradation_corner_initial_tread_mean_c": raw["initial_tread_mean_c"],
                "degradation_corner_final_tread_mean_c": raw["final_tread_mean_c"],
                "degradation_corner_tread_delta_c": raw["tread_delta_c"],
                "degradation_drive_status": "not_available_round8_lc0_cornering_only",
                "degradation_drive_peak_mu_x_delta_pct": math.nan,
                "degradation_drive_kx_delta_pct": math.nan,
            }
        )
    return rows


def vehicle_temperature_rows(
    candidates: list[Any],
    temp_rows: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    lookup = {
        (str(row["model"]), str(row["size"]), float(row["rim_width_in"]), float(row["pressure_psi"])): row
        for row in temp_rows
        if row["compound"] == "LC0"
    }
    rows: list[dict[str, Any]] = []
    for candidate in candidates:
        if candidate.is_reference:
            continue
        raw = lookup.get((*setup_key_for_candidate(candidate), float(candidate.pressure_psi)))
        if not raw:
            continue
        rows.append(
            {
                "candidate_id": candidate.candidate_id,
                "label": candidate.label,
                "temperature_status": "ok",
                "transient_run": raw["transient_run"],
                "transient_pressure_psi": raw["pressure_psi"],
                "transient_temperature_samples": raw["samples"],
                "transient_elapsed_time_min_s": raw["elapsed_time_min_s"],
                "transient_elapsed_time_max_s": raw["elapsed_time_max_s"],
                "transient_tread_mean_c": raw["tread_mean_c"],
                "transient_tread_peak_c": raw["tread_peak_c"],
                "transient_tread_start_c": raw["tread_start_c"],
                "transient_tread_end_c": raw["tread_end_c"],
                "transient_tread_rise_c": raw["tread_rise_c"],
                "transient_tread_inner_mean_c": raw["inner_mean_c"],
                "transient_tread_center_mean_c": raw["center_mean_c"],
                "transient_tread_outer_mean_c": raw["outer_mean_c"],
                "transient_tread_inner_minus_outer_c": raw["inner_minus_outer_c"],
                "transient_surface_spread_mean_c": raw["surface_spread_mean_c"],
                "transient_rim_temp_mean_c": raw["rim_temp_mean_c"],
                "transient_ambient_temp_mean_c": raw["ambient_temp_mean_c"],
            }
        )
    return rows


def run_vehicle_level_analysis(
    *,
    ds006: Any,
    vehicle_path: Path,
    reuse: bool,
    skip_standardsim: bool,
    standardsim_limit: int | None,
    standardsim_workers: int,
    standard_max_ay_mps2: float,
    standard_case_timeout_s: float,
    deg_rows: list[dict[str, Any]],
    temp_rows: list[dict[str, Any]],
) -> tuple[list[Any], list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
    candidates = discover_lc0_vehicle_candidates(ds006)
    reference_radius_m = ds006.current_reference_radius_m()
    base_vehicle, vehicle_context = ds006.build_baseline_vehicle(vehicle_path, CURRENT_TIRE)
    vehicle_context["reference_unloaded_radius_m"] = reference_radius_m
    config = ds006.make_config()
    envelope_rows, characterization_rows = ds006.run_envelopes(
        base_vehicle,
        config,
        candidates,
        reference_radius_m,
    )

    standardsim_rows: list[dict[str, Any]] = []
    standardsim_errors: list[dict[str, Any]] = []
    if not skip_standardsim:
        standardsim_rows, standardsim_errors = ds006.run_standardsim(
            vehicle_path,
            candidates,
            reuse=reuse,
            limit=standardsim_limit,
            reference_radius_m=reference_radius_m,
            workers=standardsim_workers,
            standard_max_ay_mps2=standard_max_ay_mps2,
            standard_case_timeout_s=standard_case_timeout_s,
        )

    merged_rows = ds006.merge_results(
        envelope_rows,
        characterization_rows,
        standardsim_rows,
        vehicle_temperature_rows(candidates, temp_rows),
        vehicle_degradation_rows(candidates, deg_rows),
    )
    return candidates, envelope_rows, characterization_rows, standardsim_rows, standardsim_errors, {"rows": merged_rows, "context": vehicle_context}


def family_color(row: dict[str, Any]) -> str:
    if row["compound"] == "LC0" and row["model"] == "43075":
        return "#d97706"
    if row["compound"] == "LC0":
        return "#0f766e"
    if row["model"] == "43075":
        return "#64748b"
    return "#94a3b8"


def setup_short(row: dict[str, Any]) -> str:
    return f"{row['compound']} {row['model']} {row['size']} {float(row['rim_width_in']):g}in"


def plot_lc0_degradation(deg_rows: list[dict[str, Any]]) -> None:
    rows = sorted([row for row in deg_rows if row["compound"] == "LC0"], key=lambda row: finite_float(row["peak_mu_y_delta_pct"]))
    if not rows:
        return
    y = np.arange(len(rows), dtype=float)
    fig, axes = plt.subplots(1, 2, figsize=(13.2, 5.2), sharey=True)
    for ax, metric, xlabel, title in (
        (axes[0], "peak_mu_y_delta_pct", "Final minus initial peak mu_y [%]", "LC0 Peak Lateral Change"),
        (axes[1], "ky_delta_pct", "Final minus initial Ky/Fz [%]", "LC0 Cornering Stiffness Change"),
    ):
        ax.barh(y, [finite_float(row[metric]) for row in rows], color=[family_color(row) for row in rows], alpha=0.88)
        ax.axvline(0.0, color="#111827", linewidth=1.0, alpha=0.72)
        ax.set_xlabel(xlabel)
        ax.set_title(title)
        ax.grid(True, axis="x", linestyle="--", alpha=0.3)
    axes[0].set_yticks(y)
    axes[0].set_yticklabels([setup_short(row) for row in rows])
    fig.suptitle("Round 8 LC0 Initial-To-Final 12 psi Cornering Degradation")
    fig.tight_layout()
    fig.savefig(PLOT_DIR / "lc0_degradation.png", dpi=220)
    plt.close(fig)


def plot_r20_comparison(rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    rows = sorted(rows, key=lambda row: (row["model"], float(row["rim_width_in"])))
    labels = [row["setup"] for row in rows]
    x = np.arange(len(rows), dtype=float)
    width = 0.36
    fig, axes = plt.subplots(1, 2, figsize=(13.6, 5.8))
    axes[0].bar(x - width / 2.0, [finite_float(row["r20_final_peak_mu_y_p95"]) for row in rows], width, color="#94a3b8", label="R20")
    axes[0].bar(x + width / 2.0, [finite_float(row["lc0_final_peak_mu_y_p95"]) for row in rows], width, color="#0f766e", label="LC0")
    axes[0].set_ylabel("Final 12 psi peak mu_y p95 [-]")
    axes[0].set_title("Final 12 psi Lateral Peak")
    axes[1].bar(x - width / 2.0, [finite_float(row["r20_final_ky_norm_per_rad"]) for row in rows], width, color="#94a3b8", label="R20")
    axes[1].bar(x + width / 2.0, [finite_float(row["lc0_final_ky_norm_per_rad"]) for row in rows], width, color="#0f766e", label="LC0")
    axes[1].set_ylabel("Final 12 psi Ky/Fz [1/rad]")
    axes[1].set_title("Final 12 psi Cornering Stiffness")
    for ax in axes:
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=28, ha="right")
        ax.grid(True, axis="y", linestyle="--", alpha=0.3)
        ax.legend(frameon=False)
    fig.suptitle("Round 8 LC0 Versus Round 9 R20 Matched Tire/Rim Setups")
    fig.tight_layout()
    fig.savefig(PLOT_DIR / "lc0_vs_r20_final12.png", dpi=220)
    plt.close(fig)


def plot_pressure_mu_sensitivity(rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    ordered = sorted(rows, key=lambda row: (row["model"], float(row["rim_width_in"]), row["compound"]))
    y = np.arange(len(ordered), dtype=float)
    colors = ["#0f766e" if row["compound"] == "LC0" else "#64748b" for row in ordered]
    labels = [f"{row['compound']} {row['setup']}" for row in ordered]
    fig, ax = plt.subplots(figsize=(10.8, 5.8))
    ax.barh(y, [finite_float(row["dmu_dpressure_per_psi"]) for row in ordered], color=colors, alpha=0.88)
    ax.axvline(0.0, color="#111827", linewidth=1.0, alpha=0.72)
    ax.set_yticks(y)
    ax.set_yticklabels(labels)
    ax.set_xlabel("Linear fit dmu_y / dP [1/psi]")
    ax.set_title("Pressure Sensitivity Of Lateral Friction")
    ax.grid(True, axis="x", linestyle="--", alpha=0.3)
    fig.tight_layout()
    fig.savefig(PLOT_DIR / "mu_pressure_sensitivity.png", dpi=220)
    plt.close(fig)


def plot_pressure_response(rows: list[dict[str, Any]]) -> None:
    lc0_rows = [
        row
        for row in rows
        if row["compound"] == "LC0"
        and row["window"] != "initial_run_12psi"
        and row["status"] == "ok"
    ]
    if not lc0_rows:
        return
    groups: dict[str, list[dict[str, Any]]] = {}
    for row in lc0_rows:
        groups.setdefault(row["setup_id"], []).append(row)
    fig, axes = plt.subplots(1, 2, figsize=(13.2, 5.7), sharex=True)
    colors = {"43075": "#d97706", "43070": "#0f766e"}
    for group_rows in groups.values():
        ordered = sorted(group_rows, key=lambda row: finite_float(row["pressure_psi"]))
        label = f"{ordered[0]['model']} {ordered[0]['size']} {float(ordered[0]['rim_width_in']):g}in"
        marker = "o" if ordered[0]["model"] == "43075" else "s"
        linestyle = "-" if float(ordered[0]["rim_width_in"]) in (7.0,) else "--"
        for ax, metric, ylabel in (
            (axes[0], "peak_mu_y_p95", "Measured peak mu_y p95 [-]"),
            (axes[1], "ky_norm_per_rad", "Measured Ky/Fz [1/rad]"),
        ):
            ax.plot(
                [finite_float(row["pressure_psi"]) for row in ordered],
                [finite_float(row[metric]) for row in ordered],
                marker=marker,
                linestyle=linestyle,
                linewidth=1.8,
                color=colors[str(ordered[0]["model"])],
                alpha=0.88,
                label=label,
            )
            ax.set_ylabel(ylabel)
            ax.grid(True, linestyle="--", alpha=0.3)
    for ax in axes:
        ax.set_xlabel("Pressure [psi]")
    axes[0].set_title("LC0 Pressure Response: Peak Lateral")
    axes[1].set_title("LC0 Pressure Response: Cornering Stiffness")
    handles, labels = axes[0].get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    fig.legend(unique.values(), unique.keys(), loc="center left", bbox_to_anchor=(1.0, 0.5), frameon=False)
    fig.tight_layout(rect=(0.0, 0.0, 0.84, 1.0))
    fig.savefig(PLOT_DIR / "lc0_pressure_response.png", dpi=220)
    plt.close(fig)


def plot_transient_temperature(rows: list[dict[str, Any]]) -> None:
    lc0_rows = [row for row in rows if row["compound"] == "LC0"]
    if not lc0_rows:
        return
    groups: dict[str, list[dict[str, Any]]] = {}
    for row in lc0_rows:
        groups.setdefault(row["setup_id"], []).append(row)
    fig, axes = plt.subplots(1, 2, figsize=(13.2, 5.6), sharex=True)
    colors = {"43075": "#d97706", "43070": "#0f766e"}
    for group_rows in groups.values():
        ordered = sorted(group_rows, key=lambda row: finite_float(row["pressure_psi"]))
        label = f"{ordered[0]['model']} {ordered[0]['size']} {float(ordered[0]['rim_width_in']):g}in"
        marker = "o" if ordered[0]["model"] == "43075" else "s"
        linestyle = "-" if float(ordered[0]["rim_width_in"]) in (7.0,) else "--"
        for ax, metric, ylabel in (
            (axes[0], "tread_mean_c", "Mean tread temperature [C]"),
            (axes[1], "tread_rise_c", "Transient tread rise [C]"),
        ):
            ax.plot(
                [finite_float(row["pressure_psi"]) for row in ordered],
                [finite_float(row[metric]) for row in ordered],
                marker=marker,
                linestyle=linestyle,
                linewidth=1.8,
                color=colors[str(ordered[0]["model"])],
                alpha=0.88,
                label=label,
            )
            ax.set_ylabel(ylabel)
            ax.grid(True, linestyle="--", alpha=0.3)
    for ax in axes:
        ax.set_xlabel("Pressure [psi]")
    axes[0].set_title("LC0 Transient Tread Temperature")
    axes[1].set_title("LC0 Transient Temperature Rise")
    handles, labels = axes[0].get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    fig.legend(unique.values(), unique.keys(), loc="center left", bbox_to_anchor=(1.0, 0.5), frameon=False)
    fig.tight_layout(rect=(0.0, 0.0, 0.84, 1.0))
    fig.savefig(PLOT_DIR / "lc0_transient_temperature.png", dpi=220)
    plt.close(fig)


def vehicle_candidate_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [row for row in rows if int(row.get("is_reference", 0)) == 0]


def standardsim_qa_pass(row: dict[str, Any]) -> bool:
    return (
        str(row.get("standardsim_status")) == "ok"
        and str(row.get("standardsim_quality_status", "ok")) == "ok"
    )


def vehicle_label(row: dict[str, Any]) -> str:
    return f"{row['model']} {row['tire_size']} {float(row['rim_width_in']):g}in/{float(row['pressure_psi']):g}psi"


def plot_vehicle_rank(rows: list[dict[str, Any]]) -> None:
    candidates = [
        row
        for row in vehicle_candidate_rows(rows)
        if math.isfinite(finite_float(row.get("integrated_design_score")))
    ]
    if not candidates:
        candidates = [
            row
            for row in vehicle_candidate_rows(rows)
            if math.isfinite(finite_float(row.get("envelope_score")))
        ]
        metric = "envelope_score"
        title = "LC0 EnvelopeSim Ranking"
    else:
        metric = "integrated_design_score"
        title = "LC0 Integrated Vehicle-Level Ranking"
    candidates = sorted(candidates, key=lambda row: finite_float(row.get(metric)))
    fig, ax = plt.subplots(figsize=(12.6, max(6.6, 0.34 * len(candidates) + 1.8)))
    colors = ["#d97706" if "43075" in str(row.get("model")) else "#0f766e" for row in candidates]
    ax.barh([vehicle_label(row) for row in candidates], [finite_float(row.get(metric)) for row in candidates], color=colors, alpha=0.9)
    ax.set_xlabel(metric.replace("_", " "))
    ax.set_title(title)
    ax.grid(True, axis="x", linestyle="--", alpha=0.3)
    fig.tight_layout()
    fig.savefig(PLOT_DIR / "lc0_vehicle_rank.png", dpi=220)
    plt.close(fig)


def plot_vehicle_trade_space(rows: list[dict[str, Any]]) -> None:
    candidates = [
        row
        for row in vehicle_candidate_rows(rows)
        if math.isfinite(finite_float(row.get("envelope_score")))
    ]
    if not candidates:
        return
    fig, axes = plt.subplots(1, 2, figsize=(13.0, 5.8))
    for ax, y_key, ylabel in (
        (axes[0], "standardsim_score", "StandardSim stable-window score"),
        (axes[1], "degradation_corner_peak_mu_y_delta_pct", "Initial/final peak mu_y change [%]"),
    ):
        for row in candidates:
            is_43075 = "43075" in str(row.get("model"))
            ax.scatter(
                finite_float(row.get("mean_max_lateral_g")),
                finite_float(row.get(y_key)),
                s=54.0 + 7.0 * finite_float(row.get("pressure_psi")),
                color="#d97706" if is_43075 else "#0f766e",
                edgecolor="white",
                linewidth=0.6,
                alpha=0.84,
            )
        ax.set_xlabel("EnvelopeSim mean lateral [g]")
        ax.set_ylabel(ylabel)
        ax.grid(True, linestyle="--", alpha=0.3)
    axes[0].set_title("LC0 Envelope Versus StandardSim")
    axes[1].set_title("LC0 Envelope Versus Raw Degradation")
    for row in sorted(candidates, key=lambda item: finite_float(item.get("integrated_design_score")), reverse=True)[:5]:
        axes[0].annotate(
            str(row.get("integrated_rank", row.get("envelope_rank", ""))),
            (finite_float(row.get("mean_max_lateral_g")), finite_float(row.get("standardsim_score"))),
            textcoords="offset points",
            xytext=(4, 4),
            fontsize=8,
        )
    fig.tight_layout()
    fig.savefig(PLOT_DIR / "lc0_vehicle_trade_space.png", dpi=220)
    plt.close(fig)


def plot_vehicle_pressure_response(rows: list[dict[str, Any]]) -> None:
    candidates = vehicle_candidate_rows(rows)
    if not candidates:
        return
    groups: dict[tuple[str, str, float], list[dict[str, Any]]] = {}
    for row in candidates:
        groups.setdefault((str(row["model"]), str(row["tire_size"]), float(row["rim_width_in"])), []).append(row)
    fig, axes = plt.subplots(1, 2, figsize=(13.2, 5.8), sharex=True)
    for group_rows in groups.values():
        ordered = sorted(group_rows, key=lambda row: finite_float(row["pressure_psi"]))
        label = f"{ordered[0]['model']} {ordered[0]['tire_size']} {float(ordered[0]['rim_width_in']):g}in"
        color = "#d97706" if "43075" in str(ordered[0]["model"]) else "#0f766e"
        linestyle = "-" if float(ordered[0]["rim_width_in"]) == 7.0 else "--"
        for ax, metric, ylabel in (
            (axes[0], "mean_max_lateral_g", "EnvelopeSim mean lateral [g]"),
            (axes[1], "standardsim_score", "StandardSim score"),
        ):
            values = [finite_float(row.get(metric)) for row in ordered]
            if not any(math.isfinite(value) for value in values):
                continue
            ax.plot(
                [finite_float(row["pressure_psi"]) for row in ordered],
                values,
                marker="o",
                linewidth=1.7,
                linestyle=linestyle,
                color=color,
                alpha=0.86,
                label=label,
            )
            ax.set_ylabel(ylabel)
            ax.grid(True, linestyle="--", alpha=0.3)
    for ax in axes:
        ax.set_xlabel("Pressure [psi]")
    axes[0].set_title("LC0 EnvelopeSim Pressure Response")
    axes[1].set_title("LC0 StandardSim Pressure Response")
    handles, labels = axes[0].get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    fig.legend(unique.values(), unique.keys(), loc="center left", bbox_to_anchor=(1.0, 0.5), frameon=False)
    fig.tight_layout(rect=(0.0, 0.0, 0.82, 1.0))
    fig.savefig(PLOT_DIR / "lc0_vehicle_pressure_response.png", dpi=220)
    plt.close(fig)


def plot_outputs(
    deg_rows: list[dict[str, Any]],
    pressure_summary: list[dict[str, Any]],
    temp_rows: list[dict[str, Any]],
    comparison_rows: list[dict[str, Any]],
    pressure_sensitivity_rows: list[dict[str, Any]],
    vehicle_rows: list[dict[str, Any]],
) -> None:
    PLOT_DIR.mkdir(parents=True, exist_ok=True)
    plot_lc0_degradation(deg_rows)
    plot_r20_comparison(comparison_rows)
    plot_pressure_mu_sensitivity(pressure_sensitivity_rows)
    plot_pressure_response(pressure_summary)
    plot_transient_temperature(temp_rows)
    plot_vehicle_rank(vehicle_rows)
    plot_vehicle_trade_space(vehicle_rows)
    plot_vehicle_pressure_response(vehicle_rows)


def top_row(rows: list[dict[str, Any]], metric: str, *, reverse: bool = True) -> dict[str, Any] | None:
    candidates = [row for row in rows if math.isfinite(finite_float(row.get(metric)))]
    if not candidates:
        return None
    return sorted(candidates, key=lambda row: finite_float(row[metric]), reverse=reverse)[0]


def write_report(
    *,
    started_at: str,
    elapsed_s: float,
    deg_rows: list[dict[str, Any]],
    pressure_summary: list[dict[str, Any]],
    temp_rows: list[dict[str, Any]],
    comparison_rows: list[dict[str, Any]],
    pressure_sensitivity_rows: list[dict[str, Any]],
    lc0_ranked: list[dict[str, Any]],
    vehicle_rows: list[dict[str, Any]],
    standardsim_errors: list[dict[str, Any]],
    skip_standardsim: bool,
    standard_max_ay_mps2: float,
) -> None:
    lc0_deg = [row for row in deg_rows if row["compound"] == "LC0"]
    lc0_pressure = [row for row in pressure_summary if row["compound"] == "LC0"]
    vehicle_candidates = vehicle_candidate_rows(vehicle_rows)
    vehicle_reference = next((row for row in vehicle_rows if int(row.get("is_reference", 0)) == 1), None)
    standardsim_ok_count = sum(1 for row in vehicle_rows if str(row.get("standardsim_status")) == "ok")
    standardsim_qa_pass_count = sum(1 for row in vehicle_rows if standardsim_qa_pass(row))
    standardsim_qa_fail_count = sum(
        1
        for row in vehicle_rows
        if str(row.get("standardsim_status")) == "ok"
        and str(row.get("standardsim_quality_status", "ok")) != "ok"
    )
    vehicle_top = sorted(
        [
            row
            for row in vehicle_candidates
            if math.isfinite(finite_float(row.get("integrated_design_score")))
        ],
        key=lambda row: finite_float(row.get("integrated_design_score")),
        reverse=True,
    )
    envelope_top = sorted(
        [
            row
            for row in vehicle_candidates
            if math.isfinite(finite_float(row.get("envelope_score")))
        ],
        key=lambda row: finite_float(row.get("envelope_score")),
        reverse=True,
    )
    standardsim_diagnostics = sorted(
        [
            row
            for row in vehicle_candidates
            if str(row.get("standardsim_status")) == "ok"
        ],
        key=lambda row: finite_float(row.get("envelope_score")),
        reverse=True,
    )
    winner = lc0_ranked[0] if lc0_ranked else None
    best_peak = top_row(lc0_deg, "final_peak_mu_y_p95")
    best_stiffness = top_row(lc0_deg, "final_ky_norm_per_rad")
    worst_degradation = top_row(lc0_deg, "peak_mu_y_delta_pct", reverse=False)
    best_degradation = top_row(lc0_deg, "peak_mu_y_delta_pct")
    r20_43075_rows = [row for row in comparison_rows if row["model"] == "43075"]
    r20_43070_rows = [row for row in comparison_rows if row["model"] == "43070"]
    lc0_pressure_slopes = [row for row in pressure_sensitivity_rows if row.get("compound") == "LC0"]
    r20_pressure_slopes = [row for row in pressure_sensitivity_rows if row.get("compound") == "R20"]

    lines: list[str] = []
    lines.append("# DS-008 Round 8 LC0 Tire Analysis")
    lines.append("")
    lines.append(f"Generated UTC: {started_at}")
    lines.append("")
    lines.append("## Purpose")
    lines.append("")
    lines.append(
        "This study extracts the Round 8 10 inch tire cornering archive and RunGuide matrix to "
        "characterize the Hoosier LC0 compound. It mirrors the DS-006 tire-data evidence style: "
        "initial/final 12 psi degradation, pressure response, transient temperature, matched R20 "
        "comparison against the DS-006 Round 9 Hoosier R20 rows, and vehicle-level "
        "EnvelopeSim/StandardSim screening for the existing LC0 tire files."
    )
    lines.append("")
    lines.append("The RunGuide text labels the compound as `LCO`; the existing tire-fit archives and this report use `LC0`.")
    lines.append("")
    lines.append("## Source Of Results")
    lines.append("")
    lines.append(table_line(["Result type", "Source"]))
    lines.append(table_line(["---", "---"]))
    lines.append(table_line(["Run matrix", "`RunGuide_Round8.pdf`"]))
    lines.append(table_line(["Raw cornering channels", "`RunData_Cornering_Matlab_SI_10inch_Round8.zip`"]))
    lines.append(table_line(["R20 comparison rows", "`studies/DS-006-integrated-tire-design/outputs/integrated_results.csv`"]))
    lines.append(table_line(["Envelope limits", "`BobSim/_2_EnvelopeSim/GGV/ggv_generation.py` via DS-006 helpers"]))
    lines.append(table_line(["Steady-state vehicle response", "`BobSim/_3_StandardSim/SteadyStateEval/steady_state_eval_sim.py` via DS-006 helpers"]))
    lines.append(table_line(["LC0 vehicle tire files", "`vehicles/current/tires/round_8_fabricated_longitudinal_um3`"]))
    lines.append(table_line(["Report generator", "`studies/DS-008-round8-lc0-analysis/run.py`"]))
    lines.append("")
    lines.append(
        "Vehicle-level results are screening evidence only. The Round 8 LC0 `.tir` files are PAC2002 "
        "`USE_MODE = 3` records with fabricated pure longitudinal terms and zeroed combined-slip "
        "longitudinal terms, so lateral/cornering conclusions carry more confidence than longitudinal "
        "or combined-load conclusions."
    )
    lines.append("")
    lines.append("## Coverage")
    lines.append("")
    lines.append(table_line(["Item", "Value"]))
    lines.append(table_line(["---", "---:"]))
    lines.append(table_line(["Round 8 LC0 setups", str(len(lc0_deg))]))
    lines.append(table_line(["Matched Round 9 R20 comparison setups", str(len(comparison_rows))]))
    lines.append(table_line(["Pressure-window rows", str(len(pressure_summary))]))
    lines.append(table_line(["Transient-temperature rows", str(len(temp_rows))]))
    lines.append(table_line(["LC0 vehicle candidates", str(len(vehicle_candidates))]))
    lines.append(table_line(["StandardSim errors", str(len(standardsim_errors))]))
    lines.append(table_line(["StandardSim successful rows", str(standardsim_ok_count)]))
    lines.append(table_line(["StandardSim QA-pass rows", str(standardsim_qa_pass_count)]))
    lines.append(table_line(["StandardSim QA-fail rows", str(standardsim_qa_fail_count)]))
    lines.append("")
    lines.append("## Executive Read")
    lines.append("")
    if vehicle_top:
        vehicle_winner = vehicle_top[0]
        lines.append(
            f"Best LC0 vehicle-level screening setup: **{vehicle_winner['label']}** with integrated score "
            f"`{format_float(vehicle_winner.get('integrated_design_score'), 3)}`, EnvelopeSim score "
            f"`{format_float(vehicle_winner.get('envelope_score'), 3)}`, StandardSim score "
            f"`{format_float(vehicle_winner.get('standardsim_score'), 3)}`, and raw 12 psi peak-mu degradation "
            f"`{format_pct(vehicle_winner.get('degradation_corner_peak_mu_y_delta_pct'))}`."
        )
        lines.append("")
    elif envelope_top and skip_standardsim:
        vehicle_winner = envelope_top[0]
        lines.append(
            f"StandardSim was skipped, so the LC0 vehicle-level screen is EnvelopeSim-only. "
            f"The EnvelopeSim leader is **{vehicle_winner['label']}** with envelope score "
            f"`{format_float(vehicle_winner.get('envelope_score'), 3)}`."
        )
        lines.append("")
    elif envelope_top:
        vehicle_winner = envelope_top[0]
        lines.append(
            "No integrated LC0 winner is selected because every StandardSim row failed the QA gate "
            f"(`{standardsim_qa_fail_count}` successful-but-QA-failed rows, all flagged by failed maneuver count). "
            f"The vehicle-level screen is therefore EnvelopeSim plus raw tire-data evidence. The EnvelopeSim leader is "
            f"**{vehicle_winner['label']}** with envelope score `{format_float(vehicle_winner.get('envelope_score'), 3)}`, "
            f"mean lateral capability `{format_float(vehicle_winner.get('mean_max_lateral_g'), 3)} g`, and raw 12 psi peak-mu degradation "
            f"`{format_pct(vehicle_winner.get('degradation_corner_peak_mu_y_delta_pct'))}`."
        )
        lines.append("")
    if winner:
        lines.append(
            f"Best LC0 raw cornering setup: **{winner['label']}**. It has final 12 psi peak mu "
            f"`{format_float(winner['final_peak_mu_y_p95'], 3)}`, final Ky/Fz "
            f"`{format_float(winner['final_ky_norm_per_rad'], 2)} 1/rad`, and peak-mu degradation "
            f"`{format_pct(winner['peak_mu_y_delta_pct'])}`."
        )
        lines.append("")
    if best_peak and best_stiffness and worst_degradation:
        lines.append(
            f"LC0 peak lateral capability is strongest on **{best_peak['label']}**, while small-slip "
            f"stiffness is strongest on **{best_stiffness['label']}**. The main caution is the "
            f"LC0 43075: its 12 psi peak-mu repeat falls `{format_pct(abs(finite_float(worst_degradation['peak_mu_y_delta_pct'])), signed=False)}` "
            "or more depending on rim."
        )
        lines.append("")
    lines.append(
        "The clean takeaway: **LC0 43070 looks healthier than LC0 43075 in Round 8 raw cornering data**. "
        "The 43075 LC0 variants have higher initial promise but meaningfully degrade by the final 12 psi repeat; "
        "the 43070 LC0 variants are much more stable."
    )
    lines.append("")
    lines.append("## LC0 Vehicle-Level Screening")
    lines.append("")
    lines.append(
        f"EnvelopeSim and StandardSim are run with the same DS-006 architecture correction and stable "
        f"`{format_float(standard_max_ay_mps2, 1)} m/s^2` StandardSim scoring window. This section is "
        "included to answer the vehicle-design question, but it must be read with the LC0 UM3 tire-file "
        "caveat above."
    )
    lines.append("")
    if standardsim_qa_fail_count and standardsim_qa_pass_count == 0:
        lines.append(
            "StandardSim completed numerically for the LC0 cases, but no LC0 row passes the DS-006 QA gate. "
            "Each row reports one failed maneuver point in the sweep, so StandardSim response metrics are "
            "retained below as diagnostics and are not converted into an integrated score."
        )
        lines.append("")
    if vehicle_top or envelope_top:
        ranked_vehicle = vehicle_top if vehicle_top else envelope_top
        rank_label = "Integrated rank" if vehicle_top else "Envelope rank"
        lines.append(table_line([rank_label, "Candidate", "dR", "Envelope", "Std", "Integrated", "Mean lat", "US grad", "Roll", "Raw degr", "Std QA", "Long source"]))
        lines.append(table_line(["---:", "---", "---:", "---:", "---:", "---:", "---:", "---:", "---:", "---:", "---", "---"]))
        for idx, row in enumerate(ranked_vehicle[:16], start=1):
            rank_value = row.get("integrated_rank") if vehicle_top else row.get("envelope_rank")
            if rank_value in {"", None}:
                rank_value = idx
            lines.append(
                table_line(
                    [
                        rank_value,
                        row["label"],
                        f"{format_float(row.get('architecture_radius_delta_mm'), 1)} mm",
                        format_float(row.get("envelope_score"), 3),
                        format_float(row.get("standardsim_score"), 3),
                        format_float(row.get("integrated_design_score"), 3),
                        f"{format_float(row.get('mean_max_lateral_g'), 3)} g",
                        format_float(row.get("understeer_gradient_deg_per_g"), 3),
                        format_float(row.get("roll_gradient_deg_per_g"), 3),
                        format_pct(row.get("degradation_corner_peak_mu_y_delta_pct")),
                        row.get("standardsim_quality_flags", ""),
                        row.get("longitudinal_combined_source", ""),
                    ]
                )
            )
        lines.append("")
    else:
        lines.append("No LC0 vehicle-level rows were generated.")
        lines.append("")
    if standardsim_diagnostics:
        lines.append("StandardSim diagnostic response metrics:")
        lines.append("")
        lines.append(table_line(["Candidate", "ay diag", "US grad", "Sideslip", "Roll", "HWT peak", "Failed cases", "QA flags"]))
        lines.append(table_line(["---", "---:", "---:", "---:", "---:", "---:", "---:", "---"]))
        for row in standardsim_diagnostics[:16]:
            lines.append(
                table_line(
                    [
                        row["label"],
                        f"{format_float(finite_float(row.get('ay_max')) / 9.80665, 3)} g",
                        format_float(row.get("understeer_gradient_deg_per_g"), 3),
                        format_float(row.get("sideslip_gradient_deg_per_g"), 3),
                        format_float(row.get("roll_gradient_deg_per_g"), 3),
                        format_float(row.get("handwheel_torque_peak_abs"), 2),
                        format_float(row.get("n_failed_cases"), 0),
                        row.get("standardsim_quality_flags", ""),
                    ]
                )
            )
        lines.append("")
    if vehicle_reference and (vehicle_top or envelope_top):
        comparison = vehicle_top[0] if vehicle_top else envelope_top[0]
        lines.append("Best LC0 setup versus current reference:")
        lines.append("")
        lines.append(table_line(["Metric", "Current reference", "Best LC0", "Delta"]))
        lines.append(table_line(["---", "---:", "---:", "---:"]))
        for metric, label, unit in (
            ("mean_max_lateral_g", "Envelope mean lateral", "g"),
            ("mean_ggv_area_g2", "Envelope mean GGV area", "g^2"),
            ("understeer_gradient_deg_per_g", "Understeer gradient", "deg/g"),
            ("roll_gradient_deg_per_g", "Roll gradient", "deg/g"),
            ("handwheel_torque_peak_abs", "Peak handwheel torque", "Nm"),
        ):
            ref_value = finite_float(vehicle_reference.get(metric))
            cmp_value = finite_float(comparison.get(metric))
            delta = safe_percent_delta(cmp_value, ref_value)
            lines.append(
                table_line(
                    [
                        label,
                        f"{format_float(ref_value, 3)} {unit}",
                        f"{format_float(cmp_value, 3)} {unit}",
                        format_pct(delta),
                    ]
                )
            )
        lines.append("")
    if standardsim_errors:
        lines.append("StandardSim errors:")
        lines.append("")
        lines.append(table_line(["Candidate", "Error"]))
        lines.append(table_line(["---", "---"]))
        for error in standardsim_errors:
            lines.append(table_line([error.get("label", ""), error.get("error", "")]))
        lines.append("")
    lines.append("## LC0 Initial/Final 12 psi Degradation")
    lines.append("")
    lines.append(
        "Degradation compares the initial 12 psi slip-angle sweep to the repeated final 12 psi sweep after "
        "the 10, 14, and 8 psi sequence. Metrics use the nominal 25 mph window (`34-47 km/h`), robust "
        "95th-percentile measured `|FY/FZ|`, and a small-slip Ky/Fz linear fit."
    )
    lines.append("")
    lines.append(table_line(["Rank", "LC0 setup", "Initial run", "Final run", "Peak mu_i", "Peak mu_f", "Peak delta", "Ky_i", "Ky_f", "Ky delta", "Tread delta"]))
    lines.append(table_line(["---:", "---", "---:", "---:", "---:", "---:", "---:", "---:", "---:", "---:", "---:"]))
    for row in lc0_ranked:
        lines.append(
            table_line(
                [
                    row["lc0_raw_rank"],
                    row["label"],
                    row["initial_run"],
                    row["final_run"],
                    format_float(row["initial_peak_mu_y_p95"], 3),
                    format_float(row["final_peak_mu_y_p95"], 3),
                    format_pct(row["peak_mu_y_delta_pct"]),
                    format_float(row["initial_ky_norm_per_rad"], 2),
                    format_float(row["final_ky_norm_per_rad"], 2),
                    format_pct(row["ky_delta_pct"]),
                    f"{format_float(row['tread_delta_c'], 1)} C",
                ]
            )
        )
    lines.append("")
    lines.append("## LC0 Versus R20")
    lines.append("")
    lines.append(
        "R20 is pulled from the DS-006 Round 9 Hoosier R20 study rows, matched by model, tire size, "
        "and rim width. This is a design-relevant cross-round comparison, not a same-round compound "
        "control; use it to compare LC0 against the current R20 decision set."
    )
    lines.append("")
    lines.append(table_line(["Matched setup", "LC0 peak mu", "R20 peak mu", "LC0 delta", "LC0 Ky", "R20 Ky", "LC0 Ky delta", "LC0 degr", "R20 degr", "R20 int"]))
    lines.append(table_line(["---", "---:", "---:", "---:", "---:", "---:", "---:", "---:", "---:", "---:"]))
    for row in comparison_rows:
        lines.append(
            table_line(
                [
                    row["setup"],
                    format_float(row["lc0_final_peak_mu_y_p95"], 3),
                    format_float(row["r20_final_peak_mu_y_p95"], 3),
                    format_pct(row["lc0_vs_r20_peak_mu_y_delta_pct"]),
                    format_float(row["lc0_final_ky_norm_per_rad"], 2),
                    format_float(row["r20_final_ky_norm_per_rad"], 2),
                    format_pct(row["lc0_vs_r20_ky_delta_pct"]),
                    format_pct(row["lc0_peak_mu_y_degradation_pct"]),
                    format_pct(row["r20_peak_mu_y_degradation_pct"]),
                    format_float(row["r20_integrated_design_score"], 3),
                ]
            )
        )
    lines.append("")
    if comparison_rows:
        lines.append(
            "Against matched R20, LC0 final 12 psi peak mu is "
            f"`{format_pct_span([row['lc0_vs_r20_peak_mu_y_delta_pct'] for row in comparison_rows])}` "
            "across the matched rows. The split is important: 43075 LC0 keeps similar peak mu but gives up "
            f"`{format_pct_span([row['lc0_vs_r20_ky_delta_pct'] for row in r20_43075_rows])}` in Ky/Fz and "
            "has the stronger repeat-loss warning, while 43070 LC0 is the healthier R20 comparison with peak "
            f"mu `{format_pct_span([row['lc0_vs_r20_peak_mu_y_delta_pct'] for row in r20_43070_rows])}` and "
            f"Ky/Fz `{format_pct_span([row['lc0_vs_r20_ky_delta_pct'] for row in r20_43070_rows])}` versus R20."
        )
    else:
        lines.append("No matched R20 rows were found in the DS-006 integrated output.")
    lines.append("")
    lines.append("## Lateral Friction Pressure Sensitivity")
    lines.append("")
    lines.append(
        r"The table reports $\partial \mu_y / \partial P$ as a linear-fit slope over the pressure series. "
        "LC0 uses observed raw `peak_mu_y_p95` from the Round 8 pressure windows, while R20 uses fitted "
        "`mu_y = abs(PDY1)` from the Round 9 UM14 tire files. The LC0 points mix run order "
        "(8 psi final, 10 psi initial, 12 psi final, 14 psi initial), so read those slopes as observed "
        "pressure-response evidence rather than a pure pressure-only causal derivative."
    )
    lines.append("")
    if pressure_sensitivity_rows:
        lines.append(table_line(["Source", "Setup", "dmu/dP", "%/psi @12", "8->14 delta", "R2", "Points"]))
        lines.append(table_line(["---", "---", "---:", "---:", "---:", "---:", "---:"]))
        for row in pressure_sensitivity_rows:
            lines.append(
                table_line(
                    [
                        row["compound"],
                        row["setup"],
                        f"{format_signed_float(row['dmu_dpressure_per_psi'], 4)} 1/psi",
                        format_pct(row["pct_dmu_dpressure_per_psi_at_12psi"], 2),
                        format_pct(row["mu_delta_8_to_14_pct"], 1),
                        format_float(row["linear_fit_r2"], 3),
                        row["point_count"],
                    ]
                )
            )
        lines.append("")
        lines.append(
            "In slope form, LC0 observed pressure sensitivity spans "
            f"`{format_signed_float_span([row['dmu_dpressure_per_psi'] for row in lc0_pressure_slopes], 4)} 1/psi`; "
            "the matched R20 fitted `abs(PDY1)` sensitivity spans "
            f"`{format_signed_float_span([row['dmu_dpressure_per_psi'] for row in r20_pressure_slopes], 4)} 1/psi`."
        )
    else:
        lines.append("No pressure-sensitivity rows were generated.")
    lines.append("")
    lines.append("## LC0 Pressure Response")
    lines.append("")
    lines.append(
        "The 8 psi point comes from the final-run pressure block; the 10 and 14 psi points come from the "
        "initial-run pressure block. Both initial and final 12 psi rows are retained in the CSV; the table "
        "below shows final 12 psi for consistency with degradation evidence."
    )
    lines.append("")
    lines.append(table_line(["LC0 setup", "Pressure", "Run", "Peak mu_y", "Ky/Fz", "Tread", "Samples"]))
    lines.append(table_line(["---", "---:", "---:", "---:", "---:", "---:", "---:"]))
    for row in sorted(
        [
            row
            for row in lc0_pressure
            if row["window"] in {"final_run_8psi", "initial_run_10psi", "final_run_12psi", "initial_run_14psi"}
        ],
        key=lambda row: (row["model"], row["size"], float(row["rim_width_in"]), float(row["pressure_psi"])),
    ):
        lines.append(
            table_line(
                [
                    row["label"],
                    format_float(row["pressure_psi"], 0),
                    row["run"],
                    format_float(row["peak_mu_y_p95"], 3),
                    format_float(row["ky_norm_per_rad"], 2),
                    f"{format_float(row['tread_mean_c'], 1)} C",
                    row["samples"],
                ]
            )
        )
    lines.append("")
    lines.append("## LC0 Transient Temperature")
    lines.append("")
    lc0_temp = sorted(
        [row for row in temp_rows if row["compound"] == "LC0"],
        key=lambda row: (row["model"], row["size"], float(row["rim_width_in"]), float(row["pressure_psi"])),
    )
    if lc0_temp:
        lines.append(table_line(["LC0 setup", "Pressure", "Mean tread", "Peak tread", "Rise", "I-O", "Spread", "Rim", "Ambient"]))
        lines.append(table_line(["---", "---:", "---:", "---:", "---:", "---:", "---:", "---:", "---:"]))
        for row in lc0_temp:
            lines.append(
                table_line(
                    [
                        row["label"],
                        format_float(row["pressure_psi"], 0),
                        f"{format_float(row['tread_mean_c'], 1)} C",
                        f"{format_float(row['tread_peak_c'], 1)} C",
                        f"{format_float(row['tread_rise_c'], 1)} C",
                        f"{format_float(row['inner_minus_outer_c'], 1)} C",
                        f"{format_float(row['surface_spread_mean_c'], 1)} C",
                        f"{format_float(row['rim_temp_mean_c'], 1)} C",
                        f"{format_float(row['ambient_temp_mean_c'], 1)} C",
                    ]
                )
            )
    else:
        lines.append("No LC0 transient temperature rows were extracted.")
    lines.append("")
    lines.append("## Findings")
    lines.append("")
    lines.append("- LC0 43075 shows the largest initial-to-final deterioration: peak mu falls `-6.4%` on the 8 in rim and `-7.4%` on the 7 in rim.")
    lines.append("- LC0 43070 is much more stable: peak mu falls only `-1.2%` on both rim widths, with final 12 psi Ky/Fz equal to or better than the 43075 LC0 rows.")
    lines.append("- LC0 pressure response is strongest at 8 psi in the final-run block for all LC0 setups, but that point is confounded with run order and should be treated as observed data, not a pressure-only causal result.")
    lines.append("- Against matched R20, LC0 is higher on final 12 psi peak mu in these rows, but 43075 LC0 is lower in Ky/Fz and has worse repeat degradation than R20.")
    lines.append("- Because Round 8 LC0 has no 10 inch drive/brake archive in this dataset, vehicle-level longitudinal/braking reads are lower-confidence than lateral and balance reads.")
    lines.append("")
    lines.append("## Figure Gallery")
    lines.append("")
    for caption, relative in [
        ("LC0 vehicle-level ranking", "lc0_vehicle_rank.png"),
        ("LC0 vehicle trade space", "lc0_vehicle_trade_space.png"),
        ("LC0 vehicle pressure response", "lc0_vehicle_pressure_response.png"),
        ("LC0 initial-to-final 12 psi degradation", "lc0_degradation.png"),
        ("LC0 versus matched R20 final 12 psi behavior", "lc0_vs_r20_final12.png"),
        ("Lateral friction pressure sensitivity", "mu_pressure_sensitivity.png"),
        ("LC0 measured pressure response", "lc0_pressure_response.png"),
        ("LC0 transient temperature evidence", "lc0_transient_temperature.png"),
    ]:
        lines.append(f"### {caption}")
        lines.append("")
        lines.append(markdown_image(caption, relative))
        lines.append("")
    lines.append("## Generated Files")
    lines.append("")
    for relative in [
        "outputs/degradation_summary.csv",
        "outputs/pressure_window_summary.csv",
        "outputs/transient_temperature_summary.csv",
        "outputs/r20_comparison.csv",
        "outputs/pressure_mu_sensitivity.csv",
        "outputs/lc0_ranked_summary.csv",
        "outputs/vehicle_candidate_registry.csv",
        "outputs/vehicle_tire_characterization.csv",
        "outputs/vehicle_envelope_metrics.csv",
        "outputs/vehicle_standardsim_metrics.csv",
        "outputs/vehicle_standardsim_errors.csv",
        "outputs/vehicle_integrated_results.csv",
        "outputs/run_provenance.csv",
        "plots/lc0_vehicle_rank.png",
        "plots/lc0_vehicle_trade_space.png",
        "plots/lc0_vehicle_pressure_response.png",
        "plots/lc0_degradation.png",
        "plots/lc0_vs_r20_final12.png",
        "plots/mu_pressure_sensitivity.png",
        "plots/lc0_pressure_response.png",
        "plots/lc0_transient_temperature.png",
    ]:
        lines.append(f"- `{relative}`")
    lines.append("")
    lines.append("## Run Provenance")
    lines.append("")
    lines.append(table_line(["Item", "Value"]))
    lines.append(table_line(["---", "---"]))
    lines.append(table_line(["Elapsed time", f"{elapsed_s:.1f} s"]))
    lines.append(table_line(["Python", sys.executable]))
    lines.append("")

    text = "\n".join(lines)
    (STUDY_DIR / "RESULTS.md").write_text(text.replace(PLOT_PREFIX_TOKEN, "plots"), encoding="utf-8")
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(
        text.replace(PLOT_PREFIX_TOKEN, "../studies/DS-008-round8-lc0-analysis/plots"),
        encoding="utf-8",
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--vehicle",
        type=Path,
        default=REPO_ROOT / "vehicles" / "current" / "vehicle.yml",
    )
    parser.add_argument(
        "--skip-standardsim",
        action="store_true",
        help="Only run raw tire analysis and EnvelopeSim vehicle screening.",
    )
    parser.add_argument(
        "--standardsim-limit",
        type=int,
        default=None,
        help="Debug limit for StandardSim cases. Omit for every LC0 tire plus reference.",
    )
    parser.add_argument(
        "--standardsim-workers",
        type=int,
        default=1,
        help="Number of StandardSim tire cases to run concurrently.",
    )
    parser.add_argument(
        "--standardsim-max-ay",
        type=float,
        default=STANDARD_STABLE_MAX_AY_MPS2,
        help="Commanded StandardSim SteadyStateEval maxAy in m/s^2.",
    )
    parser.add_argument(
        "--standardsim-case-timeout",
        type=float,
        default=STANDARD_CASE_TIMEOUT_S,
        help="Per-velocity StandardSim wall-clock timeout in seconds.",
    )
    parser.add_argument(
        "--reuse",
        action="store_true",
        help="Reuse StandardSim builds/results when the generated variant is unchanged.",
    )
    args = parser.parse_args()

    started_at = datetime.now(timezone.utc).isoformat(timespec="seconds")
    start = time.perf_counter()
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    PLOT_DIR.mkdir(parents=True, exist_ok=True)

    if not RUN_GUIDE.exists():
        raise SystemExit(f"Missing RunGuide: {RUN_GUIDE}")
    if not CORNERING_ARCHIVE.exists():
        raise SystemExit(f"Missing cornering archive: {CORNERING_ARCHIVE}")

    cache: dict[int, dict[str, Any]] = {}
    print("DS-008 Round 8 LC0 Tire Analysis", flush=True)
    print(f"  RunGuide: {RUN_GUIDE.name}", flush=True)
    print(f"  Cornering archive: {CORNERING_ARCHIVE.name}", flush=True)
    print(f"  Setups: {len(ROUND8_SPECS)} total, {sum(1 for spec in ROUND8_SPECS if spec.compound == 'LC0')} LC0", flush=True)

    deg_rows = degradation_rows(cache)
    pressure_summary = pressure_rows(cache)
    temp_rows = transient_temperature_rows(cache)
    comparison_rows = r20_comparison_rows(deg_rows)
    pressure_sensitivity = pressure_mu_sensitivity_rows(pressure_summary, comparison_rows)
    lc0_ranked = lc0_score_rows(deg_rows)

    ds006 = load_ds006_module()
    (
        vehicle_candidates,
        vehicle_envelope_rows,
        vehicle_characterization_rows,
        vehicle_standardsim_rows,
        vehicle_standardsim_errors,
        vehicle_result,
    ) = run_vehicle_level_analysis(
        ds006=ds006,
        vehicle_path=args.vehicle,
        reuse=args.reuse,
        skip_standardsim=args.skip_standardsim,
        standardsim_limit=args.standardsim_limit,
        standardsim_workers=args.standardsim_workers,
        standard_max_ay_mps2=args.standardsim_max_ay,
        standard_case_timeout_s=args.standardsim_case_timeout,
        deg_rows=deg_rows,
        temp_rows=temp_rows,
    )
    vehicle_rows = vehicle_result["rows"]
    print(f"  LC0 vehicle candidates: {len(vehicle_candidates) - 1}", flush=True)
    print(f"  StandardSim errors: {len(vehicle_standardsim_errors)}", flush=True)
    plot_outputs(deg_rows, pressure_summary, temp_rows, comparison_rows, pressure_sensitivity, vehicle_rows)

    write_csv(OUTPUT_DIR / "degradation_summary.csv", deg_rows)
    write_csv(OUTPUT_DIR / "pressure_window_summary.csv", pressure_summary)
    write_csv(OUTPUT_DIR / "transient_temperature_summary.csv", temp_rows)
    write_csv(OUTPUT_DIR / "r20_comparison.csv", comparison_rows)
    write_csv(OUTPUT_DIR / "pressure_mu_sensitivity.csv", pressure_sensitivity)
    write_csv(OUTPUT_DIR / "lc0_ranked_summary.csv", lc0_ranked)
    write_csv(
        OUTPUT_DIR / "vehicle_candidate_registry.csv",
        [
            {
                "candidate_id": candidate.candidate_id,
                "label": candidate.label,
                "source": candidate.source,
                "brand": candidate.brand,
                "model": candidate.model,
                "family": candidate.family,
                "tire_size": candidate.tire_size,
                "rim_width_in": candidate.rim_width_in,
                "pressure_psi": candidate.pressure_psi,
                "is_reference": int(candidate.is_reference),
                "path": str(candidate.path.relative_to(REPO_ROOT)),
                "longitudinal_combined_source": candidate.longitudinal_combined_source,
                "lateral_relaxation_source": candidate.lateral_relaxation_source,
                "notes": candidate.notes,
            }
            for candidate in vehicle_candidates
        ],
    )
    write_csv(OUTPUT_DIR / "vehicle_tire_characterization.csv", vehicle_characterization_rows)
    write_csv(OUTPUT_DIR / "vehicle_envelope_metrics.csv", vehicle_envelope_rows)
    write_csv(OUTPUT_DIR / "vehicle_standardsim_metrics.csv", vehicle_standardsim_rows)
    write_csv(OUTPUT_DIR / "vehicle_standardsim_errors.csv", vehicle_standardsim_errors)
    write_csv(OUTPUT_DIR / "vehicle_integrated_results.csv", vehicle_rows)
    write_csv(
        OUTPUT_DIR / "run_provenance.csv",
        [
            {"item": "generated_at_utc", "value": started_at},
            {"item": "vehicle", "value": str(args.vehicle.relative_to(REPO_ROOT))},
            {"item": "run_guide", "value": str(RUN_GUIDE.relative_to(REPO_ROOT))},
            {"item": "cornering_archive", "value": str(CORNERING_ARCHIVE.relative_to(REPO_ROOT))},
            {"item": "round8_tire_dir", "value": str(ROUND8_TIRE_DIR.relative_to(REPO_ROOT))},
            {"item": "lc0_setups", "value": sum(1 for spec in ROUND8_SPECS if spec.compound == "LC0")},
            {"item": "lc0_vehicle_candidates", "value": len(vehicle_candidates) - 1},
            {"item": "comparison_source", "value": str(DS006_INTEGRATED_RESULTS.relative_to(REPO_ROOT))},
            {"item": "comparison_compound", "value": "R20"},
            {"item": "comparison_setups", "value": len(comparison_rows)},
            {"item": "pressure_mu_sensitivity_rows", "value": len(pressure_sensitivity)},
            {"item": "degradation_pressure_psi", "value": DEGRADATION_PRESSURE_PSI},
            {"item": "nominal_speed_window_kph", "value": f"{NOMINAL_TEST_SPEED_KPH_MIN:g}-{NOMINAL_TEST_SPEED_KPH_MAX:g}"},
            {"item": "standardsim_skipped", "value": args.skip_standardsim},
            {"item": "standardsim_limit", "value": args.standardsim_limit},
            {"item": "standardsim_workers", "value": args.standardsim_workers},
            {"item": "standardsim_max_ay_mps2", "value": args.standardsim_max_ay},
            {"item": "standardsim_case_timeout_s", "value": args.standardsim_case_timeout},
            {"item": "reuse", "value": args.reuse},
        ],
    )

    elapsed_s = time.perf_counter() - start
    write_report(
        started_at=started_at,
        elapsed_s=elapsed_s,
        deg_rows=deg_rows,
        pressure_summary=pressure_summary,
        temp_rows=temp_rows,
        comparison_rows=comparison_rows,
        pressure_sensitivity_rows=pressure_sensitivity,
        lc0_ranked=lc0_ranked,
        vehicle_rows=vehicle_rows,
        standardsim_errors=vehicle_standardsim_errors,
        skip_standardsim=args.skip_standardsim,
        standard_max_ay_mps2=args.standardsim_max_ay,
    )
    print(f"Study report: {STUDY_DIR / 'RESULTS.md'}", flush=True)
    print(f"Top-level report: {REPORT_PATH}", flush=True)


if __name__ == "__main__":
    main()
