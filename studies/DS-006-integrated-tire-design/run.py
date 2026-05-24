#!/usr/bin/env python3
"""Run DS-006: integrated Round 9 tire design study."""

from __future__ import annotations

import argparse
import concurrent.futures
import csv
import dataclasses
import io
import importlib.util
import math
import os
import re
import shutil
import sys
import time
from collections import defaultdict
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from zipfile import ZipFile


REPO_ROOT = Path(__file__).resolve().parents[2]
STUDY_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = STUDY_DIR / "outputs"
PLOT_DIR = STUDY_DIR / "plots"
WORK_DIR = STUDY_DIR / "work"
REPORT_PATH = REPO_ROOT / "reports" / "DS-006-integrated-tire-design.md"
PLOT_PREFIX_TOKEN = "__DS006_PLOT_PREFIX__"

CURRENT_TIRE = REPO_ROOT / "vehicles" / "current" / "tires" / "16x7p5_10_12psi.tir"
ROUND9_TIRE_DIR = REPO_ROOT / "vehicles" / "current" / "tires" / "round_9_fitted_full_um14"
ROUND9_MANIFEST = ROUND9_TIRE_DIR / "manifest.csv"
ROUND9_DIAGNOSTICS = ROUND9_TIRE_DIR / "diagnostics" / "tire_diagnostic_summary.csv"
ROUND9_CORNERING_ARCHIVE = REPO_ROOT / "RunData_Cornering_Matlab_SI_Round9.zip"
ROUND9_DRIVE_ARCHIVE = REPO_ROOT / "RunData_DriveBrake_Matlab_SI_Round9.zip"
DEGRADATION_PRESSURE_PSI = 12.0
NOMINAL_TEST_SPEED_KPH_MIN = 34.0
NOMINAL_TEST_SPEED_KPH_MAX = 47.0

BOBSIM_ROOT = REPO_ROOT / "BobSim"
GENERATION_SCRIPTS = (
    BOBSIM_ROOT / "_0_Utils" / "external" / "BobLib" / "Generation" / "scripts"
)

MPLCONFIGDIR = Path("/tmp/lhre-sim-matplotlib")
MPLCONFIGDIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(MPLCONFIGDIR))

ENVELOPE_SCORE_METRICS = (
    ("mean_max_lateral_g", 0.35),
    ("max_lateral_g__25mps", 0.20),
    ("mean_ggv_area_g2", 0.25),
    ("mean_max_accel_g", 0.10),
    ("mean_max_brake_g", 0.10),
)

STANDARD_STABLE_MAX_AY_MPS2 = 8.0
STANDARD_CASE_TIMEOUT_S = 75.0

STANDARD_METRICS = (
    "ay_max",
    "roadwheel_angle_gradient_deg_per_g",
    "handwheel_angle_gradient_deg_per_g",
    "sideslip_gradient_deg_per_g",
    "understeer_gradient_deg_per_g",
    "roll_gradient_deg_per_g",
    "handwheel_torque_peak_abs",
)

STANDARD_QA_METRICS = (
    "n_cases",
    "n_successful_cases",
    "n_failed_cases",
    "standard_sweep_max_ay_mps2",
    "metric_target_velocity_mps",
    "metric_source_velocity_mps",
    "mean_curvature_error_pct",
    "max_curvature_error_pct",
    "mean_abs_rad_error",
    "max_abs_rad_error",
    "mean_abs_ay_command_error_mps2",
    "p95_abs_ay_command_error_mps2",
    "max_abs_ay_command_error_mps2",
    "roadwheel_fit_nrmse",
    "handwheel_fit_nrmse",
    "steer_excess_fit_nrmse",
    "roll_fit_nrmse",
    "sideslip_fit_nrmse",
)

REQUIRED_STANDARD_QA_METRICS = (
    "n_failed_cases",
    "standard_sweep_max_ay_mps2",
    "metric_source_velocity_mps",
    "roadwheel_fit_nrmse",
    "steer_excess_fit_nrmse",
)

TEMPERATURE_METRICS = (
    "transient_tread_mean_c",
    "transient_tread_peak_c",
    "transient_tread_rise_c",
    "transient_tread_inner_mean_c",
    "transient_tread_center_mean_c",
    "transient_tread_outer_mean_c",
    "transient_tread_inner_minus_outer_c",
    "transient_surface_spread_mean_c",
    "transient_rim_temp_mean_c",
    "transient_ambient_temp_mean_c",
)

DEGRADATION_METRICS = (
    "degradation_corner_peak_mu_y_delta_pct",
    "degradation_corner_ky_delta_pct",
    "degradation_corner_tread_delta_c",
    "degradation_drive_peak_mu_x_delta_pct",
    "degradation_drive_kx_delta_pct",
    "degradation_drive_tread_delta_c",
)


def require_dependencies() -> tuple[Any, Any, Any]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
        import yaml
    except ModuleNotFoundError as exc:
        missing = exc.name or "a required package"
        raise SystemExit(
            f"Missing Python dependency: {missing}\n"
            "Run with the study environment, for example:\n"
            "  /tmp/lhre-sim-venv/bin/python studies/DS-006-integrated-tire-design/run.py"
        ) from exc

    return np, plt, yaml


np, plt, yaml = require_dependencies()


def load_module(name: str, path: Path) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


ggv = load_module(
    "bobsim_envelopesim_ggv_ds006",
    REPO_ROOT / "BobSim" / "_2_EnvelopeSim" / "GGV" / "ggv_generation.py",
)
G = float(ggv.G)


@dataclass(frozen=True)
class TireCandidate:
    candidate_id: str
    label: str
    path: Path
    source: str
    brand: str
    model: str
    tire_size: str
    rim_width_in: float
    pressure_psi: float
    longitudinal_combined_source: str
    lateral_relaxation_source: str
    is_reference: bool
    notes: str
    manifest: dict[str, str] = dataclasses.field(default_factory=dict)
    diagnostics: dict[str, str] = dataclasses.field(default_factory=dict)

    @property
    def family(self) -> str:
        family = " ".join(part for part in (self.brand, self.model) if part).strip()
        return family or self.source

    @property
    def rim_label(self) -> str:
        return f"{self.rim_width_in:g}in" if math.isfinite(self.rim_width_in) else "unknown"


FIELD_RE = re.compile(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.*?)\s*(?:[$!].*)?$")


def as_repo_path(path: Path) -> str:
    try:
        return str(path.relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def slugify(value: str) -> str:
    value = re.sub(r"[^A-Za-z0-9]+", "_", value).strip("_")
    return value or "value"


def finite_float(value: Any) -> float:
    try:
        output = float(value)
    except (TypeError, ValueError):
        return math.nan
    return output if math.isfinite(output) else math.nan


def format_float(value: Any, digits: int = 4) -> str:
    value_f = finite_float(value)
    if not math.isfinite(value_f):
        return "nan"
    return f"{value_f:.{digits}f}"


def format_percent(value: Any, digits: int = 1) -> str:
    value_f = finite_float(value)
    if not math.isfinite(value_f):
        return "nan"
    return f"{100.0 * value_f:.{digits}f}%"


def format_pct_point(value: Any, digits: int = 1, *, signed: bool = True) -> str:
    value_f = finite_float(value)
    if not math.isfinite(value_f):
        return "n/a"
    sign = "+" if signed else ""
    return f"{value_f:{sign}.{digits}f}%"


def format_delta(value: Any, reference: Any, digits: int = 1) -> str:
    value_f = finite_float(value)
    reference_f = finite_float(reference)
    if (
        not math.isfinite(value_f)
        or not math.isfinite(reference_f)
        or abs(reference_f) <= 1e-12
    ):
        return "nan"
    return f"{100.0 * (value_f - reference_f) / reference_f:+.{digits}f}%"


def humanize_status(value: Any) -> str:
    return str(value or "").replace("_", " ")


def table_line(values: list[Any]) -> str:
    return "| " + " | ".join(str(value) for value in values) + " |"


def markdown_image(caption: str, relative_plot_path: str, *, prefix: str = PLOT_PREFIX_TOKEN) -> str:
    return f"![{caption}]({prefix}/{relative_plot_path})"


def read_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"Expected YAML mapping in {path}")
    return data


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[dict[str, Any]], fieldnames: list[str] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if fieldnames is None:
        keys: list[str] = []
        seen: set[str] = set()
        for row in rows:
            for key in row:
                if key not in seen:
                    seen.add(key)
                    keys.append(key)
        fieldnames = keys or ["empty"]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def parse_tir(path: Path) -> dict[str, float | str]:
    params: dict[str, float | str] = {}
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        match = FIELD_RE.match(line)
        if not match:
            continue
        key = match.group(1).upper()
        raw = match.group(2).strip().strip("'").strip('"')
        try:
            params[key] = float(raw)
        except ValueError:
            params[key] = raw
    return params


def tir_float(params: dict[str, float | str], key: str, default: float = math.nan) -> float:
    return finite_float(params.get(key.upper(), default))


def current_reference_radius_m() -> float:
    radius = tir_float(parse_tir(CURRENT_TIRE), "UNLOADED_RADIUS")
    if not math.isfinite(radius):
        raise ValueError(f"Reference tire is missing UNLOADED_RADIUS: {CURRENT_TIRE}")
    return radius


def tire_architecture_fields(
    tire_params: dict[str, float | str],
    reference_radius_m: float,
    baseline_cg_height_m: float,
) -> dict[str, float]:
    radius = tir_float(tire_params, "UNLOADED_RADIUS")
    rim_radius = tir_float(tire_params, "RIM_RADIUS")
    rim_width = tir_float(tire_params, "RIM_WIDTH")
    width = tir_float(tire_params, "WIDTH")
    radius_delta = radius - reference_radius_m if math.isfinite(radius) else 0.0
    return {
        "reference_unloaded_radius_m": reference_radius_m,
        "architecture_unloaded_radius_m": radius,
        "architecture_radius_delta_m": radius_delta,
        "architecture_radius_delta_mm": 1000.0 * radius_delta,
        "architecture_cg_height_m": baseline_cg_height_m + radius_delta,
        "architecture_cg_height_delta_pct": (
            100.0 * radius_delta / baseline_cg_height_m
            if baseline_cg_height_m > 0.0
            else math.nan
        ),
        "architecture_tire_width_m": width,
        "architecture_rim_radius_m": rim_radius,
        "architecture_rim_width_m": rim_width,
    }


def is_numeric_triplet(value: Any) -> bool:
    if not isinstance(value, list) or len(value) != 3:
        return False
    return all(isinstance(item, (int, float)) for item in value)


def shifted_coordinate_z(value: list[Any], delta_z_m: float) -> list[Any]:
    return [value[0], value[1], float(value[2]) + delta_z_m]


def should_shift_coordinate_triplet(key: str, parent_key: str, value: Any) -> bool:
    """Return true for chassis-layout coordinate triplets, not vectors/tables."""
    if not is_numeric_triplet(value):
        return False
    return key.endswith("_m") or parent_key.endswith("_m")


def shift_vehicle_coordinate_z_fields(
    value: Any,
    delta_z_m: float,
    key: str = "",
    parent_key: str = "",
) -> Any:
    """Translate chassis-layout coordinates upward for tire-OD variants."""
    if abs(delta_z_m) <= 1e-12:
        return value
    if isinstance(value, dict):
        return {
            child_key: shift_vehicle_coordinate_z_fields(
                child_value,
                delta_z_m,
                child_key,
                key,
            )
            for child_key, child_value in value.items()
        }
    if should_shift_coordinate_triplet(key, parent_key, value):
        return shifted_coordinate_z(value, delta_z_m)
    if isinstance(value, list):
        return [
            shift_vehicle_coordinate_z_fields(item, delta_z_m, key, parent_key)
            for item in value
        ]
    return value


def load_manifest() -> dict[str, dict[str, str]]:
    return {
        row["generated_tir"]: row
        for row in read_csv_rows(ROUND9_MANIFEST)
        if row.get("generated_tir")
    }


def load_diagnostics() -> dict[str, dict[str, str]]:
    return {
        row["generated_tir"]: row
        for row in read_csv_rows(ROUND9_DIAGNOSTICS)
        if row.get("generated_tir")
    }


def normalized_size(size: str) -> str:
    return str(size).replace(" R20", "").strip()


def setup_key(brand: str, model: str, size: str, rim_width_in: float) -> tuple[str, str, str, float]:
    return (
        str(brand).strip(),
        str(model).strip(),
        normalized_size(size),
        round(float(rim_width_in), 6),
    )


def discover_candidates() -> list[TireCandidate]:
    manifest = load_manifest()
    diagnostics = load_diagnostics()
    candidates = [
        TireCandidate(
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

    for path in sorted(ROUND9_TIRE_DIR.glob("*.tir")):
        row = manifest.get(path.name, {})
        diag = diagnostics.get(path.name, {})
        brand = row.get("brand", "").strip() or "Round9"
        model = row.get("model", "").strip() or path.stem
        size = row.get("size", "").replace(" R20", "").strip() or "unknown"
        rim_width = finite_float(row.get("rim_width_in"))
        pressure = finite_float(row.get("pressure_psi"))
        if math.isfinite(rim_width) and math.isfinite(pressure):
            label = f"{brand} {model} {size} {rim_width:g}in {pressure:g} psi"
        else:
            label = path.stem.replace("_", " ")
        long_source = row.get("longitudinal_combined_source", "").strip()
        lat_relax_source = row.get("lateral_relaxation_source", "").strip()
        notes = (
            "Round 9 fitted full UM14 tire. "
            f"Longitudinal/combined source: {long_source or 'unknown'}. "
            f"Lateral relaxation source: {lat_relax_source or 'unknown'}."
        )
        candidates.append(
            TireCandidate(
                candidate_id=slugify(path.stem),
                label=label,
                path=path,
                source="round9_fitted_full_um14",
                brand=brand,
                model=model,
                tire_size=size,
                rim_width_in=rim_width,
                pressure_psi=pressure,
                longitudinal_combined_source=long_source,
                lateral_relaxation_source=lat_relax_source,
                is_reference=False,
                notes=notes,
                manifest=row,
                diagnostics=diag,
            )
        )
    return candidates


def load_round9_fit_module() -> Any:
    return load_module(
        "round9_fit_helpers_for_ds006",
        REPO_ROOT / "tire_fits" / "fit_round9_tires.py",
    )


def load_single_mat_run(archive_path: Path, run: int) -> dict[str, Any]:
    try:
        import scipy.io
    except ModuleNotFoundError as exc:
        raise RuntimeError("scipy is required to extract Round 9 MAT data") from exc

    member = f"B2356run{run}.mat"
    with ZipFile(archive_path) as archive:
        if member not in archive.namelist():
            return {}
        mat = scipy.io.loadmat(
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
        if arr.ndim == 1 and arr.size > 10:
            output[key] = arr.astype(float, copy=False)
    return output


def load_multiple_mat_runs(
    archive_path: Path,
    runs: tuple[int, ...],
    cache: dict[tuple[str, int], dict[str, Any]] | None = None,
) -> dict[str, Any]:
    if not runs:
        return {}
    cache = cache if cache is not None else {}
    parts = []
    for run in runs:
        key = (str(archive_path), int(run))
        if key not in cache:
            cache[key] = load_single_mat_run(archive_path, int(run))
        if cache[key]:
            parts.append(cache[key])
    if not parts:
        return {}
    output: dict[str, Any] = {}
    for key in sorted(set().union(*(part.keys() for part in parts))):
        arrays = [
            np.asarray(part[key], dtype=float).ravel()
            for part in parts
            if key in part and not isinstance(part[key], str)
        ]
        if arrays:
            output[key] = np.concatenate(arrays)
    return output


def pressure_window_mask(data: dict[str, Any], pressure_psi: float) -> np.ndarray:
    pressure = np.asarray(data["P"], dtype=float)
    return np.abs(pressure - pressure_psi * 6.894757293168361) <= 2.35


def finite_array_mask(*arrays: np.ndarray) -> np.ndarray:
    if not arrays:
        return np.array([], dtype=bool)
    mask = np.ones_like(np.asarray(arrays[0], dtype=float), dtype=bool)
    for array in arrays:
        mask &= np.isfinite(np.asarray(array, dtype=float))
    return mask


def nominal_speed_mask(data: dict[str, Any]) -> np.ndarray:
    speed = np.asarray(data["V"], dtype=float)
    return (speed >= NOMINAL_TEST_SPEED_KPH_MIN) & (speed <= NOMINAL_TEST_SPEED_KPH_MAX)


def safe_percent_delta(final: Any, initial: Any) -> float:
    final_f = finite_float(final)
    initial_f = finite_float(initial)
    if not math.isfinite(final_f) or not math.isfinite(initial_f) or abs(initial_f) <= 1e-12:
        return math.nan
    return 100.0 * (final_f - initial_f) / abs(initial_f)


def robust_abs_peak_mu(force: np.ndarray, normal_load: np.ndarray, mask: np.ndarray) -> float:
    idx = np.flatnonzero(mask)
    if idx.size < 120:
        return math.nan
    mu = np.abs(force[idx] / normal_load[idx])
    mu = mu[np.isfinite(mu)]
    return float(np.nanpercentile(mu, 95.0)) if mu.size else math.nan


def linear_abs_slope(x: np.ndarray, y: np.ndarray, mask: np.ndarray) -> float:
    idx = np.flatnonzero(mask)
    if idx.size < 60:
        return math.nan
    x_i = np.asarray(x[idx], dtype=float)
    y_i = np.asarray(y[idx], dtype=float)
    valid = np.isfinite(x_i) & np.isfinite(y_i)
    if np.count_nonzero(valid) < 60:
        return math.nan
    try:
        slope = np.polyfit(x_i[valid], y_i[valid], 1)[0]
    except (np.linalg.LinAlgError, ValueError):
        return math.nan
    return abs(float(slope))


def tread_mean_c(data: dict[str, Any], mask: np.ndarray) -> float:
    channels = [
        np.asarray(data[key], dtype=float)
        for key in ("TSTI", "TSTC", "TSTO")
        if key in data
    ]
    if len(channels) != 3 or np.count_nonzero(mask) == 0:
        return math.nan
    tread = np.nanmean(np.column_stack(channels), axis=1)
    return float(np.nanmean(tread[mask])) if np.any(np.isfinite(tread[mask])) else math.nan


def lateral_degradation_run_metrics(data: dict[str, Any], pressure_psi: float) -> dict[str, Any]:
    required = ("P", "FY", "FZ", "SA", "V")
    if not all(key in data for key in required):
        return {"status": "missing_channels", "samples": 0}
    fz = -np.asarray(data["FZ"], dtype=float)
    fy = np.asarray(data["FY"], dtype=float)
    sa = np.deg2rad(np.asarray(data["SA"], dtype=float))
    base_mask = (
        pressure_window_mask(data, pressure_psi)
        & nominal_speed_mask(data)
        & finite_array_mask(fz, fy, sa)
        & (fz > 120.0)
        & (fz < 2200.0)
        & (np.abs(sa) <= np.deg2rad(15.0))
    )
    samples = int(np.count_nonzero(base_mask))
    if samples < 600:
        return {"status": "insufficient_data", "samples": samples}
    fy_norm = fy / np.maximum(fz, 1e-9)
    linear_mask = base_mask & (np.abs(sa) <= np.deg2rad(3.0)) & (np.abs(fy_norm) <= 2.5)
    return {
        "status": "ok",
        "samples": samples,
        "peak_mu_y": robust_abs_peak_mu(fy, fz, base_mask),
        "ky_norm_per_rad": linear_abs_slope(sa, fy_norm, linear_mask),
        "tread_mean_c": tread_mean_c(data, base_mask),
    }


def longitudinal_degradation_run_metrics(data: dict[str, Any], pressure_psi: float) -> dict[str, Any]:
    required = ("P", "FX", "FZ", "SR", "SA", "V")
    if not all(key in data for key in required):
        return {"status": "missing_channels", "samples": 0}
    fz = -np.asarray(data["FZ"], dtype=float)
    fx = np.asarray(data["FX"], dtype=float)
    sr = np.asarray(data["SR"], dtype=float)
    sa_deg = np.asarray(data["SA"], dtype=float)
    base_mask = (
        pressure_window_mask(data, pressure_psi)
        & nominal_speed_mask(data)
        & finite_array_mask(fz, fx, sr, sa_deg)
        & (fz > 120.0)
        & (fz < 2400.0)
        & (np.abs(sa_deg) < 0.75)
        & (np.abs(sr) <= 0.35)
    )
    samples = int(np.count_nonzero(base_mask))
    if samples < 600:
        return {"status": "insufficient_data", "samples": samples}
    fx_norm = fx / np.maximum(fz, 1e-9)
    linear_mask = base_mask & (np.abs(sr) <= 0.06) & (np.abs(fx_norm) <= 2.5)
    return {
        "status": "ok",
        "samples": samples,
        "peak_mu_x": robust_abs_peak_mu(fx, fz, base_mask),
        "kx_norm": linear_abs_slope(sr, fx_norm, linear_mask),
        "tread_mean_c": tread_mean_c(data, base_mask),
    }


def degradation_pair_row(
    *,
    prefix: str,
    initial_runs: tuple[int, ...],
    final_runs: tuple[int, ...],
    initial: dict[str, Any],
    final: dict[str, Any],
) -> dict[str, Any]:
    row: dict[str, Any] = {
        f"degradation_{prefix}_status": "ok" if initial.get("status") == "ok" and final.get("status") == "ok" else "insufficient_data",
        f"degradation_{prefix}_initial_runs": ",".join(str(run) for run in initial_runs),
        f"degradation_{prefix}_final_runs": ",".join(str(run) for run in final_runs),
        f"degradation_{prefix}_samples_initial": initial.get("samples", 0),
        f"degradation_{prefix}_samples_final": final.get("samples", 0),
    }
    if prefix == "corner":
        row.update(
            {
                "degradation_corner_initial_peak_mu_y": initial.get("peak_mu_y", math.nan),
                "degradation_corner_final_peak_mu_y": final.get("peak_mu_y", math.nan),
                "degradation_corner_peak_mu_y_delta_pct": safe_percent_delta(final.get("peak_mu_y"), initial.get("peak_mu_y")),
                "degradation_corner_initial_ky_norm_per_rad": initial.get("ky_norm_per_rad", math.nan),
                "degradation_corner_final_ky_norm_per_rad": final.get("ky_norm_per_rad", math.nan),
                "degradation_corner_ky_delta_pct": safe_percent_delta(final.get("ky_norm_per_rad"), initial.get("ky_norm_per_rad")),
                "degradation_corner_initial_tread_mean_c": initial.get("tread_mean_c", math.nan),
                "degradation_corner_final_tread_mean_c": final.get("tread_mean_c", math.nan),
                "degradation_corner_tread_delta_c": finite_float(final.get("tread_mean_c")) - finite_float(initial.get("tread_mean_c")),
            }
        )
    else:
        row.update(
            {
                "degradation_drive_initial_peak_mu_x": initial.get("peak_mu_x", math.nan),
                "degradation_drive_final_peak_mu_x": final.get("peak_mu_x", math.nan),
                "degradation_drive_peak_mu_x_delta_pct": safe_percent_delta(final.get("peak_mu_x"), initial.get("peak_mu_x")),
                "degradation_drive_initial_kx_norm": initial.get("kx_norm", math.nan),
                "degradation_drive_final_kx_norm": final.get("kx_norm", math.nan),
                "degradation_drive_kx_delta_pct": safe_percent_delta(final.get("kx_norm"), initial.get("kx_norm")),
                "degradation_drive_initial_tread_mean_c": initial.get("tread_mean_c", math.nan),
                "degradation_drive_final_tread_mean_c": final.get("tread_mean_c", math.nan),
                "degradation_drive_tread_delta_c": finite_float(final.get("tread_mean_c")) - finite_float(initial.get("tread_mean_c")),
            }
        )
    return row


def edge_mean(values: np.ndarray, order: np.ndarray, *, first: bool) -> float:
    if order.size == 0:
        return math.nan
    count = max(8, int(math.ceil(0.05 * order.size)))
    idx = order[:count] if first else order[-count:]
    return float(np.nanmean(values[idx])) if idx.size else math.nan


def extract_transient_temperature_rows(candidates: list[TireCandidate]) -> list[dict[str, Any]]:
    if not ROUND9_CORNERING_ARCHIVE.exists():
        return []

    try:
        fit_module = load_round9_fit_module()
    except Exception as exc:  # noqa: BLE001
        return [
            {
                "candidate_id": "",
                "label": "",
                "temperature_status": "error",
                "error": str(exc),
            }
        ]

    candidates_by_setup_pressure: dict[tuple[str, str, str, float, float], TireCandidate] = {}
    for candidate in candidates:
        if candidate.is_reference:
            continue
        if not math.isfinite(candidate.rim_width_in) or not math.isfinite(candidate.pressure_psi):
            continue
        key = (*setup_key(candidate.brand, candidate.model, candidate.tire_size, candidate.rim_width_in), round(candidate.pressure_psi, 6))
        candidates_by_setup_pressure[key] = candidate

    rows: list[dict[str, Any]] = []
    for tire in fit_module.round9_specs():
        if tire.transient_run is None:
            continue
        setup = setup_key(tire.brand, tire.model, tire.size, tire.rim_width_in)
        data = load_single_mat_run(ROUND9_CORNERING_ARCHIVE, tire.transient_run)
        required = ("P", "ET", "TSTI", "TSTC", "TSTO", "RST", "AMBTMP")
        if not all(key in data for key in required):
            continue
        et = np.asarray(data["ET"], dtype=float)
        tsti = np.asarray(data["TSTI"], dtype=float)
        tstc = np.asarray(data["TSTC"], dtype=float)
        tsto = np.asarray(data["TSTO"], dtype=float)
        tread = np.nanmean(np.column_stack([tsti, tstc, tsto]), axis=1)
        spread = np.nanmax(np.column_stack([tsti, tstc, tsto]), axis=1) - np.nanmin(
            np.column_stack([tsti, tstc, tsto]),
            axis=1,
        )
        finite_base = np.isfinite(et) & np.isfinite(tread) & np.isfinite(tsti) & np.isfinite(tstc) & np.isfinite(tsto)
        for pressure_psi in (8.0, 10.0, 12.0, 14.0):
            candidate = candidates_by_setup_pressure.get((*setup, round(pressure_psi, 6)))
            if candidate is None:
                continue
            mask = pressure_window_mask(data, pressure_psi) & finite_base
            if np.count_nonzero(mask) < 120:
                continue
            idx = np.flatnonzero(mask)
            order = idx[np.argsort(et[idx])]
            tread_start = edge_mean(tread, order, first=True)
            tread_end = edge_mean(tread, order, first=False)
            row = {
                "candidate_id": candidate.candidate_id,
                "label": candidate.label,
                "temperature_status": "ok",
                "transient_run": tire.transient_run,
                "transient_pressure_psi": pressure_psi,
                "transient_temperature_samples": int(idx.size),
                "transient_elapsed_time_min_s": float(np.nanmin(et[idx])),
                "transient_elapsed_time_max_s": float(np.nanmax(et[idx])),
                "transient_tread_mean_c": float(np.nanmean(tread[idx])),
                "transient_tread_peak_c": float(np.nanmax(tread[idx])),
                "transient_tread_start_c": tread_start,
                "transient_tread_end_c": tread_end,
                "transient_tread_rise_c": tread_end - tread_start if math.isfinite(tread_start) and math.isfinite(tread_end) else math.nan,
                "transient_tread_inner_mean_c": float(np.nanmean(tsti[idx])),
                "transient_tread_center_mean_c": float(np.nanmean(tstc[idx])),
                "transient_tread_outer_mean_c": float(np.nanmean(tsto[idx])),
                "transient_tread_inner_minus_outer_c": float(np.nanmean(tsti[idx] - tsto[idx])),
                "transient_surface_spread_mean_c": float(np.nanmean(spread[idx])),
                "transient_rim_temp_mean_c": float(np.nanmean(np.asarray(data["RST"], dtype=float)[idx])),
                "transient_ambient_temp_mean_c": float(np.nanmean(np.asarray(data["AMBTMP"], dtype=float)[idx])),
            }
            rows.append(row)
    return rows


def extract_tire_degradation_rows(candidates: list[TireCandidate]) -> list[dict[str, Any]]:
    if not ROUND9_CORNERING_ARCHIVE.exists():
        return []

    try:
        fit_module = load_round9_fit_module()
    except Exception as exc:  # noqa: BLE001
        return [
            {
                "candidate_id": "",
                "label": "",
                "degradation_status": "error",
                "error": str(exc),
            }
        ]

    candidates_by_setup: dict[tuple[str, str, str, float], list[TireCandidate]] = defaultdict(list)
    for candidate in candidates:
        if candidate.is_reference:
            continue
        if not math.isfinite(candidate.rim_width_in):
            continue
        candidates_by_setup[
            setup_key(candidate.brand, candidate.model, candidate.tire_size, candidate.rim_width_in)
        ].append(candidate)

    rows: list[dict[str, Any]] = []
    mat_cache: dict[tuple[str, int], dict[str, Any]] = {}
    for tire in fit_module.round9_specs():
        setup = setup_key(tire.brand, tire.model, tire.size, tire.rim_width_in)
        setup_candidates = sorted(
            candidates_by_setup.get(setup, []),
            key=lambda candidate: finite_float(candidate.pressure_psi),
        )
        if not setup_candidates or len(tire.corner_runs) < 2:
            continue

        corner_initial_runs = tuple(int(run) for run in tire.corner_runs[:-1])
        corner_final_runs = (int(tire.corner_runs[-1]),)
        corner_initial = lateral_degradation_run_metrics(
            load_multiple_mat_runs(ROUND9_CORNERING_ARCHIVE, corner_initial_runs, mat_cache),
            DEGRADATION_PRESSURE_PSI,
        )
        corner_final = lateral_degradation_run_metrics(
            load_multiple_mat_runs(ROUND9_CORNERING_ARCHIVE, corner_final_runs, mat_cache),
            DEGRADATION_PRESSURE_PSI,
        )
        base_row = {
            "degradation_status": "ok" if corner_initial.get("status") == "ok" and corner_final.get("status") == "ok" else "partial",
            "degradation_pressure_psi": DEGRADATION_PRESSURE_PSI,
            "degradation_note": (
                "Initial/final 12 psi nominal-speed repeats from RunGuide_Round9.pdf; "
                "reported as raw tire-data degradation/warmup evidence, not a vehicle-sim score."
            ),
        }
        base_row.update(
            degradation_pair_row(
                prefix="corner",
                initial_runs=corner_initial_runs,
                final_runs=corner_final_runs,
                initial=corner_initial,
                final=corner_final,
            )
        )

        if tire.drive_runs:
            drive_initial_runs = tuple(int(run) for run in tire.drive_runs[:-1])
            drive_final_runs = (int(tire.drive_runs[-1]),)
            drive_initial = longitudinal_degradation_run_metrics(
                load_multiple_mat_runs(ROUND9_DRIVE_ARCHIVE, drive_initial_runs, mat_cache),
                DEGRADATION_PRESSURE_PSI,
            )
            drive_final = longitudinal_degradation_run_metrics(
                load_multiple_mat_runs(ROUND9_DRIVE_ARCHIVE, drive_final_runs, mat_cache),
                DEGRADATION_PRESSURE_PSI,
            )
            base_row.update(
                degradation_pair_row(
                    prefix="drive",
                    initial_runs=drive_initial_runs,
                    final_runs=drive_final_runs,
                    initial=drive_initial,
                    final=drive_final,
                )
            )
        else:
            base_row.update(
                {
                    "degradation_drive_status": "not_available_for_16in_scaled_longitudinal",
                    "degradation_drive_initial_runs": "",
                    "degradation_drive_final_runs": "",
                    "degradation_drive_samples_initial": 0,
                    "degradation_drive_samples_final": 0,
                    "degradation_drive_initial_peak_mu_x": math.nan,
                    "degradation_drive_final_peak_mu_x": math.nan,
                    "degradation_drive_peak_mu_x_delta_pct": math.nan,
                    "degradation_drive_initial_kx_norm": math.nan,
                    "degradation_drive_final_kx_norm": math.nan,
                    "degradation_drive_kx_delta_pct": math.nan,
                    "degradation_drive_initial_tread_mean_c": math.nan,
                    "degradation_drive_final_tread_mean_c": math.nan,
                    "degradation_drive_tread_delta_c": math.nan,
                }
            )

        for candidate in setup_candidates:
            row = dict(base_row)
            row["candidate_id"] = candidate.candidate_id
            row["label"] = candidate.label
            row["family"] = candidate.family
            row["tire_size"] = candidate.tire_size
            row["rim_width_in"] = candidate.rim_width_in
            row["pressure_psi"] = candidate.pressure_psi
            rows.append(row)
    return rows


def add_mass_component(
    rows: list[dict[str, Any]],
    name: str,
    mass_kg: float,
    cg_m: list[float],
    count: int = 1,
) -> None:
    rows.append(
        {
            "component": name,
            "count": count,
            "unit_mass_kg": mass_kg,
            "total_mass_kg": count * mass_kg,
            "cg_x_m": float(cg_m[0]),
            "cg_y_m": float(cg_m[1]),
            "cg_z_m": float(cg_m[2]),
        }
    )


def mass_rollup(vehicle_doc: dict[str, Any]) -> tuple[float, Any, list[dict[str, Any]]]:
    rows: list[dict[str, Any]] = []
    sprung = vehicle_doc["sprung_mass"]
    add_mass_component(rows, "sprung_mass", sprung["mass_kg"], sprung["cg_m"])
    driver = vehicle_doc["driver_mass"]
    add_mass_component(rows, "driver_mass", driver["mass_kg"], driver["cg_m"])
    for axle_name in ("front", "rear"):
        for component_name, component in vehicle_doc[axle_name]["masses"].items():
            add_mass_component(
                rows,
                f"{axle_name}.{component_name}",
                component["mass_kg"],
                component["cg_m"],
                count=2,
            )
    total_mass = sum(float(row["total_mass_kg"]) for row in rows)
    weighted_cg = np.zeros(3, dtype=float)
    for row in rows:
        weighted_cg += float(row["total_mass_kg"]) * np.array(
            [row["cg_x_m"], row["cg_y_m"], row["cg_z_m"]],
            dtype=float,
        )
    return total_mass, weighted_cg / total_mass, rows


def baseline_aero_from_vehicle(vehicle_doc: dict[str, Any]) -> tuple[float, float]:
    aero = vehicle_doc["aero"]
    reference_speed = float(aero["reference_speed_m_per_s"])
    drag_n = float(aero["drag_table_n"][0][0])
    downforce_n = float(aero["downforce_table_n"][0][0])
    cl_a, cd_a = ggv.force_to_aero_area(
        downforce_n=downforce_n,
        drag_n=drag_n,
        speed_mps=reference_speed,
    )
    return float(cl_a), float(cd_a)


def build_baseline_vehicle(vehicle_path: Path, tire_path: Path) -> tuple[Any, dict[str, Any]]:
    vehicle_doc = read_yaml(vehicle_path)
    total_mass, cg, mass_rows = mass_rollup(vehicle_doc)
    front_wc = np.array(vehicle_doc["front"]["suspension"]["wheel_center_m"], dtype=float)
    rear_wc = np.array(vehicle_doc["rear"]["suspension"]["wheel_center_m"], dtype=float)
    wheelbase = abs(float(front_wc[0] - rear_wc[0]))
    track_front = 2.0 * abs(float(front_wc[1]))
    track_rear = 2.0 * abs(float(rear_wc[1]))
    front_static_frac = float((cg[0] - rear_wc[0]) / (front_wc[0] - rear_wc[0]))
    front_static_frac = min(0.70, max(0.30, front_static_frac))
    cl_a, cd_a = baseline_aero_from_vehicle(vehicle_doc)
    tire = parse_tir(tire_path)

    assumptions = {
        "lltd": 0.52,
        "aero_balance_front": 0.50,
        "max_drive_power": 80_000.0,
        "max_drive_force": 3_735.0,
        "max_brake_force": 14_000.0,
        "drive_distribution_front": 0.0,
        "brake_distribution_front": 0.62,
        "mu_min": 0.8,
    }

    vehicle = ggv.VehicleParams(
        mass=total_mass,
        wheelbase=wheelbase,
        track_front=track_front,
        track_rear=track_rear,
        cg_height=float(cg[2]),
        front_static_frac=front_static_frac,
        lltd=assumptions["lltd"],
        rho=1.225,
        cl_a=cl_a,
        cd_a=cd_a,
        aero_balance_front=assumptions["aero_balance_front"],
        max_drive_power=assumptions["max_drive_power"],
        max_drive_force=assumptions["max_drive_force"],
        max_brake_force=assumptions["max_brake_force"],
        drive_distribution_front=assumptions["drive_distribution_front"],
        brake_distribution_front=assumptions["brake_distribution_front"],
        fz_ref=tir_float(tire, "FNOMIN"),
        fz_min_valid=tir_float(tire, "FZMIN"),
        fz_max_valid=tir_float(tire, "FZMAX"),
        pdx1=tir_float(tire, "PDX1"),
        pdx2=tir_float(tire, "PDX2"),
        pdy1=tir_float(tire, "PDY1"),
        pdy2=tir_float(tire, "PDY2"),
        mu_min=assumptions["mu_min"],
    )
    context = {
        "vehicle_doc": vehicle_doc,
        "mass_components": mass_rows,
        "mass_kg": total_mass,
        "cg_x_m": float(cg[0]),
        "cg_y_m": float(cg[1]),
        "cg_z_m": float(cg[2]),
        "front_static_frac": front_static_frac,
        "assumptions": assumptions,
    }
    return vehicle, context


def vehicle_with_tire(base_vehicle: Any, tire_path: Path, reference_radius_m: float) -> Any:
    tire = parse_tir(tire_path)
    architecture = tire_architecture_fields(
        tire,
        reference_radius_m,
        float(base_vehicle.cg_height),
    )
    params = dataclasses.asdict(base_vehicle)
    params.update(
        {
            "cg_height": architecture["architecture_cg_height_m"],
            "fz_ref": tir_float(tire, "FNOMIN"),
            "fz_min_valid": tir_float(tire, "FZMIN"),
            "fz_max_valid": tir_float(tire, "FZMAX"),
            "pdx1": tir_float(tire, "PDX1"),
            "pdx2": tir_float(tire, "PDX2"),
            "pdy1": tir_float(tire, "PDY1"),
            "pdy2": tir_float(tire, "PDY2"),
        }
    )
    return ggv.VehicleParams(**params)


def make_config() -> Any:
    return ggv.GGVConfig(
        speeds=(5.0, 10.0, 15.0, 20.0, 25.0),
        ay_max_g=3.8,
        ay_points=81,
        ax_search_min_g=-3.6,
        ax_search_max_g=3.0,
        ax_search_points=201,
        include_left_right=True,
        verbose=False,
        progress_every=25,
        warn_tire_load_range=False,
    )


def speed_label(speed: float) -> str:
    if float(speed).is_integer():
        return f"{int(speed):02d}mps"
    return f"{speed:.1f}mps".replace(".", "p")


def extract_envelope_metrics(envelopes: list[Any]) -> dict[str, float]:
    metrics: dict[str, float] = {}
    series: dict[str, list[float]] = {
        "max_lateral_g": [],
        "max_accel_g": [],
        "max_brake_g": [],
        "ggv_area_g2": [],
    }
    for env in envelopes:
        ay_g = np.asarray(env.ay, dtype=float) / G
        ax_accel_g = np.asarray(env.ax_accel, dtype=float) / G
        ax_brake_g = np.asarray(env.ax_brake, dtype=float) / G
        finite_accel = np.isfinite(ax_accel_g)
        finite_brake = np.isfinite(ax_brake_g)
        finite_any = finite_accel | finite_brake
        finite_both = finite_accel & finite_brake
        max_lateral = float(np.nanmax(np.abs(ay_g[finite_any]))) if np.any(finite_any) else math.nan
        max_accel = float(np.nanmax(ax_accel_g[finite_accel])) if np.any(finite_accel) else math.nan
        max_brake = float(abs(np.nanmin(ax_brake_g[finite_brake]))) if np.any(finite_brake) else math.nan
        if np.count_nonzero(finite_both) >= 2:
            ay_valid = ay_g[finite_both]
            width = ax_accel_g[finite_both] - ax_brake_g[finite_both]
            sort_idx = np.argsort(ay_valid)
            area = float(np.trapezoid(width[sort_idx], ay_valid[sort_idx]))
        else:
            area = math.nan
        suffix = speed_label(float(env.speed))
        per_speed = {
            "max_lateral_g": max_lateral,
            "max_accel_g": max_accel,
            "max_brake_g": max_brake,
            "ggv_area_g2": area,
        }
        for key, value in per_speed.items():
            metrics[f"{key}__{suffix}"] = value
            series[key].append(value)
    for key, values in series.items():
        arr = np.asarray(values, dtype=float)
        metrics[f"mean_{key}"] = float(np.nanmean(arr)) if np.any(np.isfinite(arr)) else math.nan
    return metrics


def tire_load_range(vehicle: Any, envelopes: list[Any]) -> dict[str, float | int]:
    fz_min_seen = math.inf
    fz_max_seen = -math.inf
    for env in envelopes:
        for ax_values in (env.ax_accel, env.ax_brake):
            finite = np.isfinite(ax_values)
            for ay, ax in zip(np.asarray(env.ay)[finite], np.asarray(ax_values)[finite]):
                fz = ggv.wheel_loads(vehicle, speed=float(env.speed), ax=float(ax), ay=float(ay))
                fz_min_seen = min(fz_min_seen, float(np.min(fz)))
                fz_max_seen = max(fz_max_seen, float(np.max(fz)))
    if not math.isfinite(fz_min_seen) or not math.isfinite(fz_max_seen):
        return {
            "fz_min_seen_n": math.nan,
            "fz_max_seen_n": math.nan,
            "fz_range_exceeded": 1,
            "fz_max_excess_pct": math.nan,
        }
    exceeded = fz_min_seen < vehicle.fz_min_valid or fz_max_seen > vehicle.fz_max_valid
    max_excess_pct = (
        100.0 * (fz_max_seen - vehicle.fz_max_valid) / vehicle.fz_max_valid
        if vehicle.fz_max_valid > 0.0
        else math.nan
    )
    return {
        "fz_min_seen_n": fz_min_seen,
        "fz_max_seen_n": fz_max_seen,
        "fz_range_exceeded": int(exceeded),
        "fz_max_excess_pct": max(0.0, max_excess_pct),
    }


def cornering_stiffness_norm(params: dict[str, float | str]) -> float:
    fnom = tir_float(params, "FNOMIN")
    pky1 = tir_float(params, "PKY1")
    pky2 = tir_float(params, "PKY2")
    if not math.isfinite(fnom) or not math.isfinite(pky1) or not math.isfinite(pky2) or fnom <= 0.0 or pky2 == 0.0:
        return math.nan
    return abs(pky1 * fnom * math.sin(2.0 * math.atan(1.0 / pky2))) / fnom


def characterize_tire(candidate: TireCandidate) -> dict[str, Any]:
    params = parse_tir(candidate.path)
    diag = candidate.diagnostics
    manifest = candidate.manifest
    return {
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
        "path": as_repo_path(candidate.path),
        "use_mode": tir_float(params, "USE_MODE"),
        "fnomin_n": tir_float(params, "FNOMIN"),
        "fzmin_n": tir_float(params, "FZMIN"),
        "fzmax_n": tir_float(params, "FZMAX"),
        "unloaded_radius_m": tir_float(params, "UNLOADED_RADIUS"),
        "width_m": tir_float(params, "WIDTH"),
        "rim_width_m": tir_float(params, "RIM_WIDTH"),
        "vertical_stiffness_n_per_m": tir_float(params, "VERTICAL_STIFFNESS"),
        "vertical_damping_n_s_per_m": tir_float(params, "VERTICAL_DAMPING"),
        "mu_y": finite_float(diag.get("mu_y", abs(tir_float(params, "PDY1")))),
        "mu_x": finite_float(diag.get("mu_x", abs(tir_float(params, "PDX1")))),
        "mu_x_to_y": finite_float(diag.get("mu_x_to_y")),
        "ky_norm_per_rad": finite_float(diag.get("ky_norm_per_rad", cornering_stiffness_norm(params))),
        "kx_norm": finite_float(diag.get("kx_norm", tir_float(params, "PKX1"))),
        "sigma_alpha_m": finite_float(diag.get("sigma_alpha_m", tir_float(params, "PTY1"))),
        "sigma_kappa_m": finite_float(diag.get("sigma_kappa_m", tir_float(params, "PTX1"))),
        "lateral_nrmse": finite_float(manifest.get("lateral_nrmse", diag.get("lateral_nrmse"))),
        "longitudinal_nrmse": finite_float(manifest.get("longitudinal_nrmse", diag.get("longitudinal_nrmse"))),
        "fx_combined_nrmse": finite_float(manifest.get("fx_combined_nrmse", diag.get("fx_combined_nrmse"))),
        "fy_combined_nrmse": finite_float(manifest.get("fy_combined_nrmse", diag.get("fy_combined_nrmse"))),
        "lateral_relaxation_sigma_rmse_m": finite_float(manifest.get("lateral_relaxation_sigma_rmse_m")),
        "longitudinal_combined_source": candidate.longitudinal_combined_source,
        "lateral_relaxation_source": candidate.lateral_relaxation_source,
        "notes": candidate.notes,
    }


def run_envelopes(
    base_vehicle: Any,
    config: Any,
    candidates: list[TireCandidate],
    reference_radius_m: float,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    envelope_rows = []
    characterization_rows = []
    for idx, candidate in enumerate(candidates, start=1):
        print(f"  EnvelopeSim {idx:02d}/{len(candidates)}: {candidate.label}", flush=True)
        tire_vehicle = vehicle_with_tire(base_vehicle, candidate.path, reference_radius_m)
        envelopes = ggv.generate_ggv(tire_vehicle, config)
        metrics = extract_envelope_metrics(envelopes)
        load_range = tire_load_range(tire_vehicle, envelopes)
        char_row = characterize_tire(candidate)
        characterization_rows.append(char_row)
        architecture = tire_architecture_fields(
            parse_tir(candidate.path),
            reference_radius_m,
            float(base_vehicle.cg_height),
        )
        row = {
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
            "path": as_repo_path(candidate.path),
        }
        row.update(architecture)
        row.update(metrics)
        row.update(load_range)
        envelope_rows.append(row)
    return envelope_rows, characterization_rows


def normalized_metric_scores(
    rows: list[dict[str, Any]],
    metric: str,
    *,
    higher_is_better: bool = True,
    source_rows: list[dict[str, Any]] | None = None,
) -> dict[str, float]:
    source = source_rows if source_rows is not None else rows
    values = [finite_float(row.get(metric)) for row in source]
    values = [value for value in values if math.isfinite(value)]
    if not values:
        return {row["candidate_id"]: math.nan for row in rows}
    lo = min(values)
    hi = max(values)
    scores: dict[str, float] = {}
    for row in rows:
        value = finite_float(row.get(metric))
        if not math.isfinite(value):
            score = math.nan
        elif abs(hi - lo) <= 1e-12:
            score = 1.0
        elif higher_is_better:
            score = (value - lo) / (hi - lo)
        else:
            score = (hi - value) / (hi - lo)
        scores[row["candidate_id"]] = score
    return scores


def target_score(value: Any, target: float, span: float) -> float:
    value_f = finite_float(value)
    if not math.isfinite(value_f) or span <= 0.0:
        return math.nan
    return max(0.0, 1.0 - abs(value_f - target) / span)


def response_flags(row: dict[str, Any]) -> str:
    flags: list[str] = []
    understeer = finite_float(row.get("understeer_gradient_deg_per_g"))
    sideslip = finite_float(row.get("sideslip_gradient_deg_per_g"))
    roll = finite_float(row.get("roll_gradient_deg_per_g"))
    if math.isfinite(understeer) and abs(understeer) > 5.0:
        flags.append("understeer_outlier")
    if math.isfinite(sideslip) and abs(sideslip) > 5.0:
        flags.append("sideslip_outlier")
    if math.isfinite(roll) and (roll <= 0.0 or abs(roll) > 5.0):
        flags.append("roll_outlier")
    return ";".join(flags) if flags else "ok"


def metrics_csv_has_standard_qa(path: Path) -> bool:
    if not path.exists():
        return False
    metric_names = {
        row.get("metric", "")
        for row in read_csv_rows(path)
        if row.get("metric")
    }
    return all(metric in metric_names for metric in REQUIRED_STANDARD_QA_METRICS)


def metrics_csv_matches_standard_sweep(path: Path, expected_max_ay_mps2: float) -> bool:
    if not metrics_csv_has_standard_qa(path):
        return False
    for row in read_csv_rows(path):
        if row.get("metric") != "standard_sweep_max_ay_mps2":
            continue
        return abs(finite_float(row.get("value")) - expected_max_ay_mps2) <= 1e-6
    return False


def standardsim_quality(
    metrics: dict[str, float],
    variant_dir: Path,
    *,
    expected_max_ay_mps2: float,
) -> dict[str, Any]:
    flags: list[str] = []
    if any(metric not in metrics for metric in REQUIRED_STANDARD_QA_METRICS):
        flags.append("missing_qa_metrics")

    failed = finite_float(metrics.get("n_failed_cases", math.nan))
    if math.isfinite(failed) and failed > 0.0:
        flags.append("failed_maneuver")
    elif not math.isfinite(failed):
        flags.append("unknown_failed_maneuver_count")

    sweep_max_ay = finite_float(metrics.get("standard_sweep_max_ay_mps2"))
    if not math.isfinite(sweep_max_ay):
        flags.append("unknown_standard_sweep_max_ay")
    elif abs(sweep_max_ay - expected_max_ay_mps2) > 1e-6:
        flags.append("standard_sweep_max_ay_mismatch")

    source_velocity = finite_float(metrics.get("metric_source_velocity_mps"))
    target_velocity = finite_float(metrics.get("metric_target_velocity_mps", 15.0))
    if (
        math.isfinite(source_velocity)
        and math.isfinite(target_velocity)
        and abs(source_velocity - target_velocity) > 0.25
    ):
        flags.append("metric_velocity_mismatch")

    for metric in (
        "roadwheel_fit_nrmse",
        "steer_excess_fit_nrmse",
        "roll_fit_nrmse",
        "sideslip_fit_nrmse",
    ):
        value = finite_float(metrics.get(metric))
        if math.isfinite(value) and value > 0.12:
            flags.append(f"{metric}_high")

    if not (variant_dir / "results" / "SteadyStateEval" / "steady_state_eval_report.pdf").exists():
        flags.append("missing_pdf")

    status = "ok" if not flags else "fail"
    return {
        "standardsim_quality_status": status,
        "standardsim_quality_flags": ";".join(flags) if flags else "ok",
    }


def standardsim_row_is_scoreable(row: dict[str, Any]) -> bool:
    return (
        int(row.get("is_reference", 0)) == 0
        and str(row.get("standardsim_status")) == "ok"
        and str(row.get("standardsim_quality_status", "ok")) == "ok"
    )


def apply_envelope_scores(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    candidate_rows = [row for row in rows if int(row["is_reference"]) == 0]
    metric_scores = {
        metric: normalized_metric_scores(rows, metric, source_rows=candidate_rows)
        for metric, _weight in ENVELOPE_SCORE_METRICS
    }
    output_rows = []
    for row in rows:
        score = 0.0
        weight_sum = 0.0
        output = dict(row)
        for metric, weight in ENVELOPE_SCORE_METRICS:
            metric_score = metric_scores[metric].get(row["candidate_id"], math.nan)
            output[f"envelope_score_part_{metric}"] = metric_score
            if math.isfinite(metric_score):
                score += weight * metric_score
                weight_sum += weight
        output["envelope_score"] = score / weight_sum if weight_sum > 0.0 else math.nan
        output_rows.append(output)
    ranked = sorted(
        [row for row in output_rows if int(row["is_reference"]) == 0],
        key=lambda row: finite_float(row.get("envelope_score")),
        reverse=True,
    )
    rank_lookup = {row["candidate_id"]: idx for idx, row in enumerate(ranked, start=1)}
    for row in output_rows:
        row["envelope_rank"] = rank_lookup.get(row["candidate_id"], "")
    return output_rows


def apply_integrated_scores(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    candidate_rows = [
        row
        for row in rows
        if standardsim_row_is_scoreable(row)
    ]
    sideslip_rows = []
    for row in rows:
        output = dict(row)
        output["sideslip_gradient_abs_deg_per_g"] = abs(finite_float(row.get("sideslip_gradient_deg_per_g")))
        sideslip_rows.append(output)
    candidate_sideslip_rows = [
        row
        for row in sideslip_rows
        if standardsim_row_is_scoreable(row)
    ]
    sideslip_scores = normalized_metric_scores(
        sideslip_rows,
        "sideslip_gradient_abs_deg_per_g",
        higher_is_better=False,
        source_rows=candidate_sideslip_rows,
    )
    torque_scores = normalized_metric_scores(
        sideslip_rows,
        "handwheel_torque_peak_abs",
        higher_is_better=False,
        source_rows=candidate_sideslip_rows,
    )
    scored = []
    for row in sideslip_rows:
        scoreable = standardsim_row_is_scoreable(row)
        if scoreable:
            understeer_score = target_score(
                row.get("understeer_gradient_deg_per_g"),
                target=0.5,
                span=2.0,
            )
            roll_score = target_score(
                row.get("roll_gradient_deg_per_g"),
                target=1.0,
                span=2.0,
            )
            torque_score = torque_scores.get(row["candidate_id"], math.nan)
            parts = [
                (understeer_score, 0.40),
                (roll_score, 0.25),
                (sideslip_scores.get(row["candidate_id"], math.nan), 0.20),
                (torque_score, 0.15),
            ]
            weight_sum = sum(weight for score, weight in parts if math.isfinite(score))
            standardsim_score = (
                sum(score * weight for score, weight in parts if math.isfinite(score)) / weight_sum
                if weight_sum > 0.0
                else math.nan
            )
        else:
            understeer_score = math.nan
            roll_score = math.nan
            torque_score = math.nan
            standardsim_score = math.nan
        envelope_score = finite_float(row.get("envelope_score"))
        integrated_score = (
            0.65 * envelope_score + 0.35 * standardsim_score
            if math.isfinite(envelope_score) and math.isfinite(standardsim_score)
            else math.nan
        )
        output = dict(row)
        output["standardsim_ay_score"] = math.nan
        output["standardsim_understeer_target_score"] = understeer_score
        output["standardsim_roll_score"] = roll_score
        output["standardsim_sideslip_score"] = (
            sideslip_scores.get(row["candidate_id"], math.nan) if scoreable else math.nan
        )
        output["standardsim_torque_score"] = torque_score
        output["standardsim_score"] = standardsim_score
        output["integrated_design_score"] = integrated_score
        output["response_flags"] = response_flags(output)
        scored.append(output)
    ranked = sorted(
        [
            row
            for row in scored
            if standardsim_row_is_scoreable(row)
            and math.isfinite(finite_float(row.get("integrated_design_score")))
        ],
        key=lambda row: finite_float(row.get("integrated_design_score")),
        reverse=True,
    )
    integrated_rank_lookup = {row["candidate_id"]: idx for idx, row in enumerate(ranked, start=1)}
    for row in scored:
        row["integrated_rank"] = integrated_rank_lookup.get(row["candidate_id"], "")
    return scored


def load_standard_helpers() -> Any:
    if str(GENERATION_SCRIPTS) not in sys.path:
        sys.path.insert(0, str(GENERATION_SCRIPTS))
    if str(BOBSIM_ROOT) not in sys.path:
        sys.path.insert(0, str(BOBSIM_ROOT))
    return load_module(
        "ds002_standardsim_helpers_for_ds006",
        REPO_ROOT / "studies" / "DS-002-standardsim-steady-state-sensitivity" / "run.py",
    )


def make_standardsim_vehicle_doc(
    vehicle_doc: dict[str, Any],
    candidate: TireCandidate,
    reference_radius_m: float,
) -> dict[str, Any]:
    variant_doc = yaml.safe_load(yaml.safe_dump(vehicle_doc))
    tire_params = parse_tir(candidate.path)
    baseline_cg = mass_rollup(vehicle_doc)[1]
    architecture = tire_architecture_fields(
        tire_params,
        reference_radius_m,
        float(baseline_cg[2]),
    )
    radius_delta = finite_float(architecture["architecture_radius_delta_m"])
    if math.isfinite(radius_delta) and abs(radius_delta) > 1e-12:
        variant_doc = shift_vehicle_coordinate_z_fields(variant_doc, radius_delta)
    variant_doc["paths"]["tire_templates"] = str(candidate.path.parent.resolve())
    for axle in ("front", "rear"):
        variant_doc[axle]["tire"]["template"] = candidate.path.stem
        variant_doc[axle]["tire"]["vertical_stiffness_n_per_m"] = tir_float(
            tire_params,
            "VERTICAL_STIFFNESS",
            variant_doc[axle]["tire"]["vertical_stiffness_n_per_m"],
        )
        variant_doc[axle]["tire"]["vertical_damping_n_s_per_m"] = tir_float(
            tire_params,
            "VERTICAL_DAMPING",
            variant_doc[axle]["tire"]["vertical_damping_n_s_per_m"],
        )
        variant_doc[axle]["wheel"]["radius_m"] = tir_float(
            tire_params,
            "UNLOADED_RADIUS",
            variant_doc[axle]["wheel"]["radius_m"],
        )
        tire_radius = finite_float(variant_doc[axle]["wheel"]["radius_m"])
        rim_radius = tir_float(tire_params, "RIM_RADIUS")
        rim_width = tir_float(tire_params, "RIM_WIDTH")
        if math.isfinite(tire_radius) and tire_radius > 0.0 and math.isfinite(rim_radius):
            variant_doc[axle]["wheel"]["rim_radius_ratio"] = rim_radius / tire_radius
        if math.isfinite(rim_radius) and rim_radius > 0.0 and math.isfinite(rim_width):
            variant_doc[axle]["wheel"]["rim_width_ratio"] = rim_width / rim_radius
    return variant_doc


def run_standardsim_case(
    case_index: int,
    case_count: int,
    vehicle_path: Path,
    candidate: TireCandidate,
    *,
    reuse: bool,
    reference_radius_m: float,
    standard_max_ay_mps2: float,
    standard_case_timeout_s: float,
    std: Any | None = None,
    vehicle_doc: dict[str, Any] | None = None,
) -> dict[str, Any]:
    case_start = time.perf_counter()
    print(f"  StandardSim {case_index:02d}/{case_count}: {candidate.label}", flush=True)
    variant_dir = WORK_DIR / "standardsim_all" / f"{case_index:02d}_{candidate.candidate_id}"
    try:
        if std is None:
            std = load_standard_helpers()
        if vehicle_doc is None:
            vehicle_doc = read_yaml(vehicle_path)
        variant_doc = make_standardsim_vehicle_doc(vehicle_doc, candidate, reference_radius_m)
        record_text = std.render_record(variant_doc, vehicle_path)
        variant_changed = std.stage_variant_text(
            variant_dir,
            record_text,
            rebuild=not reuse,
        )
        build_dir = variant_dir / "build" / "SteadyStateEval"
        if not reuse or variant_changed or std.find_executable(build_dir) is None:
            std.compile_variant(variant_dir)
        metrics_path = variant_dir / "results" / "SteadyStateEval" / "metrics.csv"
        if (
            not reuse
            or variant_changed
            or not metrics_path.exists()
            or not metrics_csv_matches_standard_sweep(metrics_path, standard_max_ay_mps2)
        ):
            std.run_standard_report(
                variant_dir,
                max_ay_mps2=standard_max_ay_mps2,
                case_timeout_s=standard_case_timeout_s,
                fail_fast=False,
            )
        metrics, _raw_rows = std.read_metrics_csv(metrics_path)
        quality = standardsim_quality(
            metrics,
            variant_dir,
            expected_max_ay_mps2=standard_max_ay_mps2,
        )
        output = {
            "candidate_id": candidate.candidate_id,
            "label": candidate.label,
            "status": "ok",
            "variant_dir": as_repo_path(variant_dir),
            "elapsed_s": time.perf_counter() - case_start,
        }
        for metric in (*STANDARD_METRICS, *STANDARD_QA_METRICS):
            output[metric] = metrics.get(metric, math.nan)
        output.update(quality)
        try:
            (variant_dir / "case_error.txt").unlink()
        except FileNotFoundError:
            pass
        return {"index": case_index, "metric": output, "error": None}
    except Exception as exc:  # noqa: BLE001
        error = {
            "candidate_id": candidate.candidate_id,
            "label": candidate.label,
            "error": str(exc),
            "variant_dir": as_repo_path(variant_dir),
            "elapsed_s": time.perf_counter() - case_start,
        }
        (variant_dir / "case_error.txt").parent.mkdir(parents=True, exist_ok=True)
        (variant_dir / "case_error.txt").write_text(str(exc), encoding="utf-8")
        return {"index": case_index, "metric": None, "error": error}


def run_standardsim(
    vehicle_path: Path,
    candidates: list[TireCandidate],
    *,
    reuse: bool,
    limit: int | None,
    reference_radius_m: float,
    workers: int,
    standard_max_ay_mps2: float,
    standard_case_timeout_s: float,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    if shutil.which("omc") is None:
        return [], [
            {
                "candidate_id": "",
                "label": "",
                "error": "OpenModelica `omc` was not found on PATH.",
            }
        ]
    cases = candidates if limit is None else candidates[:limit]
    std = load_standard_helpers()
    vehicle_doc = read_yaml(vehicle_path)
    metric_rows: list[dict[str, Any]] = []
    error_rows: list[dict[str, Any]] = []
    case_count = len(cases)
    worker_count = max(1, min(int(workers), case_count)) if case_count else 1

    if worker_count == 1:
        results = [
            run_standardsim_case(
                idx,
                case_count,
                vehicle_path,
                candidate,
                reuse=reuse,
                reference_radius_m=reference_radius_m,
                standard_max_ay_mps2=standard_max_ay_mps2,
                standard_case_timeout_s=standard_case_timeout_s,
                std=std,
                vehicle_doc=vehicle_doc,
            )
            for idx, candidate in enumerate(cases, start=1)
        ]
    else:
        print(f"  StandardSim worker pool: {worker_count}", flush=True)
        results = []
        with concurrent.futures.ThreadPoolExecutor(max_workers=worker_count) as executor:
            future_lookup = {
                executor.submit(
                    run_standardsim_case,
                    idx,
                    case_count,
                    vehicle_path,
                    candidate,
                    reuse=reuse,
                    reference_radius_m=reference_radius_m,
                    standard_max_ay_mps2=standard_max_ay_mps2,
                    standard_case_timeout_s=standard_case_timeout_s,
                    std=std,
                    vehicle_doc=vehicle_doc,
                ): (idx, candidate)
                for idx, candidate in enumerate(cases, start=1)
            }
            for future in concurrent.futures.as_completed(future_lookup):
                idx, candidate = future_lookup[future]
                try:
                    result = future.result()
                except Exception as exc:  # noqa: BLE001
                    variant_dir = WORK_DIR / "standardsim_all" / f"{idx:02d}_{candidate.candidate_id}"
                    result = {
                        "index": idx,
                        "metric": None,
                        "error": {
                            "candidate_id": candidate.candidate_id,
                            "label": candidate.label,
                            "error": str(exc),
                            "variant_dir": as_repo_path(variant_dir),
                            "elapsed_s": math.nan,
                        },
                    }
                results.append(result)
                status = "ok" if result.get("metric") else "error"
                print(f"  StandardSim done {idx:02d}/{case_count}: {candidate.label} [{status}]", flush=True)

    for result in sorted(results, key=lambda item: int(item["index"])):
        if result.get("metric"):
            metric_rows.append(result["metric"])
        if result.get("error"):
            error_rows.append(result["error"])
    return metric_rows, error_rows


def merge_results(
    envelope_rows: list[dict[str, Any]],
    characterization_rows: list[dict[str, Any]],
    standardsim_rows: list[dict[str, Any]],
    transient_temperature_rows: list[dict[str, Any]],
    degradation_rows: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    char_lookup = {row["candidate_id"]: row for row in characterization_rows}
    std_lookup = {row["candidate_id"]: row for row in standardsim_rows}
    temp_lookup = {
        row["candidate_id"]: row
        for row in transient_temperature_rows
        if row.get("candidate_id") and row.get("temperature_status") == "ok"
    }
    degradation_lookup = {
        row["candidate_id"]: row
        for row in degradation_rows
        if row.get("candidate_id") and row.get("degradation_status") in {"ok", "partial"}
    }
    merged = []
    for env_row in envelope_rows:
        row = dict(env_row)
        row.update({key: value for key, value in char_lookup.get(row["candidate_id"], {}).items() if key not in row})
        temp_row = temp_lookup.get(row["candidate_id"])
        if temp_row:
            for key, value in temp_row.items():
                if key in {"candidate_id", "label"}:
                    continue
                row[key] = value
        degradation_row = degradation_lookup.get(row["candidate_id"])
        if degradation_row:
            for key, value in degradation_row.items():
                if key in {"candidate_id", "label"}:
                    continue
                row[key] = value
        std_row = std_lookup.get(row["candidate_id"])
        row["standardsim_status"] = std_row.get("status", "not_run") if std_row else "not_run"
        if std_row:
            for key, value in std_row.items():
                if key in {"candidate_id", "label"}:
                    continue
                row[key] = value
            ay_mps2 = finite_float(row.get("ay_max"))
            row["ay_max_g"] = ay_mps2 / G if math.isfinite(ay_mps2) else math.nan
        merged.append(row)
    scored = apply_envelope_scores(merged)
    return apply_integrated_scores(scored)


def family_color_map(rows: list[dict[str, Any]]) -> dict[str, Any]:
    families = sorted({str(row.get("family", "")) for row in rows})
    cmap = plt.get_cmap("tab20")
    return {family: cmap(idx % 20) for idx, family in enumerate(families)}


def plot_rank(
    rows: list[dict[str, Any]],
    metric: str,
    title: str,
    path: Path,
    *,
    top_n: int | None = None,
    require_scoreable_standardsim: bool = False,
) -> None:
    candidates = [
        row
        for row in rows
        if int(row["is_reference"]) == 0 and math.isfinite(finite_float(row.get(metric)))
        and (not require_scoreable_standardsim or standardsim_row_is_scoreable(row))
    ]
    candidates = sorted(candidates, key=lambda row: finite_float(row.get(metric)))
    if top_n is not None and len(candidates) > top_n:
        candidates = candidates[-top_n:]
    if not candidates:
        return
    colors = family_color_map(candidates)
    fig_height = max(7.5, 0.23 * len(candidates) + 1.8)
    fig, ax = plt.subplots(figsize=(12.5, fig_height))
    ax.barh(
        [row["label"] for row in candidates],
        [finite_float(row.get(metric)) for row in candidates],
        color=[colors[str(row["family"])] for row in candidates],
    )
    ax.set_xlabel(metric.replace("_", " "))
    ax.set_title(title)
    ax.grid(True, axis="x", linestyle="--", alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=220)
    plt.close(fig)


def plot_current_package_rank(rows: list[dict[str, Any]]) -> None:
    candidates = [
        row
        for row in rows
        if standardsim_row_is_scoreable(row)
        and math.isfinite(finite_float(row.get("integrated_design_score")))
        and abs(finite_float(row.get("architecture_radius_delta_mm"))) <= 0.5
    ]
    candidates = sorted(candidates, key=lambda row: finite_float(row.get("integrated_design_score")))
    if not candidates:
        return
    colors = [
        "#d97706" if str(row.get("family")) == "Hoosier 43075" else "#64748b"
        for row in candidates
    ]
    fig_height = max(7.0, 0.26 * len(candidates) + 1.8)
    fig, ax = plt.subplots(figsize=(12.5, fig_height))
    ax.barh(
        [row["label"] for row in candidates],
        [finite_float(row.get("integrated_design_score")) for row in candidates],
        color=colors,
        alpha=0.9,
    )
    ax.set_xlabel("Integrated design score")
    ax.set_title("Zero-Radius-Delta Current-Package Ranking")
    ax.grid(True, axis="x", linestyle="--", alpha=0.3)
    fig.tight_layout()
    fig.savefig(PLOT_DIR / "current_package_integrated_rank.png", dpi=220)
    plt.close(fig)


def plot_family_best_score_comparison(rows: list[dict[str, Any]]) -> None:
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        if standardsim_row_is_scoreable(row) and math.isfinite(finite_float(row.get("integrated_design_score"))):
            grouped[str(row["family"])].append(row)
    best_rows = [
        max(family_rows, key=lambda row: finite_float(row.get("integrated_design_score")))
        for _family, family_rows in sorted(grouped.items())
        if family_rows
    ]
    best_rows = sorted(best_rows, key=lambda row: finite_float(row.get("integrated_design_score")))
    if not best_rows:
        return

    labels = [
        (
            f"{row['family']}: {row['tire_size']} "
            f"{format_float(row.get('rim_width_in'), 0)}in/{format_float(row.get('pressure_psi'), 0)}psi"
        )
        for row in best_rows
    ]
    y = np.arange(len(best_rows), dtype=float)
    height = 0.24
    fig, ax = plt.subplots(figsize=(12.5, max(6.2, 0.45 * len(best_rows) + 2.0)))
    ax.barh(
        y - height,
        [finite_float(row.get("envelope_score")) for row in best_rows],
        height=height,
        label="EnvelopeSim score",
        color="#0f766e",
        alpha=0.86,
    )
    ax.barh(
        y,
        [finite_float(row.get("standardsim_score")) for row in best_rows],
        height=height,
        label="StandardSim score",
        color="#2563eb",
        alpha=0.86,
    )
    ax.barh(
        y + height,
        [finite_float(row.get("integrated_design_score")) for row in best_rows],
        height=height,
        label="Integrated score",
        color="#d97706",
        alpha=0.9,
    )
    ax.set_yticks(y)
    ax.set_yticklabels(labels)
    ax.set_xlabel("Candidate-normalized score")
    ax.set_title("Best Scoreable Setup By Tire Family")
    ax.grid(True, axis="x", linestyle="--", alpha=0.3)
    ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(PLOT_DIR / "family_best_score_comparison.png", dpi=220)
    plt.close(fig)


def plot_tire_fit_mu_stiffness_map(rows: list[dict[str, Any]]) -> None:
    candidates = [
        row
        for row in rows
        if int(row["is_reference"]) == 0
        and math.isfinite(finite_float(row.get("mu_y")))
        and math.isfinite(finite_float(row.get("ky_norm_per_rad")))
    ]
    if not candidates:
        return
    colors = family_color_map(candidates)
    fig, ax = plt.subplots(figsize=(12.5, 8.0))
    seen: set[str] = set()
    for row in candidates:
        family = str(row["family"])
        is_selected = str(row.get("integrated_rank")) == "1"
        pressure = finite_float(row.get("pressure_psi"))
        ax.scatter(
            finite_float(row.get("mu_y")),
            finite_float(row.get("ky_norm_per_rad")),
            s=120.0 if is_selected else 42.0 + 5.5 * pressure,
            color="#d97706" if is_selected else colors[family],
            edgecolor="black" if is_selected else "white",
            linewidth=1.3 if is_selected else 0.6,
            alpha=0.92 if is_selected else 0.78,
            label=family if family not in seen else None,
            zorder=4 if is_selected else 2,
        )
        seen.add(family)

    mu_values = [finite_float(row.get("mu_y")) for row in candidates]
    ky_values = [finite_float(row.get("ky_norm_per_rad")) for row in candidates]
    ax.axvline(median(mu_values), color="#334155", linestyle="--", linewidth=1.0, alpha=0.45)
    ax.axhline(median(ky_values), color="#334155", linestyle="--", linewidth=1.0, alpha=0.45)

    for row in candidates:
        rank = str(row.get("integrated_rank", ""))
        should_annotate = rank in {"1", "2", "3"} or str(row.get("envelope_rank")) == "1"
        if should_annotate:
            ax.annotate(
                rank or "env1",
                (finite_float(row.get("mu_y")), finite_float(row.get("ky_norm_per_rad"))),
                textcoords="offset points",
                xytext=(6, 5),
                fontsize=8,
                fontweight="bold" if rank == "1" else "normal",
            )

    ax.set_xlabel("Fitted lateral peak friction, mu_y [-]")
    ax.set_ylabel("Normalized cornering stiffness, Ky/Fz [1/rad]")
    ax.set_title("Tire Fit Lateral Peak Mu Versus Cornering Stiffness")
    ax.grid(True, linestyle="--", alpha=0.3)
    ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), frameon=False)
    fig.tight_layout(rect=(0.0, 0.0, 0.84, 1.0))
    fig.savefig(PLOT_DIR / "tire_fit_mu_stiffness_map.png", dpi=220)
    plt.close(fig)


def plot_tire_fit_mu_stiffness_rank(rows: list[dict[str, Any]]) -> None:
    candidates = [
        row
        for row in rows
        if int(row["is_reference"]) == 0
        and math.isfinite(finite_float(row.get("mu_y")))
        and math.isfinite(finite_float(row.get("ky_norm_per_rad")))
    ]
    if not candidates:
        return

    def _sort_score(row: dict[str, Any], metric: str) -> float:
        value = finite_float(row.get(metric))
        return value if math.isfinite(value) else -math.inf

    candidates = sorted(
        candidates,
        key=lambda row: (
            _sort_score(row, "integrated_design_score"),
            _sort_score(row, "envelope_score"),
        ),
    )
    mu_scores = normalized_metric_scores(candidates, "mu_y", source_rows=candidates)
    ky_scores = normalized_metric_scores(candidates, "ky_norm_per_rad", source_rows=candidates)
    y = np.arange(len(candidates), dtype=float)
    height = 0.36
    fig, ax = plt.subplots(figsize=(13.2, max(9.0, 0.25 * len(candidates) + 2.0)))
    edgecolors = ["black" if str(row.get("integrated_rank")) == "1" else "none" for row in candidates]
    linewidths = [1.2 if str(row.get("integrated_rank")) == "1" else 0.0 for row in candidates]
    ax.barh(
        y - height / 2.0,
        [mu_scores[row["candidate_id"]] for row in candidates],
        height=height,
        color="#0f766e",
        edgecolor=edgecolors,
        linewidth=linewidths,
        alpha=0.88,
        label="Peak lateral mu_y, normalized",
    )
    ax.barh(
        y + height / 2.0,
        [ky_scores[row["candidate_id"]] for row in candidates],
        height=height,
        color="#2563eb",
        edgecolor=edgecolors,
        linewidth=linewidths,
        alpha=0.80,
        label="Cornering stiffness Ky/Fz, normalized",
    )
    ax.set_yticks(y)
    ax.set_yticklabels([row["label"] for row in candidates])
    ax.set_xlabel("Candidate-normalized fitted tire property")
    ax.set_title("Peak Lateral Mu And Cornering Stiffness By Candidate")
    ax.grid(True, axis="x", linestyle="--", alpha=0.3)
    ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(PLOT_DIR / "tire_fit_mu_stiffness_rank.png", dpi=220)
    plt.close(fig)


def plot_pressure_trends(rows: list[dict[str, Any]]) -> None:
    candidates = [row for row in rows if int(row["is_reference"]) == 0]
    groups: dict[tuple[str, str, float], list[dict[str, Any]]] = defaultdict(list)
    for row in candidates:
        groups[(row["family"], row["tire_size"], finite_float(row["rim_width_in"]))].append(row)
    colors = family_color_map(candidates)
    fig, axes = plt.subplots(2, 1, figsize=(12.5, 10.0), sharex=True)
    for (family, tire_size, rim_width), group_rows in sorted(groups.items()):
        ordered = sorted(group_rows, key=lambda row: finite_float(row["pressure_psi"]))
        label = f"{family} {tire_size} {rim_width:g}in"
        for ax, metric in zip(axes, ("mean_max_lateral_g", "standardsim_score"), strict=False):
            if not any(math.isfinite(finite_float(row.get(metric))) for row in ordered):
                continue
            ax.plot(
                [finite_float(row["pressure_psi"]) for row in ordered],
                [finite_float(row.get(metric)) for row in ordered],
                marker="o",
                linewidth=1.4,
                markersize=4,
                alpha=0.84,
                color=colors[family],
                label=label,
            )
    axes[0].set_ylabel("Envelope mean lateral [g]")
    axes[1].set_ylabel("StandardSim stable-window score")
    axes[1].set_xlabel("Pressure [psi]")
    axes[0].set_title("Pressure Response Across Tire Architectures")
    for ax in axes:
        ax.grid(True, linestyle="--", alpha=0.3)
    handles, labels = axes[0].get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    fig.legend(unique.values(), unique.keys(), loc="center left", bbox_to_anchor=(1.0, 0.5), fontsize=8, frameon=False)
    fig.tight_layout(rect=(0.0, 0.0, 0.82, 1.0))
    fig.savefig(PLOT_DIR / "pressure_trends_vehicle_metrics.png", dpi=220)
    plt.close(fig)


def plot_trade_space(rows: list[dict[str, Any]]) -> None:
    candidates = [
        row
        for row in rows
        if int(row["is_reference"]) == 0 and math.isfinite(finite_float(row.get("standardsim_score")))
    ]
    if not candidates:
        return
    colors = family_color_map(candidates)
    fig, axes = plt.subplots(2, 2, figsize=(12.5, 9.4))
    specs = [
        ("mean_max_lateral_g", "standardsim_score", "Envelope mean lateral [g]", "StandardSim stable-window score"),
        ("understeer_gradient_deg_per_g", "standardsim_score", "Understeer gradient [deg/g]", "StandardSim stable-window score"),
        ("roll_gradient_deg_per_g", "standardsim_score", "Roll gradient [deg/g]", "StandardSim stable-window score"),
        ("sigma_alpha_m", "standardsim_score", "Lateral relaxation sigma [m]", "StandardSim stable-window score"),
    ]
    for ax, (x_key, y_key, x_label, y_label) in zip(axes.ravel(), specs, strict=False):
        ax.scatter(
            [finite_float(row.get(x_key)) for row in candidates],
            [finite_float(row.get(y_key)) for row in candidates],
            c=[colors[str(row["family"])] for row in candidates],
            s=[46.0 + 10.0 * finite_float(row.get("pressure_psi")) for row in candidates],
            alpha=0.78,
            edgecolor="white",
            linewidth=0.6,
        )
        ax.set_xlabel(x_label)
        ax.set_ylabel(y_label)
        ax.grid(True, linestyle="--", alpha=0.3)
    for row in sorted(candidates, key=lambda item: finite_float(item.get("integrated_design_score")), reverse=True)[:5]:
        axes[0, 0].annotate(
            str(row["integrated_rank"]),
            (finite_float(row.get("mean_max_lateral_g")), finite_float(row.get("standardsim_score"))),
            textcoords="offset points",
            xytext=(4, 4),
            fontsize=8,
        )
    fig.suptitle("Integrated Tire Trade Space")
    fig.tight_layout()
    fig.savefig(PLOT_DIR / "integrated_trade_space.png", dpi=220)
    plt.close(fig)


def plot_relaxation_rank(rows: list[dict[str, Any]]) -> None:
    candidates = [
        row
        for row in rows
        if int(row["is_reference"]) == 0
        and math.isfinite(finite_float(row.get("sigma_alpha_m")))
    ]
    candidates = sorted(candidates, key=lambda row: finite_float(row.get("sigma_alpha_m")), reverse=True)
    if not candidates:
        return
    colors = family_color_map(candidates)
    fig_height = max(8.0, 0.22 * len(candidates) + 1.8)
    fig, ax = plt.subplots(figsize=(12.5, fig_height))
    bar_colors = []
    for row in candidates:
        if str(row.get("family")) == "Hoosier 43075":
            bar_colors.append("#d97706")
        else:
            bar_colors.append(colors[str(row["family"])])
    ax.barh(
        [row["label"] for row in candidates],
        [finite_float(row.get("sigma_alpha_m")) for row in candidates],
        color=bar_colors,
        alpha=0.88,
    )
    ax.set_xlabel("Lateral relaxation length sigma_alpha [m], lower is faster")
    ax.set_title("Round 9 Lateral Relaxation Length Ranking")
    ax.grid(True, axis="x", linestyle="--", alpha=0.3)
    fig.tight_layout()
    fig.savefig(PLOT_DIR / "relaxation_rank.png", dpi=220)
    plt.close(fig)


def plot_relaxation_trade_space(rows: list[dict[str, Any]]) -> None:
    candidates = [
        row
        for row in rows
        if int(row["is_reference"]) == 0
        and math.isfinite(finite_float(row.get("sigma_alpha_m")))
        and math.isfinite(finite_float(row.get("integrated_design_score")))
    ]
    if not candidates:
        return
    colors = family_color_map(candidates)
    fig, axes = plt.subplots(1, 2, figsize=(13.2, 6.2))
    for ax, y_key, y_label in (
        (axes[0], "integrated_design_score", "Integrated score"),
        (axes[1], "standardsim_score", "StandardSim stable-window score"),
    ):
        for row in candidates:
            is_43075 = str(row.get("family")) == "Hoosier 43075"
            ax.scatter(
                finite_float(row.get("sigma_alpha_m")),
                finite_float(row.get(y_key)),
                s=72 if is_43075 else 46,
                color="#d97706" if is_43075 else colors[str(row["family"])],
                edgecolor="black" if is_43075 else "white",
                linewidth=1.1 if is_43075 else 0.5,
                alpha=0.86,
            )
        ax.set_xlabel("Lateral relaxation length sigma_alpha [m], lower is faster")
        ax.set_ylabel(y_label)
        ax.grid(True, linestyle="--", alpha=0.3)
    for row in sorted(
        [row for row in candidates if str(row.get("family")) == "Hoosier 43075"],
        key=lambda item: finite_float(item.get("pressure_psi")),
    ):
        axes[0].annotate(
            f"{format_float(row.get('rim_width_in'), 0)}in {format_float(row.get('pressure_psi'), 0)}psi",
            (finite_float(row.get("sigma_alpha_m")), finite_float(row.get("integrated_design_score"))),
            textcoords="offset points",
            xytext=(4, 4),
            fontsize=7,
        )
    fig.suptitle("Relaxation Length Versus Vehicle-Level Performance")
    fig.tight_layout()
    fig.savefig(PLOT_DIR / "relaxation_trade_space.png", dpi=220)
    plt.close(fig)


def plot_transient_temperature(rows: list[dict[str, Any]]) -> None:
    candidates = [
        row
        for row in rows
        if int(row["is_reference"]) == 0
        and math.isfinite(finite_float(row.get("transient_tread_mean_c")))
    ]
    if not candidates:
        return
    groups: dict[tuple[str, str, float], list[dict[str, Any]]] = defaultdict(list)
    for row in candidates:
        groups[(str(row["family"]), str(row["tire_size"]), finite_float(row["rim_width_in"]))].append(row)
    colors = family_color_map(candidates)
    fig, axes = plt.subplots(1, 2, figsize=(13.2, 6.1), sharex=True)
    for (family, tire_size, rim_width), group_rows in sorted(groups.items()):
        ordered = sorted(group_rows, key=lambda row: finite_float(row.get("pressure_psi")))
        label = f"{family} {tire_size} {rim_width:g}in"
        color = "#d97706" if family == "Hoosier 43075" else colors[family]
        width = 2.2 if family == "Hoosier 43075" else 1.3
        for ax, metric, ylabel in (
            (axes[0], "transient_tread_mean_c", "Mean tread temp [C]"),
            (axes[1], "transient_tread_rise_c", "Transient temp rise [C]"),
        ):
            ax.plot(
                [finite_float(row.get("pressure_psi")) for row in ordered],
                [finite_float(row.get(metric)) for row in ordered],
                marker="o",
                linewidth=width,
                markersize=4.5,
                alpha=0.86,
                color=color,
                label=label,
            )
            ax.set_ylabel(ylabel)
            ax.grid(True, linestyle="--", alpha=0.3)
    axes[0].set_title("Round 9 Transient Tread Temperature")
    axes[1].set_title("Transient Temperature Rise")
    for ax in axes:
        ax.set_xlabel("Pressure [psi]")
    handles, labels = axes[0].get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    fig.legend(unique.values(), unique.keys(), loc="center left", bbox_to_anchor=(1.0, 0.5), fontsize=8, frameon=False)
    fig.tight_layout(rect=(0.0, 0.0, 0.82, 1.0))
    fig.savefig(PLOT_DIR / "transient_temperature.png", dpi=220)
    plt.close(fig)


def unique_degradation_setup_rows(rows: list[dict[str, Any]], metric: str) -> list[dict[str, Any]]:
    by_setup: dict[tuple[str, str, float], dict[str, Any]] = {}
    for row in rows:
        if int(row["is_reference"]) == 1 or not math.isfinite(finite_float(row.get(metric))):
            continue
        key = (str(row["family"]), str(row["tire_size"]), finite_float(row["rim_width_in"]))
        existing = by_setup.get(key)
        if existing is None or abs(finite_float(row.get("pressure_psi")) - DEGRADATION_PRESSURE_PSI) < abs(
            finite_float(existing.get("pressure_psi")) - DEGRADATION_PRESSURE_PSI
        ):
            by_setup[key] = row
    return list(by_setup.values())


def degradation_label(row: dict[str, Any]) -> str:
    return f"{row['family']} {row['tire_size']} {format_float(row.get('rim_width_in'), 0)}in"


def plot_tire_degradation_cornering(rows: list[dict[str, Any]]) -> None:
    candidates = unique_degradation_setup_rows(rows, "degradation_corner_peak_mu_y_delta_pct")
    if not candidates:
        return
    candidates = sorted(candidates, key=lambda row: finite_float(row.get("degradation_corner_peak_mu_y_delta_pct")))
    colors = family_color_map(candidates)
    y = np.arange(len(candidates), dtype=float)
    fig, axes = plt.subplots(1, 2, figsize=(13.8, max(7.0, 0.42 * len(candidates) + 1.8)), sharey=True)
    for ax, metric, xlabel, title in (
        (
            axes[0],
            "degradation_corner_peak_mu_y_delta_pct",
            "Final minus initial peak mu_y [%]",
            "Measured Lateral Peak Change",
        ),
        (
            axes[1],
            "degradation_corner_ky_delta_pct",
            "Final minus initial Ky/Fz [%]",
            "Measured Cornering Stiffness Change",
        ),
    ):
        ax.barh(
            y,
            [finite_float(row.get(metric)) for row in candidates],
            color=[
                "#d97706" if str(row.get("family")) == "Hoosier 43075" else colors[str(row["family"])]
                for row in candidates
            ],
            alpha=0.88,
        )
        ax.axvline(0.0, color="#111827", linewidth=1.0, alpha=0.72)
        ax.set_xlabel(xlabel)
        ax.set_title(title)
        ax.grid(True, axis="x", linestyle="--", alpha=0.3)
    axes[0].set_yticks(y)
    axes[0].set_yticklabels([degradation_label(row) for row in candidates])
    fig.suptitle("Round 9 Initial-To-Final 12 psi Cornering Degradation")
    fig.tight_layout()
    fig.savefig(PLOT_DIR / "tire_degradation_cornering.png", dpi=220)
    plt.close(fig)


def plot_tire_degradation_drive_brake(rows: list[dict[str, Any]]) -> None:
    candidates = unique_degradation_setup_rows(rows, "degradation_drive_peak_mu_x_delta_pct")
    if not candidates:
        return
    candidates = sorted(candidates, key=lambda row: finite_float(row.get("degradation_drive_peak_mu_x_delta_pct")))
    colors = family_color_map(candidates)
    y = np.arange(len(candidates), dtype=float)
    fig, axes = plt.subplots(1, 2, figsize=(13.8, max(6.4, 0.48 * len(candidates) + 1.8)), sharey=True)
    for ax, metric, xlabel, title in (
        (
            axes[0],
            "degradation_drive_peak_mu_x_delta_pct",
            "Final minus initial peak mu_x [%]",
            "Measured Longitudinal Peak Change",
        ),
        (
            axes[1],
            "degradation_drive_kx_delta_pct",
            "Final minus initial Kx/Fz [%]",
            "Measured Longitudinal Stiffness Change",
        ),
    ):
        ax.barh(
            y,
            [finite_float(row.get(metric)) for row in candidates],
            color=[colors[str(row["family"])] for row in candidates],
            alpha=0.88,
        )
        ax.axvline(0.0, color="#111827", linewidth=1.0, alpha=0.72)
        ax.set_xlabel(xlabel)
        ax.set_title(title)
        ax.grid(True, axis="x", linestyle="--", alpha=0.3)
    axes[0].set_yticks(y)
    axes[0].set_yticklabels([degradation_label(row) for row in candidates])
    fig.suptitle("Round 9 Initial-To-Final 12 psi Drive/Brake Degradation")
    fig.tight_layout()
    fig.savefig(PLOT_DIR / "tire_degradation_drive_brake.png", dpi=220)
    plt.close(fig)


def plot_outputs(rows: list[dict[str, Any]]) -> None:
    PLOT_DIR.mkdir(parents=True, exist_ok=True)
    plot_rank(rows, "integrated_design_score", "Integrated Tire Design Ranking", PLOT_DIR / "integrated_score_rank.png")
    plot_rank(rows, "envelope_score", "EnvelopeSim Tire Ranking", PLOT_DIR / "envelope_score_rank.png")
    plot_rank(
        rows,
        "standardsim_score",
        "StandardSim Stable-Window Handling Ranking (QA-Passing Rows)",
        PLOT_DIR / "standardsim_score_rank.png",
        require_scoreable_standardsim=True,
    )
    plot_current_package_rank(rows)
    plot_family_best_score_comparison(rows)
    plot_tire_fit_mu_stiffness_map(rows)
    plot_tire_fit_mu_stiffness_rank(rows)
    plot_pressure_trends(rows)
    plot_trade_space(rows)
    plot_relaxation_rank(rows)
    plot_relaxation_trade_space(rows)
    plot_transient_temperature(rows)
    plot_tire_degradation_cornering(rows)
    plot_tire_degradation_drive_brake(rows)


def median(values: list[float]) -> float:
    values = [value for value in values if math.isfinite(value)]
    return float(np.median(values)) if values else math.nan


def percentile(values: list[float], pct: float) -> float:
    values = [value for value in values if math.isfinite(value)]
    return float(np.percentile(values, pct)) if values else math.nan


def pressure_delta_stats(rows: list[dict[str, Any]], metric: str) -> list[tuple[float, str]]:
    groups: dict[tuple[str, str, float], dict[float, dict[str, Any]]] = defaultdict(dict)
    for row in rows:
        if int(row["is_reference"]) == 1:
            continue
        key = (str(row["family"]), str(row["tire_size"]), finite_float(row["rim_width_in"]))
        groups[key][finite_float(row["pressure_psi"])] = row
    deltas = []
    for key, by_pressure in groups.items():
        low = by_pressure.get(8.0)
        high = by_pressure.get(14.0)
        if not low or not high:
            continue
        low_value = finite_float(low.get(metric))
        high_value = finite_float(high.get(metric))
        if math.isfinite(low_value) and math.isfinite(high_value) and abs(low_value) > 1e-12:
            label = f"{key[0]} {key[1]} {key[2]:g}in"
            deltas.append((100.0 * (high_value - low_value) / low_value, label))
    return deltas


def rim_delta_stats(rows: list[dict[str, Any]], metric: str) -> list[tuple[float, str]]:
    groups: dict[tuple[str, str, float], dict[float, dict[str, Any]]] = defaultdict(dict)
    for row in rows:
        if int(row["is_reference"]) == 1:
            continue
        key = (str(row["family"]), str(row["tire_size"]), finite_float(row["pressure_psi"]))
        groups[key][finite_float(row["rim_width_in"])] = row
    deltas = []
    for key, by_rim in groups.items():
        if len(by_rim) < 2:
            continue
        rims = sorted(by_rim)
        narrow = by_rim[rims[0]]
        wide = by_rim[rims[-1]]
        narrow_value = finite_float(narrow.get(metric))
        wide_value = finite_float(wide.get(metric))
        if math.isfinite(narrow_value) and math.isfinite(wide_value) and abs(narrow_value) > 1e-12:
            label = f"{key[0]} {key[1]} {key[2]:g} psi"
            deltas.append((100.0 * (wide_value - narrow_value) / narrow_value, label))
    return deltas


def group_summary(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    groups: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        if int(row["is_reference"]) == 1:
            continue
        for field in ("family", "tire_size", "rim_width_in", "pressure_psi", "longitudinal_combined_source"):
            groups[(field, str(row.get(field, "")))].append(row)
    output = []
    for (field, value), group_rows in sorted(groups.items()):
        ranked = sorted(
            [
                row
                for row in group_rows
                if math.isfinite(finite_float(row.get("integrated_design_score")))
            ],
            key=lambda row: finite_float(row.get("integrated_design_score")),
            reverse=True,
        )
        if not ranked:
            ranked = sorted(
                group_rows,
                key=lambda row: finite_float(row.get("envelope_score")),
                reverse=True,
            )
        best_row = ranked[0]
        output.append(
            {
                "group_field": field,
                "group_value": value,
                "n": len(group_rows),
                "mean_envelope_score": median([finite_float(row.get("envelope_score")) for row in group_rows]),
                "mean_standardsim_score": median([finite_float(row.get("standardsim_score")) for row in group_rows]),
                "mean_integrated_score": median([finite_float(row.get("integrated_design_score")) for row in group_rows]),
                "best_candidate_id": best_row["candidate_id"],
                "best_label": best_row["label"],
                "best_integrated_score": best_row.get("integrated_design_score"),
            }
        )
    return output


def source_counts(rows: list[dict[str, Any]], field: str) -> dict[str, int]:
    counts: dict[str, int] = defaultdict(int)
    for row in rows:
        if int(row["is_reference"]) == 1:
            continue
        counts[str(row.get(field, "unknown")) or "unknown"] += 1
    return dict(sorted(counts.items()))


def top_rows(rows: list[dict[str, Any]], metric: str, n: int, *, reverse: bool = True) -> list[dict[str, Any]]:
    candidates = [
        row
        for row in rows
        if int(row["is_reference"]) == 0 and math.isfinite(finite_float(row.get(metric)))
    ]
    return sorted(candidates, key=lambda row: finite_float(row.get(metric)), reverse=reverse)[:n]


def clean_positive_understeer_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [
        row
        for row in rows
        if standardsim_row_is_scoreable(row)
        and row.get("response_flags") == "ok"
        and finite_float(row.get("understeer_gradient_deg_per_g")) >= 0.0
        and finite_float(row.get("roll_gradient_deg_per_g")) > 0.0
    ]


def best_by_family_for_metric(rows: list[dict[str, Any]], metric: str, *, reverse: bool) -> list[dict[str, Any]]:
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        if int(row["is_reference"]) == 0 and math.isfinite(finite_float(row.get(metric))):
            grouped[str(row["family"])].append(row)
    output = []
    for family, family_rows in sorted(grouped.items()):
        output.append(
            sorted(
                family_rows,
                key=lambda row: finite_float(row.get(metric)),
                reverse=reverse,
            )[0]
        )
    return output


def append_delta_table(lines: list[str], rows: list[dict[str, Any]], metric_specs: list[tuple[str, str]], kind: str) -> None:
    lines.append(table_line(["Metric", "Median delta", "Largest drop", "Largest rise"]))
    lines.append(table_line(["---", "---:", "---", "---"]))
    for metric, label in metric_specs:
        deltas = pressure_delta_stats(rows, metric) if kind == "pressure" else rim_delta_stats(rows, metric)
        if not deltas:
            continue
        med = median([delta for delta, _label in deltas])
        low = min(deltas, key=lambda item: item[0])
        high = max(deltas, key=lambda item: item[0])
        lines.append(
            table_line(
                [
                    label,
                    f"{med:+.1f}%",
                    f"{low[1]} ({low[0]:+.1f}%)",
                    f"{high[1]} ({high[0]:+.1f}%)",
                ]
            )
        )


def write_report(
    *,
    started_at: str,
    elapsed_s: float,
    vehicle_context: dict[str, Any],
    candidates: list[TireCandidate],
    rows: list[dict[str, Any]],
    group_rows: list[dict[str, Any]],
    standardsim_errors: list[dict[str, Any]],
    skip_standardsim: bool,
    standard_max_ay_mps2: float,
) -> None:
    reference = next((row for row in rows if row["candidate_id"] == "current_hybrid_reference"), None)
    integrated_top = top_rows(rows, "integrated_design_score", 12)
    stable_top = sorted(
        clean_positive_understeer_rows(rows),
        key=lambda row: finite_float(row.get("integrated_design_score")),
        reverse=True,
    )
    relaxation_top = top_rows(rows, "sigma_alpha_m", 12, reverse=False)
    family_relaxation_best = best_by_family_for_metric(rows, "sigma_alpha_m", reverse=False)
    stable_response_top = sorted(
        clean_positive_understeer_rows(rows),
        key=lambda row: (
            -finite_float(row.get("integrated_design_score")),
            finite_float(row.get("sigma_alpha_m")),
        ),
    )[:12]
    zero_radius_rows = sorted(
        [
            row
            for row in rows
            if int(row["is_reference"]) == 0
            and abs(finite_float(row.get("architecture_radius_delta_mm"))) <= 0.5
            and math.isfinite(finite_float(row.get("integrated_design_score")))
        ],
        key=lambda row: finite_float(row.get("integrated_design_score")),
        reverse=True,
    )
    clean_zero_radius_rows = sorted(
        [
            row
            for row in clean_positive_understeer_rows(rows)
            if abs(finite_float(row.get("architecture_radius_delta_mm"))) <= 0.5
            and math.isfinite(finite_float(row.get("integrated_design_score")))
        ],
        key=lambda row: finite_float(row.get("integrated_design_score")),
        reverse=True,
    )
    current_package_rows = sorted(
        [
            row
            for row in clean_zero_radius_rows
            if str(row.get("tire_size")) == "16x7.5-10"
        ],
        key=lambda row: finite_float(row.get("integrated_design_score")),
        reverse=True,
    )
    hoosier_43075_rows = sorted(
        [row for row in rows if str(row.get("family")) == "Hoosier 43075"],
        key=lambda row: finite_float(row.get("integrated_design_score")),
        reverse=True,
    )
    temperature_rows = sorted(
        [
            row
            for row in rows
            if int(row["is_reference"]) == 0
            and math.isfinite(finite_float(row.get("transient_tread_mean_c")))
        ],
        key=lambda row: (str(row.get("family")), str(row.get("tire_size")), finite_float(row.get("rim_width_in")), finite_float(row.get("pressure_psi"))),
    )
    degradation_corner_rows = sorted(
        unique_degradation_setup_rows(rows, "degradation_corner_peak_mu_y_delta_pct"),
        key=lambda row: (str(row.get("family")), str(row.get("tire_size")), finite_float(row.get("rim_width_in"))),
    )
    degradation_drive_rows = sorted(
        unique_degradation_setup_rows(rows, "degradation_drive_peak_mu_x_delta_pct"),
        key=lambda row: (str(row.get("family")), str(row.get("tire_size")), finite_float(row.get("rim_width_in"))),
    )
    degradation_43075_rows = [
        row for row in degradation_corner_rows if str(row.get("family")) == "Hoosier 43075"
    ]
    standardsim_qa_fail_rows = sorted(
        [
            row
            for row in rows
            if int(row["is_reference"]) == 0
            and str(row.get("standardsim_status")) == "ok"
            and str(row.get("standardsim_quality_status", "ok")) != "ok"
        ],
        key=lambda row: row["label"],
    )
    envelope_top = top_rows(rows, "envelope_score", 10)
    standardsim_top = sorted(
        [
            row
            for row in rows
            if standardsim_row_is_scoreable(row)
            and math.isfinite(finite_float(row.get("standardsim_score")))
        ],
        key=lambda row: finite_float(row.get("standardsim_score")),
        reverse=True,
    )[:10]
    direct_rows = [
        row
        for row in rows
        if row.get("longitudinal_combined_source") == "direct_drive_brake"
        and standardsim_row_is_scoreable(row)
        and math.isfinite(finite_float(row.get("integrated_design_score")))
    ]
    scaled_rows = [
        row
        for row in rows
        if "scaled" in str(row.get("longitudinal_combined_source"))
        and standardsim_row_is_scoreable(row)
        and math.isfinite(finite_float(row.get("integrated_design_score")))
    ]

    lines: list[str] = []
    lines.append("# DS-006 Integrated Tire Design")
    lines.append("")
    lines.append(f"Generated UTC: {started_at}")
    lines.append("")
    lines.append("## Purpose")
    lines.append("")
    lines.append(
        "This study compares every Round 9 full-UM14 tire fit on the current vehicle architecture. "
        "Each tire variant updates tire radius, rim radius, rim width, vertical tire properties, "
        "and the chassis-layout z coordinates by the tire-radius delta from the reference tire. "
        "Each candidate is evaluated in EnvelopeSim and, unless explicitly skipped, StandardSim SteadyStateEval. "
        "The current hybrid tire is included as a non-candidate reference."
    )
    lines.append("")
    lines.append("## Source Of Results")
    lines.append("")
    lines.append(table_line(["Result type", "Source"]))
    lines.append(table_line(["---", "---"]))
    lines.append(table_line(["Envelope limits", "`BobSim/_2_EnvelopeSim/GGV/ggv_generation.py`"]))
    lines.append(table_line(["Steady-state vehicle response", "`BobSim/_3_StandardSim/SteadyStateEval/steady_state_eval_sim.py`"]))
    lines.append(table_line(["Tire fit diagnostics", "`vehicles/current/tires/round_9_fitted_full_um14/manifest.csv` and diagnostics CSV"]))
    lines.append(table_line(["Transient tire temperature", "`RunData_Cornering_Matlab_SI_Round9.zip` transient runs"]))
    lines.append(table_line(["Initial/final tire degradation", "`RunGuide_Round9.pdf` 12 psi repeats in the cornering and drive/brake archives"]))
    lines.append(table_line(["Report generator", "`studies/DS-006-integrated-tire-design/run.py`"]))
    lines.append("")
    lines.append("## Coverage")
    lines.append("")
    lines.append(table_line(["Item", "Value"]))
    lines.append(table_line(["---", "---:"]))
    lines.append(table_line(["Round 9 candidates", str(len(candidates) - 1)]))
    lines.append(table_line(["Reference tires", "1"]))
    lines.append(table_line(["EnvelopeSim cases", str(len(rows))]))
    ok_std = sum(1 for row in rows if row.get("standardsim_status") == "ok")
    lines.append(table_line(["StandardSim successful cases", str(ok_std)]))
    lines.append(table_line(["StandardSim errors", str(len(standardsim_errors))]))
    lines.append(table_line(["Vehicle mass", f"{format_float(vehicle_context['mass_kg'], 2)} kg"]))
    lines.append(table_line(["CG height", f"{format_float(vehicle_context['cg_z_m'], 3)} m"]))
    lines.append(table_line(["Reference tire radius", f"{format_float(vehicle_context['reference_unloaded_radius_m'], 4)} m"]))
    lines.append(table_line(["Front static fraction", f"{format_float(vehicle_context['front_static_frac'], 3)}"]))
    qa_ok = sum(
        1
        for row in rows
        if str(row.get("standardsim_status")) == "ok"
        and str(row.get("standardsim_quality_status", "ok")) == "ok"
    )
    qa_fail = sum(
        1
        for row in rows
        if str(row.get("standardsim_status")) == "ok"
        and str(row.get("standardsim_quality_status", "ok")) != "ok"
    )
    lines.append(table_line(["StandardSim QA-pass cases", str(qa_ok)]))
    lines.append(table_line(["StandardSim QA-fail cases", str(qa_fail)]))
    lines.append("")
    lines.append("## Architecture Adjustment")
    lines.append("")
    lines.append(
        "Tire OD is treated as a real vehicle architecture input, not just a tire-file swap. "
        "For each candidate, the tire `UNLOADED_RADIUS`, `RIM_RADIUS`, `RIM_WIDTH`, vertical stiffness, "
        "and vertical damping are pushed into the StandardSim vehicle record. The chassis-layout z coordinate "
        "fields are translated upward by `candidate UNLOADED_RADIUS - reference UNLOADED_RADIUS`, so the "
        "sprung/driver/unsprung mass locations, suspension points, wheel centers, and ride-height reference "
        "points all move consistently with the tire package."
    )
    lines.append("")
    lines.append(
        "EnvelopeSim does not carry the full suspension geometry, so the same architecture correction is "
        "represented by increasing the GGV vehicle CG height by the tire-radius delta before generating "
        "the candidate envelope."
    )
    lines.append("")
    size_arch: dict[str, dict[str, Any]] = {}
    for row in rows:
        if int(row["is_reference"]) == 1:
            continue
        size_arch.setdefault(str(row.get("tire_size")), row)
    lines.append(table_line(["Tire size", "Radius", "Radius delta", "Envelope CG height", "CG delta"]))
    lines.append(table_line(["---", "---:", "---:", "---:", "---:"]))
    for tire_size, row in sorted(size_arch.items(), key=lambda item: finite_float(item[1].get("architecture_unloaded_radius_m"))):
        lines.append(
            table_line(
                [
                    tire_size,
                    f"{format_float(row.get('architecture_unloaded_radius_m'), 4)} m",
                    f"{format_float(row.get('architecture_radius_delta_mm'), 1)} mm",
                    f"{format_float(row.get('architecture_cg_height_m'), 4)} m",
                    f"{format_float(row.get('architecture_cg_height_delta_pct'), 1)}%",
                ]
            )
        )
    lines.append("")
    lines.append("## Tire Fit Pedigree")
    lines.append("")
    lines.append(
        "All Round 9 candidate records resolve to generated full-UM14 PAC2002 `.tir` files "
        "and were renderable into StandardSim vehicle records. The remaining tire-fit caveats "
        "are provenance and fit quality, not missing or undefined TIR fields."
    )
    lines.append("")
    lines.append("Longitudinal/combined-source counts:")
    lines.append("")
    for source, count in source_counts(rows, "longitudinal_combined_source").items():
        lines.append(f"- `{source}`: `{count}`")
    lines.append("")
    lines.append("Lateral-relaxation-source counts:")
    lines.append("")
    for source, count in source_counts(rows, "lateral_relaxation_source").items():
        lines.append(f"- `{source}`: `{count}`")
    lines.append("")
    lines.append(table_line(["Fit metric", "Median", "P90", "Count"]))
    lines.append(table_line(["---", "---:", "---:", "---:"]))
    for metric, label, unit in (
        ("lateral_nrmse", "Lateral force NRMSE", "%"),
        ("longitudinal_nrmse", "Longitudinal force NRMSE", "%"),
        ("fx_combined_nrmse", "Combined Fx NRMSE", "%"),
        ("fy_combined_nrmse", "Combined Fy NRMSE", "%"),
        ("lateral_relaxation_sigma_rmse_m", "Lateral relaxation sigma RMSE", "m"),
    ):
        values = [finite_float(row.get(metric)) for row in rows if int(row["is_reference"]) == 0]
        values = [value for value in values if math.isfinite(value)]
        if unit == "%":
            med_text = format_percent(median(values))
            p90_text = format_percent(percentile(values, 90))
        else:
            med_text = f"{format_float(median(values), 3)} {unit}"
            p90_text = f"{format_float(percentile(values, 90), 3)} {unit}"
        lines.append(table_line([label, med_text, p90_text, str(len(values))]))
    lines.append("")
    lines.append("## Scoring Method")
    lines.append("")
    lines.append(
        "Envelope score is a candidate-normalized weighted score: mean lateral g 35%, "
        "25 m/s lateral g 20%, mean GGV area 25%, mean acceleration 10%, mean braking 10%."
    )
    lines.append(
        f"StandardSim score is evaluated in a deliberately stable `{format_float(standard_max_ay_mps2, 1)} m/s^2` "
        "ramp-steer window. It scores closeness to +0.5 deg/g understeer 40%, "
        "closeness to +1.0 deg/g roll gradient 25%, low absolute sideslip gradient 20%, "
        "and low peak handwheel torque 15%. StandardSim `ay_max` is retained only as a "
        "measured-ramp diagnostic; it is not scored and is not used as the tire limit."
    )
    lines.append(
        "Integrated design score is 65% EnvelopeSim score and 35% StandardSim score. "
        "This score is a transparent decision aid, not a replacement for reviewing the raw response metrics."
    )
    lines.append(
        "Response flags mark StandardSim outliers: absolute understeer gradient above 5 deg/g, "
        "absolute sideslip gradient above 5 deg/g, non-positive roll gradient, or absolute roll "
        "gradient above 5 deg/g. Flagged rows are still shown because they are findings, but they "
        "should be treated as stability/fit-review warnings rather than clean design wins."
    )
    lines.append(
        "StandardSim QA is a hard score gate. Any row with failed maneuver speeds, missing QA metrics, "
        "wrong metric-source velocity, or excessive fit/noise diagnostics is retained in the tables but "
        "excluded from StandardSim and integrated scoring."
    )
    lines.append("")
    lines.append("## Recommendation")
    lines.append("")
    if integrated_top:
        winner = integrated_top[0]
        lines.append(
            f"Integrated first choice: **{winner['label']}** with integrated score "
            f"`{format_float(winner['integrated_design_score'], 3)}`."
        )
        lines.append("")
        lines.append(
            f"It combines EnvelopeSim rank `{winner['envelope_rank']}` with stable-window StandardSim score "
            f"`{format_float(winner.get('standardsim_score'), 3)}`, understeer gradient "
            f"`{format_float(winner.get('understeer_gradient_deg_per_g'), 3)} deg/g`, and roll gradient "
            f"`{format_float(winner.get('roll_gradient_deg_per_g'), 3)} deg/g`."
        )
        if reference:
            lines.append("")
            comparison = (
                "Against the current reference, the winner changes mean EnvelopeSim lateral capability by "
                f"`{format_delta(winner.get('mean_max_lateral_g'), reference.get('mean_max_lateral_g'))}`, "
                f"understeer gradient by `{format_delta(winner.get('understeer_gradient_deg_per_g'), reference.get('understeer_gradient_deg_per_g'))}`, "
                f"peak handwheel torque by `{format_delta(winner.get('handwheel_torque_peak_abs'), reference.get('handwheel_torque_peak_abs'))}`, "
                f"and mean GGV area by `{format_delta(winner.get('mean_ggv_area_g2'), reference.get('mean_ggv_area_g2'))}`."
            )
            lines.append(comparison)
        if stable_top:
            stable = stable_top[0]
            lines.append("")
            lines.append(
                f"Best clean positive-understeer stability finalist: **{stable['label']}** "
                f"with integrated rank `{stable['integrated_rank']}`, StandardSim score "
                f"`{format_float(stable.get('standardsim_score'), 3)}`, understeer gradient "
                f"`{format_float(stable.get('understeer_gradient_deg_per_g'), 3)} deg/g`, "
                f"and roll gradient `{format_float(stable.get('roll_gradient_deg_per_g'), 3)} deg/g`."
            )
    elif skip_standardsim:
        lines.append("StandardSim was skipped, so this run cannot issue an integrated tire recommendation.")
    else:
        lines.append("No integrated winner could be selected because StandardSim did not return successful tire cases.")
    lines.append("")
    if direct_rows and scaled_rows:
        best_direct = max(direct_rows, key=lambda row: finite_float(row.get("integrated_design_score")))
        best_scaled = max(scaled_rows, key=lambda row: finite_float(row.get("integrated_design_score")))
        lines.append(
            f"Best direct longitudinal/combined-fit tire: **{best_direct['label']}** "
            f"(`{format_float(best_direct.get('integrated_design_score'), 3)}`)."
        )
        lines.append(
            f"Best scaled-longitudinal/combined tire: **{best_scaled['label']}** "
            f"(`{format_float(best_scaled.get('integrated_design_score'), 3)}`)."
        )
        lines.append("")
    if hoosier_43075_rows:
        best_43075 = hoosier_43075_rows[0]
        fastest_43075 = min(
            hoosier_43075_rows,
            key=lambda row: finite_float(row.get("sigma_alpha_m")),
        )
        lines.append(
            f"Practical common-tire lens: **{best_43075['label']}** is the best Hoosier 43075 by "
            f"integrated score (`{format_float(best_43075.get('integrated_design_score'), 3)}`), "
            f"while **{fastest_43075['label']}** is the fastest-relaxing 43075 "
            f"(`sigma_alpha = {format_float(fastest_43075.get('sigma_alpha_m'), 3)} m`)."
        )
        lines.append(
            "That is the important 43075 story: it is not selected by popularity or by assuming "
            "it must win. It earns consideration only after the decision is constrained to the "
            "current zero-radius-delta 16x7.5-10 vehicle package, where it is a clean, stable "
            "candidate family with no response flags across its tested pressure/rim set."
        )
        if degradation_43075_rows:
            worst_43075_mu = min(
                degradation_43075_rows,
                key=lambda row: finite_float(row.get("degradation_corner_peak_mu_y_delta_pct")),
            )
            worst_43075_ky = min(
                degradation_43075_rows,
                key=lambda row: finite_float(row.get("degradation_corner_ky_delta_pct")),
            )
            lines.append(
                "The raw initial/final tire-data check also behaves nicely for this family: "
                f"the worst 43075 lateral peak-mu change is `{format_pct_point(worst_43075_mu.get('degradation_corner_peak_mu_y_delta_pct'))}`, "
                f"and the worst cornering-stiffness change is `{format_pct_point(worst_43075_ky.get('degradation_corner_ky_delta_pct'))}` "
                "over the repeated 12 psi sweep."
            )
        lines.append("")
    lines.append("## Architecture-Constrained Read")
    lines.append("")
    lines.append(
        "The tire selection must be read as a set of nested design problems. The all-tire "
        "ranking answers `what wins after the current architecture correction is applied?`; the "
        "zero-radius-delta and 16x7.5-10 tables answer `what should be selected if the current "
        "architecture is the target?`. This prevents the 43075 from being magically selected while "
        "also preventing larger tires from receiving free packaging credit."
    )
    lines.append("")
    boundary_rows = []
    if integrated_top:
        boundary_rows.append(("Unconstrained integrated leader", integrated_top[0]))
    if stable_top:
        boundary_rows.append(("Best clean all-tire finalist", stable_top[0]))
    if zero_radius_rows:
        boundary_rows.append(("Best zero-radius-delta candidate", zero_radius_rows[0]))
    if clean_zero_radius_rows:
        boundary_rows.append(("Best clean zero-radius-delta candidate", clean_zero_radius_rows[0]))
    if current_package_rows:
        boundary_rows.append(("Best clean 16x7.5-10 package candidate", current_package_rows[0]))
    if hoosier_43075_rows:
        boundary_rows.append(("Best Hoosier 43075 candidate", hoosier_43075_rows[0]))
    if boundary_rows:
        lines.append(table_line(["Decision lens", "Candidate", "dR", "Integrated", "Envelope", "Std", "Std ay diag [g]", "US grad", "Flags"]))
        lines.append(table_line(["---", "---", "---:", "---:", "---:", "---:", "---:", "---:", "---"]))
        for lens, row in boundary_rows:
            lines.append(
                table_line(
                    [
                        lens,
                        row["label"],
                        f"{format_float(row.get('architecture_radius_delta_mm'), 1)} mm",
                        format_float(row.get("integrated_design_score"), 3),
                        format_float(row.get("envelope_score"), 3),
                        format_float(row.get("standardsim_score"), 3),
                        format_float(row.get("ay_max_g"), 3),
                        format_float(row.get("understeer_gradient_deg_per_g"), 3),
                        row.get("response_flags", ""),
                    ]
                )
            )
        lines.append("")
    if current_package_rows:
        best_current_package = current_package_rows[0]
        lines.append(
            f"Within the clean zero-radius-delta 16x7.5-10 package, the leading candidate is "
            f"**{best_current_package['label']}**. That is the only lens under which a 43075 "
            "selection can be claimed from this study. If a 13 in or larger-OD tire remains the "
            "preferred direction after the radius/CG correction, the honest next step is a vehicle "
            "architecture study, not a tire-only selection."
        )
        lines.append("")
    lines.append("## Integrated Ranking")
    lines.append("")
    lines.append(table_line(["Rank", "Candidate", "dR", "Envelope rank", "Envelope score", "Std score", "Integrated", "Std ay diag [g]", "US grad", "Roll grad", "Resp flags", "Std QA", "Long source"]))
    lines.append(table_line(["---:", "---", "---:", "---:", "---:", "---:", "---:", "---:", "---:", "---:", "---", "---", "---"]))
    for row in integrated_top:
        lines.append(
            table_line(
                [
                    row["integrated_rank"],
                    row["label"],
                    f"{format_float(row.get('architecture_radius_delta_mm'), 1)} mm",
                    row["envelope_rank"],
                    format_float(row.get("envelope_score"), 3),
                    format_float(row.get("standardsim_score"), 3),
                    format_float(row.get("integrated_design_score"), 3),
                    format_float(row.get("ay_max_g"), 3),
                    format_float(row.get("understeer_gradient_deg_per_g"), 3),
                    format_float(row.get("roll_gradient_deg_per_g"), 3),
                    row.get("response_flags", ""),
                    row.get("standardsim_quality_flags", ""),
                    row.get("longitudinal_combined_source", ""),
                ]
            )
        )
    lines.append("")
    lines.append("## EnvelopeSim Findings")
    lines.append("")
    lines.append(table_line(["Rank", "Candidate", "dR", "CG height", "Score", "Mean lat", "25 m/s lat", "Mean area", "Mean accel", "Mean brake", "Fz excess"]))
    lines.append(table_line(["---:", "---", "---:", "---:", "---:", "---:", "---:", "---:", "---:", "---:", "---:"]))
    for row in envelope_top:
        lines.append(
            table_line(
                [
                    row["envelope_rank"],
                    row["label"],
                    f"{format_float(row.get('architecture_radius_delta_mm'), 1)} mm",
                    format_float(row.get("architecture_cg_height_m"), 3),
                    format_float(row.get("envelope_score"), 3),
                    format_float(row.get("mean_max_lateral_g"), 3),
                    format_float(row.get("max_lateral_g__25mps"), 3),
                    format_float(row.get("mean_ggv_area_g2"), 3),
                    format_float(row.get("mean_max_accel_g"), 3),
                    format_float(row.get("mean_max_brake_g"), 3),
                    f"{format_float(row.get('fz_max_excess_pct'), 1)}%",
                ]
            )
        )
    lines.append("")
    lines.append("EnvelopeSim pressure deltas compare 14 psi against 8 psi for the same family/size/rim.")
    lines.append("")
    append_delta_table(
        lines,
        rows,
        [
            ("mean_max_lateral_g", "Mean lateral g"),
            ("mean_ggv_area_g2", "Mean GGV area"),
            ("mean_max_accel_g", "Mean acceleration g"),
            ("mean_max_brake_g", "Mean braking g"),
        ],
        "pressure",
    )
    lines.append("")
    lines.append("## StandardSim Findings")
    lines.append("")
    lines.append(
        f"StandardSim uses a stable handling sweep with commanded maxAy `{format_float(standard_max_ay_mps2, 1)} m/s^2`; "
        "limit capability is intentionally judged by EnvelopeSim above."
    )
    lines.append("")
    if standardsim_top:
        lines.append(table_line(["Std rank", "Candidate", "Std score", "Std ay diag [g]", "US grad", "Sideslip grad", "Roll grad", "Handwheel torque", "Resp flags", "Std QA"]))
        lines.append(table_line(["---:", "---", "---:", "---:", "---:", "---:", "---:", "---:", "---", "---"]))
        for idx, row in enumerate(standardsim_top, start=1):
            lines.append(
                table_line(
                    [
                        idx,
                        row["label"],
                        format_float(row.get("standardsim_score"), 3),
                        format_float(row.get("ay_max_g"), 3),
                        format_float(row.get("understeer_gradient_deg_per_g"), 3),
                        format_float(row.get("sideslip_gradient_deg_per_g"), 3),
                        format_float(row.get("roll_gradient_deg_per_g"), 3),
                        format_float(row.get("handwheel_torque_peak_abs"), 3),
                        row.get("response_flags", ""),
                        row.get("standardsim_quality_flags", ""),
                    ]
                )
            )
    else:
        lines.append("No successful StandardSim tire cases were available.")
    lines.append("")
    lines.append("StandardSim pressure deltas compare 14 psi against 8 psi for the same family/size/rim.")
    lines.append("Gradient percentage deltas can be dominated by flagged outlier cases; use the row-level flags when judging stability.")
    lines.append("")
    append_delta_table(
        lines,
        rows,
        [
            ("standardsim_score", "StandardSim score"),
            ("understeer_gradient_deg_per_g", "Understeer gradient"),
            ("roll_gradient_deg_per_g", "Roll gradient"),
            ("handwheel_torque_peak_abs", "Peak handwheel torque"),
        ],
        "pressure",
    )
    lines.append("")
    lines.append("### StandardSim QA Gate")
    lines.append("")
    if standardsim_qa_fail_rows:
        lines.append(
            "The following rows produced StandardSim metrics but were excluded from integrated scoring "
            "because the steady-state evidence failed QA. This is intentional: noisy or partial "
            "steady-state reports are findings, not design winners."
        )
        lines.append("")
        lines.append(table_line(["Candidate", "QA flags", "Failures", "Metric V", "Roadwheel fit", "Steer-excess fit", "Mean rad err"]))
        lines.append(table_line(["---", "---", "---:", "---:", "---:", "---:", "---:"]))
        for row in standardsim_qa_fail_rows[:24]:
            lines.append(
                table_line(
                    [
                        row["label"],
                        row.get("standardsim_quality_flags", ""),
                        format_float(row.get("n_failed_cases"), 0),
                        format_float(row.get("metric_source_velocity_mps"), 1),
                        format_float(row.get("roadwheel_fit_nrmse"), 3),
                        format_float(row.get("steer_excess_fit_nrmse"), 3),
                        format_float(row.get("mean_abs_rad_error"), 3),
                    ]
                )
            )
        if len(standardsim_qa_fail_rows) > 24:
            lines.append(table_line([f"... {len(standardsim_qa_fail_rows) - 24} more", "", "", "", "", "", ""]))
    else:
        lines.append("Every successful StandardSim row passed the QA gate.")
    lines.append("")
    lines.append("## Relaxation And Response")
    lines.append("")
    lines.append(
        "`sigma_alpha_m` is the fitted lateral relaxation length from the Round 9 transient tire data. "
        "Lower values mean the tire builds lateral force over a shorter distance, so the car should feel "
        "more immediate for the same steady-state capability. This is a tire-fit diagnostic, not an "
        "EnvelopeSim or StandardSim response metric, so it is used here as a response/feel cross-check."
    )
    lines.append("")
    lines.append("Fastest fitted relaxation lengths:")
    lines.append("")
    lines.append(table_line(["Rank", "Candidate", "sigma_alpha", "Integrated", "Std ay diag [g]", "US grad", "Flags"]))
    lines.append(table_line(["---:", "---", "---:", "---:", "---:", "---:", "---"]))
    for idx, row in enumerate(relaxation_top, start=1):
        lines.append(
            table_line(
                [
                    idx,
                    row["label"],
                    format_float(row.get("sigma_alpha_m"), 3),
                    format_float(row.get("integrated_design_score"), 3),
                    format_float(row.get("ay_max_g"), 3),
                    format_float(row.get("understeer_gradient_deg_per_g"), 3),
                    row.get("response_flags", ""),
                ]
            )
        )
    lines.append("")
    lines.append("Best relaxation case by tire family:")
    lines.append("")
    lines.append(table_line(["Family", "Best case", "sigma_alpha", "Integrated", "Std ay diag [g]", "Flags"]))
    lines.append(table_line(["---", "---", "---:", "---:", "---:", "---"]))
    for row in family_relaxation_best:
        lines.append(
            table_line(
                [
                    row["family"],
                    row["label"],
                    format_float(row.get("sigma_alpha_m"), 3),
                    format_float(row.get("integrated_design_score"), 3),
                    format_float(row.get("ay_max_g"), 3),
                    row.get("response_flags", ""),
                ]
            )
        )
    lines.append("")
    lines.append("Clean positive-understeer response finalists:")
    lines.append("")
    lines.append(table_line(["Int rank", "Candidate", "Integrated", "sigma_alpha", "Std ay diag [g]", "US grad", "Roll grad"]))
    lines.append(table_line(["---:", "---", "---:", "---:", "---:", "---:", "---:"]))
    for row in stable_response_top:
        lines.append(
            table_line(
                [
                    row["integrated_rank"],
                    row["label"],
                    format_float(row.get("integrated_design_score"), 3),
                    format_float(row.get("sigma_alpha_m"), 3),
                    format_float(row.get("ay_max_g"), 3),
                    format_float(row.get("understeer_gradient_deg_per_g"), 3),
                    format_float(row.get("roll_gradient_deg_per_g"), 3),
                ]
            )
        )
    lines.append("")
    if hoosier_43075_rows:
        lines.append("Hoosier 43075 practical lens:")
        lines.append("")
        lines.append(
            "Within the common 16x7.5-10 Hoosier 43075 family, the 7in/8psi case maximizes "
            "vehicle-level score, while the wider-rim and higher-pressure cases reduce relaxation "
            "length at the cost of some EnvelopeSim capability. This creates a real tuning choice: "
            "7in/8psi for peak simulated vehicle score, or 8in/8psi to keep most of that score while "
            "cutting relaxation length substantially."
        )
        lines.append("")
        lines.append(table_line(["43075 case", "Integrated", "Envelope", "Std", "sigma_alpha", "Std ay diag [g]", "US grad", "Roll grad", "Flags"]))
        lines.append(table_line(["---", "---:", "---:", "---:", "---:", "---:", "---:", "---:", "---"]))
        for row in hoosier_43075_rows:
            lines.append(
                table_line(
                    [
                        row["label"],
                        format_float(row.get("integrated_design_score"), 3),
                        format_float(row.get("envelope_score"), 3),
                        format_float(row.get("standardsim_score"), 3),
                        format_float(row.get("sigma_alpha_m"), 3),
                        format_float(row.get("ay_max_g"), 3),
                        format_float(row.get("understeer_gradient_deg_per_g"), 3),
                        format_float(row.get("roll_gradient_deg_per_g"), 3),
                        row.get("response_flags", ""),
                    ]
                )
            )
        lines.append("")
    lines.append("## Transient Temperature Evidence")
    lines.append("")
    lines.append(
        "The Round 9 transient step-steer runs include tread inner/center/outer temperature channels "
        "(`TSTI`, `TSTC`, `TSTO`), rim surface temperature (`RST`), and ambient temperature (`AMBTMP`). "
        "These temperatures are not EnvelopeSim or StandardSim response outputs, so they are not part of "
        "the vehicle score. They are used as a test-condition and operating-window check for the tire data."
    )
    lines.append("")
    if temperature_rows:
        lines.append(
            "Transient temperature rows exist for the 10, 12, and 14 psi transient pressure windows. "
            "The 8 psi vehicle finalists do not have direct transient-temperature windows in this dataset, "
            "so their thermal behavior still needs track validation."
        )
        lines.append("")
        lines.append(table_line(["Candidate", "Pressure", "Mean tread", "Peak tread", "Rise", "I-C-O spread", "Inner-outer", "Rim", "Ambient"]))
        lines.append(table_line(["---", "---:", "---:", "---:", "---:", "---:", "---:", "---:", "---:"]))
        for row in temperature_rows:
            if str(row.get("family")) != "Hoosier 43075" and finite_float(row.get("pressure_psi")) not in (10.0, 12.0, 14.0):
                continue
            lines.append(
                table_line(
                    [
                        row["label"],
                        format_float(row.get("pressure_psi"), 0),
                        f"{format_float(row.get('transient_tread_mean_c'), 1)} C",
                        f"{format_float(row.get('transient_tread_peak_c'), 1)} C",
                        f"{format_float(row.get('transient_tread_rise_c'), 1)} C",
                        f"{format_float(row.get('transient_surface_spread_mean_c'), 1)} C",
                        f"{format_float(row.get('transient_tread_inner_minus_outer_c'), 1)} C",
                        f"{format_float(row.get('transient_rim_temp_mean_c'), 1)} C",
                        f"{format_float(row.get('transient_ambient_temp_mean_c'), 1)} C",
                    ]
                )
            )
    else:
        lines.append("No transient temperature rows could be extracted from the Round 9 transient runs.")
    lines.append("")
    lines.append("## Tire Degradation Evidence")
    lines.append("")
    lines.append(
        "The Round 9 RunGuide repeats the 12 psi slip-angle or slip-ratio sweep after the pressure "
        "sequence. DS-006 compares those initial and final 12 psi runs at the nominal 25 mph test "
        "speed window (`34-47 km/h`) using robust 95th-percentile measured force/load and a small-slip "
        "linear stiffness fit. This is raw tire-data evidence for wear, warmup, and run-order drift; "
        "it is not included in the EnvelopeSim, StandardSim, or integrated score."
    )
    lines.append("")
    if degradation_corner_rows:
        lines.append("Cornering degradation by tire/rim setup:")
        lines.append("")
        lines.append(table_line(["Setup", "Initial runs", "Final runs", "Peak mu_y delta", "Ky/Fz delta", "Tread delta", "Samples i/f"]))
        lines.append(table_line(["---", "---:", "---:", "---:", "---:", "---:", "---:"]))
        for row in degradation_corner_rows:
            lines.append(
                table_line(
                    [
                        degradation_label(row),
                        row.get("degradation_corner_initial_runs", ""),
                        row.get("degradation_corner_final_runs", ""),
                        format_pct_point(row.get("degradation_corner_peak_mu_y_delta_pct")),
                        format_pct_point(row.get("degradation_corner_ky_delta_pct")),
                        f"{format_float(row.get('degradation_corner_tread_delta_c'), 1)} C",
                        f"{format_float(row.get('degradation_corner_samples_initial'), 0)}/{format_float(row.get('degradation_corner_samples_final'), 0)}",
                    ]
                )
            )
        lines.append("")
    else:
        lines.append("No initial/final cornering degradation rows could be extracted.")
        lines.append("")
    if degradation_drive_rows:
        lines.append("Drive/brake degradation by tire/rim setup:")
        lines.append("")
        lines.append(table_line(["Setup", "Initial runs", "Final runs", "Peak mu_x delta", "Kx/Fz delta", "Tread delta", "Samples i/f"]))
        lines.append(table_line(["---", "---:", "---:", "---:", "---:", "---:", "---:"]))
        for row in degradation_drive_rows:
            lines.append(
                table_line(
                    [
                        degradation_label(row),
                        row.get("degradation_drive_initial_runs", ""),
                        row.get("degradation_drive_final_runs", ""),
                        format_pct_point(row.get("degradation_drive_peak_mu_x_delta_pct")),
                        format_pct_point(row.get("degradation_drive_kx_delta_pct")),
                        f"{format_float(row.get('degradation_drive_tread_delta_c'), 1)} C",
                        f"{format_float(row.get('degradation_drive_samples_initial'), 0)}/{format_float(row.get('degradation_drive_samples_final'), 0)}",
                    ]
                )
            )
        lines.append("")
    else:
        lines.append("No direct drive/brake initial/final degradation rows were available.")
        lines.append("")
    if degradation_43075_rows:
        lines.append("Hoosier 43075 degradation read:")
        lines.append("")
        lines.append(
            "Both 43075 rim setups show negligible measured lateral degradation in the repeated 12 psi "
            "cornering sweep. Direct drive/brake degradation is not available for the 16 inch Hoosiers "
            "because their longitudinal/combined fits are scaled from the 18 inch Hoosier donor data."
        )
        lines.append("")
        lines.append(table_line(["43075 setup", "Peak mu_y delta", "Ky/Fz delta", "Tread delta", "Drive/brake note"]))
        lines.append(table_line(["---", "---:", "---:", "---:", "---"]))
        for row in degradation_43075_rows:
            lines.append(
                table_line(
                    [
                        degradation_label(row),
                        format_pct_point(row.get("degradation_corner_peak_mu_y_delta_pct")),
                        format_pct_point(row.get("degradation_corner_ky_delta_pct")),
                        f"{format_float(row.get('degradation_corner_tread_delta_c'), 1)} C",
                        humanize_status(row.get("degradation_drive_status", "")),
                    ]
                )
            )
        lines.append("")
    lines.append("## Rim Effects")
    lines.append("")
    lines.append("Rim deltas compare the widest fitted rim against the narrowest fitted rim for the same family/size/pressure.")
    lines.append("")
    append_delta_table(
        lines,
        rows,
        [
            ("mean_max_lateral_g", "Envelope mean lateral g"),
            ("standardsim_score", "StandardSim score"),
            ("understeer_gradient_deg_per_g", "Understeer gradient"),
            ("sigma_alpha_m", "Tire lateral relaxation sigma"),
            ("integrated_design_score", "Integrated score"),
        ],
        "rim",
    )
    lines.append("")
    lines.append("## Group Reads")
    lines.append("")
    lines.append(table_line(["Group", "Value", "N", "Median env score", "Median std score", "Median integrated", "Best candidate"]))
    lines.append(table_line(["---", "---", "---:", "---:", "---:", "---:", "---"]))
    for row in group_rows:
        if row["group_field"] in {"family", "tire_size", "rim_width_in", "pressure_psi", "longitudinal_combined_source"}:
            lines.append(
                table_line(
                    [
                        row["group_field"],
                        row["group_value"],
                        row["n"],
                        format_float(row["mean_envelope_score"], 3),
                        format_float(row["mean_standardsim_score"], 3),
                        format_float(row["mean_integrated_score"], 3),
                        row["best_label"],
                    ]
                )
            )
    lines.append("")
    if reference:
        lines.append("## Current Reference Comparison")
        lines.append("")
        lines.append(table_line(["Metric", "Reference"]))
        lines.append(table_line(["---", "---:"]))
        for metric, label in (
            ("mean_max_lateral_g", "Envelope mean lateral g"),
            ("max_lateral_g__25mps", "Envelope 25 m/s lateral g"),
            ("mean_ggv_area_g2", "Envelope mean GGV area"),
            ("ay_max_g", "StandardSim ay diagnostic [g]"),
            ("understeer_gradient_deg_per_g", "StandardSim understeer gradient"),
            ("roll_gradient_deg_per_g", "StandardSim roll gradient"),
            ("handwheel_torque_peak_abs", "StandardSim peak handwheel torque"),
        ):
            lines.append(table_line([label, format_float(reference.get(metric), 3)]))
        lines.append("")
    lines.append("## Complete Candidate Result Table")
    lines.append("")
    lines.append(
        "This table contains every candidate in the integrated study. The reference tire is listed separately above."
    )
    lines.append("")
    lines.append(table_line(["Int rank", "Env rank", "Candidate", "dR", "Pressure", "Rim", "mu_y", "mu_x", "sigma_a", "Lat mu degr", "Lat Ky degr", "Drive mu degr", "Env score", "Std ay diag [g]", "US grad", "Roll", "Std score", "Integrated", "Resp flags", "Std QA", "Long source"]))
    lines.append(table_line(["---:", "---:", "---", "---:", "---:", "---:", "---:", "---:", "---:", "---:", "---:", "---:", "---:", "---:", "---:", "---:", "---:", "---:", "---", "---", "---"]))
    sorted_all = sorted(
        [row for row in rows if int(row["is_reference"]) == 0],
        key=lambda row: (
            math.inf if row.get("integrated_rank") == "" else int(row["integrated_rank"]),
            math.inf if row.get("envelope_rank") == "" else int(row["envelope_rank"]),
        ),
    )
    for row in sorted_all:
        lines.append(
            table_line(
                [
                    row.get("integrated_rank", ""),
                    row.get("envelope_rank", ""),
                    row["label"],
                    format_float(row.get("architecture_radius_delta_mm"), 1),
                    format_float(row.get("pressure_psi"), 0),
                    format_float(row.get("rim_width_in"), 0),
                    format_float(row.get("mu_y"), 3),
                    format_float(row.get("mu_x"), 3),
                    format_float(row.get("sigma_alpha_m"), 3),
                    format_pct_point(row.get("degradation_corner_peak_mu_y_delta_pct")),
                    format_pct_point(row.get("degradation_corner_ky_delta_pct")),
                    format_pct_point(row.get("degradation_drive_peak_mu_x_delta_pct")),
                    format_float(row.get("envelope_score"), 3),
                    format_float(row.get("ay_max_g"), 3),
                    format_float(row.get("understeer_gradient_deg_per_g"), 3),
                    format_float(row.get("roll_gradient_deg_per_g"), 3),
                    format_float(row.get("standardsim_score"), 3),
                    format_float(row.get("integrated_design_score"), 3),
                    row.get("response_flags", ""),
                    row.get("standardsim_quality_flags", ""),
                    row.get("longitudinal_combined_source", ""),
                ]
            )
        )
    lines.append("")
    lines.append("## Caveats")
    lines.append("")
    lines.append("- EnvelopeSim uses the tire pure-slip/load coefficients and the current vehicle-level assumptions; it is excellent for capability screening, not final handling sign-off.")
    lines.append("- StandardSim SteadyStateEval captures vehicle response and balance, but it is still a steady-state maneuver set; transient feel should be checked separately before final tire sign-off.")
    lines.append("- Larger tire candidates are adjusted for radius-driven CG/ride-height changes, but they are not fully re-optimized for suspension kinematics, aero map quality, gearing, packaging, or wheel/brake hardware.")
    lines.append("- The 16 inch Hoosier candidates use scaled 18 inch Hoosier longitudinal/combined fits, so their longitudinal/braking conclusions carry less confidence than direct drive/brake Round 9 candidates.")
    lines.append("- Longitudinal relaxation is still estimated because the Round 9 data does not include an equivalent slip-ratio transient run.")
    lines.append("")
    lines.append("## Figure Gallery")
    lines.append("")
    lines.append("These figures are generated from the same DS-006 outputs summarized above.")
    lines.append("")
    for caption, relative in [
        ("Integrated score ranking across scoreable tire candidates", "integrated_score_rank.png"),
        ("Current-package zero-radius-delta integrated ranking", "current_package_integrated_rank.png"),
        ("Best scoreable setup by tire family", "family_best_score_comparison.png"),
        ("Fitted peak lateral mu versus cornering stiffness", "tire_fit_mu_stiffness_map.png"),
        ("Peak lateral mu and cornering stiffness by candidate", "tire_fit_mu_stiffness_rank.png"),
        ("EnvelopeSim tire ranking", "envelope_score_rank.png"),
        ("StandardSim stable-window handling ranking", "standardsim_score_rank.png"),
        ("Integrated tire trade space", "integrated_trade_space.png"),
        ("Pressure trends in EnvelopeSim and StandardSim score", "pressure_trends_vehicle_metrics.png"),
        ("Lateral relaxation length ranking", "relaxation_rank.png"),
        ("Relaxation length versus vehicle-level performance", "relaxation_trade_space.png"),
        ("Transient tire-temperature evidence", "transient_temperature.png"),
        ("Initial-to-final 12 psi cornering degradation", "tire_degradation_cornering.png"),
        ("Initial-to-final 12 psi drive/brake degradation", "tire_degradation_drive_brake.png"),
    ]:
        lines.append(f"### {caption}")
        lines.append("")
        lines.append(markdown_image(caption, relative))
        lines.append("")
    if standardsim_errors:
        lines.append("## StandardSim Errors")
        lines.append("")
        lines.append(table_line(["Candidate", "Error"]))
        lines.append(table_line(["---", "---"]))
        for error in standardsim_errors:
            lines.append(table_line([error.get("label", ""), error.get("error", "")]))
        lines.append("")
    lines.append("## Generated Files")
    lines.append("")
    for relative in [
        "outputs/candidate_registry.csv",
        "outputs/tire_characterization.csv",
        "outputs/transient_temperature_summary.csv",
        "outputs/degradation_summary.csv",
        "outputs/envelope_metrics.csv",
        "outputs/standardsim_metrics.csv",
        "outputs/standardsim_errors.csv",
        "outputs/integrated_results.csv",
        "outputs/group_summary.csv",
        "outputs/run_provenance.csv",
        "plots/integrated_score_rank.png",
        "plots/envelope_score_rank.png",
        "plots/standardsim_score_rank.png",
        "plots/current_package_integrated_rank.png",
        "plots/family_best_score_comparison.png",
        "plots/tire_fit_mu_stiffness_map.png",
        "plots/tire_fit_mu_stiffness_rank.png",
        "plots/pressure_trends_vehicle_metrics.png",
        "plots/integrated_trade_space.png",
        "plots/relaxation_rank.png",
        "plots/relaxation_trade_space.png",
        "plots/transient_temperature.png",
        "plots/tire_degradation_cornering.png",
        "plots/tire_degradation_drive_brake.png",
    ]:
        lines.append(f"- `{relative}`")
    lines.append("")
    lines.append("## Run Provenance")
    lines.append("")
    lines.append(table_line(["Item", "Value"]))
    lines.append(table_line(["---", "---"]))
    lines.append(table_line(["Elapsed time", f"{elapsed_s:.1f} s"]))
    lines.append(table_line(["Python", sys.executable]))
    lines.append(table_line(["StandardSim skipped", str(skip_standardsim)]))
    lines.append("")

    text = "\n".join(lines)
    study_text = text.replace(PLOT_PREFIX_TOKEN, "plots")
    report_text = text.replace(PLOT_PREFIX_TOKEN, "../studies/DS-006-integrated-tire-design/plots")
    (STUDY_DIR / "RESULTS.md").write_text(study_text, encoding="utf-8")
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(report_text, encoding="utf-8")


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
        help="Only run EnvelopeSim and tire-fit diagnostics.",
    )
    parser.add_argument(
        "--standardsim-limit",
        type=int,
        default=None,
        help="Debug limit for StandardSim cases. Omit for every tire plus reference.",
    )
    parser.add_argument(
        "--standardsim-workers",
        type=int,
        default=1,
        help="Number of StandardSim tire cases to run concurrently. Use 1 for safest serial execution.",
    )
    parser.add_argument(
        "--standardsim-max-ay",
        type=float,
        default=STANDARD_STABLE_MAX_AY_MPS2,
        help=(
            "Commanded StandardSim SteadyStateEval maxAy in m/s^2. "
            "DS-006 defaults to a stable handling window; EnvelopeSim carries limit capability."
        ),
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
    WORK_DIR.mkdir(parents=True, exist_ok=True)

    candidates = discover_candidates()
    transient_temperature_rows = extract_transient_temperature_rows(candidates)
    degradation_rows = extract_tire_degradation_rows(candidates)
    print("DS-006 Integrated Tire Design", flush=True)
    print(f"  Round 9 candidates: {len(candidates) - 1}", flush=True)
    print(f"  Reference: {candidates[0].label}", flush=True)
    print(f"  Transient temperature rows: {sum(1 for row in transient_temperature_rows if row.get('temperature_status') == 'ok')}", flush=True)
    print(f"  Degradation rows: {sum(1 for row in degradation_rows if row.get('degradation_status') in {'ok', 'partial'})}", flush=True)

    reference_radius_m = current_reference_radius_m()
    base_vehicle, vehicle_context = build_baseline_vehicle(args.vehicle, CURRENT_TIRE)
    vehicle_context["reference_unloaded_radius_m"] = reference_radius_m
    config = make_config()
    envelope_rows, characterization_rows = run_envelopes(
        base_vehicle,
        config,
        candidates,
        reference_radius_m,
    )

    standardsim_rows: list[dict[str, Any]] = []
    standardsim_errors: list[dict[str, Any]] = []
    if not args.skip_standardsim:
        standardsim_rows, standardsim_errors = run_standardsim(
            args.vehicle,
            candidates,
            reuse=args.reuse,
            limit=args.standardsim_limit,
            reference_radius_m=reference_radius_m,
            workers=args.standardsim_workers,
            standard_max_ay_mps2=args.standardsim_max_ay,
            standard_case_timeout_s=args.standardsim_case_timeout,
        )

    rows = merge_results(envelope_rows, characterization_rows, standardsim_rows, transient_temperature_rows, degradation_rows)
    group_rows = group_summary(rows)
    plot_outputs(rows)

    write_csv(
        OUTPUT_DIR / "candidate_registry.csv",
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
                "path": as_repo_path(candidate.path),
                "longitudinal_combined_source": candidate.longitudinal_combined_source,
                "lateral_relaxation_source": candidate.lateral_relaxation_source,
                "notes": candidate.notes,
            }
            for candidate in candidates
        ],
    )
    write_csv(OUTPUT_DIR / "tire_characterization.csv", characterization_rows)
    write_csv(OUTPUT_DIR / "transient_temperature_summary.csv", transient_temperature_rows)
    write_csv(OUTPUT_DIR / "degradation_summary.csv", degradation_rows)
    write_csv(OUTPUT_DIR / "envelope_metrics.csv", envelope_rows)
    write_csv(OUTPUT_DIR / "standardsim_metrics.csv", standardsim_rows)
    write_csv(OUTPUT_DIR / "standardsim_errors.csv", standardsim_errors)
    write_csv(OUTPUT_DIR / "integrated_results.csv", rows)
    write_csv(OUTPUT_DIR / "group_summary.csv", group_rows)
    write_csv(
        OUTPUT_DIR / "run_provenance.csv",
        [
            {"item": "generated_at_utc", "value": started_at},
            {"item": "vehicle", "value": as_repo_path(args.vehicle)},
            {"item": "current_reference", "value": as_repo_path(CURRENT_TIRE)},
            {"item": "round9_tire_dir", "value": as_repo_path(ROUND9_TIRE_DIR)},
            {"item": "round9_cornering_archive", "value": as_repo_path(ROUND9_CORNERING_ARCHIVE)},
            {"item": "round9_drive_archive", "value": as_repo_path(ROUND9_DRIVE_ARCHIVE)},
            {"item": "round9_candidates", "value": len(candidates) - 1},
            {"item": "transient_temperature_rows", "value": sum(1 for row in transient_temperature_rows if row.get("temperature_status") == "ok")},
            {"item": "degradation_rows", "value": sum(1 for row in degradation_rows if row.get("degradation_status") in {"ok", "partial"})},
            {"item": "degradation_pressure_psi", "value": DEGRADATION_PRESSURE_PSI},
            {"item": "envelopesim_speeds_mps", "value": ";".join(str(v) for v in config.speeds)},
            {"item": "standardsim_skipped", "value": args.skip_standardsim},
            {"item": "standardsim_limit", "value": args.standardsim_limit},
            {"item": "standardsim_workers", "value": args.standardsim_workers},
            {"item": "standardsim_max_ay_mps2", "value": args.standardsim_max_ay},
            {"item": "standardsim_case_timeout_s", "value": args.standardsim_case_timeout},
            {"item": "reuse", "value": args.reuse},
            {"item": "mass_kg", "value": vehicle_context["mass_kg"]},
            {"item": "cg_z_m", "value": vehicle_context["cg_z_m"]},
            {"item": "reference_unloaded_radius_m", "value": reference_radius_m},
            {"item": "architecture_adjustment", "value": "tire radius/rim dimensions plus vehicle coordinate z shift by tire radius delta"},
        ],
    )

    elapsed_s = time.perf_counter() - start
    write_report(
        started_at=started_at,
        elapsed_s=elapsed_s,
        vehicle_context=vehicle_context,
        candidates=candidates,
        rows=rows,
        group_rows=group_rows,
        standardsim_errors=standardsim_errors,
        skip_standardsim=args.skip_standardsim,
        standard_max_ay_mps2=args.standardsim_max_ay,
    )
    print(f"Study report: {STUDY_DIR / 'RESULTS.md'}", flush=True)
    print(f"Top-level report: {REPORT_PATH}", flush=True)


if __name__ == "__main__":
    main()
