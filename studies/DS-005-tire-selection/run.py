#!/usr/bin/env python3
"""Run DS-005: tire selection study."""

from __future__ import annotations

import argparse
import csv
import dataclasses
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


REPO_ROOT = Path(__file__).resolve().parents[2]
STUDY_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = STUDY_DIR / "outputs"
PLOT_DIR = STUDY_DIR / "plots"
WORK_DIR = STUDY_DIR / "work"
REPORT_PATH = REPO_ROOT / "reports" / "DS-005-tire-selection.md"

CURRENT_TIRE = REPO_ROOT / "vehicles" / "current" / "tires" / "16x7p5_10_12psi.tir"
ROUND8_TIRE_DIR = (
    REPO_ROOT / "vehicles" / "current" / "tires" / "round_8_fabricated_longitudinal_um3"
)

MPLCONFIGDIR = Path("/tmp/lhre-sim-matplotlib")
MPLCONFIGDIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(MPLCONFIGDIR))

ENVELOPE_SCORE_METRICS = (
    ("mean_max_lateral_g", 0.45),
    ("max_lateral_g__25mps", 0.30),
    ("mean_ggv_area_g2", 0.15),
    ("ggv_area_g2__25mps", 0.10),
)

REPORT_METRICS = (
    "mean_max_lateral_g",
    "max_lateral_g__25mps",
    "mean_ggv_area_g2",
    "ggv_area_g2__25mps",
    "mean_max_accel_g",
    "mean_max_brake_g",
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
            "  /tmp/lhre-sim-venv/bin/python studies/DS-005-tire-selection/run.py"
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
    "bobsim_envelopesim_ggv_ds005",
    REPO_ROOT / "BobSim" / "_2_EnvelopeSim" / "GGV" / "ggv_generation.py",
)
G = float(ggv.G)


@dataclass(frozen=True)
class TireCandidate:
    candidate_id: str
    label: str
    path: Path
    source: str
    compound: str
    tire_size: str
    rim: str
    pressure_psi: float
    is_candidate: bool
    notes: str


def as_repo_path(path: Path) -> str:
    try:
        return str(path.relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def slugify(value: str) -> str:
    value = re.sub(r"[^A-Za-z0-9]+", "_", value).strip("_")
    return value or "value"


def format_float(value: Any, digits: int = 4) -> str:
    try:
        value = float(value)
    except (TypeError, ValueError):
        return "nan"
    if not math.isfinite(value):
        return "nan"
    return f"{value:.{digits}f}"


def format_percent_delta(value: Any, reference: Any, digits: int = 1) -> str:
    try:
        value = float(value)
        reference = float(reference)
    except (TypeError, ValueError):
        return "nan"
    if not math.isfinite(value) or not math.isfinite(reference) or abs(reference) <= 1e-12:
        return "nan"
    return f"{100.0 * (value - reference) / reference:+.{digits}f}%"


def read_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"Expected YAML mapping in {path}")
    return data


FIELD_RE = re.compile(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.*?)\s*(?:[$!].*)?$")


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
    value = params.get(key.upper(), default)
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


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
        "mass_cg_x_m": float(cg[0]),
        "mass_cg_y_m": float(cg[1]),
        "mass_cg_z_m": float(cg[2]),
        "assumptions": assumptions,
    }
    return vehicle, context


def vehicle_with_tire(base_vehicle: Any, tire_path: Path) -> Any:
    tire = parse_tir(tire_path)
    params = dataclasses.asdict(base_vehicle)
    params.update(
        {
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
        ay_max_g=3.4,
        ay_points=71,
        ax_search_min_g=-3.4,
        ax_search_max_g=2.8,
        ax_search_points=181,
        include_left_right=True,
        verbose=False,
        progress_every=25,
        warn_tire_load_range=False,
    )


def speed_label(speed: float) -> str:
    if float(speed).is_integer():
        return f"{int(speed):02d}mps"
    return f"{speed:.1f}mps".replace(".", "p")


def extract_metrics(envelopes: list[Any]) -> dict[str, float]:
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

        max_lateral = (
            float(np.nanmax(np.abs(ay_g[finite_any]))) if np.any(finite_any) else math.nan
        )
        max_accel = (
            float(np.nanmax(ax_accel_g[finite_accel]))
            if np.any(finite_accel)
            else math.nan
        )
        max_brake = (
            float(abs(np.nanmin(ax_brake_g[finite_brake])))
            if np.any(finite_brake)
            else math.nan
        )

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
        metrics[f"mean_{key}"] = (
            float(np.nanmean(arr)) if np.any(np.isfinite(arr)) else math.nan
        )
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


def metadata_from_round8_path(path: Path) -> tuple[str, str, str, float]:
    stem = path.stem.replace("_fabricated_longitudinal_UM3", "")
    parts = stem.split("_")
    compound = ""
    tire_size = ""
    rim = ""
    pressure = math.nan
    for idx, part in enumerate(parts):
        if part in {"LC0", "R25B"}:
            compound = part
            tire_size = "_".join(parts[idx + 1 : idx + 3])
        if part == "on" and idx + 1 < len(parts):
            rim = parts[idx + 1]
        if part.endswith("psi"):
            try:
                pressure = float(part.removesuffix("psi"))
            except ValueError:
                pressure = math.nan
    return compound, tire_size, rim, pressure


def discover_candidates() -> list[TireCandidate]:
    candidates: list[TireCandidate] = [
        TireCandidate(
            candidate_id="current_hybrid_reference",
            label="current hybrid reference",
            path=CURRENT_TIRE,
            source="current_reference",
            compound="R20_hybrid",
            tire_size="16x7p5_10",
            rim="7in",
            pressure_psi=12.0,
            is_candidate=False,
            notes="Current hybrid 16x7.5 R20 on 7in rim, USE_MODE 14",
        )
    ]

    for path in sorted(ROUND8_TIRE_DIR.glob("*.tir")):
        compound, tire_size, rim, pressure = metadata_from_round8_path(path)
        label = f"{compound} {tire_size} on {rim} {format_float(pressure, 0)} psi"
        candidates.append(
            TireCandidate(
                candidate_id=slugify(path.stem.replace("_fabricated_longitudinal_UM3", "")),
                label=label,
                path=path,
                source="round8_fabricated_longitudinal_um3",
                compound=compound,
                tire_size=tire_size,
                rim=rim,
                pressure_psi=pressure,
                is_candidate=True,
                notes="Round 8 lateral fit with fabricated pure longitudinal support; combined slip excluded",
            )
        )

    return candidates


def mu_y_at_fz(params: dict[str, float | str], fz: float) -> float:
    fz_ref = tir_float(params, "FNOMIN")
    dfz = (max(fz, 1.0) - fz_ref) / fz_ref
    return abs(tir_float(params, "PDY1") + tir_float(params, "PDY2") * dfz)


def cornering_stiffness_at_fz(params: dict[str, float | str], fz: float) -> float:
    fz_ref = tir_float(params, "FNOMIN")
    lfzo = tir_float(params, "LFZO", 1.0)
    lky = tir_float(params, "LKY", 1.0)
    pky1 = tir_float(params, "PKY1")
    pky2 = tir_float(params, "PKY2")
    pky3 = tir_float(params, "PKY3")
    if pky2 == 0.0 or fz_ref == 0.0:
        return math.nan
    stiffness = (
        pky1
        * fz_ref
        * math.sin(2.0 * math.atan(fz / (pky2 * fz_ref * lfzo)))
        * (1.0 - pky3 * 0.0)
        * lfzo
        * lky
    )
    return abs(stiffness)


def characterize_tire(candidate: TireCandidate) -> dict[str, Any]:
    params = parse_tir(candidate.path)
    fz_ref = tir_float(params, "FNOMIN")
    mu_400 = mu_y_at_fz(params, 400.0)
    mu_ref = mu_y_at_fz(params, fz_ref)
    mu_900 = mu_y_at_fz(params, 900.0)
    mu_1090 = mu_y_at_fz(params, 1090.0)
    c_ref = cornering_stiffness_at_fz(params, fz_ref)
    c_900 = cornering_stiffness_at_fz(params, 900.0)

    return {
        "candidate_id": candidate.candidate_id,
        "label": candidate.label,
        "source": candidate.source,
        "compound": candidate.compound,
        "tire_size": candidate.tire_size,
        "rim": candidate.rim,
        "pressure_psi": candidate.pressure_psi,
        "is_candidate": int(candidate.is_candidate),
        "path": as_repo_path(candidate.path),
        "use_mode": tir_float(params, "USE_MODE"),
        "fnomin_n": fz_ref,
        "fzmin_n": tir_float(params, "FZMIN"),
        "fzmax_n": tir_float(params, "FZMAX"),
        "unloaded_radius_m": tir_float(params, "UNLOADED_RADIUS"),
        "width_m": tir_float(params, "WIDTH"),
        "rim_width_m": tir_float(params, "RIM_WIDTH"),
        "vertical_stiffness_n_per_m": tir_float(params, "VERTICAL_STIFFNESS"),
        "vertical_damping_n_s_per_m": tir_float(params, "VERTICAL_DAMPING"),
        "pdy1": tir_float(params, "PDY1"),
        "pdy2": tir_float(params, "PDY2"),
        "pky1": tir_float(params, "PKY1"),
        "pky2": tir_float(params, "PKY2"),
        "pky3": tir_float(params, "PKY3"),
        "mu_y_400n": mu_400,
        "mu_y_ref": mu_ref,
        "mu_y_900n": mu_900,
        "mu_y_1090n": mu_1090,
        "mu_y_900n_vs_400n_pct": 100.0 * (mu_900 - mu_400) / mu_400
        if mu_400 > 1e-12
        else math.nan,
        "cornering_stiffness_ref_n_per_rad": c_ref,
        "cornering_stiffness_900n_n_per_rad": c_900,
    }


def normalize_scores(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    candidate_rows = [row for row in rows if int(row["is_candidate"]) == 1]
    mins: dict[str, float] = {}
    maxs: dict[str, float] = {}
    for metric, _weight in ENVELOPE_SCORE_METRICS:
        values = [
            float(row[metric])
            for row in candidate_rows
            if math.isfinite(float(row.get(metric, math.nan)))
        ]
        mins[metric] = min(values) if values else math.nan
        maxs[metric] = max(values) if values else math.nan

    scored = []
    for row in rows:
        score = 0.0
        weight_sum = 0.0
        parts = {}
        for metric, weight in ENVELOPE_SCORE_METRICS:
            value = float(row.get(metric, math.nan))
            lo = mins[metric]
            hi = maxs[metric]
            if not math.isfinite(value) or not math.isfinite(lo) or not math.isfinite(hi):
                norm = math.nan
            elif abs(hi - lo) <= 1e-12:
                norm = 1.0
            else:
                norm = (value - lo) / (hi - lo)
            parts[f"score_part_{metric}"] = norm
            if math.isfinite(norm):
                score += weight * norm
                weight_sum += weight
        score = score / weight_sum if weight_sum > 0.0 else math.nan

        scored_row = dict(row)
        scored_row.update(parts)
        scored_row["envelope_lateral_score"] = score
        scored.append(scored_row)

    candidate_sorted = sorted(
        [row for row in scored if int(row["is_candidate"]) == 1],
        key=lambda row: float(row["envelope_lateral_score"]),
        reverse=True,
    )
    rank_lookup = {row["candidate_id"]: idx for idx, row in enumerate(candidate_sorted, start=1)}
    for row in scored:
        row["rank"] = rank_lookup.get(row["candidate_id"], "")
    return scored


def finite_float(value: Any) -> float:
    try:
        output = float(value)
    except (TypeError, ValueError):
        return math.nan
    return output if math.isfinite(output) else math.nan


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


def attach_standardsim_scores(
    scored_rows: list[dict[str, Any]],
    standardsim_rows: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    standardsim_lookup = {row["candidate_id"]: row for row in standardsim_rows}
    merged: list[dict[str, Any]] = []
    for row in scored_rows:
        output = dict(row)
        std_row = standardsim_lookup.get(row["candidate_id"])
        output["standardsim_status"] = std_row.get("status", "not_run") if std_row else "not_run"
        if std_row:
            for key, value in std_row.items():
                if key in {"candidate_id", "label"}:
                    continue
                output[f"standardsim_{key}"] = value
                if key not in output:
                    output[key] = value
        merged.append(output)

    candidate_rows = [
        row
        for row in merged
        if int(row["is_candidate"]) == 1
        and str(row.get("standardsim_status", "")) == "ok"
    ]
    ay_scores = normalized_metric_scores(
        merged,
        "ay_max",
        higher_is_better=True,
        source_rows=candidate_rows,
    )
    roll_scores = normalized_metric_scores(
        merged,
        "roll_gradient_deg_per_g",
        higher_is_better=False,
        source_rows=candidate_rows,
    )
    sideslip_abs_rows = []
    for row in merged:
        sideslip_abs = abs(finite_float(row.get("sideslip_gradient_deg_per_g")))
        output = dict(row)
        output["sideslip_gradient_abs_deg_per_g"] = sideslip_abs
        sideslip_abs_rows.append(output)
    candidate_abs_rows = [
        row
        for row in sideslip_abs_rows
        if int(row["is_candidate"]) == 1
        and str(row.get("standardsim_status", "")) == "ok"
    ]
    sideslip_scores = normalized_metric_scores(
        sideslip_abs_rows,
        "sideslip_gradient_abs_deg_per_g",
        higher_is_better=False,
        source_rows=candidate_abs_rows,
    )

    scored_with_std: list[dict[str, Any]] = []
    for row in sideslip_abs_rows:
        understeer_target = target_score(
            row.get("understeer_gradient_deg_per_g"),
            target=0.5,
            span=2.0,
        )
        parts = [
            (ay_scores.get(row["candidate_id"], math.nan), 0.60),
            (understeer_target, 0.20),
            (roll_scores.get(row["candidate_id"], math.nan), 0.10),
            (sideslip_scores.get(row["candidate_id"], math.nan), 0.10),
        ]
        weight_sum = sum(weight for score, weight in parts if math.isfinite(score))
        standardsim_score = (
            sum(score * weight for score, weight in parts if math.isfinite(score)) / weight_sum
            if weight_sum > 0.0
            else math.nan
        )
        envelope_score = finite_float(row.get("envelope_lateral_score"))
        if math.isfinite(envelope_score) and math.isfinite(standardsim_score):
            integrated_score = 0.55 * envelope_score + 0.45 * standardsim_score
        else:
            integrated_score = math.nan
        output = dict(row)
        output["standardsim_ay_score"] = ay_scores.get(row["candidate_id"], math.nan)
        output["standardsim_understeer_target_score"] = understeer_target
        output["standardsim_roll_score"] = roll_scores.get(row["candidate_id"], math.nan)
        output["standardsim_sideslip_score"] = sideslip_scores.get(row["candidate_id"], math.nan)
        output["standardsim_score"] = standardsim_score
        output["integrated_design_score"] = integrated_score
        scored_with_std.append(output)

    ranked = sorted(
        [
            row
            for row in scored_with_std
            if int(row["is_candidate"]) == 1
            and math.isfinite(finite_float(row.get("integrated_design_score")))
        ],
        key=lambda row: float(row["integrated_design_score"]),
        reverse=True,
    )
    integrated_rank_lookup = {
        row["candidate_id"]: idx for idx, row in enumerate(ranked, start=1)
    }
    for row in scored_with_std:
        row["integrated_rank"] = integrated_rank_lookup.get(row["candidate_id"], "")
    return scored_with_std


def write_csv(path: Path, rows: list[dict[str, Any]], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def group_summary(scored_rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    groups: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for row in scored_rows:
        if int(row["is_candidate"]) != 1:
            continue
        for field in ("compound", "tire_size", "rim", "pressure_psi"):
            groups[(field, str(row[field]))].append(row)

    output = []
    for (field, value), rows in sorted(groups.items()):
        rows_sorted = sorted(rows, key=lambda row: float(row["envelope_lateral_score"]), reverse=True)
        output.append(
            {
                "group_field": field,
                "group_value": value,
                "n": len(rows),
                "mean_score": float(np.nanmean([float(row["envelope_lateral_score"]) for row in rows])),
                "mean_lateral_g": float(np.nanmean([float(row["mean_max_lateral_g"]) for row in rows])),
                "mean_area_g2": float(np.nanmean([float(row["mean_ggv_area_g2"]) for row in rows])),
                "best_candidate_id": rows_sorted[0]["candidate_id"],
                "best_label": rows_sorted[0]["label"],
                "best_score": rows_sorted[0]["envelope_lateral_score"],
            }
        )
    return output


def family_color_map(rows: list[dict[str, Any]]) -> dict[str, Any]:
    families = sorted({str(row["compound"]) for row in rows})
    cmap = plt.get_cmap("tab20")
    return {family: cmap(idx % 20) for idx, family in enumerate(families)}


def plot_ranked_scores(scored_rows: list[dict[str, Any]]) -> None:
    rows = sorted(
        [row for row in scored_rows if int(row["is_candidate"]) == 1],
        key=lambda row: float(row["envelope_lateral_score"]),
    )
    labels = [row["label"] for row in rows]
    values = [float(row["envelope_lateral_score"]) for row in rows]
    colors_by_family = family_color_map(rows)
    colors = [colors_by_family[str(row["compound"])] for row in rows]
    fig_height = max(8.0, 0.20 * len(rows) + 1.8)
    fig, ax = plt.subplots(figsize=(12.5, fig_height))
    ax.barh(labels, values, color=colors)
    ax.set_xlabel("EnvelopeSim score (candidate min-max normalized)")
    ax.set_title("DS-005 EnvelopeSim Tire Selection Score")
    ax.grid(True, axis="x", linestyle="--", alpha=0.3)
    fig.tight_layout()
    fig.savefig(PLOT_DIR / "envelope_score_rank.png", dpi=220)
    plt.close(fig)


def plot_metric_scatter(scored_rows: list[dict[str, Any]]) -> None:
    rows = [row for row in scored_rows if int(row["is_candidate"]) == 1]
    compounds = sorted({str(row["compound"]) for row in rows})
    color_map = family_color_map(rows)
    marker_cycle = ("o", "s", "^", "D", "P", "X")
    tire_sizes = sorted({str(row["tire_size"]) for row in rows})
    marker_map = {
        tire_size: marker_cycle[idx % len(marker_cycle)]
        for idx, tire_size in enumerate(tire_sizes)
    }

    fig, ax = plt.subplots(figsize=(9.5, 7.0))
    for compound in compounds:
        for tire_size in tire_sizes:
            subset = [row for row in rows if row["compound"] == compound and row["tire_size"] == tire_size]
            if not subset:
                continue
            ax.scatter(
                [float(row["mean_max_lateral_g"]) for row in subset],
                [float(row["mean_ggv_area_g2"]) for row in subset],
                c=[color_map.get(compound, "#555555")],
                marker=marker_map.get(tire_size, "o"),
                s=[45.0 + 10.0 * float(row["pressure_psi"]) for row in subset],
                alpha=0.82,
                edgecolor="white",
                linewidth=0.7,
                label=f"{compound} {tire_size}",
            )
    ax.set_xlabel("Envelope mean lateral capability [g]")
    ax.set_ylabel("Envelope mean GGV area [g^2]")
    ax.set_title("EnvelopeSim Tire Capability Map")
    ax.grid(True, linestyle="--", alpha=0.3)
    handles, labels = ax.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    ax.legend(unique.values(), unique.keys(), loc="best", fontsize=9)
    fig.tight_layout()
    fig.savefig(PLOT_DIR / "envelope_capability_map.png", dpi=220)
    plt.close(fig)


def plot_pressure_trends(scored_rows: list[dict[str, Any]]) -> None:
    rows = [row for row in scored_rows if int(row["is_candidate"]) == 1]
    combos = sorted({(row["compound"], row["tire_size"], row["rim"]) for row in rows})
    fig, ax = plt.subplots(figsize=(11.5, 7.0))
    for compound, tire_size, rim in combos:
        subset = sorted(
            [
                row
                for row in rows
                if row["compound"] == compound
                and row["tire_size"] == tire_size
                and row["rim"] == rim
            ],
            key=lambda row: float(row["pressure_psi"]),
        )
        ax.plot(
            [float(row["pressure_psi"]) for row in subset],
            [float(row["mean_max_lateral_g"]) for row in subset],
            marker="o",
            linewidth=1.8,
            label=f"{compound} {tire_size} {rim}",
        )
    ax.set_xlabel("Pressure [psi]")
    ax.set_ylabel("Envelope mean lateral capability [g]")
    ax.set_title("Pressure Trend by Tire Fit")
    ax.grid(True, linestyle="--", alpha=0.3)
    ax.legend(loc="best", fontsize=8, ncols=2)
    fig.tight_layout()
    fig.savefig(PLOT_DIR / "pressure_trends_mean_lateral.png", dpi=220)
    plt.close(fig)


def plot_integrated_rank(scored_rows: list[dict[str, Any]]) -> None:
    rows = sorted(
        [
            row
            for row in scored_rows
            if int(row["is_candidate"]) == 1
            and math.isfinite(finite_float(row.get("integrated_design_score")))
        ],
        key=lambda row: float(row["integrated_design_score"]),
    )
    if not rows:
        return
    colors_by_family = family_color_map(rows)
    fig_height = max(8.0, 0.20 * len(rows) + 1.8)
    fig, ax = plt.subplots(figsize=(12.5, fig_height))
    ax.barh(
        [row["label"] for row in rows],
        [float(row["integrated_design_score"]) for row in rows],
        color=[colors_by_family[str(row["compound"])] for row in rows],
    )
    ax.set_xlabel("Integrated design score")
    ax.set_title("EnvelopeSim + StandardSim Integrated Tire Ranking")
    ax.grid(True, axis="x", linestyle="--", alpha=0.3)
    fig.tight_layout()
    fig.savefig(PLOT_DIR / "integrated_score_rank.png", dpi=220)
    plt.close(fig)


def plot_standardsim_response_space(scored_rows: list[dict[str, Any]]) -> None:
    rows = [
        row
        for row in scored_rows
        if int(row["is_candidate"]) == 1
        and math.isfinite(finite_float(row.get("ay_max")))
    ]
    if not rows:
        return
    colors_by_family = family_color_map(rows)
    colors = [colors_by_family[str(row["compound"])] for row in rows]
    sizes = [44.0 + 12.0 * finite_float(row.get("pressure_psi")) for row in rows]
    fig, axes = plt.subplots(2, 2, figsize=(12.5, 9.2))
    axis_specs = [
        ("mean_max_lateral_g", "ay_max", "Envelope mean lateral [g]", "StandardSim ay max [g]"),
        ("understeer_gradient_deg_per_g", "ay_max", "Understeer gradient [deg/g]", "StandardSim ay max [g]"),
        ("roll_gradient_deg_per_g", "ay_max", "Roll gradient [deg/g]", "StandardSim ay max [g]"),
        ("handwheel_torque_peak_abs", "ay_max", "Peak handwheel torque [N*m]", "StandardSim ay max [g]"),
    ]
    for ax, (x_key, y_key, x_label, y_label) in zip(axes.ravel(), axis_specs, strict=False):
        ax.scatter(
            [finite_float(row.get(x_key)) for row in rows],
            [finite_float(row.get(y_key)) for row in rows],
            c=colors,
            s=sizes,
            alpha=0.78,
            edgecolor="white",
            linewidth=0.6,
        )
        ax.set_xlabel(x_label)
        ax.set_ylabel(y_label)
        ax.grid(True, linestyle="--", alpha=0.3)
    ranked = sorted(
        rows,
        key=lambda row: finite_float(row.get("integrated_design_score")),
        reverse=True,
    )[:5]
    for row in ranked:
        axes[0, 0].annotate(
            str(row["integrated_rank"]),
            (
                finite_float(row.get("mean_max_lateral_g")),
                finite_float(row.get("ay_max")),
            ),
            textcoords="offset points",
            xytext=(4, 4),
            fontsize=8,
        )
    fig.suptitle("StandardSim Response Space and Envelope Agreement")
    fig.tight_layout()
    fig.savefig(PLOT_DIR / "standardsim_response_space.png", dpi=220)
    plt.close(fig)


def run_envelopes(
    base_vehicle: Any,
    config: Any,
    candidates: list[TireCandidate],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    envelope_rows = []
    characterization_rows = []

    for idx, candidate in enumerate(candidates, start=1):
        print(f"  EnvelopeSim {idx:02d}/{len(candidates)}: {candidate.label}", flush=True)
        tire_vehicle = vehicle_with_tire(base_vehicle, candidate.path)
        envelopes = ggv.generate_ggv(tire_vehicle, config)
        metrics = extract_metrics(envelopes)
        load_range = tire_load_range(tire_vehicle, envelopes)
        char_row = characterize_tire(candidate)
        characterization_rows.append(char_row)

        row = {
            "candidate_id": candidate.candidate_id,
            "label": candidate.label,
            "source": candidate.source,
            "compound": candidate.compound,
            "tire_size": candidate.tire_size,
            "rim": candidate.rim,
            "pressure_psi": candidate.pressure_psi,
            "is_candidate": int(candidate.is_candidate),
            "path": as_repo_path(candidate.path),
        }
        row.update(metrics)
        row.update(load_range)
        envelope_rows.append(row)

    return envelope_rows, characterization_rows


def write_report(
    *,
    started_at: str,
    elapsed_s: float,
    scored_rows: list[dict[str, Any]],
    group_rows: list[dict[str, Any]],
    standardsim_rows: list[dict[str, Any]],
    standardsim_errors: list[dict[str, Any]],
) -> None:
    ranked = sorted(
        [row for row in scored_rows if int(row["is_candidate"]) == 1],
        key=lambda row: float(row["envelope_lateral_score"]),
        reverse=True,
    )
    top = ranked[:10]
    reference = next(
        (row for row in scored_rows if row["candidate_id"] == "current_hybrid_reference"),
        None,
    )

    best_by_group = {
        (row["group_field"], row["group_value"]): row for row in group_rows
    }

    lines: list[str] = []
    lines.append("# DS-005 Tire Selection")
    lines.append("")
    lines.append(f"Generated UTC: {started_at}")
    lines.append("")
    lines.append("## Source of Results")
    lines.append("")
    lines.append(
        "All simulated response metrics in this report are from EnvelopeSim. "
        "No non-EnvelopeSim simulation output is used for the ranking."
    )
    if standardsim_rows or standardsim_errors:
        lines.append(
            "A StandardSim finalist section is included separately and is not mixed into the EnvelopeSim ranking."
        )
    lines.append("")
    lines.append("## Tire Data Caveat")
    lines.append("")
    lines.append(
        "The Round 8 candidates preserve real lateral/vertical/alignment tire fits, "
        "but their pure longitudinal coefficients were copied from the current hybrid "
        "reference tire. Combined-slip coefficients were intentionally zeroed. "
        "Therefore the ranking emphasizes lateral EnvelopeSim capability; acceleration, "
        "braking, and combined-slip differences are not tire-selection evidence here."
    )
    lines.append("")
    lines.append("## Recommendation")
    lines.append("")
    if top:
        winner = top[0]
        lines.append(
            f"First-pass selection winner: **{winner['label']}** "
            f"(`{winner['candidate_id']}`), with EnvelopeSim lateral score "
            f"{format_float(winner['envelope_lateral_score'], 3)}."
        )
        if int(winner["fz_range_exceeded"]):
            lines.append("")
            lines.append(
                "This is a performance winner, not a final sign-off tire: EnvelopeSim "
                f"loaded it {format_float(winner['fz_max_excess_pct'], 1)}% above the "
                "fit's stated maximum vertical load. Treat it as the first finalist, "
                "then confirm with a load-range-aware tire fit, StandardSim, or test data."
            )
    lines.append("")
    lines.append("The strongest pattern is pressure and fit dependent, not just compound-name dependent. Use the top candidates as the next StandardSim/track-test shortlist, not as a final purchasing decision.")
    lines.append("")
    lines.append("## Top EnvelopeSim Candidates")
    lines.append("")
    lines.append(table_line(["Rank", "Candidate", "Score", "Mean lat g", "Lat 25 g", "Mean area g^2", "Fz extrap?", "Fz excess %"]))
    lines.append(table_line(["---:", "---", "---:", "---:", "---:", "---:", "---", "---:"]))
    for row in top:
        lines.append(
            table_line(
                [
                    str(row["rank"]),
                    row["label"],
                    format_float(row["envelope_lateral_score"], 4),
                    format_float(row["mean_max_lateral_g"], 4),
                    format_float(row["max_lateral_g__25mps"], 4),
                    format_float(row["mean_ggv_area_g2"], 4),
                    "yes" if int(row["fz_range_exceeded"]) else "no",
                    format_float(row["fz_max_excess_pct"], 1),
                ]
            )
        )
    lines.append("")
    fz_extrapolated_top = [row for row in top if int(row["fz_range_exceeded"])]
    if fz_extrapolated_top:
        lines.append("## Fit-Range Confidence")
        lines.append("")
        lines.append(
            "Every top-ten Round 8 candidate exceeds its fitted Fz range in the current "
            "EnvelopeSim load case. That does not invalidate the directional ranking, "
            "but it lowers confidence in the absolute margin between the fastest candidates."
        )
        lines.append("")
        lines.append(table_line(["Candidate", "Fz max seen", "Fit Fz max", "Excess"]))
        lines.append(table_line(["---", "---:", "---:", "---:"]))
        for row in fz_extrapolated_top[:5]:
            lines.append(
                table_line(
                    [
                        row["label"],
                        f"{format_float(row['fz_max_seen_n'], 1)} N",
                        f"{format_float(row['fzmax_n'], 1)} N",
                        f"{format_float(row['fz_max_excess_pct'], 1)}%",
                    ]
                )
            )
        lines.append("")
    if reference:
        lines.append("## Current Reference Tire")
        lines.append("")
        lines.append(table_line(["Item", "Value"]))
        lines.append(table_line(["---", "---:"]))
        lines.append(table_line(["Label", reference["label"]]))
        lines.append(table_line(["Mean lateral", f"{format_float(reference['mean_max_lateral_g'], 4)} g"]))
        lines.append(table_line(["Lat 25 m/s", f"{format_float(reference['max_lateral_g__25mps'], 4)} g"]))
        lines.append(table_line(["Mean GGV area", f"{format_float(reference['mean_ggv_area_g2'], 4)} g^2"]))
        lines.append(table_line(["Fz max seen", f"{format_float(reference['fz_max_seen_n'], 1)} N"]))
        lines.append("")
        lines.append("## Against Current Reference Tire")
        lines.append("")
        lines.append(
            "Relative to `vehicles/current/tires/16x7p5_10_12psi.tir`, the leading "
            "Round 8 candidates show lateral-envelope upside, but with lower confidence "
            "because the fitted Fz range is exceeded. Acceleration and braking deltas "
            "are intentionally excluded because the Round 8 longitudinal coefficients "
            "are fabricated from the reference tire."
        )
        lines.append("")
        lines.append(table_line(["Rank", "Candidate", "Mean lat", "Lat 25 m/s", "Mean GGV area", "Fz confidence"]))
        lines.append(table_line(["---:", "---", "---:", "---:", "---:", "---"]))
        for row in top:
            confidence = "extrapolated" if int(row["fz_range_exceeded"]) else "in range"
            lines.append(
                table_line(
                    [
                        str(row["rank"]),
                        row["label"],
                        format_percent_delta(row["mean_max_lateral_g"], reference["mean_max_lateral_g"]),
                        format_percent_delta(row["max_lateral_g__25mps"], reference["max_lateral_g__25mps"]),
                        format_percent_delta(row["mean_ggv_area_g2"], reference["mean_ggv_area_g2"]),
                        confidence,
                    ]
                )
            )
        lines.append("")
    lines.append("## Group Reads")
    lines.append("")
    lines.append(table_line(["Group", "Value", "N", "Mean score", "Best candidate"]))
    lines.append(table_line(["---", "---", "---:", "---:", "---"]))
    for key in (
        ("compound", "LC0"),
        ("compound", "R25B"),
        ("tire_size", "16x6_10"),
        ("tire_size", "16x7p5_10"),
        ("rim", "6in"),
        ("rim", "7in"),
        ("rim", "8in"),
        ("pressure_psi", "8.0"),
        ("pressure_psi", "10.0"),
        ("pressure_psi", "12.0"),
        ("pressure_psi", "14.0"),
    ):
        row = best_by_group.get(key)
        if not row:
            continue
        lines.append(
            table_line(
                [
                    row["group_field"],
                    row["group_value"],
                    str(row["n"]),
                    format_float(row["mean_score"], 4),
                    row["best_label"],
                ]
            )
        )
    lines.append("")
    lines.append("## StandardSim Finalist Check")
    lines.append("")
    if standardsim_rows:
        lines.append(table_line(["Candidate", "Status", "ay_max", "understeer", "roll", "handwheel torque"]))
        lines.append(table_line(["---", "---", "---:", "---:", "---:", "---:"]))
        for row in standardsim_rows:
            lines.append(
                table_line(
                    [
                        row["label"],
                        row["status"],
                        format_float(row.get("ay_max"), 4),
                        format_float(row.get("understeer_gradient_deg_per_g"), 4),
                        format_float(row.get("roll_gradient_deg_per_g"), 4),
                        format_float(row.get("handwheel_torque_peak_abs"), 4),
                    ]
                )
            )
    else:
        lines.append("Not run in this pass. EnvelopeSim generated the selection ranking.")
    if standardsim_errors:
        lines.append("")
        lines.append("StandardSim errors were captured in `outputs/standardsim_errors.csv`.")
    lines.append("")
    lines.append("## Generated Files")
    lines.append("")
    for relative in [
        "outputs/candidate_registry.csv",
        "outputs/tire_characterization.csv",
        "outputs/envelope_metrics.csv",
        "outputs/candidate_scores.csv",
        "outputs/group_summary.csv",
        "outputs/standardsim_metrics.csv",
        "outputs/standardsim_errors.csv",
        "outputs/run_provenance.csv",
        "plots/envelope_score_rank.png",
        "plots/envelope_capability_map.png",
        "plots/pressure_trends_mean_lateral.png",
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
    (STUDY_DIR / "RESULTS.md").write_text(text, encoding="utf-8")
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(text, encoding="utf-8")


def table_line(values: list[str]) -> str:
    return "| " + " | ".join(str(value) for value in values) + " |"


def plot_outputs(scored_rows: list[dict[str, Any]]) -> None:
    PLOT_DIR.mkdir(parents=True, exist_ok=True)
    plot_ranked_scores(scored_rows)
    plot_metric_scatter(scored_rows)
    plot_pressure_trends(scored_rows)
    plot_integrated_rank(scored_rows)
    plot_standardsim_response_space(scored_rows)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--vehicle",
        type=Path,
        default=REPO_ROOT / "vehicles" / "current" / "vehicle.yml",
    )
    parser.add_argument(
        "--standardsim-top",
        type=int,
        default=0,
        help="Optional number of EnvelopeSim finalists to run through StandardSim.",
    )
    parser.add_argument(
        "--reuse",
        action="store_true",
        help="Reuse StandardSim finalist builds when available.",
    )
    args = parser.parse_args()

    started_at = datetime.now(timezone.utc).isoformat(timespec="seconds")
    start = time.perf_counter()
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    PLOT_DIR.mkdir(parents=True, exist_ok=True)
    WORK_DIR.mkdir(parents=True, exist_ok=True)

    candidates = discover_candidates()
    print("DS-005 Tire Selection", flush=True)
    print(f"  Candidates: {len(candidates) - 1} Round 8 + 1 current reference", flush=True)

    base_vehicle, context = build_baseline_vehicle(args.vehicle, CURRENT_TIRE)
    config = make_config()

    envelope_rows, characterization_rows = run_envelopes(base_vehicle, config, candidates)
    char_lookup = {row["candidate_id"]: row for row in characterization_rows}
    score_input = []
    for row in envelope_rows:
        merged = dict(row)
        merged.update(
            {
                key: value
                for key, value in char_lookup[row["candidate_id"]].items()
                if key not in merged
            }
        )
        score_input.append(merged)
    scored_rows = normalize_scores(score_input)
    group_rows = group_summary(scored_rows)

    standardsim_rows: list[dict[str, Any]] = []
    standardsim_errors: list[dict[str, Any]] = []
    if args.standardsim_top > 0:
        standardsim_rows, standardsim_errors = run_standardsim_finalists(
            args.vehicle,
            scored_rows,
            top_n=args.standardsim_top,
            reuse=args.reuse,
        )

    write_csv(
        OUTPUT_DIR / "candidate_registry.csv",
        [
            {
                "candidate_id": candidate.candidate_id,
                "label": candidate.label,
                "source": candidate.source,
                "compound": candidate.compound,
                "tire_size": candidate.tire_size,
                "rim": candidate.rim,
                "pressure_psi": candidate.pressure_psi,
                "is_candidate": int(candidate.is_candidate),
                "path": as_repo_path(candidate.path),
                "notes": candidate.notes,
            }
            for candidate in candidates
        ],
        [
            "candidate_id",
            "label",
            "source",
            "compound",
            "tire_size",
            "rim",
            "pressure_psi",
            "is_candidate",
            "path",
            "notes",
        ],
    )
    write_csv(OUTPUT_DIR / "tire_characterization.csv", characterization_rows, list(characterization_rows[0]))
    write_csv(OUTPUT_DIR / "envelope_metrics.csv", envelope_rows, list(envelope_rows[0]))
    write_csv(OUTPUT_DIR / "candidate_scores.csv", scored_rows, list(scored_rows[0]))
    write_csv(OUTPUT_DIR / "group_summary.csv", group_rows, list(group_rows[0]))
    write_csv(
        OUTPUT_DIR / "standardsim_metrics.csv",
        standardsim_rows,
        list(standardsim_rows[0]) if standardsim_rows else ["candidate_id", "label", "status"],
    )
    write_csv(
        OUTPUT_DIR / "standardsim_errors.csv",
        standardsim_errors,
        list(standardsim_errors[0]) if standardsim_errors else ["candidate_id", "label", "error"],
    )
    write_csv(
        OUTPUT_DIR / "run_provenance.csv",
        [
            {"item": "generated_at_utc", "value": started_at},
            {"item": "vehicle", "value": as_repo_path(args.vehicle)},
            {"item": "candidate_count_round8", "value": len(candidates) - 1},
            {"item": "current_reference", "value": as_repo_path(CURRENT_TIRE)},
            {"item": "round8_tire_dir", "value": as_repo_path(ROUND8_TIRE_DIR)},
            {"item": "envelopesim_speeds_mps", "value": ";".join(str(v) for v in config.speeds)},
            {"item": "mass_kg", "value": base_vehicle.mass},
            {"item": "cg_height_m", "value": base_vehicle.cg_height},
            {"item": "cg_x_m", "value": context["mass_cg_x_m"]},
            {"item": "standardsim_top_n", "value": args.standardsim_top},
        ],
        ["item", "value"],
    )

    plot_outputs(scored_rows)
    elapsed_s = time.perf_counter() - start
    write_report(
        started_at=started_at,
        elapsed_s=elapsed_s,
        scored_rows=scored_rows,
        group_rows=group_rows,
        standardsim_rows=standardsim_rows,
        standardsim_errors=standardsim_errors,
    )
    print(f"Study report: {STUDY_DIR / 'RESULTS.md'}", flush=True)
    print(f"Top-level report: {REPORT_PATH}", flush=True)


def run_standardsim_finalists(
    vehicle_path: Path,
    scored_rows: list[dict[str, Any]],
    *,
    top_n: int,
    reuse: bool,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    if shutil.which("omc") is None:
        return [], [
            {
                "candidate_id": "",
                "label": "",
                "error": "OpenModelica `omc` was not found on PATH.",
            }
        ]

    std = load_module(
        "ds002_standardsim_helpers",
        REPO_ROOT / "studies" / "DS-002-standardsim-steady-state-sensitivity" / "run.py",
    )
    generation_scripts = (
        REPO_ROOT / "BobSim" / "_0_Utils" / "external" / "BobLib" / "Generation" / "scripts"
    )
    if str(generation_scripts) not in sys.path:
        sys.path.insert(0, str(generation_scripts))
    from build_records import render_record  # noqa: PLC0415

    vehicle_doc = read_yaml(vehicle_path)
    finalists = sorted(
        [row for row in scored_rows if int(row["is_candidate"]) == 1],
        key=lambda row: float(row["envelope_lateral_score"]),
        reverse=True,
    )[:top_n]

    metric_rows: list[dict[str, Any]] = []
    error_rows: list[dict[str, Any]] = []

    for idx, row in enumerate(finalists, start=1):
        print(f"  StandardSim finalist {idx:02d}/{len(finalists)}: {row['label']}", flush=True)
        candidate_path = REPO_ROOT / row["path"]
        tire_params = parse_tir(candidate_path)
        variant_doc = yaml.safe_load(yaml.safe_dump(vehicle_doc))
        variant_doc["paths"]["tire_templates"] = str(candidate_path.parent.resolve())
        for axle in ("front", "rear"):
            variant_doc[axle]["tire"]["template"] = candidate_path.stem
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

        variant_dir = WORK_DIR / "standardsim_finalists" / f"{idx:02d}_{row['candidate_id']}"
        try:
            record_text = render_record(variant_doc, vehicle_path)
            variant_changed = std.stage_variant_text(
                variant_dir,
                record_text,
                rebuild=not reuse,
            )
            build_dir = variant_dir / "build" / "SteadyStateEval"
            if not reuse or variant_changed or std.find_executable(build_dir) is None:
                std.compile_variant(variant_dir)
            metrics_path = variant_dir / "results" / "SteadyStateEval" / "metrics.csv"
            if not reuse or variant_changed or not metrics_path.exists():
                std.run_standard_report(variant_dir)
            metrics, _raw_rows = std.read_metrics_csv(metrics_path)
            output = {
                "candidate_id": row["candidate_id"],
                "label": row["label"],
                "status": "ok",
                "variant_dir": as_repo_path(variant_dir),
                "envelope_rank": row["rank"],
                "envelope_lateral_score": row["envelope_lateral_score"],
            }
            for metric in (
                "ay_max",
                "roadwheel_angle_gradient_deg_per_g",
                "handwheel_angle_gradient_deg_per_g",
                "sideslip_gradient_deg_per_g",
                "understeer_gradient_deg_per_g",
                "roll_gradient_deg_per_g",
                "handwheel_torque_peak_abs",
            ):
                output[metric] = metrics.get(metric, math.nan)
            metric_rows.append(output)
        except Exception as exc:  # noqa: BLE001
            error = {
                "candidate_id": row["candidate_id"],
                "label": row["label"],
                "error": str(exc),
                "variant_dir": as_repo_path(variant_dir),
            }
            error_rows.append(error)
            (variant_dir / "case_error.txt").parent.mkdir(parents=True, exist_ok=True)
            (variant_dir / "case_error.txt").write_text(str(exc), encoding="utf-8")

    return metric_rows, error_rows


if __name__ == "__main__":
    main()
