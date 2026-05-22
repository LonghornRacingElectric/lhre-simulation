#!/usr/bin/env python3
"""Run DS-001: EnvelopeSim parameter sensitivity for the current vehicle."""

from __future__ import annotations

import argparse
import csv
import dataclasses
import importlib.util
import math
import os
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
STUDY_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = STUDY_DIR / "outputs"
PLOT_DIR = STUDY_DIR / "plots"
REPORT_PATH = REPO_ROOT / "reports" / "DS-001-envelopesim-parameter-sensitivity.md"

MPLCONFIGDIR = Path("/tmp/lhre-sim-matplotlib")
MPLCONFIGDIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(MPLCONFIGDIR))


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
            "Install the study dependencies in an external environment, for example:\n"
            "  python3 -m venv /tmp/lhre-sim-venv\n"
            "  /tmp/lhre-sim-venv/bin/python -m pip install PyYAML numpy matplotlib\n"
            "  /tmp/lhre-sim-venv/bin/python "
            "studies/DS-001-envelopesim-parameter-sensitivity/run.py"
        ) from exc

    return np, plt, yaml


np, plt, yaml = require_dependencies()


def load_ggv_module() -> Any:
    module_path = REPO_ROOT / "BobSim" / "_2_EnvelopeSim" / "GGV" / "ggv_generation.py"
    spec = importlib.util.spec_from_file_location("bobsim_envelopesim_ggv", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load EnvelopeSim module: {module_path}")

    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


ggv = load_ggv_module()
G = float(ggv.G)


@dataclass(frozen=True)
class ParameterSpec:
    name: str
    label: str
    unit: str
    baseline: float
    low: float
    high: float
    group: str
    description: str


DIRECT_FIELDS = {
    "mass_kg": "mass",
    "wheelbase_m": "wheelbase",
    "track_front_m": "track_front",
    "track_rear_m": "track_rear",
    "cg_height_m": "cg_height",
    "front_static_frac": "front_static_frac",
    "lltd": "lltd",
    "air_density_kg_m3": "rho",
    "cl_a_m2": "cl_a",
    "cd_a_m2": "cd_a",
    "aero_balance_front": "aero_balance_front",
    "max_drive_power_w": "max_drive_power",
    "max_drive_force_n": "max_drive_force",
    "max_brake_force_n": "max_brake_force",
    "drive_distribution_front": "drive_distribution_front",
    "brake_distribution_front": "brake_distribution_front",
}


def read_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"Expected YAML mapping in {path}")
    return data


def read_tir_scalar(path: Path, key: str) -> float:
    prefix = f"{key}".upper()
    with path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line.upper().startswith(prefix):
                continue
            if "=" not in line:
                continue
            value = line.split("=", 1)[1].split("$", 1)[0].strip()
            return float(value)
    raise KeyError(f"{key} not found in {path}")


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


def mass_rollup(vehicle_doc: dict[str, Any]) -> tuple[float, np.ndarray, list[dict[str, Any]]]:
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

    cg = weighted_cg / total_mass
    return total_mass, cg, rows


def baseline_aero_from_vehicle(vehicle_doc: dict[str, Any]) -> tuple[float, float, dict[str, float]]:
    aero = vehicle_doc["aero"]
    reference_speed = float(aero["reference_speed_m_per_s"])
    front_ride_height = float(aero["front_ride_height_grid_m"][0])
    rear_ride_height = float(aero["rear_ride_height_grid_m"][0])
    drag_n = float(aero["drag_table_n"][0][0])
    downforce_n = float(aero["downforce_table_n"][0][0])

    cl_a, cd_a = ggv.force_to_aero_area(
        downforce_n=downforce_n,
        drag_n=drag_n,
        speed_mps=reference_speed,
    )

    details = {
        "reference_speed_m_per_s": reference_speed,
        "front_ride_height_m": front_ride_height,
        "rear_ride_height_m": rear_ride_height,
        "downforce_n": downforce_n,
        "drag_n": drag_n,
        "cl_a_m2": float(cl_a),
        "cd_a_m2": float(cd_a),
    }
    return float(cl_a), float(cd_a), details


def build_baseline_vehicle(vehicle_path: Path) -> tuple[Any, dict[str, Any]]:
    vehicle_doc = read_yaml(vehicle_path)
    total_mass, cg, mass_rows = mass_rollup(vehicle_doc)

    front_wc = np.array(vehicle_doc["front"]["suspension"]["wheel_center_m"], dtype=float)
    rear_wc = np.array(vehicle_doc["rear"]["suspension"]["wheel_center_m"], dtype=float)

    wheelbase = abs(float(front_wc[0] - rear_wc[0]))
    track_front = 2.0 * abs(float(front_wc[1]))
    track_rear = 2.0 * abs(float(rear_wc[1]))

    front_static_frac = float((cg[0] - rear_wc[0]) / (front_wc[0] - rear_wc[0]))
    front_static_frac = min(0.70, max(0.30, front_static_frac))

    cl_a, cd_a, aero_details = baseline_aero_from_vehicle(vehicle_doc)

    tire_template = vehicle_doc["front"]["tire"]["template"]
    tire_path = vehicle_path.parent / "tires" / f"{tire_template}.tir"
    tire_values = {
        "fz_ref": read_tir_scalar(tire_path, "FNOMIN"),
        "fz_min_valid": read_tir_scalar(tire_path, "FZMIN"),
        "fz_max_valid": read_tir_scalar(tire_path, "FZMAX"),
        "pdx1": read_tir_scalar(tire_path, "PDX1"),
        "pdx2": read_tir_scalar(tire_path, "PDX2"),
        "pdy1": read_tir_scalar(tire_path, "PDY1"),
        "pdy2": read_tir_scalar(tire_path, "PDY2"),
    }

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
        fz_ref=tire_values["fz_ref"],
        fz_min_valid=tire_values["fz_min_valid"],
        fz_max_valid=tire_values["fz_max_valid"],
        pdx1=tire_values["pdx1"],
        pdx2=tire_values["pdx2"],
        pdy1=tire_values["pdy1"],
        pdy2=tire_values["pdy2"],
        mu_min=assumptions["mu_min"],
    )

    context = {
        "vehicle_name": vehicle_doc["vehicle"]["name"],
        "vehicle_path": str(vehicle_path.relative_to(REPO_ROOT)),
        "tire_path": str(tire_path.relative_to(REPO_ROOT)),
        "front_wheel_center_x_m": float(front_wc[0]),
        "rear_wheel_center_x_m": float(rear_wc[0]),
        "mass_cg_x_m": float(cg[0]),
        "mass_cg_y_m": float(cg[1]),
        "mass_cg_z_m": float(cg[2]),
        "mass_components": mass_rows,
        "aero": aero_details,
        "tire": tire_values,
        "assumptions": assumptions,
    }

    return vehicle, context


def clamp(value: float, low: float, high: float) -> float:
    return min(high, max(low, value))


def make_parameter_specs(vehicle: Any) -> list[ParameterSpec]:
    return [
        ParameterSpec(
            "mass_kg",
            "total mass",
            "kg",
            vehicle.mass,
            vehicle.mass * 0.90,
            vehicle.mass * 1.10,
            "chassis",
            "Rolled-up vehicle mass including driver and mirrored corner masses.",
        ),
        ParameterSpec(
            "cg_height_m",
            "CG height",
            "m",
            vehicle.cg_height,
            max(0.18, vehicle.cg_height - 0.05),
            vehicle.cg_height + 0.06,
            "chassis",
            "Mass-rollup CG height used for load transfer.",
        ),
        ParameterSpec(
            "front_static_frac",
            "front static load fraction",
            "fraction",
            vehicle.front_static_frac,
            clamp(vehicle.front_static_frac - 0.06, 0.35, 0.65),
            clamp(vehicle.front_static_frac + 0.06, 0.35, 0.65),
            "chassis",
            "Static front axle weight fraction from longitudinal CG.",
        ),
        ParameterSpec(
            "wheelbase_m",
            "wheelbase",
            "m",
            vehicle.wheelbase,
            vehicle.wheelbase * 0.95,
            vehicle.wheelbase * 1.05,
            "chassis",
            "Front-to-rear wheel center spacing.",
        ),
        ParameterSpec(
            "track_front_m",
            "front track",
            "m",
            vehicle.track_front,
            vehicle.track_front * 0.94,
            vehicle.track_front * 1.06,
            "chassis",
            "Front lateral wheel-center spacing.",
        ),
        ParameterSpec(
            "track_rear_m",
            "rear track",
            "m",
            vehicle.track_rear,
            vehicle.track_rear * 0.94,
            vehicle.track_rear * 1.06,
            "chassis",
            "Rear lateral wheel-center spacing.",
        ),
        ParameterSpec(
            "lltd",
            "lateral load transfer distribution",
            "front fraction",
            vehicle.lltd,
            0.40,
            0.65,
            "chassis balance",
            "Front share of total lateral load transfer in EnvelopeSim.",
        ),
        ParameterSpec(
            "air_density_kg_m3",
            "air density",
            "kg/m^3",
            vehicle.rho,
            1.00,
            1.225,
            "environment/aero",
            "Atmospheric density multiplier on aero loads and drag.",
        ),
        ParameterSpec(
            "cl_a_m2",
            "downforce area",
            "m^2",
            vehicle.cl_a,
            vehicle.cl_a * 0.55,
            vehicle.cl_a * 1.45,
            "aero",
            "Total downforce coefficient times reference area.",
        ),
        ParameterSpec(
            "cd_a_m2",
            "drag area",
            "m^2",
            vehicle.cd_a,
            vehicle.cd_a * 0.70,
            vehicle.cd_a * 1.35,
            "aero",
            "Drag coefficient times reference area.",
        ),
        ParameterSpec(
            "aero_balance_front",
            "front aero balance",
            "front fraction",
            vehicle.aero_balance_front,
            0.42,
            0.58,
            "aero balance",
            "Front axle share of total downforce.",
        ),
        ParameterSpec(
            "max_drive_power_w",
            "max drive power",
            "W",
            vehicle.max_drive_power,
            60_000.0,
            100_000.0,
            "powertrain",
            "Power cap used by EnvelopeSim's drive force limit.",
        ),
        ParameterSpec(
            "max_drive_force_n",
            "max drive force",
            "N",
            vehicle.max_drive_force,
            2_800.0,
            5_000.0,
            "powertrain",
            "Low-speed drivetrain or traction force cap before tire limits.",
        ),
        ParameterSpec(
            "drive_distribution_front",
            "front drive distribution",
            "front fraction",
            vehicle.drive_distribution_front,
            0.0,
            0.50,
            "powertrain architecture",
            "Front share of drive force. Baseline 0 is RWD.",
        ),
        ParameterSpec(
            "max_brake_force_n",
            "max brake force",
            "N",
            vehicle.max_brake_force,
            9_000.0,
            18_000.0,
            "brakes",
            "Brake system cap before tire limits.",
        ),
        ParameterSpec(
            "brake_distribution_front",
            "front brake distribution",
            "front fraction",
            vehicle.brake_distribution_front,
            0.54,
            0.70,
            "brakes",
            "Front share of brake force.",
        ),
        ParameterSpec(
            "longitudinal_mu_scale",
            "longitudinal tire peak scale",
            "scale",
            1.0,
            0.90,
            1.10,
            "tires",
            "Scale factor applied to longitudinal peak tire friction.",
        ),
        ParameterSpec(
            "longitudinal_load_sensitivity_scale",
            "longitudinal tire load sensitivity scale",
            "scale",
            1.0,
            0.50,
            1.50,
            "tires",
            "Scale factor applied to the longitudinal load-sensitivity term.",
        ),
        ParameterSpec(
            "lateral_mu_scale",
            "lateral tire peak scale",
            "scale",
            1.0,
            0.90,
            1.10,
            "tires",
            "Scale factor applied to lateral peak tire friction.",
        ),
        ParameterSpec(
            "lateral_load_sensitivity_scale",
            "lateral tire load sensitivity scale",
            "scale",
            1.0,
            0.50,
            1.50,
            "tires",
            "Scale factor applied to the lateral load-sensitivity term.",
        ),
    ]


def baseline_values(specs: list[ParameterSpec]) -> dict[str, float]:
    return {spec.name: spec.baseline for spec in specs}


def build_vehicle_from_values(base_vehicle: Any, values: dict[str, float]) -> Any:
    params = dataclasses.asdict(base_vehicle)

    for parameter_name, field_name in DIRECT_FIELDS.items():
        if parameter_name in values:
            params[field_name] = float(values[parameter_name])

    longitudinal_mu_scale = float(values.get("longitudinal_mu_scale", 1.0))
    longitudinal_load_scale = float(
        values.get("longitudinal_load_sensitivity_scale", 1.0)
    )
    lateral_mu_scale = float(values.get("lateral_mu_scale", 1.0))
    lateral_load_scale = float(values.get("lateral_load_sensitivity_scale", 1.0))

    params["pdx1"] = float(base_vehicle.pdx1) * longitudinal_mu_scale
    params["pdx2"] = (
        float(base_vehicle.pdx2) * longitudinal_mu_scale * longitudinal_load_scale
    )
    params["pdy1"] = float(base_vehicle.pdy1) * lateral_mu_scale
    params["pdy2"] = float(base_vehicle.pdy2) * lateral_mu_scale * lateral_load_scale

    return ggv.VehicleParams(**params)


def make_config() -> Any:
    return ggv.GGVConfig(
        speeds=(5.0, 10.0, 15.0, 20.0, 25.0),
        ay_max_g=3.2,
        ay_points=61,
        ax_search_min_g=-3.2,
        ax_search_max_g=2.8,
        ax_search_points=161,
        include_left_right=True,
        verbose=False,
        progress_every=25,
        warn_tire_load_range=False,
    )


def speed_label(speed: float) -> str:
    if float(speed).is_integer():
        return f"{int(speed):02d}mps"
    return f"{speed:.1f}mps".replace(".", "p")


def metric_display_name(metric_name: str) -> str:
    replacements = {
        "max_lateral_g": "lat",
        "max_accel_g": "accel",
        "max_brake_g": "brake",
        "ggv_area_g2": "area",
        "mean_max_lateral_g": "mean lat",
        "mean_max_accel_g": "mean accel",
        "mean_max_brake_g": "mean brake",
        "mean_ggv_area_g2": "mean area",
    }
    if "__" not in metric_name:
        return replacements.get(metric_name, metric_name)
    root, suffix = metric_name.split("__", 1)
    speed = suffix.replace("mps", "")
    speed = speed.lstrip("0") or "0"
    return f"{replacements.get(root, root)} {speed}"


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


def run_envelope_case(base_vehicle: Any, config: Any, values: dict[str, float]) -> tuple[dict[str, float], list[Any]]:
    vehicle = build_vehicle_from_values(base_vehicle, values)
    envelopes = ggv.generate_ggv(vehicle, config)
    return extract_metrics(envelopes), envelopes


def write_csv(path: Path, rows: list[dict[str, Any]], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def finite_pct_delta(value: float, baseline: float) -> float:
    if not math.isfinite(value) or not math.isfinite(baseline) or abs(baseline) < 1e-12:
        return math.nan
    return 100.0 * (value - baseline) / abs(baseline)


def finite_span_pct(high_value: float, low_value: float, baseline: float) -> float:
    if (
        not math.isfinite(high_value)
        or not math.isfinite(low_value)
        or not math.isfinite(baseline)
        or abs(baseline) < 1e-12
    ):
        return math.nan
    return 100.0 * (high_value - low_value) / abs(baseline)


def generate_lhs(
    specs: list[ParameterSpec],
    sample_count: int,
    seed: int,
) -> list[dict[str, float]]:
    rng = np.random.default_rng(seed)
    matrix = np.zeros((sample_count, len(specs)), dtype=float)

    for col, spec in enumerate(specs):
        bins = (np.arange(sample_count, dtype=float) + rng.random(sample_count)) / sample_count
        rng.shuffle(bins)
        matrix[:, col] = spec.low + bins * (spec.high - spec.low)

    samples: list[dict[str, float]] = []
    for row in matrix:
        samples.append({spec.name: float(row[idx]) for idx, spec in enumerate(specs)})
    return samples


def pearson(x_values: list[float], y_values: list[float]) -> float:
    x = np.asarray(x_values, dtype=float)
    y = np.asarray(y_values, dtype=float)
    mask = np.isfinite(x) & np.isfinite(y)
    if np.count_nonzero(mask) < 3:
        return math.nan
    x = x[mask]
    y = y[mask]
    if float(np.std(x)) <= 1e-12 or float(np.std(y)) <= 1e-12:
        return math.nan
    return float(np.corrcoef(x, y)[0, 1])


def format_float(value: float, digits: int = 4) -> str:
    if not math.isfinite(float(value)):
        return "nan"
    return f"{float(value):.{digits}f}"


def write_baseline_characterization(
    base_vehicle: Any,
    context: dict[str, Any],
    config: Any,
    started_at: str,
) -> None:
    rows: list[dict[str, Any]] = [
        {"item": "generated_at_utc", "value": started_at, "unit": "", "source": "study runner"},
        {"item": "vehicle_name", "value": context["vehicle_name"], "unit": "", "source": context["vehicle_path"]},
        {"item": "mass", "value": base_vehicle.mass, "unit": "kg", "source": "vehicle mass rollup"},
        {"item": "cg_x", "value": context["mass_cg_x_m"], "unit": "m", "source": "vehicle mass rollup"},
        {"item": "cg_y", "value": context["mass_cg_y_m"], "unit": "m", "source": "vehicle mass rollup"},
        {"item": "cg_height", "value": base_vehicle.cg_height, "unit": "m", "source": "vehicle mass rollup"},
        {"item": "wheelbase", "value": base_vehicle.wheelbase, "unit": "m", "source": "vehicle wheel centers"},
        {"item": "track_front", "value": base_vehicle.track_front, "unit": "m", "source": "vehicle wheel centers"},
        {"item": "track_rear", "value": base_vehicle.track_rear, "unit": "m", "source": "vehicle wheel centers"},
        {"item": "front_static_frac", "value": base_vehicle.front_static_frac, "unit": "fraction", "source": "CG over wheelbase"},
        {"item": "lltd", "value": base_vehicle.lltd, "unit": "front fraction", "source": "study assumption"},
        {"item": "cl_a", "value": base_vehicle.cl_a, "unit": "m^2", "source": "vehicle aero table"},
        {"item": "cd_a", "value": base_vehicle.cd_a, "unit": "m^2", "source": "vehicle aero table"},
        {"item": "aero_balance_front", "value": base_vehicle.aero_balance_front, "unit": "front fraction", "source": "study assumption"},
        {"item": "max_drive_power", "value": base_vehicle.max_drive_power, "unit": "W", "source": "study assumption"},
        {"item": "max_drive_force", "value": base_vehicle.max_drive_force, "unit": "N", "source": "study assumption"},
        {"item": "max_brake_force", "value": base_vehicle.max_brake_force, "unit": "N", "source": "study assumption"},
        {"item": "drive_distribution_front", "value": base_vehicle.drive_distribution_front, "unit": "front fraction", "source": "study assumption"},
        {"item": "brake_distribution_front", "value": base_vehicle.brake_distribution_front, "unit": "front fraction", "source": "study assumption"},
        {"item": "tire_file", "value": context["tire_path"], "unit": "", "source": "vehicle tire template"},
        {"item": "tire_fz_ref", "value": base_vehicle.fz_ref, "unit": "N", "source": "tire file"},
        {"item": "tire_fz_min_valid", "value": base_vehicle.fz_min_valid, "unit": "N", "source": "tire file"},
        {"item": "tire_fz_max_valid", "value": base_vehicle.fz_max_valid, "unit": "N", "source": "tire file"},
        {"item": "config_speeds", "value": " ".join(str(speed) for speed in config.speeds), "unit": "m/s", "source": "study config"},
        {"item": "config_ay_points", "value": config.ay_points, "unit": "count", "source": "study config"},
        {"item": "config_ax_search_points", "value": config.ax_search_points, "unit": "count", "source": "study config"},
    ]

    write_csv(OUTPUT_DIR / "baseline_characterization.csv", rows, ["item", "value", "unit", "source"])
    write_csv(
        OUTPUT_DIR / "mass_rollup.csv",
        context["mass_components"],
        [
            "component",
            "count",
            "unit_mass_kg",
            "total_mass_kg",
            "cg_x_m",
            "cg_y_m",
            "cg_z_m",
        ],
    )


def write_metric_catalog(metric_names: list[str]) -> None:
    rows = [
        {
            "metric": name,
            "display_name": metric_display_name(name),
            "unit": "g^2" if "area" in name else "g",
            "description": (
                "GGV envelope area between accel and brake branches"
                if "area" in name
                else "GGV-derived acceleration capability"
            ),
        }
        for name in metric_names
    ]
    write_csv(
        OUTPUT_DIR / "metric_catalog.csv",
        rows,
        ["metric", "display_name", "unit", "description"],
    )


def write_parameter_registry(specs: list[ParameterSpec]) -> None:
    rows = [dataclasses.asdict(spec) for spec in specs]
    write_csv(
        OUTPUT_DIR / "parameter_registry.csv",
        rows,
        ["name", "label", "unit", "baseline", "low", "high", "group", "description"],
    )


def plot_baseline_capability(metrics: dict[str, float], config: Any) -> None:
    speeds = list(config.speeds)
    lat = [metrics[f"max_lateral_g__{speed_label(speed)}"] for speed in speeds]
    accel = [metrics[f"max_accel_g__{speed_label(speed)}"] for speed in speeds]
    brake = [metrics[f"max_brake_g__{speed_label(speed)}"] for speed in speeds]
    area = [metrics[f"ggv_area_g2__{speed_label(speed)}"] for speed in speeds]

    fig, ax_left = plt.subplots(figsize=(9.5, 5.8))
    ax_left.plot(speeds, lat, marker="o", linewidth=2.2, label="max lateral")
    ax_left.plot(speeds, accel, marker="o", linewidth=2.2, label="max accel")
    ax_left.plot(speeds, brake, marker="o", linewidth=2.2, label="max brake")
    ax_left.set_xlabel("Speed (m/s)")
    ax_left.set_ylabel("Acceleration capability (g)")
    ax_left.grid(True, linestyle="--", alpha=0.35)

    ax_right = ax_left.twinx()
    ax_right.plot(
        speeds,
        area,
        marker="s",
        linewidth=2.0,
        linestyle=":",
        color="0.25",
        label="GGV area",
    )
    ax_right.set_ylabel("GGV area (g^2)")

    lines_left, labels_left = ax_left.get_legend_handles_labels()
    lines_right, labels_right = ax_right.get_legend_handles_labels()
    ax_left.legend(lines_left + lines_right, labels_left + labels_right, loc="best")
    ax_left.set_title("DS-001 Baseline EnvelopeSim Capability")

    fig.tight_layout()
    fig.savefig(PLOT_DIR / "baseline_capability_by_speed.png", dpi=220)
    plt.close(fig)


def plot_heatmap(
    matrix: np.ndarray,
    row_labels: list[str],
    column_labels: list[str],
    title: str,
    colorbar_label: str,
    output_path: Path,
    cmap: str = "coolwarm",
    symmetric: bool = True,
) -> None:
    finite = matrix[np.isfinite(matrix)]
    if finite.size == 0:
        limit = 1.0
    elif symmetric:
        limit = max(1e-9, float(np.nanmax(np.abs(finite))))
    else:
        limit = max(1e-9, float(np.nanmax(finite)))

    fig_width = max(12.0, 0.50 * len(column_labels))
    fig_height = max(7.0, 0.33 * len(row_labels))
    fig, ax = plt.subplots(figsize=(fig_width, fig_height))

    if symmetric:
        image = ax.imshow(matrix, aspect="auto", cmap=cmap, vmin=-limit, vmax=limit)
    else:
        image = ax.imshow(matrix, aspect="auto", cmap=cmap, vmin=0.0, vmax=limit)

    ax.set_xticks(np.arange(len(column_labels)))
    ax.set_xticklabels(column_labels, rotation=55, ha="right", fontsize=8)
    ax.set_yticks(np.arange(len(row_labels)))
    ax.set_yticklabels(row_labels, fontsize=8)
    ax.set_title(title)
    ax.set_xlabel("Response variable")
    ax.set_ylabel("Parameter")

    colorbar = fig.colorbar(image, ax=ax)
    colorbar.set_label(colorbar_label)

    fig.tight_layout()
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def plot_local_sensitivity_heatmap(
    specs: list[ParameterSpec],
    metric_names: list[str],
    sensitivity_rows: list[dict[str, Any]],
) -> None:
    lookup = {
        (str(row["parameter"]), str(row["metric"])): float(row["signed_effect_pct_span"])
        for row in sensitivity_rows
    }
    matrix = np.array(
        [
            [lookup.get((spec.name, metric), math.nan) for metric in metric_names]
            for spec in specs
        ],
        dtype=float,
    )
    plot_heatmap(
        matrix,
        [spec.label for spec in specs],
        [metric_display_name(metric) for metric in metric_names],
        "Local OFAT Response Sensitivity",
        "Signed response span from low to high (% of baseline)",
        PLOT_DIR / "response_sensitivity_heatmap.png",
    )


def plot_global_correlation_heatmap(
    specs: list[ParameterSpec],
    metric_names: list[str],
    correlation_rows: list[dict[str, Any]],
) -> None:
    lookup = {
        (str(row["parameter"]), str(row["metric"])): float(row["pearson_r"])
        for row in correlation_rows
    }
    matrix = np.array(
        [
            [lookup.get((spec.name, metric), math.nan) for metric in metric_names]
            for spec in specs
        ],
        dtype=float,
    )
    plot_heatmap(
        matrix,
        [spec.label for spec in specs],
        [metric_display_name(metric) for metric in metric_names],
        "Global Sample Parameter/Response Correlations",
        "Pearson r",
        PLOT_DIR / "global_correlation_heatmap.png",
    )


def plot_top_local_sensitivities(sensitivity_rows: list[dict[str, Any]]) -> None:
    sorted_rows = sorted(
        sensitivity_rows,
        key=lambda row: float(row["abs_effect_pct_span"])
        if math.isfinite(float(row["abs_effect_pct_span"]))
        else -1.0,
        reverse=True,
    )[:24]

    labels = [
        f"{row['parameter_label']} -> {metric_display_name(str(row['metric']))}"
        for row in sorted_rows
    ][::-1]
    values = [float(row["signed_effect_pct_span"]) for row in sorted_rows][::-1]
    colors = ["#247ba0" if value >= 0.0 else "#d1495b" for value in values]

    fig, ax = plt.subplots(figsize=(11.0, 8.0))
    ax.barh(labels, values, color=colors)
    ax.axvline(0.0, color="0.2", linewidth=0.8)
    ax.set_xlabel("Signed response span from low to high (% of baseline)")
    ax.set_title("Largest Local EnvelopeSim Sensitivities")
    ax.grid(True, axis="x", linestyle="--", alpha=0.3)

    fig.tight_layout()
    fig.savefig(PLOT_DIR / "top_local_sensitivities.png", dpi=220)
    plt.close(fig)


def baseline_metric_rows(metrics: dict[str, float]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for metric, value in metrics.items():
        rows.append(
            {
                "metric": metric,
                "display_name": metric_display_name(metric),
                "value": value,
                "unit": "g^2" if "area" in metric else "g",
            }
        )
    return rows


def local_sensitivity_rows(
    specs: list[ParameterSpec],
    metric_names: list[str],
    baseline_metrics: dict[str, float],
    low_metrics_by_parameter: dict[str, dict[str, float]],
    high_metrics_by_parameter: dict[str, dict[str, float]],
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    spec_lookup = {spec.name: spec for spec in specs}

    for spec in specs:
        low_metrics = low_metrics_by_parameter[spec.name]
        high_metrics = high_metrics_by_parameter[spec.name]
        parameter_span = spec.high - spec.low

        for metric in metric_names:
            base_response = float(baseline_metrics[metric])
            low_response = float(low_metrics[metric])
            high_response = float(high_metrics[metric])
            signed_span = high_response - low_response
            signed_effect_pct_span = finite_span_pct(
                high_response,
                low_response,
                base_response,
            )
            abs_effect_pct_span = abs(signed_effect_pct_span) if math.isfinite(signed_effect_pct_span) else math.nan
            slope_per_unit = (
                signed_span / parameter_span if abs(parameter_span) > 1e-12 else math.nan
            )

            if high_response > low_response:
                direction = "increases_with_parameter"
            elif high_response < low_response:
                direction = "decreases_with_parameter"
            else:
                direction = "flat"

            rows.append(
                {
                    "parameter": spec.name,
                    "parameter_label": spec.label,
                    "parameter_group": spec.group,
                    "metric": metric,
                    "metric_display_name": metric_display_name(metric),
                    "parameter_low": spec.low,
                    "parameter_baseline": spec.baseline,
                    "parameter_high": spec.high,
                    "response_low": low_response,
                    "response_baseline": base_response,
                    "response_high": high_response,
                    "signed_response_span": signed_span,
                    "slope_per_unit": slope_per_unit,
                    "signed_effect_pct_span": signed_effect_pct_span,
                    "abs_effect_pct_span": abs_effect_pct_span,
                    "low_delta_from_baseline_pct": finite_pct_delta(low_response, base_response),
                    "high_delta_from_baseline_pct": finite_pct_delta(high_response, base_response),
                    "direction": direction,
                }
            )

    for metric in metric_names:
        metric_rows = [row for row in rows if row["metric"] == metric]
        metric_rows.sort(
            key=lambda row: float(row["abs_effect_pct_span"])
            if math.isfinite(float(row["abs_effect_pct_span"]))
            else -1.0,
            reverse=True,
        )
        for rank, row in enumerate(metric_rows, start=1):
            row["rank_within_metric"] = rank

    for row in rows:
        row.setdefault("rank_within_metric", "")
        row["parameter_unit"] = spec_lookup[str(row["parameter"])].unit

    return rows


def run_local_sweep(
    base_vehicle: Any,
    config: Any,
    specs: list[ParameterSpec],
    metric_names: list[str],
    baseline_metrics: dict[str, float],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    case_rows: list[dict[str, Any]] = []
    low_metrics_by_parameter: dict[str, dict[str, float]] = {}
    high_metrics_by_parameter: dict[str, dict[str, float]] = {}

    base_values = baseline_values(specs)
    baseline_case = {
        "case_id": "baseline",
        "parameter": "",
        "level": "baseline",
        "value": "",
    }
    baseline_case.update(baseline_metrics)
    case_rows.append(baseline_case)

    total_cases = len(specs) * 2
    case_counter = 0
    for spec in specs:
        for level, value in (("low", spec.low), ("high", spec.high)):
            case_counter += 1
            values = dict(base_values)
            values[spec.name] = value
            print(
                f"  OFAT {case_counter:02d}/{total_cases}: "
                f"{spec.name}={format_float(value, 5)}"
            )
            metrics, _envelopes = run_envelope_case(base_vehicle, config, values)

            if level == "low":
                low_metrics_by_parameter[spec.name] = metrics
            else:
                high_metrics_by_parameter[spec.name] = metrics

            row = {
                "case_id": f"{spec.name}_{level}",
                "parameter": spec.name,
                "level": level,
                "value": value,
            }
            row.update(metrics)
            case_rows.append(row)

    sensitivity = local_sensitivity_rows(
        specs,
        metric_names,
        baseline_metrics,
        low_metrics_by_parameter,
        high_metrics_by_parameter,
    )

    return case_rows, sensitivity


def run_global_sample(
    base_vehicle: Any,
    config: Any,
    specs: list[ParameterSpec],
    metric_names: list[str],
    sample_count: int,
    seed: int,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    samples = generate_lhs(specs, sample_count, seed)
    sample_rows: list[dict[str, Any]] = []
    metric_rows: list[dict[str, Any]] = []
    parameter_series = {spec.name: [] for spec in specs}
    metric_series = {metric: [] for metric in metric_names}

    for idx, values in enumerate(samples, start=1):
        print(f"  Global sample {idx:02d}/{sample_count}")
        metrics, _envelopes = run_envelope_case(base_vehicle, config, values)

        sample_row = {"sample_id": idx}
        sample_row.update(values)
        sample_rows.append(sample_row)

        metric_row = {"sample_id": idx}
        metric_row.update(metrics)
        metric_rows.append(metric_row)

        for spec in specs:
            parameter_series[spec.name].append(values[spec.name])
        for metric in metric_names:
            metric_series[metric].append(metrics[metric])

    correlation_rows: list[dict[str, Any]] = []
    for spec in specs:
        for metric in metric_names:
            r_value = pearson(parameter_series[spec.name], metric_series[metric])
            correlation_rows.append(
                {
                    "parameter": spec.name,
                    "parameter_label": spec.label,
                    "parameter_group": spec.group,
                    "metric": metric,
                    "metric_display_name": metric_display_name(metric),
                    "pearson_r": r_value,
                    "abs_pearson_r": abs(r_value) if math.isfinite(r_value) else math.nan,
                }
            )

    for metric in metric_names:
        metric_rows_for_rank = [row for row in correlation_rows if row["metric"] == metric]
        metric_rows_for_rank.sort(
            key=lambda row: float(row["abs_pearson_r"])
            if math.isfinite(float(row["abs_pearson_r"]))
            else -1.0,
            reverse=True,
        )
        for rank, row in enumerate(metric_rows_for_rank, start=1):
            row["rank_within_metric"] = rank

    return sample_rows, metric_rows, correlation_rows


def table_line(values: list[str]) -> str:
    return "| " + " | ".join(values) + " |"


def top_rows_for_metrics(
    rows: list[dict[str, Any]],
    metric_names: list[str],
    value_key: str,
    rank_key: str = "rank_within_metric",
) -> list[dict[str, Any]]:
    top_rows: list[dict[str, Any]] = []
    for metric in metric_names:
        candidates = [row for row in rows if row["metric"] == metric]
        if not candidates:
            continue
        candidates.sort(
            key=lambda row: int(row.get(rank_key, 999999))
            if str(row.get(rank_key, "")).isdigit()
            else 999999
        )
        top_rows.append(candidates[0])
    return top_rows


def write_report(
    base_vehicle: Any,
    context: dict[str, Any],
    config: Any,
    specs: list[ParameterSpec],
    metric_names: list[str],
    baseline_metrics: dict[str, float],
    sensitivity_rows: list[dict[str, Any]],
    correlation_rows: list[dict[str, Any]],
    sample_count: int,
    started_at: str,
    elapsed_s: float,
) -> None:
    speed_rows = []
    for speed in config.speeds:
        suffix = speed_label(float(speed))
        speed_rows.append(
            [
                f"{speed:.0f}",
                format_float(baseline_metrics[f"max_lateral_g__{suffix}"], 3),
                format_float(baseline_metrics[f"max_accel_g__{suffix}"], 3),
                format_float(baseline_metrics[f"max_brake_g__{suffix}"], 3),
                format_float(baseline_metrics[f"ggv_area_g2__{suffix}"], 3),
            ]
        )

    top_local = top_rows_for_metrics(sensitivity_rows, metric_names, "abs_effect_pct_span")
    top_global = top_rows_for_metrics(correlation_rows, metric_names, "abs_pearson_r")

    summary_metrics = [
        "mean_max_lateral_g",
        "mean_max_accel_g",
        "mean_max_brake_g",
        "mean_ggv_area_g2",
        "max_lateral_g__25mps",
        "max_accel_g__25mps",
        "max_brake_g__25mps",
        "ggv_area_g2__25mps",
    ]
    top_local_summary = [
        row for row in top_local if str(row["metric"]) in set(summary_metrics)
    ]
    top_global_summary = [
        row for row in top_global if str(row["metric"]) in set(summary_metrics)
    ]

    local_frequency: dict[str, int] = {}
    for row in top_local:
        local_frequency[str(row["parameter_label"])] = (
            local_frequency.get(str(row["parameter_label"]), 0) + 1
        )
    most_common_local = sorted(local_frequency.items(), key=lambda item: item[1], reverse=True)

    lines: list[str] = []
    lines.append("# DS-001 EnvelopeSim Parameter Sensitivity")
    lines.append("")
    lines.append(f"Generated UTC: {started_at}")
    lines.append("")
    lines.append("## Source of Results")
    lines.append("")
    lines.append(
        "All response metrics in this report are generated by BobSim EnvelopeSim "
        "using `BobSim/_2_EnvelopeSim/GGV/ggv_generation.py`."
    )
    lines.append("")
    lines.append("## Sensitivity Definition")
    lines.append("")
    lines.append(
        "Each local sensitivity is one response variable with respect to one "
        "varied EnvelopeSim parameter."
    )
    lines.append("")
    lines.append(table_line(["Term", "Meaning"]))
    lines.append(table_line(["---", "---"]))
    lines.append(table_line(["Independent variable", "The parameter listed in `outputs/parameter_registry.csv`."]))
    lines.append(table_line(["Response variable", "The metric listed in `outputs/metric_catalog.csv`."]))
    lines.append(table_line(["Local cases", "One low-parameter EnvelopeSim run, one baseline run, and one high-parameter EnvelopeSim run."]))
    lines.append(table_line(["Raw response span", "`response_high - response_low`."]))
    lines.append(table_line(["Slope per unit", "`(response_high - response_low) / (parameter_high - parameter_low)`."]))
    lines.append(table_line(["Normalized span", "`100 * (response_high - response_low) / abs(response_baseline)`."]))
    lines.append("")
    lines.append(
        "The global sample reports Pearson correlation between each parameter and "
        "each response variable across the Latin-hypercube EnvelopeSim sample. "
        "That is a ranking tool, not a local derivative."
    )
    lines.append("")
    lines.append(
        "Because the local normalized span is computed over the selected low/high "
        "parameter envelope, importance rankings are sensitive to the chosen "
        "parameter bounds. Use `slope_per_unit` for the derivative-style question "
        "and `signed_effect_pct_span` for the design-space-impact question."
    )
    lines.append("")
    lines.append("## Baseline Characterization")
    lines.append("")
    lines.append(table_line(["Item", "Value"]))
    lines.append(table_line(["---", "---"]))
    lines.append(table_line(["Vehicle", str(context["vehicle_name"])]))
    lines.append(table_line(["Mass", f"{base_vehicle.mass:.2f} kg"]))
    lines.append(table_line(["CG", f"x={context['mass_cg_x_m']:.4f} m, z={base_vehicle.cg_height:.4f} m"]))
    lines.append(table_line(["Wheelbase", f"{base_vehicle.wheelbase:.4f} m"]))
    lines.append(table_line(["Front/rear track", f"{base_vehicle.track_front:.4f} / {base_vehicle.track_rear:.4f} m"]))
    lines.append(table_line(["Front static fraction", f"{base_vehicle.front_static_frac:.4f}"]))
    lines.append(table_line(["LLTD", f"{base_vehicle.lltd:.4f} (study assumption)"]))
    lines.append(table_line(["ClA/CdA", f"{base_vehicle.cl_a:.4f} / {base_vehicle.cd_a:.4f} m^2"]))
    lines.append(table_line(["Aero balance", f"{base_vehicle.aero_balance_front:.4f} front (study assumption)"]))
    lines.append(table_line(["Drive", f"{base_vehicle.max_drive_power / 1000.0:.1f} kW, {base_vehicle.max_drive_force:.0f} N, front fraction {base_vehicle.drive_distribution_front:.2f}"]))
    lines.append(table_line(["Brakes", f"{base_vehicle.max_brake_force:.0f} N, front fraction {base_vehicle.brake_distribution_front:.2f}"]))
    lines.append(table_line(["Tire", str(context["tire_path"])]))
    lines.append("")
    lines.append("## Baseline Envelope Metrics")
    lines.append("")
    lines.append(table_line(["Speed m/s", "Max lateral g", "Max accel g", "Max brake g", "GGV area g^2"]))
    lines.append(table_line(["---", "---:", "---:", "---:", "---:"]))
    for row in speed_rows:
        lines.append(table_line(row))
    lines.append("")
    lines.append("## Local Sensitivity Read")
    lines.append("")
    if most_common_local:
        common_text = ", ".join(
            f"{label} ({count} response variables)" for label, count in most_common_local[:5]
        )
        lines.append(
            "Most frequently top-ranked local drivers across response variables: "
            f"{common_text}."
        )
        lines.append("")
    lines.append(table_line(["Response", "Top local parameter", "Direction", "Signed span %", "Low", "High"]))
    lines.append(table_line(["---", "---", "---", "---:", "---:", "---:"]))
    for row in top_local_summary:
        lines.append(
            table_line(
                [
                    str(row["metric_display_name"]),
                    str(row["parameter_label"]),
                    str(row["direction"]),
                    format_float(float(row["signed_effect_pct_span"]), 2),
                    format_float(float(row["response_low"]), 4),
                    format_float(float(row["response_high"]), 4),
                ]
            )
        )
    lines.append("")
    lines.append(
        "The complete per-response local sensitivity matrix is in "
        "`outputs/metric_sensitivity_matrix.csv`."
    )
    lines.append("")
    lines.append("## Global Sample Read")
    lines.append("")
    lines.append(
        f"The deterministic Latin-hypercube sample used {sample_count} EnvelopeSim cases "
        f"over {len(specs)} parameters."
    )
    lines.append("")
    lines.append(table_line(["Response", "Top global correlate", "Pearson r"]))
    lines.append(table_line(["---", "---", "---:"]))
    for row in top_global_summary:
        lines.append(
            table_line(
                [
                    str(row["metric_display_name"]),
                    str(row["parameter_label"]),
                    format_float(float(row["pearson_r"]), 3),
                ]
            )
        )
    lines.append("")
    lines.append(
        "The complete global parameter-response correlation matrix is in "
        "`outputs/global_metric_correlations.csv`."
    )
    lines.append("")
    lines.append("## Generated Files")
    lines.append("")
    for relative_path in [
        "outputs/baseline_characterization.csv",
        "outputs/baseline_metrics.csv",
        "outputs/baseline_ggv.csv",
        "outputs/parameter_registry.csv",
        "outputs/ofat_cases.csv",
        "outputs/metric_sensitivity_matrix.csv",
        "outputs/global_samples.csv",
        "outputs/global_metrics.csv",
        "outputs/global_metric_correlations.csv",
        "plots/baseline_capability_by_speed.png",
        "plots/response_sensitivity_heatmap.png",
        "plots/top_local_sensitivities.png",
        "plots/global_correlation_heatmap.png",
    ]:
        lines.append(f"- `{relative_path}`")
    lines.append("")
    lines.append("## Run Provenance")
    lines.append("")
    lines.append(table_line(["Item", "Value"]))
    lines.append(table_line(["---", "---"]))
    lines.append(table_line(["EnvelopeSim speeds", ", ".join(str(speed) for speed in config.speeds)]))
    lines.append(table_line(["ay points", str(config.ay_points)]))
    lines.append(table_line(["ax search points", str(config.ax_search_points)]))
    lines.append(table_line(["OFAT cases", str(len(specs) * 2 + 1)]))
    lines.append(table_line(["Global cases", str(sample_count)]))
    lines.append(table_line(["Elapsed time", f"{elapsed_s:.1f} s"]))
    lines.append("")

    report_text = "\n".join(lines)
    (STUDY_DIR / "RESULTS.md").write_text(report_text, encoding="utf-8")
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(report_text, encoding="utf-8")


def write_run_provenance(
    sample_count: int,
    spec_count: int,
    metric_count: int,
    elapsed_s: float,
    started_at: str,
) -> None:
    rows = [
        {"item": "started_at_utc", "value": started_at},
        {"item": "engine", "value": "BobSim EnvelopeSim GGV"},
        {"item": "engine_path", "value": "BobSim/_2_EnvelopeSim/GGV/ggv_generation.py"},
        {"item": "parameter_count", "value": spec_count},
        {"item": "response_metric_count", "value": metric_count},
        {"item": "ofat_case_count", "value": spec_count * 2 + 1},
        {"item": "global_sample_count", "value": sample_count},
        {"item": "elapsed_seconds", "value": f"{elapsed_s:.3f}"},
    ]
    write_csv(OUTPUT_DIR / "run_provenance.csv", rows, ["item", "value"])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=48, help="Global LHS sample count.")
    parser.add_argument("--seed", type=int, default=4107, help="Deterministic random seed.")
    parser.add_argument(
        "--vehicle",
        type=Path,
        default=REPO_ROOT / "vehicles" / "current" / "vehicle.yml",
        help="Vehicle YAML file.",
    )
    args = parser.parse_args()

    start_time = time.perf_counter()
    started_at = datetime.now(timezone.utc).isoformat(timespec="seconds")
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    PLOT_DIR.mkdir(parents=True, exist_ok=True)

    print("DS-001 EnvelopeSim parameter sensitivity")
    print(f"  Vehicle: {args.vehicle.relative_to(REPO_ROOT)}")
    print(f"  Global samples: {args.samples}")

    base_vehicle, context = build_baseline_vehicle(args.vehicle)
    specs = make_parameter_specs(base_vehicle)
    config = make_config()
    values = baseline_values(specs)

    write_baseline_characterization(base_vehicle, context, config, started_at)
    write_parameter_registry(specs)

    print("Running baseline EnvelopeSim case")
    baseline_metrics, baseline_envelopes = run_envelope_case(base_vehicle, config, values)
    metric_names = list(baseline_metrics.keys())
    write_metric_catalog(metric_names)

    ggv.save_ggv_csv(baseline_envelopes, OUTPUT_DIR / "baseline_ggv.csv")
    write_csv(
        OUTPUT_DIR / "baseline_metrics.csv",
        baseline_metric_rows(baseline_metrics),
        ["metric", "display_name", "value", "unit"],
    )
    plot_baseline_capability(baseline_metrics, config)

    print("Running local one-factor parameter sweep")
    ofat_cases, sensitivity = run_local_sweep(
        base_vehicle,
        config,
        specs,
        metric_names,
        baseline_metrics,
    )
    case_fields = ["case_id", "parameter", "level", "value"] + metric_names
    write_csv(OUTPUT_DIR / "ofat_cases.csv", ofat_cases, case_fields)
    sensitivity_fields = [
        "parameter",
        "parameter_label",
        "parameter_group",
        "parameter_unit",
        "metric",
        "metric_display_name",
        "parameter_low",
        "parameter_baseline",
        "parameter_high",
        "response_low",
        "response_baseline",
        "response_high",
        "signed_response_span",
        "slope_per_unit",
        "signed_effect_pct_span",
        "abs_effect_pct_span",
        "low_delta_from_baseline_pct",
        "high_delta_from_baseline_pct",
        "direction",
        "rank_within_metric",
    ]
    write_csv(OUTPUT_DIR / "metric_sensitivity_matrix.csv", sensitivity, sensitivity_fields)
    plot_local_sensitivity_heatmap(specs, metric_names, sensitivity)
    plot_top_local_sensitivities(sensitivity)

    print("Running global parameter sample")
    sample_rows, metric_rows, correlations = run_global_sample(
        base_vehicle,
        config,
        specs,
        metric_names,
        args.samples,
        args.seed,
    )
    sample_fields = ["sample_id"] + [spec.name for spec in specs]
    metric_fields = ["sample_id"] + metric_names
    correlation_fields = [
        "parameter",
        "parameter_label",
        "parameter_group",
        "metric",
        "metric_display_name",
        "pearson_r",
        "abs_pearson_r",
        "rank_within_metric",
    ]
    write_csv(OUTPUT_DIR / "global_samples.csv", sample_rows, sample_fields)
    write_csv(OUTPUT_DIR / "global_metrics.csv", metric_rows, metric_fields)
    write_csv(OUTPUT_DIR / "global_metric_correlations.csv", correlations, correlation_fields)
    plot_global_correlation_heatmap(specs, metric_names, correlations)

    elapsed_s = time.perf_counter() - start_time
    write_run_provenance(args.samples, len(specs), len(metric_names), elapsed_s, started_at)
    write_report(
        base_vehicle,
        context,
        config,
        specs,
        metric_names,
        baseline_metrics,
        sensitivity,
        correlations,
        args.samples,
        started_at,
        elapsed_s,
    )

    print(f"Complete in {elapsed_s:.1f} s")
    print(f"Study report: {STUDY_DIR / 'RESULTS.md'}")
    print(f"Top-level report: {REPORT_PATH}")


if __name__ == "__main__":
    main()
