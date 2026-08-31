"""Physically tune dyn_py sweeps or explicit vehicle configurations.

For each requested CG height and reduced model, the script redistributes the
existing front-plus-rear anti-roll stiffness, leaving wheel spring rates and
total ARB stiffness unchanged.  It then fits one fixed brake bias to a common
distance-weighted braking-speed distribution.  All changes are analysis-only;
``vehicle.yml`` is never modified.

SPDX-License-Identifier: GPL-3.0-or-later
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict, replace
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
from scipy.optimize import minimize_scalar

LAPSIMS_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = LAPSIMS_ROOT.parents[2]
DEFAULT_BOBSIM_ROOT = WORKSPACE_ROOT / "BobDyn" / "BobSim"
DEFAULT_REFERENCE_CASE = (
    LAPSIMS_ROOT / "outputs" / "ggv_cg_smoke_midpoint_cop" / "baseline"
)
DEFAULT_OUTPUT = (
    LAPSIMS_ROOT / "inputs" / "ggv_cg_height_sweep_dyn_py_physical_tuning.json"
)
DEFAULT_RUNNER_3DOF = (
    LAPSIMS_ROOT / "inputs" / "ggv_cg_height_sweep_8_to_14in_dyn_py_3dof.json"
)
DEFAULT_RUNNER_6DOF = (
    LAPSIMS_ROOT / "inputs" / "ggv_cg_height_sweep_8_to_14in_dyn_py_6dof.json"
)
DEFAULT_HEIGHTS_IN = (8.0, 9.2, 10.4, 11.6, 12.8, 14.0)
DEFAULT_MODEL_DOFS = (3, 6)
DEFAULT_TIRE_MU_SCALE = 1.0
DEFAULT_FIXED_CG_HEIGHT_IN = 11.5
LB_TO_KG = 0.45359237
BRAKING_EVENTS = ("autocross", "michigan_endurance")
ROBUSTNESS_SPEEDS_MPS = (8.0, 11.8, 18.0, 25.0)
G = 9.80665
SAFE_CASE_NAME = re.compile(r"[A-Za-z0-9._-]+")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bobsim-root", type=Path, default=DEFAULT_BOBSIM_ROOT)
    parser.add_argument(
        "--reference-case-dir",
        type=Path,
        default=DEFAULT_REFERENCE_CASE,
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--runner-3dof", type=Path, default=DEFAULT_RUNNER_3DOF)
    parser.add_argument("--runner-6dof", type=Path, default=DEFAULT_RUNNER_6DOF)
    parser.add_argument(
        "--heights-in",
        type=float,
        nargs="+",
        default=DEFAULT_HEIGHTS_IN,
    )
    parser.add_argument(
        "--mass-offsets-lb",
        type=float,
        nargs="+",
        default=None,
        help=(
            "Run a sprung-mass sweep at --fixed-cg-height-in using offsets from "
            "the projected nominal sprung mass. Mutually exclusive with an "
            "explicit --heights-in value."
        ),
    )
    parser.add_argument(
        "--rear-weight-fractions",
        type=float,
        nargs="+",
        default=None,
        help=(
            "Run a total static rear-axle weight-fraction sweep at "
            "--fixed-cg-height-in and nominal mass. Mutually exclusive with "
            "--mass-offsets-lb."
        ),
    )
    parser.add_argument(
        "--configuration-json",
        type=Path,
        default=None,
        help=(
            "Tune explicitly named vehicle configurations whose CG height and "
            "static rear-weight fraction may both differ. The JSON must declare "
            "sweep_axis='configuration' and an explicit reference_case."
        ),
    )
    parser.add_argument(
        "--fixed-cg-height-in",
        type=float,
        default=DEFAULT_FIXED_CG_HEIGHT_IN,
        help="Fixed total CG height for a sprung-mass sweep (default: 11.5 in).",
    )
    parser.add_argument(
        "--model-dofs",
        type=int,
        nargs="+",
        choices=DEFAULT_MODEL_DOFS,
        default=DEFAULT_MODEL_DOFS,
        help="Reduced-order models to tune (default: both 3 and 6 DOF).",
    )
    parser.add_argument(
        "--tire-mu-scale",
        type=float,
        default=DEFAULT_TIRE_MU_SCALE,
        help=(
            "Temporary multiplicative scale applied to reduced-model tire "
            "friction coefficients; vehicle.yml is not modified."
        ),
    )
    parser.add_argument("--workers", type=int, default=min(6, os.cpu_count() or 1))
    return parser.parse_args()


def _case_name(height_in: float) -> str:
    return f"cg_{height_in:.1f}in".replace(".", "p")


def _mass_case_name(offset_lb: float) -> str:
    if math.isclose(offset_lb, 0.0, rel_tol=0.0, abs_tol=1e-12):
        return "mass_nominal"
    sign = "plus" if offset_lb > 0.0 else "minus"
    magnitude = f"{abs(offset_lb):g}".replace(".", "p")
    return f"mass_{sign}_{magnitude}lb"


def _rear_weight_case_name(rear_fraction: float) -> str:
    percentage = f"{100.0 * rear_fraction:g}".replace(".", "p")
    return f"rear_{percentage}pct"


def _json_default(value: object) -> object:
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(f"Cannot serialize {type(value).__name__}")


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, default=_json_default) + "\n",
        encoding="utf-8",
    )


def _load_configuration_definition(
    path: Path,
) -> tuple[dict[str, Any], list[dict[str, Any]], str]:
    """Load explicit multi-parameter cases for physical setup tuning."""

    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise TypeError("Configuration JSON must contain an object.")
    if payload.get("sweep_axis") != "configuration":
        raise ValueError("Configuration JSON must declare sweep_axis='configuration'.")
    raw_cases = payload.get("cases")
    if not isinstance(raw_cases, list) or len(raw_cases) < 2:
        raise ValueError(
            "Configuration JSON 'cases' must contain at least two entries."
        )

    case_specs: list[dict[str, Any]] = []
    names: set[str] = set()
    mass_presence: list[bool] = []
    for index, raw_case in enumerate(raw_cases):
        if not isinstance(raw_case, dict):
            raise TypeError(f"Configuration case {index} must be an object.")
        name = raw_case.get("name")
        if not isinstance(name, str) or not SAFE_CASE_NAME.fullmatch(name):
            raise ValueError(
                f"Configuration case {index}.name must use only letters, digits, "
                "'.', '_', or '-'."
            )
        if name in names:
            raise ValueError(f"Configuration case name {name!r} is duplicated.")
        names.add(name)

        supplied_height_fields = [
            field for field in ("cg_height_m", "cg_height_in") if field in raw_case
        ]
        if len(supplied_height_fields) != 1:
            raise ValueError(
                f"Configuration case {name!r} must contain exactly one of "
                "cg_height_m or cg_height_in."
            )
        height_value = raw_case[supplied_height_fields[0]]
        if (
            isinstance(height_value, bool)
            or not isinstance(height_value, (int, float))
            or not math.isfinite(float(height_value))
            or float(height_value) <= 0.0
        ):
            raise ValueError(
                f"Configuration case {name!r} CG height must be finite and positive."
            )
        height_in = float(height_value)
        if supplied_height_fields[0] == "cg_height_m":
            height_in /= 0.0254

        rear_fraction = raw_case.get("rear_static_weight_fraction")
        if (
            isinstance(rear_fraction, bool)
            or not isinstance(rear_fraction, (int, float))
            or not math.isfinite(float(rear_fraction))
            or not 0.0 < float(rear_fraction) < 1.0
        ):
            raise ValueError(
                f"Configuration case {name!r}.rear_static_weight_fraction must "
                "be finite and strictly between 0 and 1."
            )

        raw_sprung_mass = raw_case.get("sprung_mass_kg")
        if raw_sprung_mass is None:
            sprung_mass_kg = None
        elif (
            isinstance(raw_sprung_mass, bool)
            or not isinstance(raw_sprung_mass, (int, float))
            or not math.isfinite(float(raw_sprung_mass))
            or float(raw_sprung_mass) <= 0.0
        ):
            raise ValueError(
                f"Configuration case {name!r}.sprung_mass_kg must be finite and "
                "positive when supplied."
            )
        else:
            sprung_mass_kg = float(raw_sprung_mass)
        mass_presence.append(sprung_mass_kg is not None)
        case_specs.append(
            {
                "name": name,
                "height_in": height_in,
                "target_sprung_mass_kg": sprung_mass_kg,
                "static_rear_weight_fraction": float(rear_fraction),
                "configuration_order": index,
            }
        )

    if any(mass_presence) and not all(mass_presence):
        raise ValueError(
            "Configuration sprung_mass_kg must be supplied for every case or "
            "omitted for every case (nominal mass)."
        )
    physical_keys = {
        (
            round(float(case["height_in"]), 12),
            None
            if case["target_sprung_mass_kg"] is None
            else round(float(case["target_sprung_mass_kg"]), 12),
            round(float(case["static_rear_weight_fraction"]), 12),
        )
        for case in case_specs
    }
    if len(physical_keys) != len(case_specs):
        raise ValueError("Configuration cases must have unique physical inputs.")

    reference_case = payload.get("reference_case")
    if not isinstance(reference_case, str) or reference_case not in names:
        raise ValueError(
            "Configuration JSON reference_case must explicitly name one case."
        )
    return payload, case_specs, reference_case


def _load_braking_distribution(
    reference_case_dir: Path,
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    frames: list[pd.DataFrame] = []
    event_rows: dict[str, int] = {}
    event_distances: dict[str, float] = {}
    source_files: list[str] = []
    for event_slug in BRAKING_EVENTS:
        source = reference_case_dir / f"{event_slug}_trace.csv"
        trace = pd.read_csv(source)
        braking = trace.loc[
            trace["longitudinal_accel_mps2"] < -0.1,
            ["speed_mps", "dx_m"],
        ].copy()
        if braking.empty:
            raise ValueError(f"No braking rows found in {source}.")
        frames.append(braking)
        source_files.append(str(source.resolve()))
        event_rows[event_slug] = len(braking)
        event_distances[event_slug] = float(braking["dx_m"].sum())

    pooled = pd.concat(frames, ignore_index=True)
    pooled["speed_bin"] = np.floor(pooled["speed_mps"] / 0.5).astype(int)
    groups = [group for _, group in pooled.groupby("speed_bin", sort=True)]
    speeds = np.asarray(
        [np.average(group["speed_mps"], weights=group["dx_m"]) for group in groups],
        dtype=float,
    )
    weights = np.asarray(
        [float(group["dx_m"].sum()) for group in groups],
        dtype=float,
    )
    metadata = {
        "common_to_all_models_and_heights": True,
        "source_case_dir": str(reference_case_dir.resolve()),
        "source_files": source_files,
        "events": list(BRAKING_EVENTS),
        "row_filter": "longitudinal_accel_mps2 < -0.1",
        "weighting": "braking distance dx_m",
        "speed_bin_width_mps": 0.5,
        "speed_bin_rule": "floor(speed / 0.5)",
        "representative_speed": (
            "distance-weighted mean observed speed within each bin"
        ),
        "event_filtered_rows": event_rows,
        "event_braking_distance_m": event_distances,
        "filtered_rows": len(pooled),
        "braking_distance_m": float(np.sum(weights)),
        "speed_bin_count": len(speeds),
        "minimum_representative_speed_mps": float(np.min(speeds)),
        "maximum_representative_speed_mps": float(np.max(speeds)),
        "distance_weighted_mean_speed_mps": float(np.average(speeds, weights=weights)),
    }
    return speeds, weights, metadata


def _load_context(
    bobsim_root: Path,
    *,
    height_m: float,
    target_sprung_mass_kg: float | None = None,
    static_rear_weight_fraction: float | None = None,
    front_arb_fraction: float,
    front_brake_fraction: float,
    tire_mu_scale: float,
) -> tuple[Any, Any]:
    if str(bobsim_root) not in sys.path:
        sys.path.insert(0, str(bobsim_root))

    from _0_Utils.dyn_py import (
        ReducedVehicleOverrides,
        apply_reduced_vehicle_overrides,
        load_reduced_vehicle_parameters,
    )
    from _2_EnvelopeSim.vehicle_yaml import load_vehicle_yaml, project_vehicle_yaml

    projection = project_vehicle_yaml(
        load_vehicle_yaml(bobsim_root / "vehicle.yml"),
        repo_root=bobsim_root,
        aero_balance_front=0.5,
    )
    base_parameters = load_reduced_vehicle_parameters(bobsim_root / "vehicle.yml")
    parameters = apply_reduced_vehicle_overrides(
        base_parameters,
        ReducedVehicleOverrides(
            absolute_cg_height_m=height_m,
            target_sprung_mass_kg=target_sprung_mass_kg,
            static_rear_weight_fraction=static_rear_weight_fraction,
            aero_balance_front=0.5,
            brake_distribution_front=front_brake_fraction,
            front_antiroll_stiffness_fraction=front_arb_fraction,
            tire_mu_scale=tire_mu_scale,
        ),
    )
    outer_vehicle = replace(
        projection.ggv,
        mass=parameters.mass_kg,
        cg_height=height_m,
        front_static_frac=parameters.static_front_weight_fraction,
        brake_distribution_front=front_brake_fraction,
    )
    return outer_vehicle, parameters


def _lateral_limit(
    bobsim_root: Path,
    *,
    model_dof: int,
    height_m: float,
    target_sprung_mass_kg: float | None = None,
    static_rear_weight_fraction: float | None = None,
    front_arb_fraction: float,
    tire_mu_scale: float,
    speed_mps: float,
    binary_iterations: int,
) -> float:
    outer_vehicle, parameters = _load_context(
        bobsim_root,
        height_m=height_m,
        target_sprung_mass_kg=target_sprung_mass_kg,
        static_rear_weight_fraction=static_rear_weight_fraction,
        front_arb_fraction=front_arb_fraction,
        front_brake_fraction=0.84,
        tire_mu_scale=tire_mu_scale,
    )
    from _0_Utils.dyn_py import create_model
    from _2_EnvelopeSim.GGV.ggv_generation import solve_lateral_limit

    model = create_model(model_dof, parameters)
    lateral_limit, endpoint_ax = solve_lateral_limit(
        outer_vehicle,
        speed=speed_mps,
        ay_upper=3.2 * G,
        reduced_model=model,
        max_abs_beta_rad=0.25,
        max_abs_steering_rad=0.5,
        binary_iterations=binary_iterations,
    )
    if not math.isfinite(lateral_limit) or lateral_limit <= 0.0:
        raise RuntimeError("Robust pure-lateral endpoint solve found no valid limit.")
    if math.isclose(lateral_limit, 3.2 * G, rel_tol=0.0, abs_tol=1e-9):
        raise RuntimeError("Lateral search upper bound is active; expand it.")
    if not math.isclose(endpoint_ax, 0.0, rel_tol=0.0, abs_tol=1e-12):
        raise RuntimeError("Robust lateral endpoint did not retain body ax=0.")
    return float(lateral_limit)


def _lateral_trim_diagnostics(
    bobsim_root: Path,
    *,
    model_dof: int,
    height_m: float,
    target_sprung_mass_kg: float | None = None,
    static_rear_weight_fraction: float | None = None,
    front_arb_fraction: float,
    tire_mu_scale: float,
    speed_mps: float,
    lateral_acceleration_mps2: float,
) -> dict[str, Any]:
    from _0_Utils.dyn_py import create_model, solve_acceleration_trim
    from _2_EnvelopeSim.GGV.ggv_generation import _trim_is_racing_feasible

    _outer_vehicle, parameters = _load_context(
        bobsim_root,
        height_m=height_m,
        target_sprung_mass_kg=target_sprung_mass_kg,
        static_rear_weight_fraction=static_rear_weight_fraction,
        front_arb_fraction=front_arb_fraction,
        front_brake_fraction=0.84,
        tire_mu_scale=tire_mu_scale,
    )
    model = create_model(model_dof, parameters)
    trim = solve_acceleration_trim(
        model,
        speed_mps=speed_mps,
        longitudinal_acceleration_mps2=0.0,
        lateral_acceleration_mps2=lateral_acceleration_mps2,
        max_nfev=240,
        tolerance=1e-8,
    )
    racing_feasible = _trim_is_racing_feasible(
        trim,
        model=model,
        ay=lateral_acceleration_mps2,
        max_abs_beta_rad=0.25,
        max_abs_steering_rad=0.5,
    )
    normal_loads = np.asarray(trim.output.normal_loads_n, dtype=float)
    minimum_normal_load = float(np.min(normal_loads))
    maximum_normal_load = float(np.max(normal_loads))
    beta = float(trim.unknowns["beta_rad"])
    steering = float(trim.unknowns["steering_rad"])
    sideslip_bound_active = abs(beta) >= 0.25 - 1e-4
    steering_bound_active = abs(steering) >= 0.5 - 1e-4
    wheel_lift_boundary_active = minimum_normal_load <= 0.1
    active_constraints = [
        label
        for label, active in (
            ("sideslip_bound", sideslip_bound_active),
            ("steering_bound", steering_bound_active),
            ("wheel_lift_boundary", wheel_lift_boundary_active),
        )
        if active
    ]
    return {
        "racing_feasible": racing_feasible,
        "solver_success": bool(trim.success),
        "residual_norm": float(trim.residual_norm),
        "minimum_normal_load_n": minimum_normal_load,
        "maximum_normal_load_n": maximum_normal_load,
        "normal_loads_n": normal_loads.tolist(),
        "below_tir_minimum_load": bool(minimum_normal_load < parameters.tire.fz_min_n),
        "above_tir_maximum_load": bool(maximum_normal_load > parameters.tire.fz_max_n),
        "tir_valid_load_min_n": float(parameters.tire.fz_min_n),
        "tir_valid_load_max_n": float(parameters.tire.fz_max_n),
        "beta_rad": beta,
        "steering_rad": steering,
        "total_wheel_torque_nm": float(trim.unknowns["total_wheel_torque_nm"]),
        "active_constraints": active_constraints,
        "active_constraint_flags": {
            "sideslip_bound_active": sideslip_bound_active,
            "sideslip_bound_abs_rad": 0.25,
            "steering_bound_active": steering_bound_active,
            "steering_bound_abs_rad": 0.5,
            "wheel_lift_boundary_active": wheel_lift_boundary_active,
            "wheel_lift_boundary_threshold_n": 0.1,
        },
    }


def _tune_front_arb_fraction(
    bobsim_root: Path,
    *,
    model_dof: int,
    height_m: float,
    target_sprung_mass_kg: float | None = None,
    static_rear_weight_fraction: float | None = None,
    tire_mu_scale: float,
) -> dict[str, Any]:
    cache: dict[tuple[float, int], float] = {}

    def evaluate(fraction: float, iterations: int) -> float:
        bounded = float(np.clip(fraction, 0.0, 1.0))
        key = (round(bounded, 12), iterations)
        if key not in cache:
            cache[key] = _lateral_limit(
                bobsim_root,
                model_dof=model_dof,
                height_m=height_m,
                target_sprung_mass_kg=target_sprung_mass_kg,
                static_rear_weight_fraction=static_rear_weight_fraction,
                front_arb_fraction=bounded,
                tire_mu_scale=tire_mu_scale,
                speed_mps=11.8,
                binary_iterations=iterations,
            )
        return cache[key]

    coarse_fractions = np.asarray(
        [0.0, 0.20, 0.35, 0.425, 0.50, 0.575, 0.65, 0.80, 1.0]
    )
    coarse_limits = np.asarray(
        [evaluate(value, 12) for value in coarse_fractions],
        dtype=float,
    )
    best_index = int(np.argmax(coarse_limits))
    bracket_lower = float(coarse_fractions[max(0, best_index - 1)])
    bracket_upper = float(
        coarse_fractions[min(len(coarse_fractions) - 1, best_index + 1)]
    )
    result = minimize_scalar(
        lambda value: -evaluate(float(value), 18),
        bounds=(bracket_lower, bracket_upper),
        method="bounded",
        options={"xatol": 2e-4, "maxiter": 40},
    )
    if not result.success:
        raise RuntimeError(f"Front-ARB optimization failed: {result.message}")

    center = float(result.x)
    validation_fractions = sorted(
        {
            float(np.clip(center + offset, 0.0, 1.0))
            for offset in (-0.002, -0.001, 0.0, 0.001, 0.002)
        }
    )
    validation_limits = {value: evaluate(value, 22) for value in validation_fractions}
    selected_fraction = max(validation_limits, key=validation_limits.get)
    selected_limit = validation_limits[selected_fraction]
    lateral_diagnostics = _lateral_trim_diagnostics(
        bobsim_root,
        model_dof=model_dof,
        height_m=height_m,
        target_sprung_mass_kg=target_sprung_mass_kg,
        static_rear_weight_fraction=static_rear_weight_fraction,
        front_arb_fraction=selected_fraction,
        tire_mu_scale=tire_mu_scale,
        speed_mps=11.8,
        lateral_acceleration_mps2=selected_limit,
    )

    _vehicle, selected_parameters = _load_context(
        bobsim_root,
        height_m=height_m,
        target_sprung_mass_kg=target_sprung_mass_kg,
        static_rear_weight_fraction=static_rear_weight_fraction,
        front_arb_fraction=selected_fraction,
        front_brake_fraction=0.84,
        tire_mu_scale=tire_mu_scale,
    )
    original_total_arb = sum(
        _load_context(
            bobsim_root,
            height_m=height_m,
            target_sprung_mass_kg=target_sprung_mass_kg,
            static_rear_weight_fraction=static_rear_weight_fraction,
            front_arb_fraction=0.5,
            front_brake_fraction=0.84,
            tire_mu_scale=tire_mu_scale,
        )[1].antiroll_stiffness_nm_per_rad
    )
    robustness = {
        f"{speed:.1f}": _lateral_limit(
            bobsim_root,
            model_dof=model_dof,
            height_m=height_m,
            target_sprung_mass_kg=target_sprung_mass_kg,
            static_rear_weight_fraction=static_rear_weight_fraction,
            front_arb_fraction=selected_fraction,
            tire_mu_scale=tire_mu_scale,
            speed_mps=speed,
            binary_iterations=18,
        )
        / G
        for speed in ROBUSTNESS_SPEEDS_MPS
    }
    return {
        "front_antiroll_stiffness_fraction": selected_fraction,
        "front_antiroll_stiffness_nm_per_rad": float(
            selected_parameters.antiroll_stiffness_nm_per_rad[0]
        ),
        "rear_antiroll_stiffness_nm_per_rad": float(
            selected_parameters.antiroll_stiffness_nm_per_rad[1]
        ),
        "total_antiroll_stiffness_nm_per_rad": float(
            sum(selected_parameters.antiroll_stiffness_nm_per_rad)
        ),
        "total_antiroll_stiffness_preservation_error_nm_per_rad": float(
            sum(selected_parameters.antiroll_stiffness_nm_per_rad) - original_total_arb
        ),
        "front_elastic_roll_stiffness_fraction": float(
            selected_parameters.front_roll_stiffness_fraction
        ),
        "pure_lateral_limit_mps2": selected_limit,
        "pure_lateral_limit_g": selected_limit / G,
        "pure_lateral_limit_trim": lateral_diagnostics,
        "search_bounds_front_arb_fraction": [0.0, 1.0],
        "roll_optimization_at_search_bound": bool(
            math.isclose(selected_fraction, 0.0, abs_tol=2e-4)
            or math.isclose(selected_fraction, 1.0, abs_tol=2e-4)
        ),
        "arb_setup_hardware_achievable_under_model": bool(
            all(
                stiffness >= 0.0
                for stiffness in selected_parameters.antiroll_stiffness_nm_per_rad
            )
        ),
        "arb_setup_hardware_achievability_note": (
            "achievable in the continuous nonnegative-ARB model; discrete "
            "installed blade/bar adjustment ranges were not supplied"
        ),
        "roll_coarse_search": [
            {
                "front_arb_fraction": float(fraction),
                "lateral_limit_g": float(limit / G),
            }
            for fraction, limit in zip(coarse_fractions, coarse_limits)
        ],
        "roll_local_validation": [
            {
                "front_arb_fraction": fraction,
                "lateral_limit_g": limit / G,
            }
            for fraction, limit in validation_limits.items()
        ],
        "robustness_lateral_limit_g_by_speed_mps": robustness,
        "exact_endpoint_definition": (
            "EnvelopeSim solve_lateral_limit sustainable body-ax=0 endpoint with "
            "descending interval scan, branch-local continuation seeds, and "
            "adjacent-point-seeded boundary refinement"
        ),
    }


def _brake_curve(
    bobsim_root: Path,
    *,
    model_dof: int,
    height_m: float,
    target_sprung_mass_kg: float | None = None,
    static_rear_weight_fraction: float | None = None,
    front_arb_fraction: float,
    front_brake_fraction: float,
    tire_mu_scale: float,
    speeds_mps: np.ndarray,
    binary_iterations: int,
) -> np.ndarray:
    from _0_Utils.dyn_py import create_model
    from _2_EnvelopeSim.GGV.ggv_generation import solve_ax_limit

    outer_vehicle, parameters = _load_context(
        bobsim_root,
        height_m=height_m,
        target_sprung_mass_kg=target_sprung_mass_kg,
        static_rear_weight_fraction=static_rear_weight_fraction,
        front_arb_fraction=front_arb_fraction,
        front_brake_fraction=front_brake_fraction,
        tire_mu_scale=tire_mu_scale,
    )
    model = create_model(model_dof, parameters)
    limits = []
    for speed in speeds_mps:
        limit = solve_ax_limit(
            outer_vehicle,
            speed=float(speed),
            ay=0.0,
            ax_grid=np.asarray([-4.0 * G, 0.0]),
            mode="brake",
            reduced_model=model,
            ax_binary_iterations=binary_iterations,
        )
        if not math.isfinite(limit):
            raise RuntimeError(
                f"Non-finite {model_dof}DOF brake limit at {speed:.6f} m/s."
            )
        if math.isclose(limit, -4.0 * G, rel_tol=0.0, abs_tol=1e-10):
            raise RuntimeError("Brake search lower bound is active; expand it.")
        limits.append(-float(limit))
    return np.asarray(limits, dtype=float)


def _weighted_brake_objective(
    bobsim_root: Path,
    *,
    model_dof: int,
    height_m: float,
    target_sprung_mass_kg: float | None = None,
    static_rear_weight_fraction: float | None = None,
    front_arb_fraction: float,
    front_brake_fraction: float,
    tire_mu_scale: float,
    speeds_mps: np.ndarray,
    weights_m: np.ndarray,
    binary_iterations: int,
) -> tuple[float, np.ndarray]:
    curve = _brake_curve(
        bobsim_root,
        model_dof=model_dof,
        height_m=height_m,
        target_sprung_mass_kg=target_sprung_mass_kg,
        static_rear_weight_fraction=static_rear_weight_fraction,
        front_arb_fraction=front_arb_fraction,
        front_brake_fraction=front_brake_fraction,
        tire_mu_scale=tire_mu_scale,
        speeds_mps=speeds_mps,
        binary_iterations=binary_iterations,
    )
    return float(np.average(curve, weights=weights_m)), curve


def _tune_fixed_brake_bias(
    bobsim_root: Path,
    *,
    model_dof: int,
    height_m: float,
    target_sprung_mass_kg: float | None = None,
    static_rear_weight_fraction: float | None = None,
    front_arb_fraction: float,
    tire_mu_scale: float,
    speeds_mps: np.ndarray,
    weights_m: np.ndarray,
) -> dict[str, Any]:
    mean_speed = float(np.average(speeds_mps, weights=weights_m))
    representative_cache: dict[float, float] = {}

    def representative_limit(front_bias: float) -> float:
        key = round(float(front_bias), 12)
        if key not in representative_cache:
            curve = _brake_curve(
                bobsim_root,
                model_dof=model_dof,
                height_m=height_m,
                target_sprung_mass_kg=target_sprung_mass_kg,
                static_rear_weight_fraction=static_rear_weight_fraction,
                front_arb_fraction=front_arb_fraction,
                front_brake_fraction=key,
                tire_mu_scale=tire_mu_scale,
                speeds_mps=np.asarray([mean_speed]),
                binary_iterations=14,
            )
            representative_cache[key] = float(curve[0])
        return representative_cache[key]

    center_result = minimize_scalar(
        lambda value: -representative_limit(float(value)),
        bounds=(0.50, 0.98),
        method="bounded",
        options={"xatol": 0.002, "maxiter": 30},
    )
    if not center_result.success:
        raise RuntimeError(f"Brake-bias center search failed: {center_result.message}")
    center = float(center_result.x)

    coarse_biases = sorted(
        {
            0.50,
            0.98,
            *(
                float(np.clip(center + offset, 0.50, 0.98))
                for offset in (-0.025, 0.0, 0.025)
            ),
        }
    )
    coarse_results: dict[float, float] = {}
    for bias in coarse_biases:
        coarse_results[bias] = _weighted_brake_objective(
            bobsim_root,
            model_dof=model_dof,
            height_m=height_m,
            target_sprung_mass_kg=target_sprung_mass_kg,
            static_rear_weight_fraction=static_rear_weight_fraction,
            front_arb_fraction=front_arb_fraction,
            front_brake_fraction=bias,
            tire_mu_scale=tire_mu_scale,
            speeds_mps=speeds_mps,
            weights_m=weights_m,
            binary_iterations=8,
        )[0]
    coarse_best = max(coarse_results, key=coarse_results.get)

    refine_biases = sorted(
        {
            float(np.clip(coarse_best + offset, 0.50, 0.98))
            for offset in (-0.004, -0.002, 0.0, 0.002, 0.004)
        }
    )
    refined_results: dict[float, float] = {}
    for bias in refine_biases:
        refined_results[bias] = _weighted_brake_objective(
            bobsim_root,
            model_dof=model_dof,
            height_m=height_m,
            target_sprung_mass_kg=target_sprung_mass_kg,
            static_rear_weight_fraction=static_rear_weight_fraction,
            front_arb_fraction=front_arb_fraction,
            front_brake_fraction=bias,
            tire_mu_scale=tire_mu_scale,
            speeds_mps=speeds_mps,
            weights_m=weights_m,
            binary_iterations=11,
        )[0]
    refined_best = max(refined_results, key=refined_results.get)

    local_biases = sorted(
        {
            float(np.clip(refined_best + offset, 0.50, 0.98))
            for offset in (-0.001, 0.0, 0.001)
        }
    )
    final_results: dict[float, tuple[float, np.ndarray]] = {}
    for bias in local_biases:
        final_results[bias] = _weighted_brake_objective(
            bobsim_root,
            model_dof=model_dof,
            height_m=height_m,
            target_sprung_mass_kg=target_sprung_mass_kg,
            static_rear_weight_fraction=static_rear_weight_fraction,
            front_arb_fraction=front_arb_fraction,
            front_brake_fraction=bias,
            tire_mu_scale=tire_mu_scale,
            speeds_mps=speeds_mps,
            weights_m=weights_m,
            binary_iterations=15,
        )
    selected_bias = max(final_results, key=lambda value: final_results[value][0])
    selected_objective, selected_curve = final_results[selected_bias]
    return {
        "brake_distribution_front": selected_bias,
        "weighted_pure_braking_limit_mps2": selected_objective,
        "weighted_pure_braking_limit_g": selected_objective / G,
        "minimum_bin_braking_limit_g": float(np.min(selected_curve) / G),
        "maximum_bin_braking_limit_g": float(np.max(selected_curve) / G),
        "search_bounds_front_brake_fraction": [0.50, 0.98],
        "brake_optimization_at_search_bound": bool(
            math.isclose(selected_bias, 0.50, abs_tol=5e-4)
            or math.isclose(selected_bias, 0.98, abs_tol=5e-4)
        ),
        "brake_bias_within_search_bounds": bool(0.50 <= selected_bias <= 0.98),
        "brake_bias_hardware_achievability": (
            "not assessed; physical balance-bar/caliper adjustment limits "
            "were not supplied"
        ),
        "representative_speed_center_fraction": center,
        "brake_coarse_search": [
            {"front_brake_fraction": bias, "weighted_limit_g": value / G}
            for bias, value in coarse_results.items()
        ],
        "brake_refined_search": [
            {"front_brake_fraction": bias, "weighted_limit_g": value / G}
            for bias, value in refined_results.items()
        ],
        "brake_local_validation": [
            {
                "front_brake_fraction": bias,
                "weighted_limit_g": value[0] / G,
            }
            for bias, value in final_results.items()
        ],
        "solver": (
            "BobSim reduced-QSS solve_ax_limit at ay=0 for every weighted "
            "speed bin; fixed front fraction shared across all bins"
        ),
    }


def _tune_case(
    task: tuple[Any, ...],
) -> dict[str, Any]:
    if len(task) == 6:
        (
            bobsim_root_raw,
            model_dof,
            height_in,
            tire_mu_scale,
            speeds_raw,
            weights_raw,
        ) = task
        target_sprung_mass_kg = None
        static_rear_weight_fraction = None
        case_name = _case_name(float(height_in))
    elif len(task) == 8:
        (
            bobsim_root_raw,
            model_dof,
            height_in,
            tire_mu_scale,
            speeds_raw,
            weights_raw,
            target_sprung_mass_kg,
            case_name,
        ) = task
        static_rear_weight_fraction = None
    elif len(task) == 9:
        (
            bobsim_root_raw,
            model_dof,
            height_in,
            tire_mu_scale,
            speeds_raw,
            weights_raw,
            target_sprung_mass_kg,
            static_rear_weight_fraction,
            case_name,
        ) = task
    else:
        raise ValueError(
            "Tuning task must contain 6 CG, 8 mass, or 9 static-weight fields."
        )
    bobsim_root = Path(bobsim_root_raw)
    if str(bobsim_root) not in sys.path:
        sys.path.insert(0, str(bobsim_root))
    height_m = height_in * 0.0254
    speeds = np.asarray(speeds_raw, dtype=float)
    weights = np.asarray(weights_raw, dtype=float)
    roll = _tune_front_arb_fraction(
        bobsim_root,
        model_dof=model_dof,
        height_m=height_m,
        target_sprung_mass_kg=target_sprung_mass_kg,
        static_rear_weight_fraction=static_rear_weight_fraction,
        tire_mu_scale=tire_mu_scale,
    )
    braking = _tune_fixed_brake_bias(
        bobsim_root,
        model_dof=model_dof,
        height_m=height_m,
        target_sprung_mass_kg=target_sprung_mass_kg,
        static_rear_weight_fraction=static_rear_weight_fraction,
        front_arb_fraction=float(roll["front_antiroll_stiffness_fraction"]),
        tire_mu_scale=tire_mu_scale,
        speeds_mps=speeds,
        weights_m=weights,
    )
    mass_fields: dict[str, float] = {}
    weight_fields: dict[str, float] = {}
    if target_sprung_mass_kg is not None or static_rear_weight_fraction is not None:
        _outer_vehicle, selected_parameters = _load_context(
            bobsim_root,
            height_m=height_m,
            target_sprung_mass_kg=target_sprung_mass_kg,
            static_rear_weight_fraction=static_rear_weight_fraction,
            front_arb_fraction=float(roll["front_antiroll_stiffness_fraction"]),
            front_brake_fraction=float(braking["brake_distribution_front"]),
            tire_mu_scale=tire_mu_scale,
        )
        mass_fields = {
            "sprung_mass_kg": float(selected_parameters.sprung_mass_kg),
            "total_mass_kg": float(selected_parameters.mass_kg),
        }
        if static_rear_weight_fraction is not None:
            weight_fields = {
                "rear_static_weight_fraction": float(
                    selected_parameters.static_rear_weight_fraction
                ),
                "front_static_weight_fraction": float(
                    selected_parameters.static_front_weight_fraction
                ),
                "cg_x_m": float(selected_parameters.center_of_gravity_m[0]),
            }
    return {
        "name": str(case_name),
        "model_dof": model_dof,
        "cg_height_in": height_in,
        "cg_height_m": height_m,
        **mass_fields,
        **weight_fields,
        "tire_mu_scale": tire_mu_scale,
        **roll,
        **braking,
    }


def _runner_payload(
    model_dof: int,
    cases: list[dict[str, Any]],
    *,
    tire_mu_scale: float,
    sweep_axis: str = "cg_height_in",
    reference_case: str | None = None,
) -> dict[str, Any]:
    tire_label = "Raw-TIR" if math.isclose(tire_mu_scale, 1.0) else "Scaled-mu"
    if sweep_axis not in {
        "cg_height_in",
        "sprung_mass_kg",
        "rear_static_weight_fraction",
        "configuration",
    }:
        raise ValueError(f"Unsupported sweep axis {sweep_axis!r}.")
    case_names = {str(case["name"]) for case in cases}
    if reference_case is not None and reference_case not in case_names:
        raise ValueError("reference_case must name one of the tuned cases.")
    if reference_case is None:
        reference_case = (
            "cg_11p6in"
            if sweep_axis == "cg_height_in"
            else (
                str(cases[0]["name"])
                if sweep_axis == "configuration"
                else min(
                    cases,
                    key=(
                        (lambda case: abs(float(case.get("mass_offset_lb", math.inf))))
                        if sweep_axis == "sprung_mass_kg"
                        else (
                            lambda case: abs(
                                float(case["rear_static_weight_fraction"]) - 0.5
                            )
                        )
                    ),
                )["name"]
            )
        )
    study_axis = {
        "cg_height_in": "CG-height",
        "sprung_mass_kg": "sprung-mass",
        "rear_static_weight_fraction": "longitudinal-CG/static-weight",
        "configuration": "vehicle-configuration comparison",
    }[sweep_axis]
    study_suffix = "" if sweep_axis == "configuration" else " sweep"
    return {
        "study": (
            f"{tire_label} dyn_py {model_dof}DOF physical {study_axis}{study_suffix}"
        ),
        "model_dof": model_dof,
        "sweep_axis": sweep_axis,
        "reference_case": reference_case,
        "aero_balance_front": 0.5,
        "tire_mu_scale": tire_mu_scale,
        "drive_model": "RWD with equal rear-wheel force split; no LSD model",
        "cases": [
            {
                "name": case["name"],
                "cg_height_in": case["cg_height_in"],
                **(
                    {
                        "sprung_mass_kg": case["sprung_mass_kg"],
                        "total_mass_kg": case["total_mass_kg"],
                        "mass_offset_lb": case["mass_offset_lb"],
                    }
                    if sweep_axis == "sprung_mass_kg"
                    else {}
                ),
                **(
                    {
                        "rear_static_weight_fraction": case[
                            "rear_static_weight_fraction"
                        ],
                        "front_static_weight_fraction": case[
                            "front_static_weight_fraction"
                        ],
                        "cg_x_m": case["cg_x_m"],
                        "total_mass_kg": case["total_mass_kg"],
                    }
                    if sweep_axis in {"rear_static_weight_fraction", "configuration"}
                    else {}
                ),
                **(
                    {
                        "sprung_mass_kg": case["sprung_mass_kg"],
                        "total_mass_kg": case["total_mass_kg"],
                    }
                    if sweep_axis == "configuration"
                    else {}
                ),
                "front_antiroll_stiffness_fraction": case[
                    "front_antiroll_stiffness_fraction"
                ],
                "brake_distribution_front": case["brake_distribution_front"],
            }
            for case in cases
        ],
    }


def run(args: argparse.Namespace) -> dict[str, Any]:
    bobsim_root = args.bobsim_root.resolve()
    if str(bobsim_root) not in sys.path:
        sys.path.insert(0, str(bobsim_root))
    from _0_Utils.dyn_py import load_reduced_vehicle_parameters
    from _2_EnvelopeSim.vehicle_yaml import load_vehicle_yaml, project_vehicle_yaml

    model_dofs = [int(value) for value in args.model_dofs]
    if len(set(model_dofs)) != len(model_dofs):
        raise ValueError("Model DOFs must be unique.")
    if any(value not in DEFAULT_MODEL_DOFS for value in model_dofs):
        raise ValueError("Model DOFs must be 3 or 6.")
    tire_mu_scale = float(args.tire_mu_scale)
    if not math.isfinite(tire_mu_scale) or tire_mu_scale <= 0.0:
        raise ValueError("Tire mu scale must be finite and positive.")
    speeds, weights, distribution = _load_braking_distribution(
        args.reference_case_dir.resolve()
    )
    base_parameters = load_reduced_vehicle_parameters(bobsim_root / "vehicle.yml")
    mass_offsets_raw = getattr(args, "mass_offsets_lb", None)
    rear_weight_fractions_raw = getattr(args, "rear_weight_fractions", None)
    configuration_path = getattr(args, "configuration_json", None)
    if mass_offsets_raw is not None and rear_weight_fractions_raw is not None:
        raise ValueError(
            "--mass-offsets-lb and --rear-weight-fractions are mutually exclusive."
        )
    if configuration_path is not None and (
        mass_offsets_raw is not None or rear_weight_fractions_raw is not None
    ):
        raise ValueError(
            "--configuration-json is mutually exclusive with --mass-offsets-lb "
            "and --rear-weight-fractions."
        )
    configuration_definition: dict[str, Any] | None = None
    configuration_reference_case: str | None = None
    if configuration_path is not None:
        sweep_axis = "configuration"
        (
            configuration_definition,
            case_specs,
            configuration_reference_case,
        ) = _load_configuration_definition(configuration_path.resolve())
        declared_model_dof = configuration_definition.get("model_dof")
        if declared_model_dof is not None and model_dofs != [int(declared_model_dof)]:
            raise ValueError(
                "Configuration JSON model_dof must exactly match --model-dofs."
            )
        declared_mu_scale = configuration_definition.get("tire_mu_scale")
        if declared_mu_scale is not None and not math.isclose(
            float(declared_mu_scale), tire_mu_scale, rel_tol=0.0, abs_tol=1e-15
        ):
            raise ValueError(
                "Configuration JSON tire_mu_scale must exactly match --tire-mu-scale."
            )
        declared_aero_balance = configuration_definition.get("aero_balance_front")
        if declared_aero_balance is not None and not math.isclose(
            float(declared_aero_balance), 0.5, rel_tol=0.0, abs_tol=1e-15
        ):
            raise ValueError(
                "Configuration tuning currently requires 50/50 aero balance."
            )
        heights = [float(case["height_in"]) for case in case_specs]
    elif mass_offsets_raw is None and rear_weight_fractions_raw is None:
        sweep_axis = "cg_height_in"
        heights = [float(value) for value in args.heights_in]
        if any(not math.isfinite(value) or value <= 0.0 for value in heights):
            raise ValueError("CG heights must be finite and positive.")
        if len(set(heights)) != len(heights):
            raise ValueError("CG heights must be unique.")
        case_specs = [
            {
                "name": _case_name(height),
                "height_in": height,
                "target_sprung_mass_kg": None,
                "static_rear_weight_fraction": None,
            }
            for height in heights
        ]
    elif mass_offsets_raw is not None:
        sweep_axis = "sprung_mass_kg"
        mass_offsets_lb = [float(value) for value in mass_offsets_raw]
        if any(not math.isfinite(value) for value in mass_offsets_lb):
            raise ValueError("Mass offsets must be finite.")
        if len(set(mass_offsets_lb)) != len(mass_offsets_lb):
            raise ValueError("Mass offsets must be unique.")
        fixed_height_in = float(args.fixed_cg_height_in)
        if not math.isfinite(fixed_height_in) or fixed_height_in <= 0.0:
            raise ValueError("Fixed CG height must be finite and positive.")
        heights = [fixed_height_in]
        case_specs = [
            {
                "name": _mass_case_name(offset_lb),
                "height_in": fixed_height_in,
                "target_sprung_mass_kg": (
                    base_parameters.sprung_mass_kg + offset_lb * LB_TO_KG
                ),
                "static_rear_weight_fraction": None,
                "mass_offset_lb": offset_lb,
            }
            for offset_lb in mass_offsets_lb
        ]
        if any(float(case["target_sprung_mass_kg"]) <= 0.0 for case in case_specs):
            raise ValueError("Mass offsets must leave positive sprung mass.")
    else:
        sweep_axis = "rear_static_weight_fraction"
        rear_weight_fractions = [float(value) for value in rear_weight_fractions_raw]
        if any(
            not math.isfinite(value) or not 0.0 < value < 1.0
            for value in rear_weight_fractions
        ):
            raise ValueError(
                "Rear weight fractions must be finite and strictly between 0 and 1."
            )
        if len(set(rear_weight_fractions)) != len(rear_weight_fractions):
            raise ValueError("Rear weight fractions must be unique.")
        fixed_height_in = float(args.fixed_cg_height_in)
        if not math.isfinite(fixed_height_in) or fixed_height_in <= 0.0:
            raise ValueError("Fixed CG height must be finite and positive.")
        heights = [fixed_height_in]
        case_specs = [
            {
                "name": _rear_weight_case_name(rear_fraction),
                "height_in": fixed_height_in,
                "target_sprung_mass_kg": None,
                "static_rear_weight_fraction": rear_fraction,
            }
            for rear_fraction in rear_weight_fractions
        ]
    projection = project_vehicle_yaml(
        load_vehicle_yaml(bobsim_root / "vehicle.yml"),
        repo_root=bobsim_root,
        aero_balance_front=0.5,
    )
    tire_path = (
        bobsim_root
        / "_0_Utils"
        / "tire_templates"
        / f"{projection.summary['tire_template']}"
    )
    tasks = []
    for model_dof in model_dofs:
        for case in case_specs:
            base_task: tuple[Any, ...] = (
                str(bobsim_root),
                model_dof,
                case["height_in"],
                tire_mu_scale,
                speeds.tolist(),
                weights.tolist(),
            )
            if sweep_axis == "sprung_mass_kg":
                base_task = (
                    *base_task,
                    case["target_sprung_mass_kg"],
                    case["name"],
                )
            elif sweep_axis in {"rear_static_weight_fraction", "configuration"}:
                base_task = (
                    *base_task,
                    case["target_sprung_mass_kg"],
                    case["static_rear_weight_fraction"],
                    case["name"],
                )
            tasks.append(base_task)
    results: list[dict[str, Any]] = []
    workers = max(1, min(int(args.workers), len(tasks)))
    if workers == 1:
        for task in tasks:
            result = _tune_case(task)
            print(
                f"{result['model_dof']}DOF {result['name']}: "
                f"ARB={result['front_antiroll_stiffness_fraction']:.6f}, "
                f"brake={result['brake_distribution_front']:.6f}",
                flush=True,
            )
            results.append(result)
    else:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            futures = {pool.submit(_tune_case, task): task for task in tasks}
            for future in as_completed(futures):
                result = future.result()
                print(
                    f"{result['model_dof']}DOF {result['name']}: "
                    f"ARB={result['front_antiroll_stiffness_fraction']:.6f}, "
                    f"brake={result['brake_distribution_front']:.6f}",
                    flush=True,
                )
                results.append(result)

    if sweep_axis == "sprung_mass_kg":
        for result in results:
            result["mass_offset_lb"] = (
                float(result["sprung_mass_kg"]) - base_parameters.sprung_mass_kg
            ) / LB_TO_KG

    models: dict[str, Any] = {}
    runner_paths = {3: args.runner_3dof.resolve(), 6: args.runner_6dof.resolve()}
    configuration_order = {
        str(case["name"]): int(case.get("configuration_order", index))
        for index, case in enumerate(case_specs)
    }
    for model_dof in model_dofs:
        model_cases = sorted(
            (case for case in results if case["model_dof"] == model_dof),
            key=(
                (lambda case: case["cg_height_m"])
                if sweep_axis == "cg_height_in"
                else (
                    (lambda case: case["sprung_mass_kg"])
                    if sweep_axis == "sprung_mass_kg"
                    else (
                        (lambda case: case["rear_static_weight_fraction"])
                        if sweep_axis == "rear_static_weight_fraction"
                        else (lambda case: configuration_order[str(case["name"])])
                    )
                )
            ),
        )
        runner_payload = _runner_payload(
            model_dof,
            model_cases,
            tire_mu_scale=tire_mu_scale,
            sweep_axis=sweep_axis,
            reference_case=configuration_reference_case,
        )
        _write_json(runner_paths[model_dof], runner_payload)
        models[f"dyn_py_{model_dof}dof_qss"] = {
            "model_dof": model_dof,
            "runner_sweep_json": str(runner_paths[model_dof]),
            "cases": model_cases,
        }

    spring_front, spring_rear = base_parameters.spring_roll_stiffness_nm_per_rad
    arb_front, arb_rear = base_parameters.antiroll_stiffness_nm_per_rad
    total_elastic = spring_front + spring_rear + arb_front + arb_rear
    payload = {
        "schema": (
            "lapsims.dyn-py-cg-physical-tuning.v1"
            if sweep_axis == "cg_height_in"
            else (
                "lapsims.dyn-py-mass-physical-tuning.v1"
                if sweep_axis == "sprung_mass_kg"
                else (
                    "lapsims.dyn-py-longitudinal-cg-physical-tuning.v1"
                    if sweep_axis == "rear_static_weight_fraction"
                    else "lapsims.dyn-py-configuration-physical-tuning.v1"
                )
            )
        ),
        "sweep_axis": sweep_axis,
        "study": (
            f"dyn_py physical setup tuning for {len(case_specs)} "
            f"{ {'cg_height_in': 'CG heights', 'sprung_mass_kg': 'sprung masses', 'rear_static_weight_fraction': 'static rear-weight fractions', 'configuration': 'explicit vehicle configurations'}[sweep_axis] } "
            f"at tire mu scale {tire_mu_scale:.15g}"
        ),
        "vehicle_yaml": str((bobsim_root / "vehicle.yml").resolve()),
        "aero_balance_front": 0.5,
        "drive_model": "RWD with equal rear-wheel force split; no LSD model",
        "configuration_definition": configuration_definition,
        "configuration_reference_case": configuration_reference_case,
        "mass_sweep_assumptions": (
            None
            if sweep_axis != "sprung_mass_kg"
            else {
                "fixed_total_cg_height_in": heights[0],
                "nominal_sprung_mass_kg": base_parameters.sprung_mass_kg,
                "fixed_unsprung_mass_kg": sum(base_parameters.unsprung_mass_kg),
                "sprung_inertia_rule": (
                    "scale the full sprung inertia tensor linearly with sprung "
                    "mass, preserving sprung radii of gyration"
                ),
                "static_load_rule": (
                    "recompute total static load at the unchanged projected front "
                    "fraction and fixed total CG"
                ),
                "held_fixed": (
                    "spring and damper rates, total ARB stiffness, CG x/y/z, "
                    "unsprung mass/inertia contribution, aero, powertrain, and tires"
                ),
                "not_retuned": "ride and roll natural frequencies",
            }
        ),
        "longitudinal_cg_sweep_assumptions": (
            None
            if sweep_axis != "rear_static_weight_fraction"
            else {
                "fixed_total_cg_height_in": heights[0],
                "nominal_total_mass_kg": base_parameters.mass_kg,
                "rear_static_weight_fraction_definition": (
                    "rear axle static load divided by total static load"
                ),
                "cg_x_rule": (
                    "translate total CG along the fixed wheelbase to achieve the "
                    "target static rear fraction"
                ),
                "held_fixed": (
                    "total and sprung mass, inertia tensors about the CG, CG y/z, "
                    "global axle/contact locations, total aero load and 50/50 "
                    "aero balance, powertrain, tire model, and tracks"
                ),
                "retuned_per_case": (
                    "front/rear ARB distribution at fixed total ARB stiffness and "
                    "one fixed front brake fraction"
                ),
            }
        ),
        "configuration_comparison_assumptions": (
            None
            if sweep_axis != "configuration"
            else {
                "varied_per_case": (
                    "total CG height and longitudinal total-CG position/static "
                    "rear-weight fraction"
                ),
                "mass_rule": (
                    "nominal vehicle.yml sprung and total mass when sprung_mass_kg "
                    "is omitted; otherwise use the explicit per-case sprung mass "
                    "with fixed unsprung masses"
                ),
                "cg_x_rule": (
                    "translate total CG along the fixed wheelbase to achieve each "
                    "declared static rear fraction"
                ),
                "held_fixed": (
                    "CG y, inertia tensors about the CG (except established mass "
                    "scaling when explicitly requested), global axle/contact "
                    "locations, total aero load and 50/50 aero balance, powertrain, "
                    "tire model, tire-mu scale, and tracks"
                ),
                "retuned_per_case": (
                    "front/rear ARB distribution at fixed total ARB stiffness and "
                    "one fixed front brake fraction"
                ),
                "interpretation": (
                    "categorical configuration comparison; not a one-variable CG "
                    "height or longitudinal-CG sensitivity"
                ),
            }
        ),
        "tire": {
            "path": str(tire_path.resolve()),
            "sha256": hashlib.sha256(tire_path.read_bytes()).hexdigest(),
            "mu_scale": tire_mu_scale,
            "coefficients": asdict(base_parameters.tire),
        },
        "roll_tuning_method": {
            "target_speed_mps": 11.8,
            "objective": (
                "maximize the EnvelopeSim reduced-QSS sustainable body-ax=0 "
                "pure-lateral endpoint while RWD force balances aero drag"
            ),
            "endpoint_solver": (
                "_2_EnvelopeSim.GGV.ggv_generation.solve_lateral_limit with "
                "descending interval scan, branch-local continuation seeds, and "
                "adjacent-point-seeded boundary refinement"
            ),
            "cold_monotonic_binary_assumption": False,
            "tire_fit_normal_load_bounds_enforced": False,
            "front_antiroll_fraction_bounds": [0.0, 1.0],
            "held_fixed": (
                "four wheel spring rates and total front-plus-rear ARB stiffness"
            ),
            "spring_roll_stiffness_nm_per_rad": [spring_front, spring_rear],
            "baseline_antiroll_stiffness_nm_per_rad": [arb_front, arb_rear],
            "total_antiroll_stiffness_nm_per_rad": arb_front + arb_rear,
            "total_elastic_roll_stiffness_nm_per_rad": total_elastic,
            "achievable_front_elastic_fraction": [
                spring_front / total_elastic,
                (spring_front + arb_front + arb_rear) / total_elastic,
            ],
            "racing_feasibility_bounds": {
                "maximum_absolute_sideslip_rad": 0.25,
                "maximum_absolute_steering_rad": 0.5,
                "minimum_normal_load_n": 0.0,
            },
            "tire_fit_normal_load_range_n": [
                float(base_parameters.tire.fz_min_n),
                float(base_parameters.tire.fz_max_n),
            ],
            "active_constraint_flag_tolerances": {
                "sideslip_or_steering_rad": 1e-4,
                "wheel_lift_boundary_n": 0.1,
            },
            "hardware_achievability_scope": (
                "continuous nonnegative front/rear ARB stiffness only; no "
                "discrete installed blade/bar adjustment ranges supplied"
            ),
            "robustness_speeds_mps": list(ROBUSTNESS_SPEEDS_MPS),
        },
        "brake_bias_method": {
            "objective": (
                "maximize distance-weighted exact reduced-QSS straight-braking "
                "limit over one common prior speed distribution"
            ),
            "front_fraction_bounds": [0.50, 0.98],
            "hardware_achievability_scope": (
                "not assessed because balance-bar/caliper adjustment limits "
                "were not supplied"
            ),
            "representative_distribution": distribution,
        },
        "models": models,
    }
    _write_json(args.output.resolve(), payload)
    return payload


def main() -> None:
    run(parse_args())


if __name__ == "__main__":
    main()
