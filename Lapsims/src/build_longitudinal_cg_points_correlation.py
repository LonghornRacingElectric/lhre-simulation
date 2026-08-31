"""Build a longitudinal-CG sweep points report and optional cohort comparison.

The builder consumes a completed scaled-mu reduced-6DOF sweep at fixed mass and
CG height.  It validates an explicitly supplied rear-static-weight grid, scores
all simulations in one field with valid 2026 Michigan EV results, and writes
auditable tables, fits, plots, and a Markdown report.  An optional prior sweep
can be included so every unique simulation shares one real-plus-simulation Tmin
per event.  Source studies are read-only; this module writes only to the
requested report directory.
"""

from __future__ import annotations

import argparse
import json
import math
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from build_reduced_dof_cg_comparison import (
    DEFAULT_SCORING_REFERENCE,
    EVENT_LABELS,
    EXPECTED_EVENTS,
    _load_scoring_reference,
    _rescore_common_field,
)

DEFAULT_REAR_FRACTIONS = (0.45, 0.50, 0.55)
DEFAULT_COMPARISON_REAR_FRACTIONS = (0.45, 0.50, 0.55)
DEFAULT_FINE_VALIDATION_REAR_FRACTIONS = (0.55, 0.56, 0.58)
DEFAULT_CG_HEIGHT_IN = 11.5
DEFAULT_MU_SCALE = 0.6225437130779028
MODEL_ABSOLUTE_CG_Z_M = 0.292100
MODEL_CONTACT_PLANE_Z_M = -0.004602
MODEL_CG_HEIGHT_ABOVE_CONTACT_PLANE_M = (
    MODEL_ABSOLUTE_CG_Z_M - MODEL_CONTACT_PLANE_Z_M
)
M_PER_INCH = 0.0254
SOURCE_NOMINAL_REAR_STATIC_WEIGHT_FRACTION = 0.5165037110580026
POINT_DELTA_BASELINE_REAR_WEIGHT_PERCENT = 50.0
REAR_FRACTION_KEYS = (
    "rear_static_weight_fraction",
    "rear_static_fraction",
    "rear_weight_fraction",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Build an arbitrary-grid rear-static-weight report from one "
            "completed scaled-mu reduced-6DOF sweep, optionally rescored "
            "together with a prior sweep."
        )
    )
    parser.add_argument("--study-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--tuning-json", type=Path)
    parser.add_argument("--comparison-study-root", type=Path)
    parser.add_argument("--comparison-tuning-json", type=Path)
    parser.add_argument(
        "--fine-validation-root",
        type=Path,
        help=(
            "Optional root containing fine-grid case caches below prior/ and "
            "new/; these cases are scored separately and do not alter the "
            "main coarse comparison field."
        ),
    )
    parser.add_argument(
        "--scoring-reference", type=Path, default=DEFAULT_SCORING_REFERENCE
    )
    parser.add_argument(
        "--expected-rear-fractions",
        type=float,
        nargs="+",
        default=list(DEFAULT_REAR_FRACTIONS),
    )
    parser.add_argument(
        "--expected-cg-height-in", type=float, default=DEFAULT_CG_HEIGHT_IN
    )
    parser.add_argument(
        "--comparison-rear-fractions",
        type=float,
        nargs="+",
        default=list(DEFAULT_COMPARISON_REAR_FRACTIONS),
    )
    parser.add_argument(
        "--fine-validation-rear-fractions",
        type=float,
        nargs="+",
        default=list(DEFAULT_FINE_VALIDATION_REAR_FRACTIONS),
    )
    parser.add_argument("--expected-mu-scale", type=float, default=DEFAULT_MU_SCALE)
    parser.add_argument("--fraction-tolerance", type=float, default=1e-9)
    return parser.parse_args()


def _json_default(value: object) -> object:
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(f"Cannot serialize {type(value).__name__}")


def _write_json(path: Path, payload: object) -> None:
    path.write_text(
        json.dumps(payload, indent=2, default=_json_default) + "\n",
        encoding="utf-8",
    )


def _finite_float(value: object) -> float:
    if value is None:
        return math.nan
    try:
        result = float(value)
    except (TypeError, ValueError):
        return math.nan
    return result if math.isfinite(result) else math.nan


def _as_bool(value: object) -> bool:
    if isinstance(value, (bool, np.bool_)):
        return bool(value)
    return str(value).strip().lower() in {"true", "1", "yes"}


def _nested(payload: Mapping[str, Any], *paths: Sequence[str]) -> Any:
    for path in paths:
        value: Any = payload
        for key in path:
            if not isinstance(value, Mapping) or key not in value:
                break
            value = value[key]
        else:
            return value
    return None


def _require_columns(frame: pd.DataFrame, columns: Sequence[str], label: str) -> None:
    missing = sorted(set(columns).difference(frame.columns))
    if missing:
        raise ValueError(f"{label} is missing required columns: {missing}")


def _first_finite(*values: object) -> float:
    for value in values:
        result = _finite_float(value)
        if math.isfinite(result):
            return result
    return math.nan


def _load_tuning_cases(
    path: Path | None,
) -> tuple[dict[str, Mapping[str, Any]], str, dict[str, Any]]:
    if path is None:
        return {}, "", {}
    if not path.is_file():
        raise FileNotFoundError(f"Missing tuning JSON: {path.resolve()}")
    payload = json.loads(path.read_text(encoding="utf-8"))
    cases: object = None
    if isinstance(payload, Mapping):
        models = payload.get("models")
        if isinstance(models, Mapping):
            model = models.get("dyn_py_6dof_qss")
            if isinstance(model, Mapping):
                cases = model.get("cases")
        if cases is None:
            cases = payload.get("cases")
    if not isinstance(cases, list):
        raise TypeError(f"{path} does not contain a 6DOF tuning case list.")
    result: dict[str, Mapping[str, Any]] = {}
    for case in cases:
        if not isinstance(case, Mapping) or not isinstance(case.get("name"), str):
            raise TypeError(f"{path} contains an invalid tuning case.")
        name = str(case["name"])
        if name in result:
            raise ValueError(f"{path} contains duplicate tuning case {name!r}.")
        result[name] = case
    context = {
        "schema": payload.get("schema", ""),
        "sweep_axis": payload.get("sweep_axis", ""),
        "roll_tuning_method": payload.get("roll_tuning_method"),
        "brake_bias_method": payload.get("brake_bias_method"),
        "longitudinal_cg_sweep_assumptions": payload.get(
            "longitudinal_cg_sweep_assumptions"
        ),
    }
    return result, str(path.resolve()), context


def _candidate_rear_fractions(
    payload: Mapping[str, Any], reduced: Mapping[str, Any]
) -> list[tuple[str, float]]:
    candidates: list[tuple[str, float]] = []
    for key in REAR_FRACTION_KEYS:
        value = _finite_float(payload.get(key))
        if math.isfinite(value):
            candidates.append((key, value))
    case_tuning = payload.get("case_tuning")
    if isinstance(case_tuning, Mapping):
        for key in REAR_FRACTION_KEYS:
            value = _finite_float(case_tuning.get(key))
            if math.isfinite(value):
                candidates.append((f"case_tuning.{key}", value))
    vehicle = payload.get("vehicle")
    if isinstance(vehicle, Mapping):
        rear = _finite_float(vehicle.get("rear_static_fraction"))
        front = _finite_float(vehicle.get("front_static_frac"))
        if math.isfinite(rear):
            candidates.append(("vehicle.rear_static_fraction", rear))
        if math.isfinite(front):
            candidates.append(("1-vehicle.front_static_frac", 1.0 - front))
    static_loads = reduced.get("static_wheel_loads_n")
    try:
        loads = np.asarray(static_loads, dtype=float)
    except (TypeError, ValueError):
        loads = np.asarray([], dtype=float)
    if loads.shape == (4,) and np.isfinite(loads).all() and loads.sum() > 0.0:
        candidates.append(
            ("reduced_vehicle_parameters.static_wheel_loads_n", float(loads[2:].sum() / loads.sum()))
        )
    return candidates


def _load_case_metadata(
    study_root: Path,
    case_name: str,
    tuning_case: Mapping[str, Any],
    *,
    expected_cg_height_in: float,
    expected_mu_scale: float,
    fraction_tolerance: float,
) -> dict[str, Any]:
    path = study_root / case_name / "case_metadata.json"
    if not path.is_file():
        raise FileNotFoundError(f"Missing case metadata: {path.resolve()}")
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise TypeError(f"{path} must contain a JSON object.")
    if str(payload.get("cg_case")) != case_name:
        raise ValueError(f"{path} has a mismatched cg_case.")
    if str(payload.get("sweep_axis")) != "rear_static_weight_fraction":
        raise ValueError(f"{path} does not declare the longitudinal-CG sweep axis.")
    if payload.get("model_key") != "dyn_py_6dof_qss" or int(
        payload.get("model_dof", -1)
    ) != 6:
        raise ValueError(f"{path} is not a dyn_py reduced 6DOF case.")
    if payload.get("limited_slip_differential_model") is not False:
        raise ValueError(f"{path} does not explicitly disable the LSD model.")

    height_in = _finite_float(payload.get("cg_height_in"))
    if not math.isclose(height_in, expected_cg_height_in, abs_tol=1e-9):
        raise ValueError(
            f"{path} CG height is {height_in!r} in; expected "
            f"{expected_cg_height_in!r} in."
        )
    mu_scale = _first_finite(
        payload.get("tire_mu_scale"),
        _nested(payload, ("projection_summary", "tire_mu_scale")),
    )
    if not math.isclose(mu_scale, expected_mu_scale, abs_tol=1e-9):
        raise ValueError(
            f"{path} tire_mu_scale={mu_scale!r}, expected {expected_mu_scale!r}."
        )
    drive_front = _first_finite(
        payload.get("drive_distribution_front"),
        _nested(payload, ("vehicle", "drive_distribution_front")),
    )
    if not math.isclose(drive_front, 0.0, abs_tol=1e-12):
        raise ValueError(f"{path} is not rear-wheel drive.")
    aero_front = _first_finite(
        payload.get("effective_aero_balance_front"),
        _nested(payload, ("vehicle", "aero_balance_front")),
    )
    if not math.isclose(aero_front, 0.5, abs_tol=1e-9):
        raise ValueError(f"{path} does not use 50/50 aero balance.")

    reduced = payload.get("reduced_vehicle_parameters")
    if not isinstance(reduced, Mapping):
        raise TypeError(f"{path} lacks reduced_vehicle_parameters.")
    total_mass_kg = _first_finite(payload.get("total_mass_kg"), reduced.get("mass_kg"))
    sprung_mass_kg = _first_finite(
        payload.get("sprung_mass_kg"), reduced.get("sprung_mass_kg")
    )
    if not (total_mass_kg > 0.0 and 0.0 < sprung_mass_kg < total_mass_kg):
        raise ValueError(f"{path} contains invalid sprung/total masses.")
    reduced_total = _finite_float(reduced.get("mass_kg"))
    reduced_sprung = _finite_float(reduced.get("sprung_mass_kg"))
    if not (
        math.isclose(total_mass_kg, reduced_total, abs_tol=1e-9)
        and math.isclose(sprung_mass_kg, reduced_sprung, abs_tol=1e-9)
    ):
        raise ValueError(f"{path} top-level and reduced-model masses disagree.")

    candidates = _candidate_rear_fractions(payload, reduced)
    if not candidates:
        raise ValueError(f"{path} does not expose rear static weight fraction.")
    rear_fraction = candidates[0][1]
    if not 0.0 < rear_fraction < 1.0:
        raise ValueError(f"{path} has invalid rear static fraction {rear_fraction!r}.")
    disagreement = [
        (source, value)
        for source, value in candidates[1:]
        if not math.isclose(value, rear_fraction, abs_tol=fraction_tolerance)
    ]
    if disagreement:
        raise ValueError(
            f"{path} rear static fraction sources disagree: "
            f"{candidates[0]!r} versus {disagreement!r}."
        )

    center = reduced.get("center_of_gravity_m")
    try:
        center_array = np.asarray(center, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{path} has invalid center_of_gravity_m.") from exc
    if center_array.shape != (3,) or not np.isfinite(center_array).all():
        raise ValueError(f"{path} has invalid center_of_gravity_m.")
    cg_x_m = _first_finite(payload.get("longitudinal_cg_x_m"), center_array[0])
    if not math.isclose(cg_x_m, float(center_array[0]), abs_tol=1e-9):
        raise ValueError(f"{path} longitudinal CG fields disagree.")
    wheelbase_m = _first_finite(
        payload.get("wheelbase_m"), _nested(payload, ("vehicle", "wheelbase"))
    )
    if wheelbase_m > 0.0:
        cg_implied_rear = -cg_x_m / wheelbase_m
        if not math.isclose(
            cg_implied_rear, rear_fraction, abs_tol=max(fraction_tolerance, 2e-8)
        ):
            raise ValueError(
                f"{path} CG x implies {cg_implied_rear:.12f} rear but stored "
                f"static loads imply {rear_fraction:.12f}."
            )

    try:
        static_loads = np.asarray(reduced.get("static_wheel_loads_n"), dtype=float)
        sprung_inertia = np.asarray(reduced.get("sprung_inertia_kg_m2"), dtype=float)
        suspension_stiffness = np.asarray(
            reduced.get("suspension_stiffness_n_per_m"), dtype=float
        )
        antiroll_stiffness = np.asarray(
            reduced.get("antiroll_stiffness_nm_per_rad"), dtype=float
        )
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{path} contains invalid reduced-model arrays.") from exc
    required_shapes = (
        (static_loads, (4,), "static_wheel_loads_n"),
        (sprung_inertia, (3, 3), "sprung_inertia_kg_m2"),
        (suspension_stiffness, (4,), "suspension_stiffness_n_per_m"),
        (antiroll_stiffness, (2,), "antiroll_stiffness_nm_per_rad"),
    )
    for array, shape, label in required_shapes:
        if array.shape != shape or not np.isfinite(array).all():
            raise ValueError(f"{path} {label} must have finite shape {shape}.")
    static_load_sum_error_n = float(static_loads.sum() - total_mass_kg * 9.80665)
    if not math.isclose(static_load_sum_error_n, 0.0, abs_tol=1e-6):
        raise ValueError(f"{path} static wheel loads do not sum to total weight.")

    tune_rear = _first_finite(
        *(tuning_case.get(key) for key in REAR_FRACTION_KEYS)
    )
    if math.isfinite(tune_rear) and not math.isclose(
        tune_rear, rear_fraction, abs_tol=fraction_tolerance
    ):
        raise ValueError(f"{path} and tuning artifact rear fractions disagree.")

    reuse = payload.get("ggv_reuse_validation")
    if not isinstance(reuse, Mapping):
        reuse = {}
    speed_slices = payload.get("ggv_speed_slices")
    try:
        speeds = np.asarray(speed_slices, dtype=float)
    except (TypeError, ValueError):
        speeds = np.asarray([], dtype=float)
    positive_steps = np.diff(np.unique(speeds))
    positive_steps = positive_steps[positive_steps > 1e-9]
    ay_points = _finite_float(payload.get("ggv_ay_points"))
    ay_max_g = _finite_float(payload.get("ggv_ay_max_g"))
    ay_spacing_g = (
        ay_max_g / (ay_points - 1.0)
        if ay_points > 1.0 and math.isfinite(ay_max_g)
        else math.nan
    )
    tune = tuning_case
    lateral_trim = tune.get("pure_lateral_limit_trim")
    if not isinstance(lateral_trim, Mapping):
        lateral_trim = {}
    active_constraints = lateral_trim.get("active_constraints")
    if not isinstance(active_constraints, list):
        active_constraints = []
    return {
        "cg_case": case_name,
        "rear_static_weight_fraction": rear_fraction,
        "rear_static_weight_percent": rear_fraction * 100.0,
        "front_static_weight_fraction": 1.0 - rear_fraction,
        "longitudinal_cg_x_m": cg_x_m,
        "wheelbase_m": wheelbase_m,
        "cg_height_in": height_in,
        "sprung_mass_kg": sprung_mass_kg,
        "total_mass_kg": total_mass_kg,
        "unsprung_mass_kg": total_mass_kg - sprung_mass_kg,
        "static_load_sum_error_n": static_load_sum_error_n,
        "sprung_inertia_xx_kg_m2": float(sprung_inertia[0, 0]),
        "sprung_inertia_yy_kg_m2": float(sprung_inertia[1, 1]),
        "sprung_inertia_zz_kg_m2": float(sprung_inertia[2, 2]),
        "sprung_inertia_xy_kg_m2": float(sprung_inertia[0, 1]),
        "sprung_inertia_xz_kg_m2": float(sprung_inertia[0, 2]),
        "sprung_inertia_yz_kg_m2": float(sprung_inertia[1, 2]),
        "front_suspension_stiffness_n_per_m": float(
            np.mean(suspension_stiffness[:2])
        ),
        "rear_suspension_stiffness_n_per_m": float(
            np.mean(suspension_stiffness[2:])
        ),
        "total_antiroll_stiffness_nm_per_rad": float(antiroll_stiffness.sum()),
        "cl_area_m2": _finite_float(reduced.get("cl_area_m2")),
        "cd_area_m2": _finite_float(reduced.get("cd_area_m2")),
        "peak_drive_power_w": _finite_float(reduced.get("peak_drive_power_w")),
        "maximum_drive_speed_mps": _finite_float(
            reduced.get("maximum_drive_speed_mps")
        ),
        "effective_cop_from_front_m": _finite_float(
            payload.get("effective_cop_from_front_m")
        ),
        "tire_mu_scale": mu_scale,
        "aero_balance_front": aero_front,
        "drive_distribution_front": drive_front,
        "front_antiroll_stiffness_fraction": _first_finite(
            payload.get("front_antiroll_stiffness_fraction"),
            tune.get("front_antiroll_stiffness_fraction"),
        ),
        "front_elastic_roll_stiffness_fraction": _first_finite(
            payload.get("front_elastic_roll_stiffness_fraction"),
            payload.get("effective_lltd"),
            tune.get("front_elastic_roll_stiffness_fraction"),
        ),
        "front_brake_bias": _first_finite(
            payload.get("effective_brake_distribution_front"),
            tune.get("brake_distribution_front"),
            tune.get("front_brake_bias"),
        ),
        "pure_lateral_limit_g": _finite_float(tune.get("pure_lateral_limit_g")),
        "weighted_pure_braking_limit_g": _finite_float(
            tune.get("weighted_pure_braking_limit_g")
        ),
        "tuned_lateral_beta_rad": _finite_float(lateral_trim.get("beta_rad")),
        "tuned_lateral_minimum_normal_load_n": _finite_float(
            lateral_trim.get("minimum_normal_load_n")
        ),
        "roll_optimization_at_search_bound": _as_bool(
            tune.get("roll_optimization_at_search_bound", False)
        ),
        "brake_optimization_at_search_bound": _as_bool(
            tune.get("brake_optimization_at_search_bound", False)
        ),
        "tuned_lateral_active_constraints": ",".join(
            str(value) for value in active_constraints
        ),
        "wheel_load_min_n": _finite_float(payload.get("wheel_load_min_n")),
        "wheel_load_max_n": _finite_float(payload.get("wheel_load_max_n")),
        "tire_valid_load_min_n": _first_finite(
            payload.get("tire_valid_load_min_n"),
            _nested(payload, ("vehicle", "fz_min_valid")),
        ),
        "tire_valid_load_max_n": _first_finite(
            payload.get("tire_valid_load_max_n"),
            _nested(payload, ("vehicle", "fz_max_valid")),
        ),
        "wheel_load_outside_tire_validity": _as_bool(
            payload.get("wheel_load_outside_tire_validity", False)
        ),
        "ggv_source_sha256": str(payload.get("ggv_source_sha256", "")),
        "ggv_speed_step_mps": (
            float(np.median(positive_steps)) if positive_steps.size else math.nan
        ),
        "ggv_speed_slice_count": int(speeds.size),
        "ggv_requested_speed_max_mps": (
            float(speeds.max()) if speeds.size else math.nan
        ),
        "ggv_ay_points": ay_points,
        "ggv_ay_spacing_g": ay_spacing_g,
        "ggv_ax_search_points": _finite_float(payload.get("ggv_ax_search_points")),
        "ggv_ax_binary_iterations": _finite_float(
            payload.get("ggv_ax_binary_iterations")
        ),
        "requested_terminal_slice_empty": _as_bool(
            reuse.get("requested_terminal_slice_had_no_serialized_rows", False)
        ),
        "case_metadata_path": str(path.resolve()),
    }


def _load_audit(
    study_root: Path, metadata: pd.DataFrame
) -> tuple[pd.DataFrame, pd.DataFrame, dict[str, Any]]:
    audit_root = study_root / "reduced_qss_audit"
    case_path = audit_root / "case_qss_audit_summary.csv"
    state_path = audit_root / "sampled_trim_states.csv"
    json_path = audit_root / "reduced_qss_audit.json"
    present = [path.is_file() for path in (case_path, state_path, json_path)]
    if not any(present):
        return pd.DataFrame(), pd.DataFrame(), {"present": False}
    if not all(present):
        missing = [
            str(path.resolve())
            for path, exists in zip((case_path, state_path, json_path), present)
            if not exists
        ]
        raise FileNotFoundError("Incomplete reduced_qss_audit bundle: " + ", ".join(missing))
    cases = pd.read_csv(case_path)
    states = pd.read_csv(state_path)
    payload = json.loads(json_path.read_text(encoding="utf-8"))
    _require_columns(
        cases,
        (
            "cg_case",
            "model_dof",
            "sweep_axis",
            "sample_count",
            "successful_trim_count",
            "failed_trim_count",
            "sampled_normal_load_min_n",
            "sampled_normal_load_max_n",
            "sampled_wheel_lift_state_count",
        ),
        str(case_path),
    )
    if str(payload.get("sweep_axis")) != "rear_static_weight_fraction":
        raise ValueError(f"{json_path} is not a longitudinal-CG audit.")
    if set(cases["cg_case"].astype(str)) != set(metadata["cg_case"].astype(str)):
        raise ValueError("Reduced-QSS audit cases do not match the completed sweep.")
    if not bool((pd.to_numeric(cases["model_dof"], errors="coerce") == 6).all()):
        raise ValueError("Reduced-QSS audit is not uniformly 6DOF.")
    if not bool(
        (cases["sweep_axis"].astype(str) == "rear_static_weight_fraction").all()
    ):
        raise ValueError("Reduced-QSS case summaries have a mismatched sweep axis.")

    rear_map = metadata.set_index("cg_case")["rear_static_weight_fraction"]
    if "rear_static_weight_fraction" in cases:
        expected = cases["cg_case"].astype(str).map(rear_map)
        actual = pd.to_numeric(cases["rear_static_weight_fraction"], errors="coerce")
        if not np.allclose(actual, expected, atol=1e-9, rtol=0.0):
            raise ValueError("Reduced-QSS audit rear fractions do not match metadata.")
    cases = cases.copy()
    cases["rear_static_weight_fraction"] = cases["cg_case"].astype(str).map(rear_map)
    cases["rear_static_weight_percent"] = cases["rear_static_weight_fraction"] * 100.0
    if "valid_for_load_summary" in states:
        valid_states = states[states["valid_for_load_summary"].map(_as_bool)]
    elif "trim_success" in states:
        valid_states = states[states["trim_success"].map(_as_bool)]
    else:
        valid_states = states.iloc[0:0]
    failed_states = (
        states[~states["trim_success"].map(_as_bool)].copy()
        if "trim_success" in states
        else states.iloc[0:0].copy()
    )
    summary = {
        "present": True,
        "case_count": len(cases),
        "sample_count": int(pd.to_numeric(cases["sample_count"]).sum()),
        "successful_trim_count": int(
            pd.to_numeric(cases["successful_trim_count"]).sum()
        ),
        "failed_trim_count": int(pd.to_numeric(cases["failed_trim_count"]).sum()),
        "sampled_wheel_lift_state_count": int(
            pd.to_numeric(cases["sampled_wheel_lift_state_count"]).sum()
        ),
        "successful_load_state_count": len(valid_states),
        "normal_load_min_n": float(
            pd.to_numeric(cases["sampled_normal_load_min_n"]).min()
        ),
        "normal_load_max_n": float(
            pd.to_numeric(cases["sampled_normal_load_max_n"]).max()
        ),
        "audit_json_path": str(json_path.resolve()),
    }
    return cases, failed_states, summary


def _load_study(
    study_root: Path,
    tuning_cases: Mapping[str, Mapping[str, Any]],
    *,
    cohort_label: str = "primary_sweep",
    expected_rear_fractions: Sequence[float],
    expected_cg_height_in: float,
    expected_mu_scale: float,
    fraction_tolerance: float,
) -> tuple[pd.DataFrame, pd.DataFrame, dict[str, Any]]:
    event_path = study_root / "event_results.csv"
    if not event_path.is_file():
        raise FileNotFoundError(f"Missing event results: {event_path.resolve()}")
    events = pd.read_csv(event_path)
    _require_columns(
        events,
        ("cg_case", "cg_height_in", "event_slug", "lap_time_s", "track_length_m"),
        str(event_path),
    )
    if events.empty or events.duplicated(["cg_case", "event_slug"]).any():
        raise ValueError(f"{event_path} is empty or contains duplicate case/event rows.")
    case_names = events["cg_case"].astype(str).drop_duplicates().tolist()
    if len(case_names) != len(expected_rear_fractions):
        raise ValueError(
            f"Expected {len(expected_rear_fractions)} cases; found {len(case_names)}."
        )
    for case_name, group in events.groupby(events["cg_case"].astype(str)):
        if len(group) != len(EXPECTED_EVENTS) or set(group["event_slug"].astype(str)) != set(EXPECTED_EVENTS):
            raise ValueError(f"Case {case_name!r} must contain exactly four timed events.")
    if tuning_cases and set(tuning_cases) != set(case_names):
        raise ValueError("Tuning cases do not exactly match event-result cases.")

    metadata = pd.DataFrame(
        [
            _load_case_metadata(
                study_root,
                case_name,
                tuning_cases.get(case_name, {}),
                expected_cg_height_in=expected_cg_height_in,
                expected_mu_scale=expected_mu_scale,
                fraction_tolerance=fraction_tolerance,
            )
            for case_name in case_names
        ]
    ).sort_values("rear_static_weight_fraction").reset_index(drop=True)
    metadata.insert(0, "source_cohort", cohort_label)
    metadata.insert(1, "source_study_root", str(study_root.resolve()))
    actual = metadata["rear_static_weight_fraction"].to_numpy(dtype=float)
    expected = np.sort(np.asarray(expected_rear_fractions, dtype=float))
    if actual.shape != expected.shape or not np.allclose(
        actual, expected, atol=fraction_tolerance, rtol=0.0
    ):
        raise ValueError(
            f"Rear-static-weight design is {actual.tolist()}; expected {expected.tolist()}."
        )
    for column in ("cg_height_in", "sprung_mass_kg", "total_mass_kg", "unsprung_mass_kg"):
        values = metadata[column].to_numpy(dtype=float)
        if not np.allclose(values, values[0], atol=1e-9, rtol=0.0):
            raise ValueError(f"{column} is not fixed across the longitudinal-CG sweep.")
    invariant_columns = (
        "sprung_inertia_xx_kg_m2",
        "sprung_inertia_yy_kg_m2",
        "sprung_inertia_zz_kg_m2",
        "sprung_inertia_xy_kg_m2",
        "sprung_inertia_xz_kg_m2",
        "sprung_inertia_yz_kg_m2",
        "front_suspension_stiffness_n_per_m",
        "rear_suspension_stiffness_n_per_m",
        "total_antiroll_stiffness_nm_per_rad",
        "cl_area_m2",
        "cd_area_m2",
        "peak_drive_power_w",
        "maximum_drive_speed_mps",
        "effective_cop_from_front_m",
    )
    for column in invariant_columns:
        values = metadata[column].to_numpy(dtype=float)
        finite = values[np.isfinite(values)]
        if finite.size and not np.allclose(finite, finite[0], atol=1e-9, rtol=0.0):
            raise ValueError(f"{column} is not fixed across the longitudinal-CG sweep.")
    if not bool(
        (
            np.diff(metadata["longitudinal_cg_x_m"].to_numpy(dtype=float))
            < 0.0
        ).all()
    ):
        raise ValueError("Longitudinal CG x must move rearward as rear weight increases.")

    events = events.copy()
    if "sweep_axis" in events and not bool(
        (events["sweep_axis"].astype(str) == "rear_static_weight_fraction").all()
    ):
        raise ValueError("Event results contain a mismatched sweep axis.")
    for column in ("cg_height_in", "lap_time_s", "track_length_m"):
        events[column] = pd.to_numeric(events[column], errors="coerce")
    if not bool(
        np.isfinite(events[["cg_height_in", "lap_time_s", "track_length_m"]]).all(axis=None)
        and (events["lap_time_s"] > 0.0).all()
        and (events["track_length_m"] > 0.0).all()
    ):
        raise ValueError(f"{event_path} contains invalid event time or distance values.")
    metadata_by_case = metadata.set_index("cg_case")
    for key in REAR_FRACTION_KEYS:
        if key not in events:
            continue
        event_fraction = pd.to_numeric(events[key], errors="coerce")
        expected_fraction = events["cg_case"].astype(str).map(
            metadata_by_case["rear_static_weight_fraction"]
        )
        if not np.allclose(
            event_fraction, expected_fraction, atol=fraction_tolerance, rtol=0.0
        ):
            raise ValueError(f"Event-result {key} values disagree with case metadata.")
        events.rename(columns={key: f"source_{key}"}, inplace=True)
    if "model_key" in events:
        events.rename(columns={"model_key": "source_model_key"}, inplace=True)
    events = events.merge(
        metadata[
            [
                "cg_case",
                "rear_static_weight_fraction",
                "rear_static_weight_percent",
                "front_static_weight_fraction",
                "longitudinal_cg_x_m",
                "sprung_mass_kg",
                "total_mass_kg",
            ]
        ],
        on="cg_case",
        how="left",
        validate="many_to_one",
    )
    events.insert(0, "model_key", "dyn_py_6dof_qss")
    events.insert(1, "source_cohort", cohort_label)
    events.insert(2, "source_study_root", str(study_root.resolve()))
    events["all_track_solvers_converged"] = (
        events["converged"].map(_as_bool) if "converged" in events else True
    )
    if "ggv_speed_cap_segments" in events:
        events["ggv_speed_cap_segments"] = pd.to_numeric(
            events["ggv_speed_cap_segments"], errors="coerce"
        ).fillna(0)
    else:
        events["ggv_speed_cap_segments"] = 0
    design = {
        "sweep_axis": "rear_static_weight_fraction",
        "rear_static_weight_fractions": actual.tolist(),
        "rear_static_weight_percents": np.round(actual * 100.0, 12).tolist(),
        "rear_weight_step_percentage_points": np.diff(actual * 100.0).tolist(),
        "cg_height_in": float(metadata["cg_height_in"].iloc[0]),
        "sprung_mass_kg": float(metadata["sprung_mass_kg"].iloc[0]),
        "total_mass_kg": float(metadata["total_mass_kg"].iloc[0]),
        "fixed_mass_and_height": True,
        "rear_weight_design_passed": True,
    }
    return events, metadata, design


def _load_fine_validation_study(
    fine_root: Path,
    coarse_metadata: pd.DataFrame,
    *,
    expected_rear_fractions: Sequence[float],
    expected_cg_height_in: float,
    expected_mu_scale: float,
    fraction_tolerance: float,
) -> tuple[pd.DataFrame, pd.DataFrame, dict[str, Any]]:
    """Load split fine-grid case caches and verify controlled inputs."""
    fine_root = fine_root.resolve()
    cache_paths = sorted(fine_root.glob("*/*/raw_case_results.json"))
    if not cache_paths:
        raise FileNotFoundError(
            f"No fine-grid case caches found below {fine_root}."
        )

    metadata_rows: list[dict[str, Any]] = []
    event_rows: list[dict[str, Any]] = []
    seen_cases: set[str] = set()
    for cache_path in cache_paths:
        case_dir = cache_path.parent
        case_name = case_dir.name
        if case_name in seen_cases:
            raise ValueError(f"Duplicate fine-grid case cache for {case_name!r}.")
        seen_cases.add(case_name)
        study_root = case_dir.parent
        metadata_path = case_dir / "case_metadata.json"
        if not metadata_path.is_file():
            raise FileNotFoundError(f"Missing fine case metadata: {metadata_path}.")
        cache = json.loads(cache_path.read_text(encoding="utf-8"))
        standalone = json.loads(metadata_path.read_text(encoding="utf-8"))
        if not isinstance(cache, Mapping) or cache.get("schema") != (
            "lapsims.ggv-cg-case-cache.v1"
        ):
            raise ValueError(f"{cache_path} is not a completed GGV case cache.")
        if not isinstance(standalone, Mapping):
            raise TypeError(f"{metadata_path} must contain a JSON object.")
        embedded = cache.get("case_metadata")
        if not isinstance(embedded, Mapping):
            raise TypeError(f"{cache_path} lacks embedded case metadata.")
        fingerprints = {
            str(cache.get("input_fingerprint_sha256", "")),
            str(embedded.get("input_fingerprint_sha256", "")),
            str(standalone.get("input_fingerprint_sha256", "")),
        }
        if "" in fingerprints or len(fingerprints) != 1:
            raise ValueError(
                f"{cache_path} cache and standalone metadata fingerprints disagree."
            )
        for label, payload in (("embedded", embedded), ("standalone", standalone)):
            if str(payload.get("cg_case")) != case_name:
                raise ValueError(
                    f"{cache_path} {label} metadata has a mismatched cg_case."
                )
        for key in (
            "rear_static_weight_fraction",
            "cg_height_in",
            "tire_mu_scale",
        ):
            if not math.isclose(
                _finite_float(embedded.get(key)),
                _finite_float(standalone.get(key)),
                abs_tol=fraction_tolerance,
            ):
                raise ValueError(
                    f"{cache_path} embedded and standalone {key} disagree."
                )

        metadata_row = _load_case_metadata(
            study_root,
            case_name,
            {},
            expected_cg_height_in=expected_cg_height_in,
            expected_mu_scale=expected_mu_scale,
            fraction_tolerance=fraction_tolerance,
        )
        metadata_row["fine_case_study_root"] = str(study_root.resolve())
        metadata_rows.append(metadata_row)

        raw_events = cache.get("raw_event_results")
        if not isinstance(raw_events, list) or len(raw_events) != len(EXPECTED_EVENTS):
            raise ValueError(f"{cache_path} must contain exactly four timed events.")
        for raw_event in raw_events:
            if not isinstance(raw_event, Mapping):
                raise TypeError(f"{cache_path} contains an invalid event result.")
            event = dict(raw_event)
            if str(event.get("cg_case")) != case_name:
                raise ValueError(f"{cache_path} contains a mismatched event cg_case.")
            if event.get("model_key") != "dyn_py_6dof_qss" or int(
                event.get("model_dof", -1)
            ) != 6:
                raise ValueError(f"{cache_path} contains a non-6DOF event result.")
            if str(event.get("sweep_axis")) != "rear_static_weight_fraction":
                raise ValueError(f"{cache_path} contains a mismatched sweep axis.")
            if not math.isclose(
                _finite_float(event.get("rear_static_weight_fraction")),
                metadata_row["rear_static_weight_fraction"],
                abs_tol=fraction_tolerance,
            ):
                raise ValueError(f"{cache_path} event rear fraction disagrees.")
            if not math.isclose(
                _finite_float(event.get("tire_mu_scale")),
                expected_mu_scale,
                abs_tol=1e-9,
            ):
                raise ValueError(f"{cache_path} event tire mu scale disagrees.")
            event["fine_case_study_root"] = str(study_root.resolve())
            event_rows.append(event)

    metadata = pd.DataFrame(metadata_rows).sort_values(
        "rear_static_weight_fraction"
    ).reset_index(drop=True)
    metadata.insert(0, "source_cohort", "fine_validation")
    metadata.insert(1, "source_study_root", str(fine_root))
    expected = np.sort(np.asarray(expected_rear_fractions, dtype=float))
    actual = metadata["rear_static_weight_fraction"].to_numpy(dtype=float)
    if actual.shape != expected.shape or not np.allclose(
        actual, expected, atol=fraction_tolerance, rtol=0.0
    ):
        raise ValueError(
            f"Fine rear-weight design is {actual.tolist()}; expected "
            f"{expected.tolist()}."
        )

    events = pd.DataFrame(event_rows)
    _require_columns(
        events,
        (
            "cg_case",
            "cg_height_in",
            "event_slug",
            "lap_time_s",
            "track_length_m",
            "converged",
        ),
        str(fine_root),
    )
    if events.duplicated(["cg_case", "event_slug"]).any():
        raise ValueError("Fine validation contains duplicate case/event rows.")
    for case_name, group in events.groupby(events["cg_case"].astype(str)):
        if set(group["event_slug"].astype(str)) != set(EXPECTED_EVENTS):
            raise ValueError(f"Fine case {case_name!r} does not contain four events.")
    for column in ("cg_height_in", "lap_time_s", "track_length_m"):
        events[column] = pd.to_numeric(events[column], errors="coerce")
    if not bool(
        np.isfinite(
            events[["cg_height_in", "lap_time_s", "track_length_m"]]
        ).all(axis=None)
        and (events["lap_time_s"] > 0.0).all()
        and (events["track_length_m"] > 0.0).all()
    ):
        raise ValueError("Fine validation contains invalid event times or distances.")
    metadata_by_case = metadata.set_index("cg_case")
    source_rear = pd.to_numeric(
        events["rear_static_weight_fraction"], errors="coerce"
    )
    expected_rear = events["cg_case"].astype(str).map(
        metadata_by_case["rear_static_weight_fraction"]
    )
    if not np.allclose(
        source_rear, expected_rear, atol=fraction_tolerance, rtol=0.0
    ):
        raise ValueError("Fine event rear fractions disagree with case metadata.")
    events.rename(
        columns={
            "model_key": "source_model_key",
            "rear_static_weight_fraction": "source_rear_static_weight_fraction",
        },
        inplace=True,
    )
    events = events.merge(
        metadata[
            [
                "cg_case",
                "rear_static_weight_fraction",
                "rear_static_weight_percent",
                "front_static_weight_fraction",
                "longitudinal_cg_x_m",
                "sprung_mass_kg",
                "total_mass_kg",
            ]
        ],
        on="cg_case",
        how="left",
        validate="many_to_one",
    )
    events.insert(0, "model_key", "dyn_py_6dof_qss")
    events.insert(1, "source_cohort", "fine_validation")
    events.insert(2, "source_study_root", str(fine_root))
    events["all_track_solvers_converged"] = events["converged"].map(_as_bool)
    if "ggv_speed_cap_segments" in events:
        events["ggv_speed_cap_segments"] = pd.to_numeric(
            events["ggv_speed_cap_segments"], errors="coerce"
        ).fillna(0)
    else:
        events["ggv_speed_cap_segments"] = 0

    coarse_selected = coarse_metadata[
        coarse_metadata["rear_static_weight_fraction"].apply(
            lambda value: bool(
                np.isclose(value, expected, atol=fraction_tolerance, rtol=0.0).any()
            )
        )
    ].sort_values("rear_static_weight_fraction")
    if len(coarse_selected) != len(metadata):
        raise ValueError("Each fine validation case must match one coarse case.")
    controlled_columns = (
        "rear_static_weight_fraction",
        "front_static_weight_fraction",
        "longitudinal_cg_x_m",
        "cg_height_in",
        "sprung_mass_kg",
        "total_mass_kg",
        "unsprung_mass_kg",
        "sprung_inertia_xx_kg_m2",
        "sprung_inertia_yy_kg_m2",
        "sprung_inertia_zz_kg_m2",
        "sprung_inertia_xy_kg_m2",
        "sprung_inertia_xz_kg_m2",
        "sprung_inertia_yz_kg_m2",
        "front_suspension_stiffness_n_per_m",
        "rear_suspension_stiffness_n_per_m",
        "total_antiroll_stiffness_nm_per_rad",
        "cl_area_m2",
        "cd_area_m2",
        "peak_drive_power_w",
        "maximum_drive_speed_mps",
        "effective_cop_from_front_m",
        "tire_mu_scale",
        "aero_balance_front",
        "drive_distribution_front",
        "front_antiroll_stiffness_fraction",
        "front_elastic_roll_stiffness_fraction",
        "front_brake_bias",
        "tire_valid_load_min_n",
        "tire_valid_load_max_n",
    )
    fine_by_case = metadata.set_index("cg_case")
    coarse_by_case = coarse_selected.set_index("cg_case")
    if set(fine_by_case.index) != set(coarse_by_case.index):
        raise ValueError("Fine and coarse validation case names do not match.")
    for case_name in fine_by_case.index:
        for column in controlled_columns:
            fine_value = _finite_float(fine_by_case.loc[case_name, column])
            coarse_value = _finite_float(coarse_by_case.loc[case_name, column])
            if not (
                math.isfinite(fine_value)
                and math.isfinite(coarse_value)
                and math.isclose(fine_value, coarse_value, abs_tol=1e-9)
            ):
                raise ValueError(
                    f"Fine/coarse controlled input {column!r} differs for "
                    f"{case_name!r}: {fine_value!r} versus {coarse_value!r}."
                )

    def uniform_value(frame: pd.DataFrame, column: str) -> float:
        values = frame[column].to_numpy(dtype=float)
        if not np.isfinite(values).all() or not np.allclose(
            values, values[0], atol=1e-9, rtol=0.0
        ):
            raise ValueError(f"Resolution field {column!r} is not uniform.")
        return float(values[0])

    fine_grid = {
        "nominal_speed_step_mps": uniform_value(metadata, "ggv_speed_step_mps"),
        "ay_points": round(uniform_value(metadata, "ggv_ay_points")),
        "ax_binary_iterations": round(
            uniform_value(metadata, "ggv_ax_binary_iterations")
        ),
        "ax_search_points": round(uniform_value(metadata, "ggv_ax_search_points")),
    }
    coarse_grid = {
        "nominal_speed_step_mps": uniform_value(
            coarse_selected, "ggv_speed_step_mps"
        ),
        "ay_points": round(uniform_value(coarse_selected, "ggv_ay_points")),
        "ax_binary_iterations": round(
            uniform_value(coarse_selected, "ggv_ax_binary_iterations")
        ),
        "ax_search_points": round(
            uniform_value(coarse_selected, "ggv_ax_search_points")
        ),
    }
    expected_fine_grid = {
        "nominal_speed_step_mps": 1.0,
        "ay_points": 23,
        "ax_binary_iterations": 12,
        "ax_search_points": 301,
    }
    expected_coarse_grid = {
        "nominal_speed_step_mps": 2.0,
        "ay_points": 23,
        "ax_binary_iterations": 10,
        "ax_search_points": 301,
    }
    if fine_grid != expected_fine_grid or coarse_grid != expected_coarse_grid:
        raise ValueError(
            f"Unexpected coarse/fine grids: {coarse_grid!r}, {fine_grid!r}."
        )
    validation = {
        "passed": True,
        "case_count": len(metadata),
        "rear_static_weight_percents": np.round(actual * 100.0, 12).tolist(),
        "controlled_inputs_match_coarse": True,
        "controlled_columns": list(controlled_columns),
        "coarse_grid": coarse_grid,
        "fine_grid": fine_grid,
        "tire_mu_scale": expected_mu_scale,
        "serialized_tire_mu_scale": uniform_value(metadata, "tire_mu_scale"),
        "legacy_serialized_load_diagnostics": [
            {
                "rear_static_weight_percent": float(
                    row.rear_static_weight_percent
                ),
                "wheel_load_min_n": float(row.wheel_load_min_n),
                "wheel_load_max_n": float(row.wheel_load_max_n),
                "tire_valid_load_min_n": float(row.tire_valid_load_min_n),
                "tire_valid_load_max_n": float(row.tire_valid_load_max_n),
                "outside_tire_validity": bool(
                    row.wheel_load_outside_tire_validity
                ),
            }
            for row in metadata.itertuples(index=False)
        ],
        "fine_reduced_qss_audit_present": bool(
            list(fine_root.rglob("reduced_qss_audit.json"))
        ),
        "requested_terminal_slice_empty_cases": sorted(
            {
                str(row.cg_case)
                for row in events.itertuples(index=False)
                if math.isfinite(
                    _finite_float(getattr(row, "ggv_csv_speed_max_mps", math.nan))
                )
                and _finite_float(
                    metadata_by_case.loc[
                        str(row.cg_case), "ggv_requested_speed_max_mps"
                    ]
                )
                > _finite_float(
                    getattr(row, "ggv_csv_speed_max_mps", math.nan)
                )
                + 1e-9
            }
            | set(
                metadata.loc[
                    metadata["requested_terminal_slice_empty"].map(_as_bool),
                    "cg_case",
                ].astype(str)
            )
        ),
        "all_track_solvers_converged": bool(
            events["all_track_solvers_converged"].all()
        ),
        "total_ggv_speed_cap_segments": int(
            events["ggv_speed_cap_segments"].sum()
        ),
        "fine_validation_root": str(fine_root),
    }
    return events, metadata, validation


def _build_fine_resolution_validation(
    *,
    fine_root: Path,
    coarse_events: pd.DataFrame,
    coarse_metadata: pd.DataFrame,
    scoring_reference: Mapping[str, Any],
    expected_rear_fractions: Sequence[float],
    expected_cg_height_in: float,
    expected_mu_scale: float,
    fraction_tolerance: float,
) -> dict[str, Any]:
    fine_events, fine_metadata, validation = _load_fine_validation_study(
        fine_root,
        coarse_metadata,
        expected_rear_fractions=expected_rear_fractions,
        expected_cg_height_in=expected_cg_height_in,
        expected_mu_scale=expected_mu_scale,
        fraction_tolerance=fraction_tolerance,
    )
    expected = np.asarray(expected_rear_fractions, dtype=float)
    coarse_mask = coarse_events["rear_static_weight_fraction"].apply(
        lambda value: bool(
            np.isclose(value, expected, atol=fraction_tolerance, rtol=0.0).any()
        )
    )
    coarse_meta_mask = coarse_metadata["rear_static_weight_fraction"].apply(
        lambda value: bool(
            np.isclose(value, expected, atol=fraction_tolerance, rtol=0.0).any()
        )
    )
    coarse_selected_events = coarse_events[coarse_mask].copy()
    coarse_selected_metadata = coarse_metadata[coarse_meta_mask].copy()
    coarse_selected_result = _score_cohort(
        coarse_selected_events, coarse_selected_metadata, scoring_reference
    )
    fine_result = _score_cohort(fine_events, fine_metadata, scoring_reference)

    coarse_scored = coarse_selected_result["events"][
        [
            "cg_case",
            "event_slug",
            "rear_static_weight_percent",
            "lap_time_s",
            "common_projected_points",
        ]
    ].rename(
        columns={
            "lap_time_s": "coarse_time_s",
            "common_projected_points": "coarse_three_case_points",
        }
    )
    fine_scored = fine_result["events"][
        [
            "cg_case",
            "event_slug",
            "rear_static_weight_percent",
            "lap_time_s",
            "common_projected_points",
        ]
    ].rename(
        columns={
            "lap_time_s": "fine_time_s",
            "common_projected_points": "fine_three_case_points",
        }
    )
    event_comparison = coarse_scored.merge(
        fine_scored,
        on=["cg_case", "event_slug", "rear_static_weight_percent"],
        how="inner",
        validate="one_to_one",
    )
    event_comparison["event_name"] = event_comparison["event_slug"].map(
        EVENT_LABELS
    )
    event_comparison["fine_minus_coarse_time_s"] = (
        event_comparison["fine_time_s"] - event_comparison["coarse_time_s"]
    )
    event_comparison["fine_minus_coarse_time_percent"] = 100.0 * (
        event_comparison["fine_time_s"] / event_comparison["coarse_time_s"]
        - 1.0
    )
    event_comparison["fine_minus_coarse_event_points"] = (
        event_comparison["fine_three_case_points"]
        - event_comparison["coarse_three_case_points"]
    )
    event_comparison = event_comparison.sort_values(
        ["rear_static_weight_percent", "event_slug"]
    ).reset_index(drop=True)

    coarse_totals = coarse_selected_result["totals"][
        ["cg_case", "rear_static_weight_percent", "timed_event_points"]
    ].rename(columns={"timed_event_points": "coarse_three_case_total_points"})
    fine_totals = fine_result["totals"][
        ["cg_case", "rear_static_weight_percent", "timed_event_points"]
    ].rename(columns={"timed_event_points": "fine_three_case_total_points"})
    ranking = coarse_totals.merge(
        fine_totals,
        on=["cg_case", "rear_static_weight_percent"],
        how="inner",
        validate="one_to_one",
    ).sort_values("rear_static_weight_percent")
    ranking["coarse_rank"] = (
        ranking["coarse_three_case_total_points"]
        .rank(method="min", ascending=False)
        .astype(int)
    )
    ranking["fine_rank"] = (
        ranking["fine_three_case_total_points"]
        .rank(method="min", ascending=False)
        .astype(int)
    )
    ranking["fine_minus_coarse_total_points"] = (
        ranking["fine_three_case_total_points"]
        - ranking["coarse_three_case_total_points"]
    )
    coarse_best = float(
        ranking.loc[
            ranking["coarse_three_case_total_points"].idxmax(),
            "rear_static_weight_percent",
        ]
    )
    fine_best = float(
        ranking.loc[
            ranking["fine_three_case_total_points"].idxmax(),
            "rear_static_weight_percent",
        ]
    )
    validation.update(
        {
            "coarse_three_case_best_rear_static_weight_percent": coarse_best,
            "fine_three_case_best_rear_static_weight_percent": fine_best,
            "best_case_ranking_survives": math.isclose(
                coarse_best, fine_best, abs_tol=fraction_tolerance
            ),
            "rear_58_remains_best": math.isclose(
                fine_best, 58.0, abs_tol=fraction_tolerance
            ),
            "fine_event_scoring": fine_result["event_scoring"],
            "coarse_selected_event_scoring": coarse_selected_result[
                "event_scoring"
            ],
        }
    )
    return {
        "validation": validation,
        "fine_result": fine_result,
        "coarse_selected_result": coarse_selected_result,
        "event_comparison": event_comparison,
        "ranking": ranking.reset_index(drop=True),
    }


def _validate_cross_study_invariants(
    primary: pd.DataFrame,
    comparison: pd.DataFrame,
    *,
    fraction_tolerance: float,
) -> dict[str, Any]:
    """Require the two source sweeps to differ only in CG position and setup."""
    combined = pd.concat([primary, comparison], ignore_index=True)
    if combined["cg_case"].astype(str).duplicated().any():
        duplicates = sorted(
            combined.loc[
                combined["cg_case"].astype(str).duplicated(keep=False), "cg_case"
            ].astype(str).unique()
        )
        raise ValueError(f"Comparison sweeps contain duplicate case names: {duplicates}.")
    fractions = np.sort(
        combined["rear_static_weight_fraction"].to_numpy(dtype=float)
    )
    if np.any(np.diff(fractions) <= fraction_tolerance):
        raise ValueError(
            "Comparison sweeps contain duplicate or indistinguishable rear-weight "
            "fractions; this report requires every simulation to remain in the "
            "common scoring cohort."
        )
    invariant_columns = (
        "cg_height_in",
        "sprung_mass_kg",
        "total_mass_kg",
        "unsprung_mass_kg",
        "sprung_inertia_xx_kg_m2",
        "sprung_inertia_yy_kg_m2",
        "sprung_inertia_zz_kg_m2",
        "sprung_inertia_xy_kg_m2",
        "sprung_inertia_xz_kg_m2",
        "sprung_inertia_yz_kg_m2",
        "front_suspension_stiffness_n_per_m",
        "rear_suspension_stiffness_n_per_m",
        "total_antiroll_stiffness_nm_per_rad",
        "cl_area_m2",
        "cd_area_m2",
        "peak_drive_power_w",
        "maximum_drive_speed_mps",
        "effective_cop_from_front_m",
        "tire_mu_scale",
        "aero_balance_front",
        "drive_distribution_front",
        "tire_valid_load_min_n",
        "tire_valid_load_max_n",
    )
    for column in invariant_columns:
        values = combined[column].to_numpy(dtype=float)
        finite = values[np.isfinite(values)]
        if finite.size and not np.allclose(finite, finite[0], atol=1e-9, rtol=0.0):
            raise ValueError(
                f"Controlled-input invariant {column!r} differs between sweeps."
            )
    return {
        "passed": True,
        "case_count": len(combined),
        "all_rear_fractions_unique": True,
        "rear_static_weight_percents": np.round(fractions * 100.0, 12).tolist(),
        "invariant_columns": list(invariant_columns),
    }


def _fit_summary(x_percent: np.ndarray, y: np.ndarray) -> dict[str, Any]:
    if x_percent.ndim != 1 or y.ndim != 1 or x_percent.size != y.size:
        raise ValueError("Correlation inputs must be equal-length one-dimensional arrays.")
    if x_percent.size < 3:
        raise ValueError("At least three distinct rear-weight points are required.")
    order = np.argsort(x_percent)
    x_percent = x_percent[order]
    y = y[order]
    if np.any(np.diff(x_percent) <= 0.0):
        raise ValueError("Rear-weight fit coordinates must be strictly increasing.")
    center = 50.0
    centered = x_percent - center
    linear_coefficients = np.polyfit(centered, y, 1)
    fitted = np.polyval(linear_coefficients, centered)
    residual = y - fitted
    ss_res = float(np.sum(residual**2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0.0 else 1.0
    pearson = float(np.corrcoef(x_percent, y)[0, 1]) if ss_tot > 0.0 else 0.0
    quadratic = np.polyfit(centered, y, 2)
    a, b, c = (float(value) for value in quadratic)
    vertex_percent = 50.0 - b / (2.0 * a) if abs(a) > 1e-15 else math.nan
    vertex_is_in_range = bool(
        math.isfinite(vertex_percent)
        and float(x_percent.min()) <= vertex_percent <= float(x_percent.max())
    )
    vertex_is_local_maximum = bool(a < 0.0 and vertex_is_in_range)
    vertex_points = (
        float(np.polyval(quadratic, vertex_percent - center))
        if math.isfinite(vertex_percent)
        else math.nan
    )
    endpoint_change = float(y[-1] - y[0])
    return {
        "sample_count": int(x_percent.size),
        "x_unit": "rear_static_weight_percentage_point",
        "rear_weight_min_percent": float(x_percent[0]),
        "rear_weight_max_percent": float(x_percent[-1]),
        "fit_center_rear_weight_percent": center,
        "linear_slope_points_per_rear_weight_percentage_point": float(
            linear_coefficients[0]
        ),
        "linear_intercept_points_at_50_percent_rear": float(linear_coefficients[1]),
        "linear_r_squared": r2,
        "pearson_r": pearson,
        "linear_rmse_points": float(np.sqrt(np.mean(residual**2))),
        "quadratic_coefficients_centered_at_50_percent": [a, b, c],
        "quadratic_vertex_rear_weight_percent": vertex_percent,
        "quadratic_vertex_is_in_sample_range": vertex_is_in_range,
        "quadratic_vertex_is_local_maximum": vertex_is_local_maximum,
        "quadratic_vertex_predicted_points": vertex_points,
        "quadratic_diagnostic_only": True,
        "quadratic_vertex_is_three_point_interpolation_only": bool(
            x_percent.size == 3
        ),
        "endpoint_change_points_max_minus_min": endpoint_change,
        # Compatibility alias for reports generated before arbitrary grids were
        # supported.  New code must use endpoint_change_points_max_minus_min.
        "endpoint_change_points_55_minus_45": endpoint_change,
        "adjacent_slopes_points_per_percentage_point": (
            np.diff(y) / np.diff(x_percent)
        ).tolist(),
    }


def _build_case_totals(
    events: pd.DataFrame, metadata: pd.DataFrame
) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for case_name, group in events.groupby("cg_case", sort=False):
        meta = metadata[metadata["cg_case"] == case_name].iloc[0]
        row: dict[str, Any] = {
            **meta.to_dict(),
            "timed_event_points": float(group["common_projected_points"].sum()),
            "maximum_timed_event_points": float(group["common_maximum_points"].sum()),
            "all_track_solvers_converged": bool(
                group["all_track_solvers_converged"].all()
            ),
            "total_ggv_speed_cap_segments": int(group["ggv_speed_cap_segments"].sum()),
        }
        for event_slug in EXPECTED_EVENTS:
            event = group[group["event_slug"] == event_slug].iloc[0]
            row[f"{event_slug}_time_s"] = float(event["lap_time_s"])
            row[f"{event_slug}_competition_time_s"] = float(
                event["common_projected_competition_time_s"]
            )
            row[f"{event_slug}_points"] = float(event["common_projected_points"])
        rows.append(row)
    return pd.DataFrame(rows).sort_values("rear_static_weight_fraction").reset_index(drop=True)


def _build_event_sensitivity(events: pd.DataFrame, total_endpoint_change: float) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for event_slug in EXPECTED_EVENTS:
        group = events[events["event_slug"] == event_slug].sort_values(
            "rear_static_weight_percent"
        )
        x = group["rear_static_weight_percent"].to_numpy(dtype=float)
        raw_time = group["lap_time_s"].to_numpy(dtype=float)
        competition_time = group["common_projected_competition_time_s"].to_numpy(
            dtype=float
        )
        points = group["common_projected_points"].to_numpy(dtype=float)
        endpoint_change = float(points[-1] - points[0])
        time_change_percent = float(100.0 * (raw_time[-1] / raw_time[0] - 1.0))
        rows.append(
            {
                "event_slug": event_slug,
                "event_name": EVENT_LABELS[event_slug],
                "rear_weight_min_percent": float(x[0]),
                "rear_weight_max_percent": float(x[-1]),
                "time_at_min_rear_s": float(raw_time[0]),
                "time_at_max_rear_s": float(raw_time[-1]),
                "time_endpoint_change_percent": time_change_percent,
                "points_at_min_rear": float(points[0]),
                "points_at_max_rear": float(points[-1]),
                "points_endpoint_change": endpoint_change,
                # Backward-compatible names retained for existing bundles.
                "time_45_s": float(raw_time[0]),
                "time_50_s": float(raw_time[len(raw_time) // 2]),
                "time_55_s": float(raw_time[-1]),
                "time_change_55_vs_45_percent": time_change_percent,
                "competition_time_slope_per_rear_weight_percentage_point": float(
                    np.polyfit(x, competition_time, 1)[0]
                ),
                "points_45": float(points[0]),
                "points_50": float(points[len(points) // 2]),
                "points_55": float(points[-1]),
                "points_change_55_minus_45": endpoint_change,
                "linear_points_per_rear_weight_percentage_point": float(
                    np.polyfit(x, points, 1)[0]
                ),
                "share_of_total_endpoint_change_percent": (
                    100.0 * endpoint_change / total_endpoint_change
                    if abs(total_endpoint_change) > 1e-12
                    else math.nan
                ),
            }
        )
    return pd.DataFrame(rows)


def _build_event_points_delta_vs_50pct(
    events: pd.DataFrame,
    *,
    fraction_tolerance: float,
) -> pd.DataFrame:
    """Return common-field projected event-point deltas from the exact 50% case."""

    required = (
        "cg_case",
        "event_slug",
        "event_name",
        "rear_static_weight_fraction",
        "rear_static_weight_percent",
        "common_projected_competition_time_s",
        "common_field_tmin_s",
        "common_maximum_points",
        "common_projected_points",
    )
    _require_columns(events, required, "common-field event results")
    rows: list[pd.DataFrame] = []
    for event_slug in EXPECTED_EVENTS:
        group = events[events["event_slug"] == event_slug].copy()
        rear_percent = group["rear_static_weight_percent"].to_numpy(dtype=float)
        baseline_mask = np.isclose(
            rear_percent,
            POINT_DELTA_BASELINE_REAR_WEIGHT_PERCENT,
            rtol=0.0,
            atol=100.0 * fraction_tolerance,
        )
        if int(baseline_mask.sum()) != 1:
            raise ValueError(
                f"{event_slug} requires exactly one simulated 50% rear case to "
                "build point deltas; interpolation is not permitted."
            )
        baseline = group.loc[baseline_mask].iloc[0]
        baseline_points = float(baseline["common_projected_points"])
        group["baseline_rear_static_weight_fraction"] = 0.5
        group["baseline_rear_static_weight_percent"] = (
            POINT_DELTA_BASELINE_REAR_WEIGHT_PERCENT
        )
        group["baseline_cg_case"] = str(baseline["cg_case"])
        group["baseline_projected_points"] = baseline_points
        group["projected_points_delta_vs_50pct_rear"] = (
            group["common_projected_points"].astype(float) - baseline_points
        )
        group["is_50pct_baseline_case"] = baseline_mask
        rows.append(group)

    result = pd.concat(rows, ignore_index=True)
    columns = [
        "cg_case",
        "source_cohort",
        "rear_static_weight_fraction",
        "rear_static_weight_percent",
        "event_slug",
        "event_name",
        "common_projected_competition_time_s",
        "common_field_tmin_s",
        "common_maximum_points",
        "common_projected_points",
        "baseline_rear_static_weight_fraction",
        "baseline_rear_static_weight_percent",
        "baseline_cg_case",
        "baseline_projected_points",
        "projected_points_delta_vs_50pct_rear",
        "is_50pct_baseline_case",
    ]
    if "source_cohort" not in result.columns:
        columns.remove("source_cohort")
    result = result.loc[:, columns].sort_values(
        ["rear_static_weight_fraction", "event_slug"]
    )
    baseline_delta = result.loc[
        result["is_50pct_baseline_case"].map(_as_bool),
        "projected_points_delta_vs_50pct_rear",
    ].to_numpy(dtype=float)
    if not bool((baseline_delta == 0.0).all()):
        raise AssertionError("The exact 50% rear baseline must have zero point delta.")
    return result.reset_index(drop=True)


def _validate_exact_tmin_scoring(rescored: pd.DataFrame) -> None:
    awarded_maximum = np.isclose(
        rescored["common_projected_points"].to_numpy(dtype=float),
        rescored["common_maximum_points"].to_numpy(dtype=float),
        rtol=0.0,
        atol=0.0,
    )
    exact_tmin = (
        rescored["common_projected_competition_time_s"].to_numpy(dtype=float)
        == rescored["common_field_tmin_s"].to_numpy(dtype=float)
    )
    if not np.array_equal(awarded_maximum, exact_tmin):
        raise AssertionError("Maximum points were not limited to exact Tmin ties.")
    for event_slug in EXPECTED_EVENTS:
        winners = rescored[
            (rescored["event_slug"] == event_slug)
            & rescored["common_is_event_fastest"].map(_as_bool)
        ]["common_projected_competition_time_s"].to_numpy(dtype=float)
        if winners.size > 1 and not bool((winners == winners[0]).all()):
            raise AssertionError(f"{event_slug} awarded a non-identical simulated tie.")


def _build_event_optima(events: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for event_slug in EXPECTED_EVENTS:
        group = events[events["event_slug"] == event_slug].copy()
        fastest_time = float(group["lap_time_s"].min())
        winners = group[group["lap_time_s"] == fastest_time].sort_values(
            "rear_static_weight_percent"
        )
        winner = winners.iloc[0]
        rows.append(
            {
                "event_slug": event_slug,
                "event_name": EVENT_LABELS[event_slug],
                "fastest_raw_time_s": fastest_time,
                "fastest_case_ids": ",".join(winners["cg_case"].astype(str)),
                "fastest_rear_weight_percents": ",".join(
                    f"{value:.12g}"
                    for value in winners["rear_static_weight_percent"].astype(float)
                ),
                "fastest_source_cohorts": ",".join(
                    winners["source_cohort"].astype(str).drop_duplicates()
                ),
                "points_at_fastest_case": float(winner["common_projected_points"]),
                "competition_time_at_fastest_case_s": float(
                    winner["common_projected_competition_time_s"]
                ),
            }
        )
    return pd.DataFrame(rows)


def _score_cohort(
    events: pd.DataFrame,
    metadata: pd.DataFrame,
    scoring_reference: Mapping[str, Any],
) -> dict[str, Any]:
    rescored, event_scoring = _rescore_common_field(events, scoring_reference)
    _validate_exact_tmin_scoring(rescored)
    totals = _build_case_totals(rescored, metadata)
    fit = _fit_summary(
        totals["rear_static_weight_percent"].to_numpy(dtype=float),
        totals["timed_event_points"].to_numpy(dtype=float),
    )
    event_sensitivity = _build_event_sensitivity(
        rescored, float(fit["endpoint_change_points_max_minus_min"])
    )
    return {
        "events": rescored,
        "event_scoring": event_scoring,
        "totals": totals,
        "fit": fit,
        "event_sensitivity": event_sensitivity,
        "event_optima": _build_event_optima(rescored),
    }


def _attach_audit_to_totals(
    totals: pd.DataFrame, audit_cases: pd.DataFrame
) -> pd.DataFrame:
    totals = totals.copy()
    if audit_cases.empty:
        return totals
    audit_by_case = audit_cases.set_index("cg_case")
    for index, row in totals.iterrows():
        case_name = str(row["cg_case"])
        if case_name not in audit_by_case.index:
            raise ValueError(f"Missing QSS audit summary for {case_name!r}.")
        audit = audit_by_case.loc[case_name]
        totals.loc[index, "sampled_qss_sample_count"] = int(audit["sample_count"])
        totals.loc[index, "sampled_qss_successful_trim_count"] = int(
            audit["successful_trim_count"]
        )
        totals.loc[index, "sampled_qss_normal_load_min_n"] = _finite_float(
            audit["sampled_normal_load_min_n"]
        )
        totals.loc[index, "sampled_qss_normal_load_max_n"] = _finite_float(
            audit["sampled_normal_load_max_n"]
        )
        totals.loc[index, "sampled_qss_failed_trim_count"] = int(
            audit["failed_trim_count"]
        )
        totals.loc[index, "sampled_qss_wheel_lift_state_count"] = int(
            audit["sampled_wheel_lift_state_count"]
        )
        totals.loc[index, "sampled_qss_state_count_below_tir"] = int(
            audit.get(
                "sampled_state_count_below_100n",
                _finite_float(audit["sampled_normal_load_min_n"]) < 100.0,
            )
        )
        totals.loc[index, "sampled_qss_state_count_above_tir"] = int(
            audit.get(
                "sampled_state_count_above_1800n",
                _finite_float(audit["sampled_normal_load_max_n"]) > 1800.0,
            )
        )
    return totals


def _plot_points(
    totals: pd.DataFrame,
    fit: Mapping[str, Any],
    path: Path,
    *,
    highlight_cohort: str | None = None,
) -> None:
    x = totals["rear_static_weight_percent"].to_numpy(dtype=float)
    y = totals["timed_event_points"].to_numpy(dtype=float)
    grid = np.linspace(float(x.min()), float(x.max()), 200)
    slope = float(fit["linear_slope_points_per_rear_weight_percentage_point"])
    intercept = float(fit["linear_intercept_points_at_50_percent_rear"])
    quadratic = np.asarray(
        fit["quadratic_coefficients_centered_at_50_percent"], dtype=float
    )
    fig, axis = plt.subplots(figsize=(9.4, 6.0))
    if highlight_cohort and "source_cohort" in totals:
        highlighted = totals[totals["source_cohort"] == highlight_cohort]
        prior = totals[totals["source_cohort"] != highlight_cohort]
        axis.scatter(
            prior["rear_static_weight_percent"],
            prior["timed_event_points"],
            facecolors="white",
            edgecolors="#6B7280",
            linewidths=2.0,
            s=78,
            zorder=3,
            label="Prior 45/50/55 cases",
        )
        axis.scatter(
            highlighted["rear_static_weight_percent"],
            highlighted["timed_event_points"],
            color="#DC2626",
            s=78,
            zorder=4,
            label="New sweep cases",
        )
    else:
        axis.scatter(x, y, color="#DC2626", s=70, zorder=3, label="6DOF cases")
    axis.plot(grid, intercept + slope * (grid - 50.0), color="#111827", linewidth=2, label="Linear fit")
    axis.plot(
        grid,
        np.polyval(quadratic, grid - 50.0),
        color="#2563EB",
        linestyle="--",
        linewidth=1.7,
        label=f"{len(totals)}-point quadratic diagnostic",
    )
    for row in totals.itertuples(index=False):
        axis.annotate(f"{row.timed_event_points:.1f}", (row.rear_static_weight_percent, row.timed_event_points), xytext=(0, 8), textcoords="offset points", ha="center")
    axis.set_xlabel("Rear static weight (%)")
    axis.set_ylabel("Timed dynamic-event points / 575")
    axis.set_title("Longitudinal CG smoke test: rear weight vs points")
    axis.grid(alpha=0.25)
    axis.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def _plot_event_points(
    events: pd.DataFrame,
    path: Path,
    *,
    highlight_cohort: str | None = None,
) -> None:
    fig, axis = plt.subplots(figsize=(9.4, 6.0))
    for event_slug, color in zip(EXPECTED_EVENTS, ("#2563EB", "#059669", "#D97706", "#7C3AED")):
        group = events[events["event_slug"] == event_slug].sort_values(
            "rear_static_weight_percent"
        )
        axis.plot(
            group["rear_static_weight_percent"],
            group["common_projected_points"],
            linewidth=2,
            color=color,
            label=EVENT_LABELS[event_slug],
        )
        if highlight_cohort and "source_cohort" in group:
            prior = group[group["source_cohort"] != highlight_cohort]
            highlighted = group[group["source_cohort"] == highlight_cohort]
            axis.scatter(
                prior["rear_static_weight_percent"],
                prior["common_projected_points"],
                facecolors="white",
                edgecolors=color,
                s=48,
                linewidths=1.5,
                zorder=3,
            )
            axis.scatter(
                highlighted["rear_static_weight_percent"],
                highlighted["common_projected_points"],
                color=color,
                s=52,
                zorder=4,
            )
        else:
            axis.scatter(
                group["rear_static_weight_percent"],
                group["common_projected_points"],
                color=color,
                s=46,
                zorder=3,
            )
    axis.set_xlabel("Rear static weight (%)")
    axis.set_ylabel("Common-field event points")
    axis.set_title("Per-event contribution to longitudinal-CG sensitivity")
    axis.grid(alpha=0.25)
    axis.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def _plot_event_points_delta_vs_50pct(
    deltas: pd.DataFrame,
    path: Path,
    *,
    highlight_cohort: str | None = None,
) -> None:
    series = (
        ("acceleration", "#2563EB", "o"),
        ("skidpad", "#059669", "s"),
        ("autocross", "#D97706", "^"),
        ("michigan_endurance", "#7C3AED", "D"),
    )
    fig, axis = plt.subplots(figsize=(9.4, 6.0))
    for event_slug, color, marker in series:
        group = deltas[deltas["event_slug"] == event_slug].sort_values(
            "rear_static_weight_percent"
        )
        axis.plot(
            group["rear_static_weight_percent"],
            group["projected_points_delta_vs_50pct_rear"],
            linewidth=2,
            color=color,
            label=EVENT_LABELS[event_slug],
        )
        if highlight_cohort and "source_cohort" in group:
            highlighted = group["source_cohort"] == highlight_cohort
            axis.scatter(
                group.loc[~highlighted, "rear_static_weight_percent"],
                group.loc[
                    ~highlighted, "projected_points_delta_vs_50pct_rear"
                ],
                facecolors="white",
                edgecolors=color,
                marker=marker,
                linewidths=1.5,
                s=50,
                zorder=3,
            )
            axis.scatter(
                group.loc[highlighted, "rear_static_weight_percent"],
                group.loc[
                    highlighted, "projected_points_delta_vs_50pct_rear"
                ],
                color=color,
                marker=marker,
                s=54,
                zorder=4,
            )
        else:
            axis.scatter(
                group["rear_static_weight_percent"],
                group["projected_points_delta_vs_50pct_rear"],
                color=color,
                marker=marker,
                s=50,
                zorder=3,
            )
    axis.axhline(0.0, color="#6B7280", linewidth=1.0)
    axis.axvline(
        POINT_DELTA_BASELINE_REAR_WEIGHT_PERCENT,
        color="#9CA3AF",
        linewidth=1.0,
        linestyle="--",
    )
    axis.set_xlabel("Rear static weight (%)")
    axis.set_ylabel("Projected points delta vs 50% rear (points)")
    axis.set_title("Per-event projected-point change from the 50% rear case")
    axis.grid(alpha=0.25)
    axis.legend(ncol=2)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def _plot_time_change(
    events: pd.DataFrame,
    path: Path,
    *,
    highlight_cohort: str | None = None,
) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(12.0, 8.0), sharex=True)
    for axis, event_slug in zip(axes.flat, EXPECTED_EVENTS):
        group = events[events["event_slug"] == event_slug].sort_values(
            "rear_static_weight_percent"
        )
        baseline = group.iloc[int(np.argmin(np.abs(group["rear_static_weight_percent"].to_numpy(dtype=float) - 50.0)))]["lap_time_s"]
        change = 100.0 * (group["lap_time_s"] / float(baseline) - 1.0)
        axis.plot(group["rear_static_weight_percent"], change, linewidth=2)
        if highlight_cohort and "source_cohort" in group:
            highlighted = group["source_cohort"] == highlight_cohort
            axis.scatter(
                group.loc[~highlighted, "rear_static_weight_percent"],
                change.loc[~highlighted],
                facecolors="white",
                edgecolors="#2563EB",
                linewidths=1.5,
                s=46,
                zorder=3,
            )
            axis.scatter(
                group.loc[highlighted, "rear_static_weight_percent"],
                change.loc[highlighted],
                color="#DC2626",
                s=48,
                zorder=4,
            )
        else:
            axis.scatter(group["rear_static_weight_percent"], change, s=46)
        axis.axhline(0.0, color="#6B7280", linewidth=0.8)
        axis.set_title(EVENT_LABELS[event_slug])
        axis.set_ylabel("Time change vs 50% rear (%)")
        axis.grid(alpha=0.25)
    fig.suptitle("Raw event-time response to longitudinal CG", fontweight="bold")
    fig.supxlabel("Rear static weight (%)")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def _plot_tuning(
    metadata: pd.DataFrame,
    path: Path,
    *,
    highlight_cohort: str | None = None,
) -> None:
    columns = (
        ("front_antiroll_stiffness_fraction", "Front ARB allocation"),
        ("front_brake_bias", "Front brake bias"),
        ("pure_lateral_limit_g", "Tuned lateral limit (g)"),
        ("weighted_pure_braking_limit_g", "Weighted braking limit (g)"),
    )
    fig, axes = plt.subplots(2, 2, figsize=(12.0, 8.0), sharex=True)
    for axis, (column, label) in zip(axes.flat, columns):
        values = pd.to_numeric(metadata[column], errors="coerce")
        if values.notna().any():
            axis.plot(metadata["rear_static_weight_percent"], values, linewidth=2)
            if highlight_cohort and "source_cohort" in metadata:
                highlighted = metadata["source_cohort"] == highlight_cohort
                axis.scatter(
                    metadata.loc[~highlighted, "rear_static_weight_percent"],
                    values.loc[~highlighted],
                    facecolors="white",
                    edgecolors="#2563EB",
                    linewidths=1.5,
                    s=46,
                    zorder=3,
                )
                axis.scatter(
                    metadata.loc[highlighted, "rear_static_weight_percent"],
                    values.loc[highlighted],
                    color="#DC2626",
                    s=48,
                    zorder=4,
                )
            else:
                axis.scatter(metadata["rear_static_weight_percent"], values, s=46)
        else:
            axis.text(0.5, 0.5, "Not recorded", transform=axis.transAxes, ha="center")
        axis.set_ylabel(label)
        axis.grid(alpha=0.25)
    fig.suptitle("Per-CG-position setup retuning", fontweight="bold")
    fig.supxlabel("Rear static weight (%)")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def _plot_fine_resolution_validation(
    ranking: pd.DataFrame,
    event_comparison: pd.DataFrame,
    path: Path,
) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(13.0, 5.5))
    x = ranking["rear_static_weight_percent"].to_numpy(dtype=float)
    coarse_points = ranking["coarse_three_case_total_points"].to_numpy(
        dtype=float
    )
    fine_points = ranking["fine_three_case_total_points"].to_numpy(dtype=float)
    axes[0].plot(x, coarse_points, color="#6B7280", linewidth=2)
    axes[0].scatter(
        x,
        coarse_points,
        facecolors="white",
        edgecolors="#6B7280",
        linewidths=1.8,
        s=70,
        label="Coarse 3-case field",
        zorder=3,
    )
    axes[0].plot(x, fine_points, color="#DC2626", linewidth=2)
    axes[0].scatter(
        x,
        fine_points,
        color="#DC2626",
        s=70,
        label="Fine 3-case field",
        zorder=4,
    )
    for rear, coarse, fine in zip(x, coarse_points, fine_points):
        axes[0].annotate(
            f"{coarse:.2f}",
            (rear, coarse),
            xytext=(0, -16),
            textcoords="offset points",
            ha="center",
            color="#4B5563",
        )
        axes[0].annotate(
            f"{fine:.2f}",
            (rear, fine),
            xytext=(0, 9),
            textcoords="offset points",
            ha="center",
            color="#991B1B",
        )
    axes[0].set_xlabel("Rear static weight (%)")
    axes[0].set_ylabel("Timed event points / 575")
    axes[0].set_title("Relative points ranking")
    axes[0].set_ylim(
        float(min(coarse_points.min(), fine_points.min()) - 0.5),
        float(max(coarse_points.max(), fine_points.max()) + 0.4),
    )
    axes[0].grid(alpha=0.25)
    axes[0].legend()

    colors = ("#2563EB", "#059669", "#D97706", "#7C3AED")
    for event_slug, color in zip(EXPECTED_EVENTS, colors):
        group = event_comparison[
            event_comparison["event_slug"] == event_slug
        ].sort_values("rear_static_weight_percent")
        delta_ms = 1000.0 * group["fine_minus_coarse_time_s"]
        axes[1].plot(
            group["rear_static_weight_percent"],
            delta_ms,
            marker="o",
            linewidth=2,
            color=color,
            label=EVENT_LABELS[event_slug],
        )
        axes[1].annotate(
            EVENT_LABELS[event_slug],
            (
                float(group["rear_static_weight_percent"].iloc[-1]),
                float(delta_ms.iloc[-1]),
            ),
            xytext=(7, 0),
            textcoords="offset points",
            color=color,
            va="center",
            fontsize=9,
        )
    axes[1].axhline(0.0, color="#6B7280", linewidth=0.9)
    axes[1].set_xlabel("Rear static weight (%)")
    axes[1].set_ylabel("Fine minus coarse raw time (ms)")
    axes[1].set_title("Raw-time resolution shift")
    axes[1].set_xlim(float(x.min() - 0.15), float(x.max() + 0.55))
    axes[1].grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def _markdown_table(headers: Sequence[str], rows: Sequence[Sequence[object]]) -> str:
    def render(value: object) -> str:
        if isinstance(value, float):
            if not math.isfinite(value):
                return "—"
            return f"{value:.4f}"
        return str(value)

    header = "| " + " | ".join(headers) + " |"
    divider = "| " + " | ".join("---" for _ in headers) + " |"
    body = ["| " + " | ".join(render(value) for value in row) + " |" for row in rows]
    return "\n".join([header, divider, *body])


def _write_report(
    path: Path,
    *,
    study_root: Path,
    scoring_reference_path: Path,
    tuning_path: str,
    tuning_context: Mapping[str, Any],
    totals: pd.DataFrame,
    event_sensitivity: pd.DataFrame,
    event_points_delta: pd.DataFrame,
    fit: Mapping[str, Any],
    design: Mapping[str, Any],
    event_scoring: Mapping[str, Mapping[str, Any]],
    audit_summary: Mapping[str, Any],
) -> None:
    best = totals.loc[totals["timed_event_points"].idxmax()]
    worst = totals.loc[totals["timed_event_points"].idxmin()]
    raw_rows = [
        (
            row.rear_static_weight_percent,
            row.longitudinal_cg_x_m,
            row.acceleration_time_s,
            row.skidpad_time_s,
            row.autocross_time_s,
            row.michigan_endurance_time_s,
        )
        for row in totals.itertuples(index=False)
    ]
    point_rows = [
        (
            row.rear_static_weight_percent,
            row.acceleration_points,
            row.skidpad_points,
            row.autocross_points,
            row.michigan_endurance_points,
            row.timed_event_points,
        )
        for row in totals.itertuples(index=False)
    ]
    event_rows = [
        (
            row.event_name,
            row.time_change_55_vs_45_percent,
            row.points_change_55_minus_45,
            row.linear_points_per_rear_weight_percentage_point,
            row.share_of_total_endpoint_change_percent,
        )
        for row in event_sensitivity.itertuples(index=False)
    ]
    tuning_rows = [
        (
            row.rear_static_weight_percent,
            row.front_antiroll_stiffness_fraction,
            row.front_elastic_roll_stiffness_fraction,
            row.front_brake_bias,
            row.pure_lateral_limit_g,
            row.weighted_pure_braking_limit_g,
        )
        for row in totals.itertuples(index=False)
    ]
    validity_rows = [
        (
            row.rear_static_weight_percent,
            row.wheel_load_min_n,
            row.wheel_load_max_n,
            row.wheel_load_outside_tire_validity,
            row.ggv_speed_step_mps,
            row.ggv_ay_points,
            row.all_track_solvers_converged,
            row.total_ggv_speed_cap_segments,
        )
        for row in totals.itertuples(index=False)
    ]
    qss_rows = [
        (
            row.rear_static_weight_percent,
            getattr(row, "sampled_qss_successful_trim_count", math.nan),
            getattr(row, "sampled_qss_sample_count", math.nan),
            getattr(row, "sampled_qss_normal_load_min_n", math.nan),
            getattr(row, "sampled_qss_normal_load_max_n", math.nan),
            getattr(row, "sampled_qss_state_count_below_tir", math.nan),
            getattr(row, "sampled_qss_state_count_above_tir", math.nan),
            getattr(row, "sampled_qss_wheel_lift_state_count", math.nan),
        )
        for row in totals.itertuples(index=False)
    ]
    scoring_rows = [
        (
            EVENT_LABELS[event_slug],
            summary["real_field_fastest_time_s"],
            summary["simulated_fastest_time_s"],
            summary["common_field_tmin_s"],
            summary["common_field_tmin_source"],
        )
        for event_slug, summary in event_scoring.items()
    ]
    delta_baseline_rows = [
        (
            EVENT_LABELS[event_slug],
            str(row["baseline_cg_case"]),
            f"{float(row['baseline_projected_points']):.9f}",
            f"{float(row['common_field_tmin_s']):.9f}",
        )
        for event_slug in EXPECTED_EVENTS
        for _, row in event_points_delta[
            (event_points_delta["event_slug"] == event_slug)
            & event_points_delta["is_50pct_baseline_case"].map(_as_bool)
        ].iterrows()
    ]
    endpoint_change = float(fit["endpoint_change_points_55_minus_45"])
    slope = float(fit["linear_slope_points_per_rear_weight_percentage_point"])
    hypothesis = (
        "supports"
        if endpoint_change > 0.0
        else "does not support" if endpoint_change < 0.0 else "is neutral on"
    )
    vertex = float(fit["quadratic_vertex_rear_weight_percent"])
    if bool(fit["quadratic_vertex_is_local_maximum"]):
        vertex_text = (
            f"The exact three-point quadratic has an in-range maximum at "
            f"{vertex:.3f}% rear. This is only an interpolation diagnostic; three "
            "smoke points are insufficient to establish a physical optimum."
        )
    else:
        vertex_text = (
            "The three-point quadratic does not establish an in-range maximum. "
            "Do not extrapolate an optimum from this smoke test."
        )
    audit_text = (
        f"The sampled QSS audit reconstructed {audit_summary['successful_trim_count']} "
        f"states successfully and {audit_summary['failed_trim_count']} unsuccessfully; "
        f"it found {audit_summary['sampled_wheel_lift_state_count']} sampled wheel-lift "
        "states."
        if audit_summary.get("present")
        else "No reduced-QSS sampled-audit bundle was present at report-build time."
    )
    constrained_tuning = totals[
        totals["tuned_lateral_active_constraints"].astype(str).str.len() > 0
    ]
    constraint_text = (
        "; ".join(
            f"{row.rear_static_weight_percent:.1f}% rear: "
            f"{row.tuned_lateral_active_constraints}, "
            f"minimum Fz {row.tuned_lateral_minimum_normal_load_n:.2f} N"
            for row in constrained_tuning.itertuples(index=False)
        )
        if not constrained_tuning.empty
        else "none"
    )
    report = f"""# Longitudinal-CG to dynamic-event points: three-point smoke test

This fixed-mass, fixed-height study moves total static weight distribution from
45% to 55% rear at **{design['cg_height_in']:.3f} in CG height** and
**{design['total_mass_kg']:.3f} kg total mass**. Each CG position has its own
front ARB allocation and fixed front brake bias. The model is scaled-mu reduced
6DOF, rear-wheel drive, 50/50 aero, and no LSD.

The inherited prior-study **11.5-in** convention sets the model's absolute CG
z-coordinate to **{MODEL_ABSOLUTE_CG_Z_M:.6f} m**. Relative to the modeled
contact plane at z = **{MODEL_CONTACT_PLANE_Z_M:.6f} m**, the physical CG height
is **{MODEL_CG_HEIGHT_ABOVE_CONTACT_PLANE_M:.6f} m = {MODEL_CG_HEIGHT_ABOVE_CONTACT_PLANE_M / M_PER_INCH:.3f} in**.
The source vehicle's nominal static rear fraction is
**{100.0 * SOURCE_NOMINAL_REAR_STATIC_WEIGHT_FRACTION:.5f}%**; therefore, 50%
rear is the requested sweep midpoint, not the exact nominal vehicle balance.

The best observed case is **{best.rear_static_weight_percent:.1f}% rear** at
**{best.timed_event_points:.3f} timed-event points**. The linear screening
sensitivity is **{slope:+.4f} points per rear-weight percentage point** and the
55%-minus-45% endpoint change is **{endpoint_change:+.3f} points**. Thus this
three-point result **{hypothesis}** the hypothesis that some rearward bias helps
this RWD car over the sampled interval. The worst observed case is
{worst.rear_static_weight_percent:.1f}% rear at {worst.timed_event_points:.3f}
points.

This is explicitly a **smoke-resolution result**, not a production-resolution
optimum search. {vertex_text} Do not extrapolate beyond 45-55% rear from these
three points.

![Rear weight to points](rear_weight_to_points.png)

## Raw simulated event times

Endurance values below are modeled lap times; the official-distance multiplier
is applied only when scoring.

{_markdown_table(("Rear wt. (%)", "CG x (m)", "Accel (s)", "Skidpad (s)", "Autocross (s)", "Endurance lap (s)"), raw_rows)}

## Common-field dynamic-event points

All three simulations are scored together with the valid 2026 Michigan EV
field. Each event uses the single fastest real-or-simulated competition time as
Tmin. Only simulations with a time exactly equal to that Tmin receive maximum
points; near-identical times remain below maximum.

{_markdown_table(("Rear wt. (%)", "Accel", "Skidpad", "Autocross", "Endurance", "Total / 575"), point_rows)}

## Event contribution and the RWD hypothesis

Positive point changes mean the 55%-rear case beats the 45%-rear case. A share
can be negative or exceed 100% when one event offsets another, so the signed
point delta is the primary quantity.

{_markdown_table(("Event", "Time 55 vs 45 (%)", "Points 55-45", "Point slope / pp", "Share of net (%)"), event_rows)}

![Event points](event_points_vs_rear_weight.png)

### Projected-point delta from the exact 50% rear case

For each event `e` and sampled rear fraction `r`, the plotted quantity is
`Delta P_e(r) = P_e(r) - P_e(50%)`. Both terms are projected event points from
the same real-plus-simulation common scoring field and therefore share the same
event-specific Tmin. The baseline is the simulated **exact 50% rear case**, not
an interpolation and not the source vehicle's 51.65037% nominal balance. Every
event is zero at 50% by construction; positive values indicate more projected
points than the 50% case.

{_markdown_table(("Event", "50% case", "50% baseline points", "Common Tmin (s)"), delta_baseline_rows)}

![Projected event-point delta from 50% rear](event_points_delta_vs_50pct_rear_weight.png)

The exact plotted values are in
[`event_points_delta_vs_50pct_rear.csv`](event_points_delta_vs_50pct_rear.csv).

![Raw time change](raw_event_time_percent_change.png)

## Correlation and optimum interpretation

- Linear fit: {slope:+.6f} pt per rear-weight percentage point.
- Pearson r: {fit['pearson_r']:+.6f}; linear R²: {fit['linear_r_squared']:.6f}.
- Adjacent slopes: {', '.join(f'{value:+.6f}' for value in fit['adjacent_slopes_points_per_percentage_point'])} pt/percentage point.
- Endpoint 55%-minus-45%: {endpoint_change:+.6f} points.
- The quadratic passes exactly through three points and is recorded only to
  propose the next denser sweep; it is not evidence of a validated optimum.

## Per-case retuning

The front elastic roll fraction is an LLTD proxy, not a directly imposed axle
load-transfer percentage. Brake bias is one fixed front fraction per case; an
ideal speed/deceleration-varying schedule is not modeled.

Tuned pure-lateral endpoint constraints: {constraint_text}.

{_markdown_table(("Rear wt. (%)", "Front ARB", "Front elastic roll", "Front brake", "Lateral (g)", "Braking (g)"), tuning_rows)}

![Tuning](tuning_vs_rear_weight.png)

## Numerical and tire-load validity

Serialized GGV envelope extrema (TIR load range: 100-1800 N):

{_markdown_table(("Rear wt. (%)", "Min Fz (N)", "Max Fz (N)", "Outside TIR", "GGV dV (m/s)", "Ay points", "Track converged", "Speed caps"), validity_rows)}

{audit_text}

Event-relevant sampled 6DOF trim reconstruction:

{_markdown_table(("Rear wt. (%)", "Successful", "Sampled", "Min Fz (N)", "Max Fz (N)", "States <100 N", "States >1800 N", "Wheel-lift states"), qss_rows)}

The stored GGV load extrema and the sampled QSS reconstruction answer different
questions: the former covers serialized envelope states, while the latter checks
selected event-relevant trim reconstructions. Both should pass before treating a
dense follow-up as final.

## Common scoring field

{_markdown_table(("Event", "Real fastest (s)", "Sim fastest (s)", "Common Tmin (s)", "Tmin source"), scoring_rows)}

## Provenance

- Source study: `{study_root.resolve()}`
- Scoring reference: `{scoring_reference_path.resolve()}`
- Tuning artifact: `{tuning_path or 'not supplied'}`
- Tuning sweep axis: `{tuning_context.get('sweep_axis', '')}`
- Permanent vehicle and tire model files are not modified by this report builder.
"""
    path.write_text(report, encoding="utf-8")


def _quadratic_diagnostic_text(fit: Mapping[str, Any], label: str) -> str:
    vertex = float(fit["quadratic_vertex_rear_weight_percent"])
    predicted = float(fit["quadratic_vertex_predicted_points"])
    if bool(fit["quadratic_vertex_is_local_maximum"]):
        location = "inside"
        nature = "a local maximum"
    elif bool(fit["quadratic_vertex_is_in_sample_range"]):
        location = "inside"
        nature = "a local minimum, not an optimum"
    else:
        location = "outside"
        nature = "an extrapolated stationary point"
    return (
        f"{label}: the quadratic vertex is {vertex:.3f}% rear and "
        f"{predicted:.3f} points, {location} the sampled interval and {nature}. "
        "This is a quadratic diagnostic only, not a validated physical optimum."
    )


def _fine_resolution_report_section(fine_validation: Mapping[str, Any]) -> str:
    validation = fine_validation["validation"]
    fine_result = fine_validation["fine_result"]
    coarse_result = fine_validation["coarse_selected_result"]
    ranking = fine_validation["ranking"]
    event_comparison = fine_validation["event_comparison"]
    fine_totals = fine_result["totals"]
    fine_point_rows = [
        (
            row.rear_static_weight_percent,
            row.acceleration_points,
            row.skidpad_points,
            row.autocross_points,
            row.michigan_endurance_points,
            row.timed_event_points,
        )
        for row in fine_totals.itertuples(index=False)
    ]
    ranking_rows = [
        (
            row.rear_static_weight_percent,
            row.coarse_three_case_total_points,
            row.coarse_rank,
            row.fine_three_case_total_points,
            row.fine_rank,
            row.fine_minus_coarse_total_points,
        )
        for row in ranking.itertuples(index=False)
    ]
    time_rows = [
        (
            row.rear_static_weight_percent,
            EVENT_LABELS[row.event_slug],
            row.coarse_time_s,
            row.fine_time_s,
            1000.0 * row.fine_minus_coarse_time_s,
            row.fine_minus_coarse_time_percent,
        )
        for row in event_comparison.itertuples(index=False)
    ]
    scoring_rows = [
        (
            EVENT_LABELS[event_slug],
            summary["real_field_fastest_time_s"],
            summary["simulated_fastest_time_s"],
            summary["common_field_tmin_s"],
            summary["common_field_tmin_source"],
        )
        for event_slug, summary in fine_result["event_scoring"].items()
    ]
    coarse_optima = coarse_result["event_optima"].set_index("event_slug")
    fine_optima = fine_result["event_optima"].set_index("event_slug")
    optimum_rows = [
        (
            EVENT_LABELS[event_slug],
            coarse_optima.loc[event_slug, "fastest_rear_weight_percents"],
            fine_optima.loc[event_slug, "fastest_rear_weight_percents"],
        )
        for event_slug in EXPECTED_EVENTS
    ]
    load_rows = [
        (
            row["rear_static_weight_percent"],
            row["wheel_load_min_n"],
            row["wheel_load_max_n"],
            row["tire_valid_load_min_n"],
            row["tire_valid_load_max_n"],
            row["outside_tire_validity"],
        )
        for row in validation["legacy_serialized_load_diagnostics"]
    ]
    fine_grid = validation["fine_grid"]
    coarse_grid = validation["coarse_grid"]
    ranking_statement = (
        "The **58% rear case remains the best observed configuration** after "
        "fine-grid rerunning."
        if validation["rear_58_remains_best"]
        else "The 58% rear ranking does **not** survive the fine-grid rerun."
    )
    fine_ranked = ranking.sort_values(
        "fine_three_case_total_points", ascending=False
    ).reset_index(drop=True)
    fine_winner_margin = float(
        fine_ranked.loc[0, "fine_three_case_total_points"]
        - fine_ranked.loc[1, "fine_three_case_total_points"]
    )
    qss_audit_statement = (
        "A fine-grid reduced-QSS audit bundle is present."
        if validation["fine_reduced_qss_audit_present"]
        else "No fine-grid reduced-QSS sampled-trim audit bundle is present."
    )
    return f"""
## Fine-grid resolution validation: 55%, 56%, and 58% rear

This validation does **not** replace or rescore the main coarse eight-case field
above. It reruns only the three candidate cases and scores those three fine
simulations together with the real 2026 field. The comparison coarse totals are
also rescored as their own three-case field so the relative rankings use the
same cohort size at each resolution.

The coarse grid is {coarse_grid['nominal_speed_step_mps']:.0f} m/s speed spacing,
{coarse_grid['ay_points']} ay points, and
{coarse_grid['ax_binary_iterations']} longitudinal binary iterations. The fine
grid is {fine_grid['nominal_speed_step_mps']:.0f} m/s,
{fine_grid['ay_points']} ay points, and
{fine_grid['ax_binary_iterations']} iterations. Both use
{fine_grid['ax_search_points']} longitudinal search points. The configured tire
mu scale is **{validation['tire_mu_scale']:.16f}** in every coarse and fine case;
the serialized JSON round-trip is
{validation['serialized_tire_mu_scale']:.16f}. All controlled vehicle and
per-case tuning inputs match exactly.

{ranking_statement} The coarse three-case ranking is led by
{validation['coarse_three_case_best_rear_static_weight_percent']:.0f}% rear; the
fine three-case ranking is led by
{validation['fine_three_case_best_rear_static_weight_percent']:.0f}% rear, by
{fine_winner_margin:.4f} points over the runner-up.

### Fine-field event points

{_markdown_table(("Rear wt. (%)", "Accel", "Skidpad", "Autocross", "Endurance", "Total / 575"), fine_point_rows)}

### Coarse-versus-fine relative ranking

Point changes between these columns are resolution diagnostics, not additions
to the main eight-case score, because each column has its own simulation Tmin.

{_markdown_table(("Rear wt. (%)", "Coarse 3-case total", "Coarse rank", "Fine 3-case total", "Fine rank", "Fine - coarse pts"), ranking_rows)}

The total order remains 58%, 56%, 55%, although the event-level raw-time
winners do change:

{_markdown_table(("Event", "Coarse winner rear (%)", "Fine winner rear (%)"), optimum_rows)}

![Fine-grid resolution validation](fine_resolution_validation.png)

### Raw event-time convergence

{_markdown_table(("Rear wt. (%)", "Event", "Coarse time (s)", "Fine time (s)", "Fine - coarse (ms)", "Change (%)"), time_rows)}

### Fine-field scoring minima

{_markdown_table(("Event", "Real fastest (s)", "Fine-sim fastest (s)", "Fine-field Tmin (s)", "Tmin source"), scoring_rows)}

### Fine validation limitations

The following extrema are the serialized **legacy algebraic** load diagnostic,
not a reduced-QSS sampled-trim audit:

{_markdown_table(("Rear wt. (%)", "Min Fz (N)", "Max Fz (N)", "Tire min (N)", "Tire max (N)", "Outside fit"), load_rows)}

{qss_audit_statement} The requested terminal speed slice is empty for
{', '.join(validation['requested_terminal_slice_empty_cases']) or 'no cases'},
but no fine event used the speed cap.

All fine track solvers converged:
{validation['all_track_solvers_converged']}; total fine speed-cap segments:
{validation['total_ggv_speed_cap_segments']}.

- Fine validation source: `{validation['fine_validation_root']}`
"""


def _write_grid_comparison_report(
    path: Path,
    *,
    primary_root: Path,
    comparison_root: Path,
    scoring_reference_path: Path,
    primary_tuning_path: str,
    comparison_tuning_path: str,
    primary_result: Mapping[str, Any],
    combined_result: Mapping[str, Any],
    combined_metadata: pd.DataFrame,
    primary_design: Mapping[str, Any],
    comparison_design: Mapping[str, Any],
    primary_audit: Mapping[str, Any],
    comparison_audit: Mapping[str, Any],
    expected_mu_scale: float,
    fine_validation: Mapping[str, Any] | None = None,
) -> None:
    primary_totals = primary_result["totals"]
    combined_totals = combined_result["totals"]
    primary_fit = primary_result["fit"]
    combined_fit = combined_result["fit"]
    primary_sensitivity = primary_result["event_sensitivity"]
    combined_sensitivity = combined_result["event_sensitivity"]
    combined_delta = combined_result["event_points_delta_vs_50pct"]
    primary_optima = primary_result["event_optima"].set_index("event_slug")
    combined_optima = combined_result["event_optima"].set_index("event_slug")
    primary_best = primary_totals.loc[
        primary_totals["timed_event_points"].idxmax()
    ]
    combined_best = combined_totals.loc[
        combined_totals["timed_event_points"].idxmax()
    ]
    primary_min = float(primary_fit["rear_weight_min_percent"])
    primary_max = float(primary_fit["rear_weight_max_percent"])
    combined_min = float(combined_fit["rear_weight_min_percent"])
    combined_max = float(combined_fit["rear_weight_max_percent"])

    def cohort_name(value: object) -> str:
        return "New" if str(value) == "new_sweep" else "Prior"

    raw_rows = [
        (
            cohort_name(row.source_cohort),
            row.rear_static_weight_percent,
            row.longitudinal_cg_x_m,
            row.acceleration_time_s,
            row.skidpad_time_s,
            row.autocross_time_s,
            row.michigan_endurance_time_s,
        )
        for row in combined_totals.itertuples(index=False)
    ]
    primary_point_rows = [
        (
            row.rear_static_weight_percent,
            row.acceleration_points,
            row.skidpad_points,
            row.autocross_points,
            row.michigan_endurance_points,
            row.timed_event_points,
        )
        for row in primary_totals.itertuples(index=False)
    ]
    combined_point_rows = [
        (
            cohort_name(row.source_cohort),
            row.rear_static_weight_percent,
            row.acceleration_points,
            row.skidpad_points,
            row.autocross_points,
            row.michigan_endurance_points,
            row.timed_event_points,
        )
        for row in combined_totals.itertuples(index=False)
    ]

    def sensitivity_rows(frame: pd.DataFrame) -> list[tuple[object, ...]]:
        return [
            (
                row.event_name,
                row.time_endpoint_change_percent,
                row.points_endpoint_change,
                row.linear_points_per_rear_weight_percentage_point,
                row.share_of_total_endpoint_change_percent,
            )
            for row in frame.itertuples(index=False)
        ]

    tuning_rows = [
        (
            cohort_name(row.source_cohort),
            row.rear_static_weight_percent,
            row.front_antiroll_stiffness_fraction,
            row.front_elastic_roll_stiffness_fraction,
            row.front_brake_bias,
            row.pure_lateral_limit_g,
            row.weighted_pure_braking_limit_g,
            row.tuned_lateral_active_constraints or "none",
            row.tuned_lateral_minimum_normal_load_n,
        )
        for row in combined_totals.itertuples(index=False)
    ]
    qss_rows = [
        (
            cohort_name(row.source_cohort),
            row.rear_static_weight_percent,
            getattr(row, "sampled_qss_successful_trim_count", math.nan),
            getattr(row, "sampled_qss_sample_count", math.nan),
            getattr(row, "sampled_qss_normal_load_min_n", math.nan),
            getattr(row, "sampled_qss_normal_load_max_n", math.nan),
            getattr(row, "sampled_qss_state_count_below_tir", math.nan),
            getattr(row, "sampled_qss_state_count_above_tir", math.nan),
            getattr(row, "sampled_qss_wheel_lift_state_count", math.nan),
        )
        for row in combined_totals.itertuples(index=False)
    ]
    scoring_rows = [
        (
            EVENT_LABELS[event_slug],
            summary["real_field_fastest_time_s"],
            summary["simulated_fastest_time_s"],
            summary["common_field_tmin_s"],
            summary["common_field_tmin_source"],
        )
        for event_slug, summary in combined_result["event_scoring"].items()
    ]
    optimum_rows = [
        (
            EVENT_LABELS[event_slug],
            primary_optima.loc[event_slug, "fastest_rear_weight_percents"],
            float(primary_optima.loc[event_slug, "fastest_raw_time_s"]),
            combined_optima.loc[event_slug, "fastest_rear_weight_percents"],
            float(combined_optima.loc[event_slug, "fastest_raw_time_s"]),
        )
        for event_slug in EXPECTED_EVENTS
    ]
    delta_baseline_rows = [
        (
            EVENT_LABELS[event_slug],
            str(row["baseline_cg_case"]),
            cohort_name(row["source_cohort"]),
            f"{float(row['baseline_projected_points']):.9f}",
            f"{float(row['common_field_tmin_s']):.9f}",
        )
        for event_slug in EXPECTED_EVENTS
        for _, row in combined_delta[
            (combined_delta["event_slug"] == event_slug)
            & combined_delta["is_50pct_baseline_case"].map(_as_bool)
        ].iterrows()
    ]
    primary_grid = ", ".join(
        f"{value:g}%" for value in primary_design["rear_static_weight_percents"]
    )
    comparison_grid = ", ".join(
        f"{value:g}%" for value in comparison_design["rear_static_weight_percents"]
    )
    observed_mu_scale = float(combined_totals["tire_mu_scale"].iloc[0])
    if not math.isclose(observed_mu_scale, expected_mu_scale, abs_tol=1e-9):
        raise ValueError(
            "Report mu scale does not match the configured validation value."
        )
    fine_section = (
        _fine_resolution_report_section(fine_validation)
        if fine_validation is not None
        else ""
    )
    report = f"""# Longitudinal-CG points: five-case sweep and eight-case common field

## Result

The new **{primary_grid} rear** sweep is compared with the prior
**{comparison_grid} rear** smoke cases. All eight simulations retain
{primary_design['cg_height_in']:.3f}-in model-coordinate CG height,
{primary_design['total_mass_kg']:.3f} kg total mass, scaled-mu reduced 6DOF,
RWD, 50/50 aero, and no LSD. Each case is independently retuned for front ARB
allocation and one fixed front brake bias. The configured tire mu scale is
**{expected_mu_scale:.16f}**.

Within the five new cases alone, the best observed configuration is
**{primary_best.rear_static_weight_percent:.1f}% rear** at
**{primary_best.timed_event_points:.3f} points**. After rescoring all eight
simulations together with the 2026 real field, the best observed configuration
is **{combined_best.rear_static_weight_percent:.1f}% rear** at
**{combined_best.timed_event_points:.3f} points**.

The new-sweep OLS sensitivity is
**{primary_fit['linear_slope_points_per_rear_weight_percentage_point']:+.4f}
pt/rear-weight percentage point** (R²={primary_fit['linear_r_squared']:.4f}).
Across the common eight-case field it is
**{combined_fit['linear_slope_points_per_rear_weight_percentage_point']:+.4f}
pt/percentage point** (R²={combined_fit['linear_r_squared']:.4f}).

This remains a **smoke-resolution study**. The five new cases use the same
screening GGV discretization as the prior three; fit quality does not turn the
screening grid into production-resolution validation.

![Eight-case rear weight to points](rear_weight_to_points.png)

## Height and nominal-balance conventions

The inherited prior-study **11.5-in** convention sets absolute model CG z to
**{MODEL_ABSOLUTE_CG_Z_M:.6f} m**. Relative to the modeled contact plane at
z={MODEL_CONTACT_PLANE_Z_M:.6f} m, height above the contact plane is
**{MODEL_CG_HEIGHT_ABOVE_CONTACT_PLANE_M:.6f} m = {MODEL_CG_HEIGHT_ABOVE_CONTACT_PLANE_M / M_PER_INCH:.3f} in**.
The source vehicle nominal static rear fraction is
**{100.0 * SOURCE_NOMINAL_REAR_STATIC_WEIGHT_FRACTION:.5f}%**. Thus 50% is the
requested earlier midpoint, not exact nominal.

## Raw simulated event times: all eight cases

Endurance entries are modeled lap times; official-distance conversion is
applied only for scoring.

{_markdown_table(("Cohort", "Rear wt. (%)", "CG x (m)", "Accel (s)", "Skidpad (s)", "Autocross (s)", "Endurance lap (s)"), raw_rows)}

## Five-case points in the new-sweep scoring field

These values use the 2026 real field plus only the five new simulations.

{_markdown_table(("Rear wt. (%)", "Accel", "Skidpad", "Autocross", "Endurance", "Total / 575"), primary_point_rows)}

## Eight-case common-field points

These are the final comparison values. The 2026 real results and all eight
simulations share one Tmin per event. No case is deduplicated. Only a simulation
whose competition time exactly equals common Tmin receives maximum points.

{_markdown_table(("Cohort", "Rear wt. (%)", "Accel", "Skidpad", "Autocross", "Endurance", "Total / 575"), combined_point_rows)}

{fine_section}

## Per-event fastest observed configurations

The new-only optimum is selected from 52-60%; the common optimum is selected
from all eight simulated cases. These are discrete observed minima, not fitted
optima.

{_markdown_table(("Event", "New best rear (%)", "New best time (s)", "All-8 best rear (%)", "All-8 best time (s)"), optimum_rows)}

## Per-event endpoint changes

New sweep ({primary_min:g}% to {primary_max:g}% rear):

{_markdown_table(("Event", "Time change (%)", "Point change", "Point slope / pp", "Share of net (%)"), sensitivity_rows(primary_sensitivity))}

Common eight-case field ({combined_min:g}% to {combined_max:g}% rear):

{_markdown_table(("Event", "Time change (%)", "Point change", "Point slope / pp", "Share of net (%)"), sensitivity_rows(combined_sensitivity))}

![Eight-case event points](event_points_vs_rear_weight.png)

### Projected-point delta from the exact 50% rear case

For each event `e` and sampled rear fraction `r`, the plotted quantity is
`Delta P_e(r) = P_e(r) - P_e(50%)`. Both terms use the **final common eight-case
scoring field**: the 2026 real field plus all eight simulations share the same
event-specific Tmin. The baseline is the simulated exact 50% rear case from the
prior cohort, not an interpolation and not the source vehicle's 51.65037%
nominal balance. Every event is zero at 50% by construction; positive values
mean that case earns more projected points than the 50% case.

{_markdown_table(("Event", "50% case", "Cohort", "50% baseline points", "Common Tmin (s)"), delta_baseline_rows)}

![Projected event-point delta from 50% rear](event_points_delta_vs_50pct_rear_weight.png)

The exact plotted values are in
[`common_8case_event_points_delta_vs_50pct_rear.csv`](common_8case_event_points_delta_vs_50pct_rear.csv).

![Eight-case raw time response](raw_event_time_percent_change.png)

## OLS, curvature, and optimum diagnostics

- New five-case OLS: {primary_fit['linear_slope_points_per_rear_weight_percentage_point']:+.6f} pt/pp; Pearson r={primary_fit['pearson_r']:+.6f}; R²={primary_fit['linear_r_squared']:.6f}; endpoint change={primary_fit['endpoint_change_points_max_minus_min']:+.6f} points.
- Common eight-case OLS: {combined_fit['linear_slope_points_per_rear_weight_percentage_point']:+.6f} pt/pp; Pearson r={combined_fit['pearson_r']:+.6f}; R²={combined_fit['linear_r_squared']:.6f}; endpoint change={combined_fit['endpoint_change_points_max_minus_min']:+.6f} points.
- New adjacent slopes: {', '.join(f'{value:+.6f}' for value in primary_fit['adjacent_slopes_points_per_percentage_point'])} pt/pp.
- Common adjacent slopes: {', '.join(f'{value:+.6f}' for value in combined_fit['adjacent_slopes_points_per_percentage_point'])} pt/pp.
- {_quadratic_diagnostic_text(primary_fit, 'New five-case diagnostic')}
- {_quadratic_diagnostic_text(combined_fit, 'Common eight-case diagnostic')}

The best observed discrete case and a fitted quadratic vertex are different
claims. Only the best observed case is directly supported by a simulation; any
quadratic optimum requires denser confirmation around the candidate region.

## Per-case tuning

{_markdown_table(("Cohort", "Rear wt. (%)", "Front ARB", "Front elastic roll", "Front brake", "Lateral (g)", "Braking (g)", "Lateral constraint", "Tuned min Fz (N)"), tuning_rows)}

![Eight-case tuning](tuning_vs_rear_weight.png)

## QSS and tire-load validation

{_markdown_table(("Cohort", "Rear wt. (%)", "Successful", "Sampled", "Min Fz (N)", "Max Fz (N)", "States <100 N", "States >1800 N", "Wheel lift"), qss_rows)}

- New-sweep audit: {primary_audit.get('successful_trim_count', 0)} successful,
  {primary_audit.get('failed_trim_count', 0)} failed reconstructed trims.
- Prior-sweep audit: {comparison_audit.get('successful_trim_count', 0)} successful,
  {comparison_audit.get('failed_trim_count', 0)} failed reconstructed trims.
- All event solvers converged:
  {bool(combined_totals['all_track_solvers_converged'].all())}; total speed-cap
  segments: {int(combined_totals['total_ggv_speed_cap_segments'].sum())}.

## Eight-case common scoring field

{_markdown_table(("Event", "Real fastest (s)", "Sim fastest (s)", "Common Tmin (s)", "Tmin source"), scoring_rows)}

## Provenance

- New source study: `{primary_root.resolve()}`
- Prior source study: `{comparison_root.resolve()}`
- New tuning artifact: `{primary_tuning_path}`
- Prior tuning artifact: `{comparison_tuning_path}`
- Scoring reference: `{scoring_reference_path.resolve()}`
- All source vehicle/tire files remain unmodified by this report builder.
"""
    path.write_text(report, encoding="utf-8")


def build_report_bundle(
    *,
    study_root: Path,
    output_root: Path,
    tuning_json_path: Path | None = None,
    comparison_study_root: Path | None = None,
    comparison_tuning_json_path: Path | None = None,
    fine_validation_root: Path | None = None,
    scoring_reference_path: Path = DEFAULT_SCORING_REFERENCE,
    expected_rear_fractions: Sequence[float] = DEFAULT_REAR_FRACTIONS,
    comparison_expected_rear_fractions: Sequence[float] = (
        DEFAULT_COMPARISON_REAR_FRACTIONS
    ),
    fine_validation_expected_rear_fractions: Sequence[float] = (
        DEFAULT_FINE_VALIDATION_REAR_FRACTIONS
    ),
    expected_cg_height_in: float = DEFAULT_CG_HEIGHT_IN,
    expected_mu_scale: float = DEFAULT_MU_SCALE,
    fraction_tolerance: float = 1e-9,
) -> dict[str, Any]:
    study_root = study_root.resolve()
    output_root = output_root.resolve()
    scoring_reference_path = scoring_reference_path.resolve()
    if fine_validation_root is not None and comparison_study_root is None:
        raise ValueError(
            "Fine resolution validation requires a comparison study so the "
            "matching coarse cases are available."
        )
    tuning_cases, tuning_path, tuning_context = _load_tuning_cases(tuning_json_path)
    events, metadata, design = _load_study(
        study_root,
        tuning_cases,
        cohort_label=("new_sweep" if comparison_study_root is not None else "primary_sweep"),
        expected_rear_fractions=expected_rear_fractions,
        expected_cg_height_in=expected_cg_height_in,
        expected_mu_scale=expected_mu_scale,
        fraction_tolerance=fraction_tolerance,
    )
    if comparison_study_root is not None:
        comparison_study_root = comparison_study_root.resolve()
        (
            comparison_tuning_cases,
            comparison_tuning_path,
            _comparison_tuning_context,
        ) = _load_tuning_cases(comparison_tuning_json_path)
        comparison_events, comparison_metadata, comparison_design = _load_study(
            comparison_study_root,
            comparison_tuning_cases,
            cohort_label="prior_sweep",
            expected_rear_fractions=comparison_expected_rear_fractions,
            expected_cg_height_in=expected_cg_height_in,
            expected_mu_scale=expected_mu_scale,
            fraction_tolerance=fraction_tolerance,
        )
        cross_validation = _validate_cross_study_invariants(
            metadata,
            comparison_metadata,
            fraction_tolerance=fraction_tolerance,
        )
        combined_events = pd.concat(
            [comparison_events, events], ignore_index=True
        ).sort_values(["rear_static_weight_fraction", "event_slug"])
        combined_metadata = pd.concat(
            [comparison_metadata, metadata], ignore_index=True
        ).sort_values("rear_static_weight_fraction").reset_index(drop=True)
        scoring_reference = _load_scoring_reference(scoring_reference_path)
        primary_result = _score_cohort(events, metadata, scoring_reference)
        combined_result = _score_cohort(
            combined_events, combined_metadata, scoring_reference
        )
        combined_result["event_points_delta_vs_50pct"] = (
            _build_event_points_delta_vs_50pct(
                combined_result["events"],
                fraction_tolerance=fraction_tolerance,
            )
        )
        fine_validation = None
        if fine_validation_root is not None:
            fine_validation_root = fine_validation_root.resolve()
            fine_validation = _build_fine_resolution_validation(
                fine_root=fine_validation_root,
                coarse_events=combined_events,
                coarse_metadata=combined_metadata,
                scoring_reference=scoring_reference,
                expected_rear_fractions=fine_validation_expected_rear_fractions,
                expected_cg_height_in=expected_cg_height_in,
                expected_mu_scale=expected_mu_scale,
                fraction_tolerance=fraction_tolerance,
            )
        primary_audit_cases, primary_failed, primary_audit = _load_audit(
            study_root, metadata
        )
        (
            comparison_audit_cases,
            comparison_failed,
            comparison_audit,
        ) = _load_audit(comparison_study_root, comparison_metadata)
        primary_result["totals"] = _attach_audit_to_totals(
            primary_result["totals"], primary_audit_cases
        )
        combined_audit_cases = pd.concat(
            [comparison_audit_cases, primary_audit_cases], ignore_index=True
        )
        combined_result["totals"] = _attach_audit_to_totals(
            combined_result["totals"], combined_audit_cases
        )
        primary_failed = primary_failed.copy()
        comparison_failed = comparison_failed.copy()
        if not primary_failed.empty:
            primary_failed.insert(0, "source_cohort", "new_sweep")
        if not comparison_failed.empty:
            comparison_failed.insert(0, "source_cohort", "prior_sweep")
        failed_states = pd.concat(
            [comparison_failed, primary_failed], ignore_index=True
        )

        output_root.mkdir(parents=True, exist_ok=True)
        common_prefix = f"common_{len(combined_result['totals'])}case"
        primary_result["events"].sort_values(
            ["rear_static_weight_fraction", "event_slug"]
        ).to_csv(output_root / "new_sweep_event_results.csv", index=False)
        primary_result["totals"].to_csv(
            output_root / "new_sweep_case_performance_and_points.csv", index=False
        )
        primary_result["event_sensitivity"].to_csv(
            output_root / "new_sweep_event_sensitivity.csv", index=False
        )
        primary_result["event_optima"].to_csv(
            output_root / "new_sweep_event_optima.csv", index=False
        )
        combined_result["events"].sort_values(
            ["rear_static_weight_fraction", "event_slug"]
        ).to_csv(output_root / f"{common_prefix}_event_results.csv", index=False)
        combined_result["totals"].to_csv(
            output_root / f"{common_prefix}_case_performance_and_points.csv",
            index=False,
        )
        combined_result["event_sensitivity"].to_csv(
            output_root / f"{common_prefix}_event_sensitivity.csv", index=False
        )
        combined_result["event_optima"].to_csv(
            output_root / f"{common_prefix}_event_optima.csv", index=False
        )
        combined_result["event_points_delta_vs_50pct"].to_csv(
            output_root / f"{common_prefix}_event_points_delta_vs_50pct_rear.csv",
            index=False,
        )
        combined_metadata.to_csv(
            output_root / "validity_and_tuning_by_case.csv", index=False
        )
        failed_states.to_csv(
            output_root / "failed_qss_trim_reconstructions.csv", index=False
        )
        if fine_validation is not None:
            fine_validation["fine_result"]["events"].sort_values(
                ["rear_static_weight_fraction", "event_slug"]
            ).to_csv(output_root / "fine_validation_event_results.csv", index=False)
            fine_validation["fine_result"]["totals"].to_csv(
                output_root / "fine_validation_case_performance_and_points.csv",
                index=False,
            )
            fine_validation["fine_result"]["event_optima"].to_csv(
                output_root / "fine_validation_event_optima.csv", index=False
            )
            fine_validation["event_comparison"].to_csv(
                output_root / "coarse_vs_fine_event_validation.csv", index=False
            )
            fine_validation["ranking"].to_csv(
                output_root / "coarse_vs_fine_case_ranking.csv", index=False
            )
            fine_json = dict(fine_validation["validation"])
            fine_json["case_ranking"] = fine_validation["ranking"].to_dict(
                orient="records"
            )
            fine_json["event_time_comparison"] = fine_validation[
                "event_comparison"
            ].to_dict(orient="records")
            _write_json(output_root / "fine_resolution_validation.json", fine_json)
            _plot_fine_resolution_validation(
                fine_validation["ranking"],
                fine_validation["event_comparison"],
                output_root / "fine_resolution_validation.png",
            )
        correlations = {
            "new_sweep_design": design,
            "prior_sweep_design": comparison_design,
            "cross_study_validation": cross_validation,
            "new_sweep_fit": primary_result["fit"],
            "common_field_fit": combined_result["fit"],
            "new_sweep_event_sensitivity": primary_result[
                "event_sensitivity"
            ].to_dict(orient="records"),
            "common_field_event_sensitivity": combined_result[
                "event_sensitivity"
            ].to_dict(orient="records"),
            "common_field_event_points_delta_vs_50pct": combined_result[
                "event_points_delta_vs_50pct"
            ].to_dict(orient="records"),
        }
        _write_json(output_root / "rear_weight_correlations.json", correlations)
        _plot_points(
            combined_result["totals"],
            combined_result["fit"],
            output_root / "rear_weight_to_points.png",
            highlight_cohort="new_sweep",
        )
        _plot_event_points(
            combined_result["events"],
            output_root / "event_points_vs_rear_weight.png",
            highlight_cohort="new_sweep",
        )
        _plot_event_points_delta_vs_50pct(
            combined_result["event_points_delta_vs_50pct"],
            output_root / "event_points_delta_vs_50pct_rear_weight.png",
            highlight_cohort="new_sweep",
        )
        _plot_time_change(
            combined_result["events"],
            output_root / "raw_event_time_percent_change.png",
            highlight_cohort="new_sweep",
        )
        _plot_tuning(
            combined_metadata,
            output_root / "tuning_vs_rear_weight.png",
            highlight_cohort="new_sweep",
        )
        _write_grid_comparison_report(
            output_root / "study_report.md",
            primary_root=study_root,
            comparison_root=comparison_study_root,
            scoring_reference_path=scoring_reference_path,
            primary_tuning_path=tuning_path,
            comparison_tuning_path=comparison_tuning_path,
            primary_result=primary_result,
            combined_result=combined_result,
            combined_metadata=combined_metadata,
            primary_design=design,
            comparison_design=comparison_design,
            primary_audit=primary_audit,
            comparison_audit=comparison_audit,
            expected_mu_scale=expected_mu_scale,
            fine_validation=fine_validation,
        )
        validation: dict[str, Any] = {
            "status": "passed",
            "study_resolution": "smoke",
            "production_resolution_validation": False,
            "new_sweep_case_count": len(primary_result["totals"]),
            "common_field_case_count": len(combined_result["totals"]),
            "new_sweep_event_row_count": len(primary_result["events"]),
            "common_field_event_row_count": len(combined_result["events"]),
            "new_sweep_design": design,
            "prior_sweep_design": comparison_design,
            "cross_study_validation": cross_validation,
            "all_rear_fractions_retained_without_deduplication": True,
            "common_scoring_includes_2026_real_field_and_all_simulations": True,
            "only_exact_common_tmin_ties_receive_maximum": True,
            "event_points_delta_baseline_rear_static_weight_percent": (
                POINT_DELTA_BASELINE_REAR_WEIGHT_PERCENT
            ),
            "event_points_delta_uses_exact_simulated_baseline_without_interpolation": True,
            "event_points_delta_uses_final_common_scoring_field": True,
            "configured_tire_mu_scale": expected_mu_scale,
            "new_sweep_best_observed_rear_static_weight_percent": float(
                primary_result["totals"].loc[
                    primary_result["totals"]["timed_event_points"].idxmax(),
                    "rear_static_weight_percent",
                ]
            ),
            "common_field_best_observed_rear_static_weight_percent": float(
                combined_result["totals"].loc[
                    combined_result["totals"]["timed_event_points"].idxmax(),
                    "rear_static_weight_percent",
                ]
            ),
            "new_sweep_event_scoring": primary_result["event_scoring"],
            "common_field_event_scoring": combined_result["event_scoring"],
            "new_sweep_correlation": primary_result["fit"],
            "common_field_correlation": combined_result["fit"],
            "new_sweep_reduced_qss_audit": primary_audit,
            "prior_sweep_reduced_qss_audit": comparison_audit,
            "fine_resolution_validation": (
                fine_validation["validation"]
                if fine_validation is not None
                else {"present": False}
            ),
            "all_track_solvers_converged": bool(
                combined_result["totals"]["all_track_solvers_converged"].all()
            ),
            "total_ggv_speed_cap_segments": int(
                combined_result["totals"]["total_ggv_speed_cap_segments"].sum()
            ),
            "input_paths": {
                "new_study_root": str(study_root),
                "prior_study_root": str(comparison_study_root),
                "scoring_reference": str(scoring_reference_path),
                "new_tuning_json": tuning_path,
                "prior_tuning_json": comparison_tuning_path,
                "fine_validation_root": (
                    str(fine_validation_root)
                    if fine_validation_root is not None
                    else ""
                ),
            },
        }
        validation["artifacts"] = sorted(
            {path.name for path in output_root.iterdir()}
            | {"validation_summary.json"}
        )
        _write_json(output_root / "validation_summary.json", validation)
        return validation

    scoring_reference = _load_scoring_reference(scoring_reference_path)
    rescored, event_scoring = _rescore_common_field(events, scoring_reference)
    awarded_maximum = np.isclose(
        rescored["common_projected_points"].to_numpy(dtype=float),
        rescored["common_maximum_points"].to_numpy(dtype=float),
        rtol=0.0,
        atol=0.0,
    )
    exact_tmin = (
        rescored["common_projected_competition_time_s"].to_numpy(dtype=float)
        == rescored["common_field_tmin_s"].to_numpy(dtype=float)
    )
    if not np.array_equal(awarded_maximum, exact_tmin):
        raise AssertionError("Maximum points were not limited to exact Tmin ties.")
    for event_slug in EXPECTED_EVENTS:
        winners = rescored[
            (rescored["event_slug"] == event_slug)
            & rescored["common_is_event_fastest"].map(_as_bool)
        ]["common_projected_competition_time_s"].to_numpy(dtype=float)
        if winners.size > 1 and not bool((winners == winners[0]).all()):
            raise AssertionError(f"{event_slug} awarded a non-identical simulated tie.")

    totals = _build_case_totals(rescored, metadata)
    x = totals["rear_static_weight_percent"].to_numpy(dtype=float)
    y = totals["timed_event_points"].to_numpy(dtype=float)
    fit = _fit_summary(x, y)
    event_sensitivity = _build_event_sensitivity(
        rescored, float(fit["endpoint_change_points_55_minus_45"])
    )
    event_points_delta = _build_event_points_delta_vs_50pct(
        rescored,
        fraction_tolerance=fraction_tolerance,
    )
    audit_cases, failed_states, audit_summary = _load_audit(study_root, metadata)
    if not audit_cases.empty:
        audit_by_case = audit_cases.set_index("cg_case")
        for index, row in totals.iterrows():
            audit = audit_by_case.loc[str(row["cg_case"])]
            totals.loc[index, "sampled_qss_sample_count"] = int(
                audit["sample_count"]
            )
            totals.loc[index, "sampled_qss_successful_trim_count"] = int(
                audit["successful_trim_count"]
            )
            totals.loc[index, "sampled_qss_normal_load_min_n"] = _finite_float(
                audit["sampled_normal_load_min_n"]
            )
            totals.loc[index, "sampled_qss_normal_load_max_n"] = _finite_float(
                audit["sampled_normal_load_max_n"]
            )
            totals.loc[index, "sampled_qss_failed_trim_count"] = int(
                audit["failed_trim_count"]
            )
            totals.loc[index, "sampled_qss_wheel_lift_state_count"] = int(
                audit["sampled_wheel_lift_state_count"]
            )
            totals.loc[index, "sampled_qss_state_count_below_tir"] = int(
                audit.get(
                    "sampled_state_count_below_100n",
                    _finite_float(audit["sampled_normal_load_min_n"]) < 100.0,
                )
            )
            totals.loc[index, "sampled_qss_state_count_above_tir"] = int(
                audit.get(
                    "sampled_state_count_above_1800n",
                    _finite_float(audit["sampled_normal_load_max_n"]) > 1800.0,
                )
            )

    output_root.mkdir(parents=True, exist_ok=True)
    rescored.sort_values(["rear_static_weight_fraction", "event_slug"]).to_csv(
        output_root / "common_field_event_results.csv", index=False
    )
    totals.to_csv(output_root / "case_performance_and_points.csv", index=False)
    event_sensitivity.to_csv(output_root / "event_sensitivity.csv", index=False)
    event_points_delta.to_csv(
        output_root / "event_points_delta_vs_50pct_rear.csv", index=False
    )
    metadata.to_csv(output_root / "validity_and_tuning_by_case.csv", index=False)
    failed_states.to_csv(output_root / "failed_qss_trim_reconstructions.csv", index=False)
    _write_json(
        output_root / "rear_weight_correlation.json",
        {
            "design": design,
            "fit": fit,
            "event_sensitivity": event_sensitivity.to_dict(orient="records"),
            "event_points_delta_vs_50pct": event_points_delta.to_dict(
                orient="records"
            ),
        },
    )
    _plot_points(totals, fit, output_root / "rear_weight_to_points.png")
    _plot_event_points(rescored, output_root / "event_points_vs_rear_weight.png")
    _plot_event_points_delta_vs_50pct(
        event_points_delta,
        output_root / "event_points_delta_vs_50pct_rear_weight.png",
    )
    _plot_time_change(rescored, output_root / "raw_event_time_percent_change.png")
    _plot_tuning(metadata, output_root / "tuning_vs_rear_weight.png")
    _write_report(
        output_root / "study_report.md",
        study_root=study_root,
        scoring_reference_path=scoring_reference_path,
        tuning_path=tuning_path,
        tuning_context=tuning_context,
        totals=totals,
        event_sensitivity=event_sensitivity,
        event_points_delta=event_points_delta,
        fit=fit,
        design=design,
        event_scoring=event_scoring,
        audit_summary=audit_summary,
    )
    validation: dict[str, Any] = {
        "status": "passed",
        "study_resolution": "smoke",
        "production_resolution_validation": False,
        "case_count": len(totals),
        "event_row_count": len(rescored),
        "design": design,
        "common_scoring_includes_2026_real_field_and_all_simulations": True,
        "only_exact_common_tmin_ties_receive_maximum": True,
        "event_points_delta_baseline_rear_static_weight_percent": (
            POINT_DELTA_BASELINE_REAR_WEIGHT_PERCENT
        ),
        "event_points_delta_uses_exact_simulated_baseline_without_interpolation": True,
        "event_points_delta_uses_common_scoring_field": True,
        "all_track_solvers_converged": bool(
            totals["all_track_solvers_converged"].all()
        ),
        "total_ggv_speed_cap_segments": int(
            totals["total_ggv_speed_cap_segments"].sum()
        ),
        "best_observed_rear_static_weight_percent": float(
            totals.loc[totals["timed_event_points"].idxmax(), "rear_static_weight_percent"]
        ),
        "best_observed_timed_event_points": float(totals["timed_event_points"].max()),
        "tuned_lateral_constraint_cases": totals.loc[
            totals["tuned_lateral_active_constraints"].astype(str).str.len() > 0,
            "cg_case",
        ].astype(str).tolist(),
        "serialized_ggv_load_outside_tir_cases": totals.loc[
            totals["wheel_load_outside_tire_validity"].map(_as_bool), "cg_case"
        ].astype(str).tolist(),
        "event_scoring": event_scoring,
        "rear_weight_correlation": fit,
        "reduced_qss_audit": audit_summary,
        "input_paths": {
            "study_root": str(study_root),
            "scoring_reference": str(scoring_reference_path),
            "tuning_json": tuning_path,
        },
    }
    validation["artifacts"] = sorted(
        {path.name for path in output_root.iterdir()} | {"validation_summary.json"}
    )
    _write_json(output_root / "validation_summary.json", validation)
    return validation


def main() -> None:
    args = parse_args()
    validation = build_report_bundle(
        study_root=args.study_root,
        output_root=args.output_root,
        tuning_json_path=args.tuning_json,
        comparison_study_root=args.comparison_study_root,
        comparison_tuning_json_path=args.comparison_tuning_json,
        fine_validation_root=args.fine_validation_root,
        scoring_reference_path=args.scoring_reference,
        expected_rear_fractions=args.expected_rear_fractions,
        comparison_expected_rear_fractions=args.comparison_rear_fractions,
        fine_validation_expected_rear_fractions=(
            args.fine_validation_rear_fractions
        ),
        expected_cg_height_in=args.expected_cg_height_in,
        expected_mu_scale=args.expected_mu_scale,
        fraction_tolerance=args.fraction_tolerance,
    )
    print(json.dumps(validation, indent=2, default=_json_default))


if __name__ == "__main__":
    main()
