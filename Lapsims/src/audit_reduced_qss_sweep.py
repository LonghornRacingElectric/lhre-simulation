"""Sampled reduced-QSS load/state audit for completed reduced-model sweeps.

This post-processor reconstructs each frozen reduced model from the active raw
TIR/YAML projection plus the study-local setup recorded in case metadata. It
then solves deterministic representative trace and GGV-boundary states. The
result is a sampled trim audit, not an exact time- or distance-weighted history.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from itertools import pairwise
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

LAPSIMS_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = LAPSIMS_ROOT.parents[2]
DEFAULT_BOBSIM_ROOT = WORKSPACE_ROOT / "BobDyn" / "BobSim"
DEFAULT_BOUNDARY_SPEED_MPS = 11.8
DEFAULT_MIN_QSS_SPEED_MPS = 0.1
DEFAULT_LOAD_MIN_N = 100.0
DEFAULT_LOAD_MAX_N = 1800.0
MASS_SWEEP_AXIS = "sprung_mass_kg"
EXPECTED_MASS_SWEEP_HEIGHT_IN = 11.5
EXPECTED_MASS_SWEEP_OFFSETS_LB = (-100.0, -50.0, 0.0, 50.0, 100.0)
LONGITUDINAL_CG_SWEEP_AXIS = "rear_static_weight_fraction"
EXPECTED_LONGITUDINAL_CG_HEIGHT_IN = 11.5
EXPECTED_NOMINAL_SPRUNG_MASS_KG = 230.72288408
EXPECTED_NOMINAL_TOTAL_MASS_KG = 261.07265114
EXPECTED_FRONT_AXLE_GLOBAL_X_M = 0.0
EXPECTED_REAR_AXLE_GLOBAL_X_M = -1.5494
EXPECTED_LONGITUDINAL_CG_TIRE_MU_SCALE = 0.6225437130779028
G_MPS2 = 9.80665
LB_TO_KG = 0.45359237
TRACE_COLUMNS = {
    "longitudinal_accel_mps2",
    "ggv_constraint_speed_mps",
    "ggv_constraint_lateral_accel_mps2",
    "ggv_accel_limit_mps2",
    "ggv_brake_limit_mps2",
    "ggv_lateral_utilization",
    "ggv_speed_domain_fraction",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--model-dof", type=int, choices=(3, 6), required=True)
    parser.add_argument("--bobsim-root", type=Path, default=DEFAULT_BOBSIM_ROOT)
    parser.add_argument(
        "--sweep-json",
        type=Path,
        default=None,
        help=(
            "Optional source sweep definition. Longitudinal-CG case names, "
            "fractions, and reference metadata are checked against it."
        ),
    )
    parser.add_argument("--audit-output-dir", type=Path, default=None)
    parser.add_argument("--max-trace-samples-per-case", type=int, default=36)
    parser.add_argument("--speed-bins", type=int, default=6)
    parser.add_argument("--lateral-bins", type=int, default=7)
    parser.add_argument("--longitudinal-bins", type=int, default=4)
    parser.add_argument(
        "--pure-lateral-speed-mps",
        type=float,
        default=DEFAULT_BOUNDARY_SPEED_MPS,
    )
    parser.add_argument("--trim-max-nfev", type=int, default=220)
    parser.add_argument("--trim-tolerance", type=float, default=1e-7)
    parser.add_argument("--resolution-compare-root", type=Path, default=None)
    return parser.parse_args()


def _write_json(path: Path, payload: object) -> None:
    def default(value: object) -> object:
        if isinstance(value, np.generic):
            return value.item()
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, Path):
            return value.as_posix()
        raise TypeError(f"Cannot serialize {type(value).__name__}.")

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, default=default) + "\n",
        encoding="utf-8",
    )


def _finite_numeric(frame: pd.DataFrame, columns: set[str]) -> pd.DataFrame:
    converted = frame.copy()
    for column in columns:
        converted[column] = pd.to_numeric(converted[column], errors="coerce")
    mask = np.ones(len(converted), dtype=bool)
    for column in columns:
        mask &= np.isfinite(converted[column].to_numpy(dtype=float))
    return converted.loc[mask].copy()


def _bin_index(values: np.ndarray, *, low: float, high: float, count: int) -> np.ndarray:
    if count <= 0:
        raise ValueError("Bin counts must be positive.")
    if high <= low:
        return np.zeros(values.size, dtype=int)
    normalized = (np.clip(values, low, high) - low) / (high - low)
    return np.minimum((normalized * count).astype(int), count - 1)


def _evenly_spaced_positions(count: int, target: int) -> list[int]:
    if target <= 0 or count <= 0:
        return []
    if count <= target:
        return list(range(count))
    return sorted(set(np.linspace(0, count - 1, target).round().astype(int).tolist()))


def select_trace_states(
    traces: pd.DataFrame,
    *,
    maximum_samples: int,
    speed_bins: int,
    lateral_bins: int,
    longitudinal_bins: int,
    minimum_qss_speed_mps: float = DEFAULT_MIN_QSS_SPEED_MPS,
) -> list[dict[str, Any]]:
    """Select deterministic representatives across branch and utilization bins."""

    missing = TRACE_COLUMNS.difference(traces.columns)
    if missing:
        raise ValueError(f"Trace data are missing columns: {sorted(missing)}")
    required = set(TRACE_COLUMNS)
    frame = _finite_numeric(traces, required)
    frame = frame.loc[
        frame["ggv_constraint_speed_mps"] >= minimum_qss_speed_mps - 1e-12
    ].copy()
    if frame.empty:
        return []
    if "event_slug" not in frame:
        frame["event_slug"] = "unknown"
    if "trace_row_index" not in frame:
        frame["trace_row_index"] = np.arange(len(frame), dtype=int)

    ax = frame["longitudinal_accel_mps2"].to_numpy(dtype=float)
    ay = frame["ggv_constraint_lateral_accel_mps2"].to_numpy(dtype=float)
    lateral_util = frame["ggv_lateral_utilization"].to_numpy(dtype=float)
    signed_lateral_util = np.sign(ay) * np.clip(lateral_util, 0.0, 1.5)
    branch = np.where(ax >= -1e-9, "accel", "brake")
    accel_limit = np.maximum(
        frame["ggv_accel_limit_mps2"].to_numpy(dtype=float), 1e-9
    )
    brake_limit = np.maximum(
        np.abs(frame["ggv_brake_limit_mps2"].to_numpy(dtype=float)), 1e-9
    )
    longitudinal_util = np.where(
        branch == "accel",
        np.maximum(ax, 0.0) / accel_limit,
        np.abs(np.minimum(ax, 0.0)) / brake_limit,
    )
    speed_fraction = np.clip(
        frame["ggv_speed_domain_fraction"].to_numpy(dtype=float), 0.0, 1.0
    )
    frame["audit_branch"] = branch
    frame["audit_speed_fraction"] = speed_fraction
    frame["audit_signed_lateral_utilization"] = signed_lateral_util
    frame["audit_longitudinal_utilization"] = longitudinal_util
    frame["audit_speed_bin"] = _bin_index(
        speed_fraction, low=0.0, high=1.0, count=speed_bins
    )
    frame["audit_lateral_bin"] = _bin_index(
        signed_lateral_util, low=-1.0, high=1.0, count=lateral_bins
    )
    frame["audit_longitudinal_bin"] = _bin_index(
        longitudinal_util, low=0.0, high=1.0, count=longitudinal_bins
    )

    representatives: list[pd.Series] = []
    group_columns = [
        "audit_branch",
        "audit_speed_bin",
        "audit_lateral_bin",
        "audit_longitudinal_bin",
    ]
    for _, group in frame.groupby(group_columns, sort=True):
        speed_center = (float(group["audit_speed_bin"].iloc[0]) + 0.5) / speed_bins
        lateral_center = (
            -1.0
            + 2.0
            * (float(group["audit_lateral_bin"].iloc[0]) + 0.5)
            / lateral_bins
        )
        longitudinal_center = (
            float(group["audit_longitudinal_bin"].iloc[0]) + 0.5
        ) / longitudinal_bins
        distance = (
            (group["audit_speed_fraction"] - speed_center) ** 2
            + (group["audit_signed_lateral_utilization"] - lateral_center) ** 2
            + (group["audit_longitudinal_utilization"] - longitudinal_center) ** 2
        )
        ranked = group.assign(_distance=distance).sort_values(
            ["_distance", "event_slug", "trace_row_index"], kind="mergesort"
        )
        representatives.append(ranked.iloc[0])

    representatives.sort(
        key=lambda row: (
            str(row["audit_branch"]),
            int(row["audit_speed_bin"]),
            int(row["audit_lateral_bin"]),
            int(row["audit_longitudinal_bin"]),
            str(row["event_slug"]),
            int(row["trace_row_index"]),
        )
    )
    selected = [representatives[index] for index in _evenly_spaced_positions(
        len(representatives), maximum_samples
    )]

    mandatory = {
        "trace_strongest_accel": frame["longitudinal_accel_mps2"].idxmax(),
        "trace_strongest_brake": frame["longitudinal_accel_mps2"].idxmin(),
        "trace_highest_lateral_utilization": frame[
            "ggv_lateral_utilization"
        ].idxmax(),
        "trace_highest_speed": frame["ggv_constraint_speed_mps"].idxmax(),
    }
    by_index: dict[tuple[str, int], dict[str, Any]] = {}
    for row in selected:
        key = (str(row["event_slug"]), int(row["trace_row_index"]))
        by_index[key] = _trace_row_to_state(row, "trace_binned")
    for source_kind, index in mandatory.items():
        row = frame.loc[index]
        key = (str(row["event_slug"]), int(row["trace_row_index"]))
        state = _trace_row_to_state(row, source_kind)
        if key in by_index:
            existing = by_index[key]
            existing["selection_reasons"] = sorted(
                set(existing["selection_reasons"] + [source_kind])
            )
        else:
            by_index[key] = state
    return sorted(
        by_index.values(),
        key=lambda item: (
            item["event_slug"],
            item["trace_row_index"],
            item["source_kind"],
        ),
    )


def _trace_row_to_state(row: pd.Series, source_kind: str) -> dict[str, Any]:
    return {
        "source_kind": source_kind,
        "selection_reasons": [source_kind],
        "event_slug": str(row["event_slug"]),
        "trace_row_index": int(row["trace_row_index"]),
        "speed_mps": float(row["ggv_constraint_speed_mps"]),
        "ax_mps2": float(row["longitudinal_accel_mps2"]),
        "ay_mps2": float(row["ggv_constraint_lateral_accel_mps2"]),
        "branch": str(row["audit_branch"]),
        "speed_fraction": float(row["audit_speed_fraction"]),
        "signed_lateral_utilization": float(
            row["audit_signed_lateral_utilization"]
        ),
        "longitudinal_utilization": float(row["audit_longitudinal_utilization"]),
    }


def select_boundary_states(
    ggv_csv: Path,
    *,
    pure_lateral_speed_mps: float,
) -> list[dict[str, Any]]:
    """Return pure-lateral and strongest serialized accel/brake candidates."""

    from ggv_lookup_solver import GGVMap

    ggv = GGVMap.from_csv(ggv_csv)
    speed = float(
        np.clip(
            pure_lateral_speed_mps,
            max(ggv.minimum_speed_mps, DEFAULT_MIN_QSS_SPEED_MPS),
            ggv.maximum_speed_mps,
        )
    )
    positive, _negative = ggv.lateral_limits(np.asarray([speed], dtype=float))
    states = [
        {
            "source_kind": "boundary_pure_lateral_11p8",
            "selection_reasons": ["boundary_pure_lateral"],
            "event_slug": "GGV",
            "trace_row_index": -1,
            "speed_mps": speed,
            "ax_mps2": 0.0,
            "ay_mps2": float(positive[0]),
            "branch": "lateral",
            "speed_fraction": speed / ggv.maximum_speed_mps,
            "signed_lateral_utilization": 1.0,
            "longitudinal_utilization": 0.0,
        }
    ]

    frame = pd.read_csv(ggv_csv)
    for column in (
        "speed_mps",
        "ay_mps2",
        "ax_accel_mps2",
        "ax_brake_mps2",
        "accel_feasible",
        "brake_feasible",
    ):
        frame[column] = pd.to_numeric(frame[column], errors="coerce")
    positive_speed = frame["speed_mps"] >= DEFAULT_MIN_QSS_SPEED_MPS - 1e-12
    accel = frame.loc[
        positive_speed
        & (frame["accel_feasible"] > 0.5)
        & np.isfinite(frame["ax_accel_mps2"])
    ]
    brake = frame.loc[
        positive_speed
        & (frame["brake_feasible"] > 0.5)
        & np.isfinite(frame["ax_brake_mps2"])
    ]
    if not accel.empty:
        row = accel.loc[accel["ax_accel_mps2"].idxmax()]
        states.append(_ggv_row_to_state(row, "boundary_strongest_accel", "accel"))
    if not brake.empty:
        row = brake.loc[brake["ax_brake_mps2"].idxmin()]
        states.append(_ggv_row_to_state(row, "boundary_strongest_brake", "brake"))
    return states


def _ggv_row_to_state(row: pd.Series, source_kind: str, branch: str) -> dict[str, Any]:
    ax_column = "ax_accel_mps2" if branch == "accel" else "ax_brake_mps2"
    return {
        "source_kind": source_kind,
        "selection_reasons": [source_kind],
        "event_slug": "GGV",
        "trace_row_index": -1,
        "speed_mps": float(row["speed_mps"]),
        "ax_mps2": float(row[ax_column]),
        "ay_mps2": float(row["ay_mps2"]),
        "branch": branch,
        "speed_fraction": math.nan,
        "signed_lateral_utilization": math.nan,
        "longitudinal_utilization": 1.0,
    }


def audit_trim_state(
    model: Any,
    state: dict[str, Any],
    *,
    model_dof: int,
    max_nfev: int,
    tolerance: float,
    load_min_n: float = DEFAULT_LOAD_MIN_N,
    load_max_n: float = DEFAULT_LOAD_MAX_N,
) -> dict[str, Any]:
    from _0_Utils.dyn_py import solve_acceleration_trim

    trim = solve_acceleration_trim(
        model,
        speed_mps=float(state["speed_mps"]),
        longitudinal_acceleration_mps2=float(state["ax_mps2"]),
        lateral_acceleration_mps2=float(state["ay_mps2"]),
        max_nfev=max_nfev,
        tolerance=tolerance,
    )
    loads = np.asarray(trim.output.normal_loads_n, dtype=float)
    result = {
        **state,
        "model_dof": model_dof,
        "trim_success": bool(trim.success),
        "trim_message": str(trim.message),
        "trim_residual_norm": float(trim.residual_norm),
        "beta_rad": float(trim.unknowns.get("beta_rad", math.nan)),
        "steering_rad": float(trim.unknowns.get("steering_rad", math.nan)),
        "heave_m": float(trim.unknowns.get("heave_m", math.nan)),
        "roll_rad": float(trim.unknowns.get("roll_rad", math.nan)),
        "pitch_rad": float(trim.unknowns.get("pitch_rad", math.nan)),
        "fz_fl_n": float(loads[0]),
        "fz_fr_n": float(loads[1]),
        "fz_rl_n": float(loads[2]),
        "fz_rr_n": float(loads[3]),
        "normal_load_min_n": float(np.min(loads)),
        "normal_load_max_n": float(np.max(loads)),
        "wheel_observations_below_100n": int(np.count_nonzero(loads < load_min_n)),
        "wheel_observations_above_1800n": int(np.count_nonzero(loads > load_max_n)),
        "wheel_lift": bool(np.any(loads <= 0.0)),
        "valid_for_load_summary": bool(trim.success and np.all(np.isfinite(loads))),
    }
    return result


def summarize_trim_rows(rows: list[dict[str, Any]]) -> dict[str, Any]:
    """Aggregate sampled trim rows without implying full-lap occurrence rates."""

    if not rows:
        return {
            "sample_count": 0,
            "successful_trim_count": 0,
            "failed_trim_count": 0,
        }
    frame = pd.DataFrame(rows)
    valid = frame.loc[frame["valid_for_load_summary"].astype(bool)].copy()
    result: dict[str, Any] = {
        "sample_count": len(frame),
        "successful_trim_count": int(frame["trim_success"].sum()),
        "failed_trim_count": int((~frame["trim_success"].astype(bool)).sum()),
        "maximum_abs_beta_rad": float(frame["beta_rad"].abs().max()),
        "maximum_abs_steering_rad": float(frame["steering_rad"].abs().max()),
        "maximum_residual_norm": float(frame["trim_residual_norm"].max()),
        "sampled_state_count_below_100n": int(
            np.count_nonzero(valid["wheel_observations_below_100n"] > 0)
        ),
        "sampled_wheel_observations_below_100n": int(
            valid["wheel_observations_below_100n"].sum()
        ),
        "sampled_state_count_above_1800n": int(
            np.count_nonzero(valid["wheel_observations_above_1800n"] > 0)
        ),
        "sampled_wheel_observations_above_1800n": int(
            valid["wheel_observations_above_1800n"].sum()
        ),
        "sampled_wheel_lift_state_count": int(valid["wheel_lift"].sum()),
        "load_and_threshold_statistics_include_only_successful_trims": True,
    }
    if valid.empty:
        result.update(
            {
                "sampled_normal_load_min_n": math.nan,
                "sampled_normal_load_max_n": math.nan,
            }
        )
    else:
        result.update(
            {
                "sampled_normal_load_min_n": float(valid["normal_load_min_n"].min()),
                "sampled_normal_load_max_n": float(valid["normal_load_max_n"].max()),
            }
        )
    for name in ("heave_m", "roll_rad", "pitch_rad"):
        values = pd.to_numeric(valid[name], errors="coerce").to_numpy(dtype=float)
        finite = values[np.isfinite(values)]
        result[f"sampled_{name}_min"] = (
            float(np.min(finite)) if finite.size else math.nan
        )
        result[f"sampled_{name}_max"] = (
            float(np.max(finite)) if finite.size else math.nan
        )
    return result


def _load_case_traces(case_dir: Path) -> pd.DataFrame:
    frames: list[pd.DataFrame] = []
    for path in sorted(case_dir.glob("*_trace.csv")):
        event_slug = path.stem.removesuffix("_trace")
        frame = pd.read_csv(path)
        frame["event_slug"] = event_slug
        frame["trace_row_index"] = np.arange(len(frame), dtype=int)
        frames.append(frame)
    if not frames:
        raise FileNotFoundError(f"No event trace CSVs found in {case_dir}.")
    return pd.concat(frames, ignore_index=True)


def _validate_case_metadata(
    metadata: dict[str, Any],
    *,
    model_dof: int,
) -> float:
    if int(metadata.get("model_dof", -1)) != model_dof:
        raise ValueError(
            f"Case {metadata.get('cg_case')} metadata model_dof does not match "
            f"requested {model_dof}DOF."
        )
    if metadata.get("model_family") != "dyn_py_reduced_order_qss":
        raise ValueError("Audit requires dyn_py_reduced_order_qss case metadata.")
    raw_tire_mu_scale = metadata.get("tire_mu_scale")
    if (
        isinstance(raw_tire_mu_scale, bool)
        or not isinstance(raw_tire_mu_scale, (int, float))
        or not math.isfinite(float(raw_tire_mu_scale))
        or float(raw_tire_mu_scale) <= 0.0
    ):
        raise ValueError(
            "Reduced-QSS case metadata requires a finite positive tire_mu_scale."
        )
    tire_mu_scale = float(raw_tire_mu_scale)
    if not math.isclose(
        float(metadata.get("effective_aero_balance_front", math.nan)),
        0.5,
        abs_tol=1e-12,
    ):
        raise ValueError("Reduced-QSS CG audit requires the 50/50 aero setup.")
    value = metadata.get("front_antiroll_stiffness_fraction")
    if value is None or not math.isfinite(float(value)):
        raise ValueError(
            "Reduced-QSS case metadata requires front_antiroll_stiffness_fraction."
        )
    _mass_case_metadata(metadata)
    longitudinal_case = _longitudinal_cg_case_metadata(metadata)
    if longitudinal_case is not None:
        if model_dof != 6:
            raise ValueError("Longitudinal-CG audit requires the 6DOF model.")
        if not math.isclose(
            tire_mu_scale,
            EXPECTED_LONGITUDINAL_CG_TIRE_MU_SCALE,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise ValueError("Longitudinal-CG audit requires the calibrated mu scale.")
        if not math.isclose(
            float(metadata.get("drive_distribution_front", math.nan)),
            0.0,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise ValueError("Longitudinal-CG audit requires rear-wheel drive.")
        if metadata.get("limited_slip_differential_model") is not False:
            raise ValueError("Longitudinal-CG audit requires no LSD model.")
    return tire_mu_scale


def _required_finite_float(
    mapping: dict[str, Any],
    field: str,
    *,
    context: str,
    positive: bool = False,
) -> float:
    value = mapping.get(field)
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or (positive and float(value) <= 0.0)
    ):
        qualifier = "finite positive" if positive else "finite"
        raise ValueError(f"{context}.{field} must be {qualifier}.")
    return float(value)


def _required_finite_sequence(
    mapping: dict[str, Any],
    field: str,
    *,
    context: str,
    length: int,
) -> list[float]:
    value = mapping.get(field)
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{context}.{field} must contain {length} values.")
    try:
        numbers = [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{context}.{field} must be numeric.") from exc
    if any(not math.isfinite(item) for item in numbers):
        raise ValueError(f"{context}.{field} must be finite.")
    return numbers


def _mass_case_metadata(metadata: dict[str, Any]) -> dict[str, float] | None:
    """Validate and return the serialized mass identity for a mass-sweep case.

    Older CG-height artifacts omit ``sweep_axis`` and retain their historical
    validation path.  Declared sprung-mass cases are deliberately stricter:
    their top-level, override, reduced-model, and outer-vehicle mass identities
    must all agree before the live model is reconstructed.
    """

    sweep_axis = str(metadata.get("sweep_axis", "cg_height_in"))
    if sweep_axis in {"cg_height_in", LONGITUDINAL_CG_SWEEP_AXIS}:
        return None
    if sweep_axis != MASS_SWEEP_AXIS:
        raise ValueError(f"Unsupported reduced-QSS sweep_axis {sweep_axis!r}.")

    case_name = str(metadata.get("cg_case", "unknown"))
    context = f"Case {case_name} metadata"
    cg_height_in = _required_finite_float(
        metadata, "cg_height_in", context=context, positive=True
    )
    if not math.isclose(
        cg_height_in,
        EXPECTED_MASS_SWEEP_HEIGHT_IN,
        rel_tol=0.0,
        abs_tol=1e-10,
    ):
        raise ValueError(
            f"{context} mass sweep must remain at "
            f"{EXPECTED_MASS_SWEEP_HEIGHT_IN:.1f} in CG height."
        )
    sprung_mass = _required_finite_float(
        metadata, "sprung_mass_kg", context=context, positive=True
    )
    total_mass = _required_finite_float(
        metadata, "total_mass_kg", context=context, positive=True
    )
    mass_offset_lb = _required_finite_float(
        metadata, "mass_offset_lb", context=context
    )

    overrides = metadata.get("reduced_parameter_overrides")
    if not isinstance(overrides, dict):
        raise TypeError(f"{context}.reduced_parameter_overrides must be an object.")
    override_sprung = _required_finite_float(
        overrides,
        "sprung_mass_kg",
        context=f"{context}.reduced_parameter_overrides",
        positive=True,
    )
    override_total = _required_finite_float(
        overrides,
        "total_mass_kg",
        context=f"{context}.reduced_parameter_overrides",
        positive=True,
    )
    fixed_unsprung = _required_finite_float(
        overrides,
        "fixed_unsprung_mass_kg",
        context=f"{context}.reduced_parameter_overrides",
        positive=True,
    )

    reduced = metadata.get("reduced_vehicle_parameters")
    if not isinstance(reduced, dict):
        raise TypeError(f"{context}.reduced_vehicle_parameters must be an object.")
    reduced_sprung = _required_finite_float(
        reduced,
        "sprung_mass_kg",
        context=f"{context}.reduced_vehicle_parameters",
        positive=True,
    )
    reduced_total = _required_finite_float(
        reduced,
        "mass_kg",
        context=f"{context}.reduced_vehicle_parameters",
        positive=True,
    )
    unsprung_values = reduced.get("unsprung_mass_kg")
    if not isinstance(unsprung_values, list) or len(unsprung_values) != 4:
        raise ValueError(
            f"{context}.reduced_vehicle_parameters.unsprung_mass_kg must contain "
            "four corner masses."
        )
    try:
        unsprung = [float(value) for value in unsprung_values]
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"{context}.reduced_vehicle_parameters.unsprung_mass_kg must be numeric."
        ) from exc
    if any(not math.isfinite(value) or value <= 0.0 for value in unsprung):
        raise ValueError(
            f"{context}.reduced_vehicle_parameters.unsprung_mass_kg must be "
            "finite and positive."
        )
    reduced_unsprung = sum(unsprung)

    vehicle = metadata.get("vehicle")
    if not isinstance(vehicle, dict):
        raise TypeError(f"{context}.vehicle must be an object.")
    outer_total = _required_finite_float(
        vehicle, "mass", context=f"{context}.vehicle", positive=True
    )

    comparisons = (
        (sprung_mass, override_sprung, "top-level and override sprung mass"),
        (sprung_mass, reduced_sprung, "top-level and reduced sprung mass"),
        (total_mass, override_total, "top-level and override total mass"),
        (total_mass, reduced_total, "top-level and reduced total mass"),
        (total_mass, outer_total, "top-level and outer vehicle total mass"),
        (fixed_unsprung, reduced_unsprung, "fixed and reduced unsprung mass"),
        (total_mass, sprung_mass + fixed_unsprung, "sprung plus unsprung total mass"),
    )
    for left, right, label in comparisons:
        if not math.isclose(left, right, rel_tol=0.0, abs_tol=1e-8):
            raise ValueError(f"{context} mismatch between {label}.")
    return {
        "cg_height_in": cg_height_in,
        "sprung_mass_kg": sprung_mass,
        "total_mass_kg": total_mass,
        "fixed_unsprung_mass_kg": fixed_unsprung,
        "mass_offset_lb": mass_offset_lb,
    }


def _longitudinal_cg_case_metadata(
    metadata: dict[str, Any],
) -> dict[str, float] | None:
    """Validate one serialized total-static rear-weight case."""

    sweep_axis = str(metadata.get("sweep_axis", "cg_height_in"))
    if sweep_axis in {"cg_height_in", MASS_SWEEP_AXIS}:
        return None
    if sweep_axis != LONGITUDINAL_CG_SWEEP_AXIS:
        raise ValueError(f"Unsupported reduced-QSS sweep_axis {sweep_axis!r}.")

    case_name = str(metadata.get("cg_case", "unknown"))
    context = f"Case {case_name} metadata"
    height_in = _required_finite_float(
        metadata, "cg_height_in", context=context, positive=True
    )
    sprung_mass = _required_finite_float(
        metadata, "sprung_mass_kg", context=context, positive=True
    )
    total_mass = _required_finite_float(
        metadata, "total_mass_kg", context=context, positive=True
    )
    rear_fraction = _required_finite_float(
        metadata, "rear_static_weight_fraction", context=context, positive=True
    )
    front_fraction = _required_finite_float(
        metadata, "front_static_weight_fraction", context=context, positive=True
    )
    expected_cg_x = (
        EXPECTED_FRONT_AXLE_GLOBAL_X_M
        + rear_fraction
        * (EXPECTED_REAR_AXLE_GLOBAL_X_M - EXPECTED_FRONT_AXLE_GLOBAL_X_M)
    )
    required_values = (
        (height_in, EXPECTED_LONGITUDINAL_CG_HEIGHT_IN, "CG height"),
        (sprung_mass, EXPECTED_NOMINAL_SPRUNG_MASS_KG, "sprung mass"),
        (total_mass, EXPECTED_NOMINAL_TOTAL_MASS_KG, "total mass"),
        (front_fraction, 1.0 - rear_fraction, "front/rear static fractions"),
    )
    for actual, expected, label in required_values:
        if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-8):
            raise ValueError(f"{context} longitudinal-CG {label} mismatch.")
    if not 0.0 < rear_fraction < 1.0:
        raise ValueError(f"{context} rear static fraction must be inside (0, 1).")

    overrides = metadata.get("reduced_parameter_overrides")
    reduced = metadata.get("reduced_vehicle_parameters")
    vehicle = metadata.get("vehicle")
    if not isinstance(overrides, dict):
        raise TypeError(f"{context}.reduced_parameter_overrides must be an object.")
    if not isinstance(reduced, dict):
        raise TypeError(f"{context}.reduced_vehicle_parameters must be an object.")
    if not isinstance(vehicle, dict):
        raise TypeError(f"{context}.vehicle must be an object.")

    override_context = f"{context}.reduced_parameter_overrides"
    reduced_context = f"{context}.reduced_vehicle_parameters"
    vehicle_context = f"{context}.vehicle"
    override_cg = _required_finite_sequence(
        overrides, "center_of_gravity_m", context=override_context, length=3
    )
    reduced_cg = _required_finite_sequence(
        reduced, "center_of_gravity_m", context=reduced_context, length=3
    )
    corner_positions = _required_finite_sequence(
        {
            "values": [
                item
                for corner in reduced.get("corner_positions_m", [])
                if isinstance(corner, (list, tuple))
                for item in corner
            ]
        },
        "values",
        context=f"{reduced_context}.corner_positions_m",
        length=12,
    )
    corners = np.asarray(corner_positions, dtype=float).reshape(4, 3)
    static_loads = _required_finite_sequence(
        reduced, "static_wheel_loads_n", context=reduced_context, length=4
    )
    unsprung = _required_finite_sequence(
        reduced, "unsprung_mass_kg", context=reduced_context, length=4
    )
    if any(value <= 0.0 for value in unsprung):
        raise ValueError(f"{reduced_context}.unsprung_mass_kg must be positive.")

    fixed_unsprung = sum(unsprung)
    scalar_comparisons = (
        (
            _required_finite_float(
                overrides,
                "rear_static_weight_fraction",
                context=override_context,
                positive=True,
            ),
            rear_fraction,
            "override rear static fraction",
        ),
        (
            _required_finite_float(
                overrides,
                "front_static_weight_fraction",
                context=override_context,
                positive=True,
            ),
            front_fraction,
            "override front static fraction",
        ),
        (
            _required_finite_float(
                overrides, "sprung_mass_kg", context=override_context, positive=True
            ),
            sprung_mass,
            "override sprung mass",
        ),
        (
            _required_finite_float(
                overrides, "total_mass_kg", context=override_context, positive=True
            ),
            total_mass,
            "override total mass",
        ),
        (
            _required_finite_float(
                overrides,
                "fixed_unsprung_mass_kg",
                context=override_context,
                positive=True,
            ),
            fixed_unsprung,
            "override fixed unsprung mass",
        ),
        (
            _required_finite_float(
                reduced, "sprung_mass_kg", context=reduced_context, positive=True
            ),
            sprung_mass,
            "reduced sprung mass",
        ),
        (
            _required_finite_float(
                reduced, "mass_kg", context=reduced_context, positive=True
            ),
            total_mass,
            "reduced total mass",
        ),
        (
            _required_finite_float(
                vehicle, "mass", context=vehicle_context, positive=True
            ),
            total_mass,
            "outer total mass",
        ),
        (
            _required_finite_float(
                vehicle, "front_static_frac", context=vehicle_context, positive=True
            ),
            front_fraction,
            "outer front static fraction",
        ),
        (
            _required_finite_float(
                overrides, "aero_balance_front", context=override_context
            ),
            0.5,
            "override aero balance",
        ),
        (
            _required_finite_float(
                reduced, "aero_balance_front", context=reduced_context
            ),
            0.5,
            "reduced aero balance",
        ),
        (
            _required_finite_float(
                vehicle, "aero_balance_front", context=vehicle_context
            ),
            0.5,
            "outer aero balance",
        ),
        (
            _required_finite_float(
                reduced, "drive_distribution_front", context=reduced_context
            ),
            0.0,
            "reduced drive distribution",
        ),
        (
            _required_finite_float(
                vehicle, "drive_distribution_front", context=vehicle_context
            ),
            0.0,
            "outer drive distribution",
        ),
    )
    for actual, expected, label in scalar_comparisons:
        if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-8):
            raise ValueError(f"{context} {label} mismatch.")
    for label, cg in (("override", override_cg), ("reduced", reduced_cg)):
        if not math.isclose(cg[0], expected_cg_x, rel_tol=0.0, abs_tol=1e-10):
            raise ValueError(f"{context} {label} CG x mismatch.")
        if not math.isclose(
            cg[2],
            EXPECTED_LONGITUDINAL_CG_HEIGHT_IN * 0.0254,
            rel_tol=0.0,
            abs_tol=1e-10,
        ):
            raise ValueError(f"{context} {label} CG height mismatch.")

    expected_loads = (
        0.5 * front_fraction * total_mass * G_MPS2,
        0.5 * front_fraction * total_mass * G_MPS2,
        0.5 * rear_fraction * total_mass * G_MPS2,
        0.5 * rear_fraction * total_mass * G_MPS2,
    )
    if not np.allclose(static_loads, expected_loads, rtol=0.0, atol=1e-8):
        raise ValueError(f"{context} reduced static wheel loads mismatch.")
    front_axle_global_x = reduced_cg[0] + float(np.mean(corners[:2, 0]))
    rear_axle_global_x = reduced_cg[0] + float(np.mean(corners[2:, 0]))
    if not math.isclose(
        front_axle_global_x,
        EXPECTED_FRONT_AXLE_GLOBAL_X_M,
        rel_tol=0.0,
        abs_tol=1e-10,
    ) or not math.isclose(
        rear_axle_global_x,
        EXPECTED_REAR_AXLE_GLOBAL_X_M,
        rel_tol=0.0,
        abs_tol=1e-10,
    ):
        raise ValueError(f"{context} global axle locations moved with CG.")

    aero_cop = _required_finite_sequence(
        reduced, "aero_cop_m", context=reduced_context, length=3
    )
    expected_cop_global_x = 0.5 * (
        EXPECTED_FRONT_AXLE_GLOBAL_X_M + EXPECTED_REAR_AXLE_GLOBAL_X_M
    )
    if not math.isclose(
        reduced_cg[0] + aero_cop[0],
        expected_cop_global_x,
        rel_tol=0.0,
        abs_tol=1e-10,
    ):
        raise ValueError(f"{context} global 50/50 aero CoP x mismatch.")

    return {
        "cg_height_in": height_in,
        "sprung_mass_kg": sprung_mass,
        "total_mass_kg": total_mass,
        "fixed_unsprung_mass_kg": fixed_unsprung,
        "rear_static_weight_fraction": rear_fraction,
        "front_static_weight_fraction": front_fraction,
        "cg_x_m": expected_cg_x,
        "is_reference_case": bool(metadata.get("is_reference_case", False)),
    }


def validate_completed_sweep_metadata(
    metadata_rows: list[dict[str, Any]],
    *,
    sweep_definition: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Validate controlled mass or longitudinal-CG completed designs.

    When a longitudinal-CG source definition is supplied, its declared case
    names, rear-static fractions, and reference case are authoritative.  The
    metadata-only path remains available for auditing legacy completed output.
    """

    axes = {str(metadata.get("sweep_axis", "cg_height_in")) for metadata in metadata_rows}
    if len(axes) != 1:
        raise ValueError(f"Completed output root mixes sweep axes: {sorted(axes)}.")
    sweep_axis = next(iter(axes))
    if sweep_definition is not None:
        declared_axis = str(sweep_definition.get("sweep_axis", "cg_height_in"))
        if declared_axis != sweep_axis:
            raise ValueError(
                "Sweep definition axis does not match completed case metadata: "
                f"{declared_axis!r} != {sweep_axis!r}."
            )
    if sweep_axis == "cg_height_in":
        return {"sweep_axis": sweep_axis, "mass_sweep_validation": None}
    if sweep_axis == LONGITUDINAL_CG_SWEEP_AXIS:
        validated_longitudinal: list[tuple[str, dict[str, float]]] = []
        for metadata in metadata_rows:
            item = _longitudinal_cg_case_metadata(metadata)
            if item is not None:
                validated_longitudinal.append((str(metadata["cg_case"]), item))
        if len(validated_longitudinal) < 2:
            raise ValueError(
                "Longitudinal-CG audit requires at least two completed cases."
            )
        case_names = [case_name for case_name, _item in validated_longitudinal]
        if len(set(case_names)) != len(case_names):
            raise ValueError("Longitudinal-CG completed case names must be unique.")
        fractions = sorted(
            item["rear_static_weight_fraction"]
            for _case_name, item in validated_longitudinal
        )
        steps = [right - left for left, right in pairwise(fractions)]
        if any(step <= 1e-10 for step in steps):
            raise ValueError(
                "Longitudinal-CG completed rear static fractions must be unique."
            )
        fraction_step = steps[0]
        if any(
            not math.isclose(step, fraction_step, rel_tol=0.0, abs_tol=1e-10)
            for step in steps[1:]
        ):
            raise ValueError(
                "Longitudinal-CG completed rear static fractions must form an "
                "evenly spaced grid."
            )

        actual_by_name = dict(validated_longitudinal)
        declared_reference: str | None = None
        validation_source = "completed_case_metadata"
        if sweep_definition is not None:
            raw_cases = sweep_definition.get("cases")
            if not isinstance(raw_cases, list):
                raise TypeError("Longitudinal-CG sweep definition must contain a cases list.")
            if len(raw_cases) < 2:
                raise ValueError(
                    "Longitudinal-CG sweep definition must contain at least two cases."
                )
            declared_by_name: dict[str, float] = {}
            for index, raw_case in enumerate(raw_cases):
                if not isinstance(raw_case, dict):
                    raise TypeError(
                        f"Longitudinal-CG declared case {index} must be an object."
                    )
                name = raw_case.get("name")
                if not isinstance(name, str) or not name:
                    raise ValueError(
                        f"Longitudinal-CG declared case {index} has no valid name."
                    )
                if name in declared_by_name:
                    raise ValueError(
                        f"Longitudinal-CG sweep definition repeats case name {name!r}."
                    )
                rear_fraction = _required_finite_float(
                    raw_case,
                    "rear_static_weight_fraction",
                    context=f"Declared case {name}",
                    positive=True,
                )
                if rear_fraction >= 1.0:
                    raise ValueError(
                        f"Declared case {name} rear static fraction must be inside (0, 1)."
                    )
                declared_by_name[name] = rear_fraction
            declared_fractions = sorted(declared_by_name.values())
            declared_steps = [
                right - left
                for left, right in pairwise(declared_fractions)
            ]
            if any(step <= 1e-10 for step in declared_steps):
                raise ValueError(
                    "Longitudinal-CG declared rear static fractions must be unique."
                )
            declared_step = declared_steps[0]
            if any(
                not math.isclose(
                    step,
                    declared_step,
                    rel_tol=0.0,
                    abs_tol=1e-10,
                )
                for step in declared_steps[1:]
            ):
                raise ValueError(
                    "Longitudinal-CG declared rear static fractions must form an "
                    "evenly spaced grid."
                )
            if set(actual_by_name) != set(declared_by_name):
                missing = sorted(set(declared_by_name) - set(actual_by_name))
                unexpected = sorted(set(actual_by_name) - set(declared_by_name))
                raise ValueError(
                    "Completed longitudinal-CG cases do not match the sweep "
                    f"definition; missing={missing}, unexpected={unexpected}."
                )
            for name, expected_fraction in declared_by_name.items():
                actual_fraction = actual_by_name[name][
                    "rear_static_weight_fraction"
                ]
                if not math.isclose(
                    actual_fraction,
                    expected_fraction,
                    rel_tol=0.0,
                    abs_tol=1e-10,
                ):
                    raise ValueError(
                        f"Completed case {name} rear static fraction does not "
                        "match the sweep definition."
                    )
            declared_reference = str(sweep_definition.get("reference_case", ""))
            if declared_reference not in declared_by_name:
                raise ValueError(
                    "Longitudinal-CG sweep definition reference_case does not "
                    "name a declared case."
                )
            validation_source = "sweep_json"

        references = [
            (case_name, item)
            for case_name, item in validated_longitudinal
            if item["is_reference_case"]
        ]
        if len(references) != 1:
            raise ValueError(
                "Longitudinal-CG sweep must have exactly one completed reference case."
            )
        reference_name, reference_item = references[0]
        if declared_reference is not None and reference_name != declared_reference:
            raise ValueError(
                "Completed longitudinal-CG reference case does not match the "
                "sweep definition."
            )
        return {
            "sweep_axis": LONGITUDINAL_CG_SWEEP_AXIS,
            "mass_sweep_validation": None,
            "longitudinal_cg_sweep_validation": {
                "case_count": len(validated_longitudinal),
                "fixed_cg_height_in": EXPECTED_LONGITUDINAL_CG_HEIGHT_IN,
                "rear_static_weight_fractions": fractions,
                "rear_static_weight_fraction_step": fraction_step,
                "reference_case": reference_name,
                "reference_rear_static_weight_fraction": reference_item[
                    "rear_static_weight_fraction"
                ],
                "declared_fraction_validation_source": validation_source,
                "nominal_sprung_mass_kg": EXPECTED_NOMINAL_SPRUNG_MASS_KG,
                "nominal_total_mass_kg": EXPECTED_NOMINAL_TOTAL_MASS_KG,
                "cg_x_m": [
                    item["cg_x_m"]
                    for item in sorted(
                        (item for _case_name, item in validated_longitudinal),
                        key=lambda item: item["rear_static_weight_fraction"],
                    )
                ],
            },
        }
    if sweep_axis != MASS_SWEEP_AXIS:
        raise ValueError(f"Unsupported reduced-QSS sweep_axis {sweep_axis!r}.")

    mass_rows = [_mass_case_metadata(metadata) for metadata in metadata_rows]
    if any(item is None for item in mass_rows):
        raise ValueError("Completed output root mixes CG-height and sprung-mass cases.")
    validated = [item for item in mass_rows if item is not None]
    if len(validated) != len(EXPECTED_MASS_SWEEP_OFFSETS_LB):
        raise ValueError(
            "Sprung-mass audit requires exactly five completed cases at "
            "-100, -50, 0, +50, and +100 lb."
        )
    offsets = sorted(item["mass_offset_lb"] for item in validated)
    if any(
        not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-8)
        for actual, expected in zip(offsets, EXPECTED_MASS_SWEEP_OFFSETS_LB)
    ):
        raise ValueError(
            "Sprung-mass audit offsets must be exactly -100, -50, 0, +50, "
            f"and +100 lb; found {offsets}."
        )
    unsprung = [item["fixed_unsprung_mass_kg"] for item in validated]
    if max(unsprung) - min(unsprung) > 1e-8:
        raise ValueError("Explicit unsprung mass is not fixed across the mass sweep.")
    nominal = next(
        item for item in validated if math.isclose(item["mass_offset_lb"], 0.0)
    )
    for item in validated:
        expected_sprung = (
            nominal["sprung_mass_kg"] + item["mass_offset_lb"] * LB_TO_KG
        )
        if not math.isclose(
            item["sprung_mass_kg"], expected_sprung, rel_tol=0.0, abs_tol=1e-8
        ):
            raise ValueError(
                "Serialized sprung mass does not match the declared pound offset."
            )
    return {
        "sweep_axis": MASS_SWEEP_AXIS,
        "mass_sweep_validation": {
            "case_count": len(validated),
            "fixed_cg_height_in": EXPECTED_MASS_SWEEP_HEIGHT_IN,
            "mass_offsets_lb": offsets,
            "nominal_sprung_mass_kg": nominal["sprung_mass_kg"],
            "fixed_unsprung_mass_kg": unsprung[0],
        },
    }


def _validate_reconstructed_mass(
    parameters: Any,
    serialized: dict[str, float] | None,
    *,
    case_name: str,
) -> None:
    if serialized is None:
        return
    reconstructed = {
        "sprung_mass_kg": float(parameters.sprung_mass_kg),
        "total_mass_kg": float(parameters.mass_kg),
        "fixed_unsprung_mass_kg": float(sum(parameters.unsprung_mass_kg)),
    }
    for field, actual in reconstructed.items():
        if not math.isclose(
            actual, serialized[field], rel_tol=0.0, abs_tol=1e-8
        ):
            raise ValueError(
                f"Case {case_name} reconstructed {field}={actual:.12g} does not "
                f"match metadata {serialized[field]:.12g}."
            )


def _validate_reconstructed_longitudinal_cg(
    parameters: Any,
    serialized: dict[str, float] | None,
    *,
    case_name: str,
) -> None:
    if serialized is None:
        return
    actual_values = {
        "sprung_mass_kg": float(parameters.sprung_mass_kg),
        "total_mass_kg": float(parameters.mass_kg),
        "fixed_unsprung_mass_kg": float(sum(parameters.unsprung_mass_kg)),
        "rear_static_weight_fraction": float(
            parameters.static_rear_weight_fraction
        ),
        "front_static_weight_fraction": float(
            parameters.static_front_weight_fraction
        ),
        "cg_x_m": float(parameters.center_of_gravity_m[0]),
    }
    for field, actual in actual_values.items():
        if not math.isclose(
            actual, serialized[field], rel_tol=0.0, abs_tol=1e-8
        ):
            raise ValueError(
                f"Case {case_name} reconstructed {field}={actual:.12g} does not "
                f"match metadata {serialized[field]:.12g}."
            )
    expected_loads = np.asarray(
        (
            0.5
            * serialized["front_static_weight_fraction"]
            * serialized["total_mass_kg"]
            * G_MPS2,
            0.5
            * serialized["front_static_weight_fraction"]
            * serialized["total_mass_kg"]
            * G_MPS2,
            0.5
            * serialized["rear_static_weight_fraction"]
            * serialized["total_mass_kg"]
            * G_MPS2,
            0.5
            * serialized["rear_static_weight_fraction"]
            * serialized["total_mass_kg"]
            * G_MPS2,
        )
    )
    if not np.allclose(
        np.asarray(parameters.static_wheel_loads_n, dtype=float),
        expected_loads,
        rtol=0.0,
        atol=1e-8,
    ):
        raise ValueError(f"Case {case_name} reconstructed static loads mismatch.")


def compare_resolution_outputs(
    primary_root: Path,
    comparison_root: Path,
) -> pd.DataFrame:
    """Join two completed grids and report raw time/point differences."""

    columns = ["model_key", "cg_case", "event_slug"]
    primary = pd.read_csv(primary_root / "event_results.csv")
    comparison = pd.read_csv(comparison_root / "event_results.csv")
    required = set(columns + ["lap_time_s", "projected_points"])
    for label, frame in (("primary", primary), ("comparison", comparison)):
        missing = required.difference(frame.columns)
        if missing:
            raise ValueError(f"{label} resolution output is missing {sorted(missing)}")
    joined = primary[columns + ["lap_time_s", "projected_points"]].merge(
        comparison[columns + ["lap_time_s", "projected_points"]],
        on=columns,
        how="inner",
        validate="one_to_one",
        suffixes=("_primary", "_comparison"),
    )
    joined["primary_minus_comparison_time_s"] = (
        joined["lap_time_s_primary"] - joined["lap_time_s_comparison"]
    )
    joined["primary_minus_comparison_time_pct"] = 100.0 * (
        joined["lap_time_s_primary"] / joined["lap_time_s_comparison"] - 1.0
    )
    joined["primary_minus_comparison_points"] = (
        joined["projected_points_primary"] - joined["projected_points_comparison"]
    )
    return joined.sort_values(columns).reset_index(drop=True)


def run(args: argparse.Namespace) -> dict[str, Any]:
    output_root = args.output_root.resolve()
    bobsim_root = args.bobsim_root.resolve()
    audit_dir = (
        (output_root / "reduced_qss_audit")
        if args.audit_output_dir is None
        else args.audit_output_dir.resolve()
    )
    audit_dir.mkdir(parents=True, exist_ok=True)
    for source in (bobsim_root, Path(__file__).resolve().parent):
        if str(source) not in sys.path:
            sys.path.insert(0, str(source))

    from _0_Utils.dyn_py import (
        ReducedVehicleOverrides,
        apply_reduced_vehicle_overrides,
        create_model,
        load_reduced_vehicle_parameters,
    )

    base_parameters = load_reduced_vehicle_parameters(bobsim_root / "vehicle.yml")
    case_dirs = sorted(
        path for path in output_root.iterdir() if (path / "case_metadata.json").exists()
    )
    if not case_dirs:
        raise FileNotFoundError(f"No completed case directories found in {output_root}.")

    case_entries = [
        (
            case_dir,
            json.loads((case_dir / "case_metadata.json").read_text(encoding="utf-8")),
        )
        for case_dir in case_dirs
    ]
    sweep_json = getattr(args, "sweep_json", None)
    sweep_definition = None
    if sweep_json is not None:
        sweep_definition = json.loads(
            Path(sweep_json).resolve().read_text(encoding="utf-8")
        )
        if not isinstance(sweep_definition, dict):
            raise TypeError("Sweep JSON root must be an object.")
    sweep_validation = validate_completed_sweep_metadata(
        [metadata for _case_dir, metadata in case_entries],
        sweep_definition=sweep_definition,
    )

    sampled_rows: list[dict[str, Any]] = []
    case_summaries: list[dict[str, Any]] = []
    audited_tire_mu_scales: set[float] = set()
    for case_dir, metadata in case_entries:
        tire_mu_scale = _validate_case_metadata(
            metadata,
            model_dof=int(args.model_dof),
        )
        audited_tire_mu_scales.add(tire_mu_scale)
        case_name = str(metadata["cg_case"])
        mass_case = _mass_case_metadata(metadata)
        longitudinal_case = _longitudinal_cg_case_metadata(metadata)
        overrides = ReducedVehicleOverrides(
            absolute_cg_height_m=float(metadata["cg_height_m"]),
            target_sprung_mass_kg=(
                None if mass_case is None else mass_case["sprung_mass_kg"]
            ),
            static_rear_weight_fraction=(
                None
                if longitudinal_case is None
                else longitudinal_case["rear_static_weight_fraction"]
            ),
            aero_balance_front=float(metadata["effective_aero_balance_front"]),
            brake_distribution_front=float(
                metadata["effective_brake_distribution_front"]
            ),
            front_antiroll_stiffness_fraction=float(
                metadata["front_antiroll_stiffness_fraction"]
            ),
            tire_mu_scale=tire_mu_scale,
        )
        parameters = apply_reduced_vehicle_overrides(base_parameters, overrides)
        _validate_reconstructed_mass(parameters, mass_case, case_name=case_name)
        _validate_reconstructed_longitudinal_cg(
            parameters,
            longitudinal_case,
            case_name=case_name,
        )
        model = create_model(int(args.model_dof), parameters)

        traces = _load_case_traces(case_dir)
        states = select_trace_states(
            traces,
            maximum_samples=int(args.max_trace_samples_per_case),
            speed_bins=int(args.speed_bins),
            lateral_bins=int(args.lateral_bins),
            longitudinal_bins=int(args.longitudinal_bins),
        )
        states.extend(
            select_boundary_states(
                case_dir / "ggv.csv",
                pure_lateral_speed_mps=float(args.pure_lateral_speed_mps),
            )
        )
        case_rows: list[dict[str, Any]] = []
        for sample_index, state in enumerate(states):
            print(
                f"{case_name} {sample_index + 1}/{len(states)} "
                f"{state['source_kind']}",
                flush=True,
            )
            row = audit_trim_state(
                model,
                state,
                model_dof=int(args.model_dof),
                max_nfev=int(args.trim_max_nfev),
                tolerance=float(args.trim_tolerance),
            )
            row.update(
                {
                    "model_family": str(metadata["model_family"]),
                    "model_key": str(metadata["model_key"]),
                    "cg_case": case_name,
                    "cg_height_m": float(metadata["cg_height_m"]),
                    "cg_height_in": float(metadata["cg_height_in"]),
                    "sweep_axis": str(
                        metadata.get("sweep_axis", "cg_height_in")
                    ),
                    "sprung_mass_kg": float(parameters.sprung_mass_kg),
                    "total_mass_kg": float(parameters.mass_kg),
                    "fixed_unsprung_mass_kg": float(
                        sum(parameters.unsprung_mass_kg)
                    ),
                    "mass_offset_lb": (
                        math.nan if mass_case is None else mass_case["mass_offset_lb"]
                    ),
                    "rear_static_weight_fraction": float(
                        parameters.static_rear_weight_fraction
                    ),
                    "front_static_weight_fraction": float(
                        parameters.static_front_weight_fraction
                    ),
                    "cg_x_m": float(parameters.center_of_gravity_m[0]),
                    "front_antiroll_stiffness_fraction": float(
                        parameters.front_antiroll_stiffness_fraction
                    ),
                    "front_elastic_roll_stiffness_fraction": float(
                        parameters.front_roll_stiffness_fraction
                    ),
                    "brake_distribution_front": float(
                        parameters.brake_distribution_front
                    ),
                    "aero_balance_front": float(parameters.aero_balance_front),
                    "tire_mu_scale": tire_mu_scale,
                    "tir_valid_load_min_n": float(parameters.tire.fz_min_n),
                    "tir_valid_load_max_n": float(parameters.tire.fz_max_n),
                    "sample_index": sample_index,
                }
            )
            case_rows.append(row)
        sampled_rows.extend(case_rows)
        case_summaries.append(
            {
                "model_family": str(metadata["model_family"]),
                "model_key": str(metadata["model_key"]),
                "model_dof": int(args.model_dof),
                "cg_case": case_name,
                "cg_height_m": float(metadata["cg_height_m"]),
                "cg_height_in": float(metadata["cg_height_in"]),
                "sweep_axis": str(metadata.get("sweep_axis", "cg_height_in")),
                "sprung_mass_kg": float(parameters.sprung_mass_kg),
                "total_mass_kg": float(parameters.mass_kg),
                "fixed_unsprung_mass_kg": float(sum(parameters.unsprung_mass_kg)),
                "mass_offset_lb": (
                    math.nan if mass_case is None else mass_case["mass_offset_lb"]
                ),
                "rear_static_weight_fraction": float(
                    parameters.static_rear_weight_fraction
                ),
                "front_static_weight_fraction": float(
                    parameters.static_front_weight_fraction
                ),
                "cg_x_m": float(parameters.center_of_gravity_m[0]),
                "front_antiroll_stiffness_fraction": float(
                    parameters.front_antiroll_stiffness_fraction
                ),
                "front_elastic_roll_stiffness_fraction": float(
                    parameters.front_roll_stiffness_fraction
                ),
                "brake_distribution_front": float(
                    parameters.brake_distribution_front
                ),
                "tire_mu_scale": tire_mu_scale,
                "tir_valid_load_min_n": float(parameters.tire.fz_min_n),
                "tir_valid_load_max_n": float(parameters.tire.fz_max_n),
                **summarize_trim_rows(case_rows),
            }
        )

    sampled_frame = pd.DataFrame(sampled_rows)
    summary_frame = pd.DataFrame(case_summaries)
    sampled_frame.to_csv(audit_dir / "sampled_trim_states.csv", index=False)
    summary_frame.to_csv(audit_dir / "case_qss_audit_summary.csv", index=False)

    resolution_payload = None
    if args.resolution_compare_root is not None:
        comparison = compare_resolution_outputs(
            output_root,
            args.resolution_compare_root.resolve(),
        )
        comparison.to_csv(audit_dir / "resolution_comparison.csv", index=False)
        resolution_payload = comparison.to_dict(orient="records")

    payload = {
        "schema": "lapsims.reduced-qss-sampled-audit.v1",
        "output_root": output_root.as_posix(),
        "model_dof": int(args.model_dof),
        **sweep_validation,
        "tire_mu_scales": sorted(audited_tire_mu_scales),
        "sampling": {
            "method": (
                "deterministic one-state-per occupied speed/signed-lateral/"
                "longitudinal-utilization/branch bin, evenly thinned to the "
                "configured cap, plus trace extrema and three GGV boundary states"
            ),
            "maximum_binned_trace_samples_per_case": int(
                args.max_trace_samples_per_case
            ),
            "speed_bins": int(args.speed_bins),
            "lateral_bins": int(args.lateral_bins),
            "longitudinal_bins": int(args.longitudinal_bins),
            "pure_lateral_speed_mps": float(args.pure_lateral_speed_mps),
            "approximation_warning": (
                "Counts are occurrences among sampled QSS target states only. "
                "They are not exact segment, time, or distance exposure counts, "
                "and unsampled transient peaks may exist."
            ),
        },
        "load_thresholds_n": {
            "below": DEFAULT_LOAD_MIN_N,
            "above": DEFAULT_LOAD_MAX_N,
            "wheel_lift_at_or_below": 0.0,
        },
        "parameter_reconstruction": {
            "source": (
                "active BobSim vehicle.yml and raw TIR coefficients, with the "
                "case-metadata tire_mu_scale applied through "
                "ReducedVehicleOverrides"
            ),
            "study_overrides": (
                "absolute CG height, 50/50 aero balance with preserved global "
                "CoP z, optional target sprung mass with fixed unsprung hardware, "
                "optional total-static rear fraction with fixed global axles and "
                "aero hardware, "
                "fixed front brake fraction, fixed front ARB fraction, and finite "
                "positive tire mu scale"
            ),
            "tire_mu_scale_source": "case_metadata.tire_mu_scale",
            "permanent_vehicle_model_modified": False,
        },
        "case_summaries": case_summaries,
        "sampled_trim_states": sampled_rows,
        "resolution_comparison": resolution_payload,
    }
    _write_json(audit_dir / "reduced_qss_audit.json", payload)
    print(json.dumps({"case_summaries": case_summaries}, indent=2), flush=True)
    return payload


def main() -> None:
    run(parse_args())


if __name__ == "__main__":
    main()
