"""Bounded reduced-QSS sampler for the coupled two-configuration study.

This is deliberately a study-local wrapper around the mature state-selection
and trim-audit primitives in :mod:`audit_reduced_qss_sweep`.  The older entry
point only accepts one-axis mass or longitudinal-CG sweeps.  Here both total CG
height and longitudinal static balance are varied together, so the completed
metadata is validated as a categorical configuration comparison before either
model is reconstructed.

The script does not generate GGV maps or run laps.  It only solves a bounded,
deterministic set of representative QSS trim states from already-completed
production traces and GGV boundaries.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from audit_reduced_qss_sweep import (
    _load_case_traces,
    _write_json,
    audit_trim_state,
    select_boundary_states,
    select_trace_states,
    summarize_trim_rows,
)

LAPSIMS_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = LAPSIMS_ROOT.parents[2]
DEFAULT_BOBSIM_ROOT = WORKSPACE_ROOT / "BobDyn" / "BobSim"
CONFIGURATION_AXIS = "configuration"
MODEL_FAMILY = "dyn_py_reduced_order_qss"
MODEL_KEY = "dyn_py_6dof_qss"
EXPECTED_MODEL_DOF = 6
EXPECTED_MU_SCALE = 0.6225437130779028
EXPECTED_AERO_BALANCE_FRONT = 0.5
EXPECTED_SPRUNG_MASS_KG = 230.72288408
EXPECTED_TOTAL_MASS_KG = 261.07265114
EXPECTED_WHEELBASE_M = 1.5494
EXPECTED_FRONT_AXLE_X_M = 0.0
EXPECTED_REAR_AXLE_X_M = -EXPECTED_WHEELBASE_M
G_MPS2 = 9.80665
INCH_TO_M = 0.0254
EXPECTED_CASES: tuple[tuple[str, float, float], ...] = (
    ("config_nominal_54", 11.5, 0.54),
    ("config_plus0p25_56p5", 11.75, 0.565),
)
EXPECTED_EVENTS = {
    "acceleration",
    "skidpad",
    "autocross",
    "michigan_endurance",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Sample existing two-configuration production traces with the "
            "pinned 6DOF reduced-QSS model."
        )
    )
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--bobsim-root", type=Path, default=DEFAULT_BOBSIM_ROOT)
    parser.add_argument("--sweep-json", type=Path, required=True)
    parser.add_argument("--audit-output-dir", type=Path, default=None)
    parser.add_argument("--max-trace-samples-per-case", type=int, default=36)
    parser.add_argument("--speed-bins", type=int, default=6)
    parser.add_argument("--lateral-bins", type=int, default=7)
    parser.add_argument("--longitudinal-bins", type=int, default=4)
    parser.add_argument("--pure-lateral-speed-mps", type=float, default=11.8)
    parser.add_argument("--trim-max-nfev", type=int, default=220)
    parser.add_argument("--trim-tolerance", type=float, default=1e-7)
    return parser.parse_args()


def _read_json(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8-sig"))
    if not isinstance(payload, dict):
        raise TypeError(f"{path} must contain a JSON object.")
    return payload


def _finite_float(
    mapping: Mapping[str, Any],
    field: str,
    *,
    context: str,
    positive: bool = False,
) -> float:
    value = mapping.get(field)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{context}.{field} must be a finite number.")
    number = float(value)
    if not math.isfinite(number) or (positive and number <= 0.0):
        qualifier = "finite positive" if positive else "finite"
        raise ValueError(f"{context}.{field} must be {qualifier}.")
    return number


def _close(
    actual: float,
    expected: float,
    *,
    context: str,
    tolerance: float = 1e-9,
) -> None:
    if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=tolerance):
        raise ValueError(
            f"{context} mismatch: expected {expected:.12g}, got {actual:.12g}."
        )


def _case_design(case_name: str) -> tuple[float, float]:
    for expected_name, height_in, rear_fraction in EXPECTED_CASES:
        if case_name == expected_name:
            return height_in, rear_fraction
    raise ValueError(f"Unexpected configuration name {case_name!r}.")


def _validate_tuning(
    payload: Mapping[str, Any],
    *,
    context: str,
) -> dict[str, float]:
    front_arb = _finite_float(
        payload,
        "front_antiroll_stiffness_fraction",
        context=context,
    )
    brake_field = (
        "brake_distribution_front"
        if "brake_distribution_front" in payload
        else "effective_brake_distribution_front"
    )
    brake = _finite_float(payload, brake_field, context=context)
    if not 0.0 <= front_arb <= 1.0:
        raise ValueError(f"{context} front ARB fraction must be within [0, 1].")
    if not 0.0 <= brake <= 1.0:
        raise ValueError(f"{context} front brake fraction must be within [0, 1].")
    return {
        "front_antiroll_stiffness_fraction": front_arb,
        "brake_distribution_front": brake,
    }


def validate_configuration_sweep(
    sweep: Mapping[str, Any],
) -> dict[str, dict[str, float]]:
    """Validate the exact coupled design and return authoritative case tuning."""

    if str(sweep.get("sweep_axis")) != CONFIGURATION_AXIS:
        raise ValueError("Two-configuration QSS audit requires sweep_axis=configuration.")
    if int(sweep.get("model_dof", -1)) != EXPECTED_MODEL_DOF:
        raise ValueError("Two-configuration QSS audit requires model_dof=6.")
    if str(sweep.get("reference_case")) != EXPECTED_CASES[0][0]:
        raise ValueError("The nominal 54% rear case must be the reference case.")
    _close(
        _finite_float(sweep, "aero_balance_front", context="sweep"),
        EXPECTED_AERO_BALANCE_FRONT,
        context="sweep aero balance",
    )
    _close(
        _finite_float(sweep, "tire_mu_scale", context="sweep", positive=True),
        EXPECTED_MU_SCALE,
        context="sweep tire mu scale",
    )
    drive_model = str(sweep.get("drive_model", "")).lower()
    if "rwd" not in drive_model or "no lsd" not in drive_model:
        raise ValueError("Sweep must explicitly declare RWD with no LSD model.")

    cases = sweep.get("cases")
    if not isinstance(cases, list) or len(cases) != len(EXPECTED_CASES):
        raise ValueError("Sweep must contain exactly the two requested configurations.")
    observed_names = [str(case.get("name")) for case in cases if isinstance(case, dict)]
    expected_names = [case[0] for case in EXPECTED_CASES]
    if observed_names != expected_names:
        raise ValueError(
            f"Configuration order/names must be exactly {expected_names}; "
            f"got {observed_names}."
        )

    tuning: dict[str, dict[str, float]] = {}
    for case, (name, height_in, rear_fraction) in zip(cases, EXPECTED_CASES):
        assert isinstance(case, dict)
        context = f"sweep case {name}"
        _close(
            _finite_float(case, "cg_height_in", context=context, positive=True),
            height_in,
            context=f"{context} CG height",
        )
        _close(
            _finite_float(
                case,
                "rear_static_weight_fraction",
                context=context,
                positive=True,
            ),
            rear_fraction,
            context=f"{context} rear fraction",
        )
        optional_expected = {
            "front_static_weight_fraction": 1.0 - rear_fraction,
            "cg_x_m": -rear_fraction * EXPECTED_WHEELBASE_M,
            "sprung_mass_kg": EXPECTED_SPRUNG_MASS_KG,
            "total_mass_kg": EXPECTED_TOTAL_MASS_KG,
        }
        for field, expected in optional_expected.items():
            if field in case:
                _close(
                    _finite_float(case, field, context=context),
                    expected,
                    context=f"{context} {field}",
                    tolerance=1e-8,
                )
        tuning[name] = _validate_tuning(case, context=context)
    return tuning


def _numeric_sequence(
    payload: Mapping[str, Any],
    field: str,
    length: int,
    *,
    context: str,
) -> np.ndarray:
    value = payload.get(field)
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{context}.{field} must contain {length} values.")
    values = np.asarray(value, dtype=float)
    if not np.all(np.isfinite(values)):
        raise ValueError(f"{context}.{field} must be finite.")
    return values


def validate_configuration_metadata(
    metadata: Mapping[str, Any],
    *,
    expected_tuning: Mapping[str, Mapping[str, float]],
) -> dict[str, float]:
    """Validate serialized geometry, mass, setup, and force-distribution identity."""

    case_name = str(metadata.get("cg_case"))
    height_in, rear_fraction = _case_design(case_name)
    front_fraction = 1.0 - rear_fraction
    expected_cg_x = -rear_fraction * EXPECTED_WHEELBASE_M
    expected_cg_z = height_in * INCH_TO_M
    context = f"case metadata {case_name}"

    scalar_expected = {
        "model_dof": float(EXPECTED_MODEL_DOF),
        "cg_height_in": height_in,
        "cg_height_m": expected_cg_z,
        "sprung_mass_kg": EXPECTED_SPRUNG_MASS_KG,
        "total_mass_kg": EXPECTED_TOTAL_MASS_KG,
        "rear_static_weight_fraction": rear_fraction,
        "front_static_weight_fraction": front_fraction,
        "effective_aero_balance_front": EXPECTED_AERO_BALANCE_FRONT,
        "drive_distribution_front": 0.0,
        "tire_mu_scale": EXPECTED_MU_SCALE,
    }
    for field, expected in scalar_expected.items():
        _close(
            _finite_float(metadata, field, context=context),
            expected,
            context=f"{context} {field}",
            tolerance=1e-8,
        )
    if str(metadata.get("sweep_axis")) != CONFIGURATION_AXIS:
        raise ValueError(f"{context} has the wrong sweep axis.")
    if str(metadata.get("model_family")) != MODEL_FAMILY:
        raise ValueError(f"{context} has the wrong model family.")
    if str(metadata.get("model_key")) != MODEL_KEY:
        raise ValueError(f"{context} has the wrong model key.")
    if metadata.get("limited_slip_differential_model") is not False:
        raise ValueError(f"{context} must explicitly disable the LSD model.")

    authoritative_tuning = expected_tuning[case_name]
    serialized_tuning = _validate_tuning(metadata, context=context)
    for field, expected in authoritative_tuning.items():
        _close(
            serialized_tuning[field],
            expected,
            context=f"{context} {field}",
            tolerance=1e-10,
        )
    case_tuning = metadata.get("case_tuning")
    if not isinstance(case_tuning, Mapping):
        raise TypeError(f"{context}.case_tuning must be an object.")
    nested_tuning = _validate_tuning(case_tuning, context=f"{context}.case_tuning")
    for field, expected in authoritative_tuning.items():
        _close(
            nested_tuning[field],
            expected,
            context=f"{context}.case_tuning {field}",
            tolerance=1e-10,
        )

    overrides = metadata.get("reduced_parameter_overrides")
    reduced = metadata.get("reduced_vehicle_parameters")
    vehicle = metadata.get("vehicle")
    if not isinstance(overrides, Mapping):
        raise TypeError(f"{context}.reduced_parameter_overrides must be an object.")
    if not isinstance(reduced, Mapping):
        raise TypeError(f"{context}.reduced_vehicle_parameters must be an object.")
    if not isinstance(vehicle, Mapping):
        raise TypeError(f"{context}.vehicle must be an object.")

    for block_name, block, fields in (
        (
            "overrides",
            overrides,
            {
                "sprung_mass_kg": EXPECTED_SPRUNG_MASS_KG,
                "total_mass_kg": EXPECTED_TOTAL_MASS_KG,
                "rear_static_weight_fraction": rear_fraction,
                "front_static_weight_fraction": front_fraction,
                "aero_balance_front": EXPECTED_AERO_BALANCE_FRONT,
                "brake_distribution_front": authoritative_tuning[
                    "brake_distribution_front"
                ],
                "front_antiroll_stiffness_fraction": authoritative_tuning[
                    "front_antiroll_stiffness_fraction"
                ],
            },
        ),
        (
            "reduced",
            reduced,
            {
                "sprung_mass_kg": EXPECTED_SPRUNG_MASS_KG,
                "mass_kg": EXPECTED_TOTAL_MASS_KG,
                "aero_balance_front": EXPECTED_AERO_BALANCE_FRONT,
                "drive_distribution_front": 0.0,
                "brake_distribution_front": authoritative_tuning[
                    "brake_distribution_front"
                ],
            },
        ),
        (
            "vehicle",
            vehicle,
            {
                "mass": EXPECTED_TOTAL_MASS_KG,
                "front_static_frac": front_fraction,
                "aero_balance_front": EXPECTED_AERO_BALANCE_FRONT,
                "drive_distribution_front": 0.0,
                "brake_distribution_front": authoritative_tuning[
                    "brake_distribution_front"
                ],
            },
        ),
    ):
        for field, expected in fields.items():
            _close(
                _finite_float(block, field, context=f"{context}.{block_name}"),
                expected,
                context=f"{context}.{block_name}.{field}",
                tolerance=1e-8,
            )

    override_cg = _numeric_sequence(
        overrides, "center_of_gravity_m", 3, context=f"{context}.overrides"
    )
    reduced_cg = _numeric_sequence(
        reduced, "center_of_gravity_m", 3, context=f"{context}.reduced"
    )
    for label, cg in (("override", override_cg), ("reduced", reduced_cg)):
        _close(cg[0], expected_cg_x, context=f"{context} {label} CG x", tolerance=1e-10)
        _close(cg[2], expected_cg_z, context=f"{context} {label} CG z", tolerance=1e-10)

    static_loads = _numeric_sequence(
        reduced, "static_wheel_loads_n", 4, context=f"{context}.reduced"
    )
    expected_loads = np.asarray(
        (
            0.5 * front_fraction * EXPECTED_TOTAL_MASS_KG * G_MPS2,
            0.5 * front_fraction * EXPECTED_TOTAL_MASS_KG * G_MPS2,
            0.5 * rear_fraction * EXPECTED_TOTAL_MASS_KG * G_MPS2,
            0.5 * rear_fraction * EXPECTED_TOTAL_MASS_KG * G_MPS2,
        )
    )
    if not np.allclose(static_loads, expected_loads, rtol=0.0, atol=1e-8):
        raise ValueError(f"{context} reduced static wheel loads mismatch.")
    corners = _numeric_sequence(
        {"corner_positions_m": [
            item
            for corner in reduced.get("corner_positions_m", [])
            if isinstance(corner, (list, tuple))
            for item in corner
        ]},
        "corner_positions_m",
        12,
        context=f"{context}.reduced",
    ).reshape(4, 3)
    front_axle_x = reduced_cg[0] + float(np.mean(corners[:2, 0]))
    rear_axle_x = reduced_cg[0] + float(np.mean(corners[2:, 0]))
    _close(front_axle_x, EXPECTED_FRONT_AXLE_X_M, context=f"{context} front axle x")
    _close(rear_axle_x, EXPECTED_REAR_AXLE_X_M, context=f"{context} rear axle x")
    aero_cop = _numeric_sequence(reduced, "aero_cop_m", 3, context=f"{context}.reduced")
    _close(
        reduced_cg[0] + aero_cop[0],
        -0.5 * EXPECTED_WHEELBASE_M,
        context=f"{context} global aero CoP x",
        tolerance=1e-10,
    )
    return {
        "cg_height_in": height_in,
        "cg_height_m": expected_cg_z,
        "rear_static_weight_fraction": rear_fraction,
        "front_static_weight_fraction": front_fraction,
        "cg_x_m": expected_cg_x,
        "front_antiroll_stiffness_fraction": authoritative_tuning[
            "front_antiroll_stiffness_fraction"
        ],
        "brake_distribution_front": authoritative_tuning[
            "brake_distribution_front"
        ],
    }


def _ensure_event_coverage(
    traces: pd.DataFrame,
    selected: list[dict[str, Any]],
    *,
    speed_bins: int,
    lateral_bins: int,
    longitudinal_bins: int,
) -> list[dict[str, Any]]:
    """Add one deterministic state for any event absent after global thinning."""

    output = list(selected)
    present = {str(state["event_slug"]) for state in output}
    available = set(traces["event_slug"].astype(str))
    missing = sorted(EXPECTED_EVENTS.intersection(available).difference(present))
    for event_slug in missing:
        candidates = select_trace_states(
            traces.loc[traces["event_slug"].astype(str) == event_slug].copy(),
            maximum_samples=1,
            speed_bins=speed_bins,
            lateral_bins=lateral_bins,
            longitudinal_bins=longitudinal_bins,
        )
        if not candidates:
            raise ValueError(f"No valid QSS trace state available for {event_slug}.")
        candidate = dict(candidates[0])
        candidate["selection_reasons"] = sorted(
            set(candidate["selection_reasons"] + ["event_coverage"])
        )
        output.append(candidate)
    return output


def _validate_reconstructed(parameters: Any, design: Mapping[str, float], case_name: str) -> None:
    actual = {
        "cg_height_m": float(parameters.center_of_gravity_m[2]),
        "cg_x_m": float(parameters.center_of_gravity_m[0]),
        "sprung_mass_kg": float(parameters.sprung_mass_kg),
        "total_mass_kg": float(parameters.mass_kg),
        "rear_static_weight_fraction": float(parameters.static_rear_weight_fraction),
        "front_static_weight_fraction": float(parameters.static_front_weight_fraction),
        "front_antiroll_stiffness_fraction": float(
            parameters.front_antiroll_stiffness_fraction
        ),
        "brake_distribution_front": float(parameters.brake_distribution_front),
        "aero_balance_front": float(parameters.aero_balance_front),
        "drive_distribution_front": float(parameters.drive_distribution_front),
    }
    expected = {
        **design,
        "sprung_mass_kg": EXPECTED_SPRUNG_MASS_KG,
        "total_mass_kg": EXPECTED_TOTAL_MASS_KG,
        "aero_balance_front": EXPECTED_AERO_BALANCE_FRONT,
        "drive_distribution_front": 0.0,
    }
    expected.pop("cg_height_in", None)
    for field, expected_value in expected.items():
        _close(
            actual[field],
            float(expected_value),
            context=f"reconstructed {case_name} {field}",
            tolerance=1e-8,
        )


def run(args: argparse.Namespace) -> dict[str, Any]:
    output_root = Path(args.output_root).resolve()
    bobsim_root = Path(args.bobsim_root).resolve()
    sweep_path = Path(args.sweep_json).resolve()
    audit_dir = (
        output_root / "reduced_qss_audit"
        if args.audit_output_dir is None
        else Path(args.audit_output_dir).resolve()
    )
    if not output_root.is_dir():
        raise FileNotFoundError(output_root)
    if not (bobsim_root / "vehicle.yml").is_file():
        raise FileNotFoundError(bobsim_root / "vehicle.yml")
    for source in (bobsim_root, Path(__file__).resolve().parent):
        if str(source) not in sys.path:
            sys.path.insert(0, str(source))

    from _0_Utils.dyn_py import (
        ReducedVehicleOverrides,
        apply_reduced_vehicle_overrides,
        create_model,
        load_reduced_vehicle_parameters,
    )

    sweep = _read_json(sweep_path)
    tuning = validate_configuration_sweep(sweep)
    base_parameters = load_reduced_vehicle_parameters(bobsim_root / "vehicle.yml")
    _close(
        float(base_parameters.sprung_mass_kg),
        EXPECTED_SPRUNG_MASS_KG,
        context="pinned source nominal sprung mass",
        tolerance=1e-8,
    )
    _close(
        float(base_parameters.mass_kg),
        EXPECTED_TOTAL_MASS_KG,
        context="pinned source nominal total mass",
        tolerance=1e-8,
    )

    expected_dirs = [output_root / name for name, _height, _rear in EXPECTED_CASES]
    observed_dirs = sorted(
        path.name
        for path in output_root.iterdir()
        if path.is_dir() and (path / "case_metadata.json").is_file()
    )
    if observed_dirs != sorted(path.name for path in expected_dirs):
        raise ValueError(
            "Completed output root must contain exactly the two requested case "
            f"directories; got {observed_dirs}."
        )

    sampled_rows: list[dict[str, Any]] = []
    case_summaries: list[dict[str, Any]] = []
    audit_dir.mkdir(parents=True, exist_ok=True)
    for case_dir in expected_dirs:
        metadata = _read_json(case_dir / "case_metadata.json")
        design = validate_configuration_metadata(metadata, expected_tuning=tuning)
        overrides = ReducedVehicleOverrides(
            absolute_cg_height_m=design["cg_height_m"],
            static_rear_weight_fraction=design["rear_static_weight_fraction"],
            aero_balance_front=EXPECTED_AERO_BALANCE_FRONT,
            brake_distribution_front=design["brake_distribution_front"],
            front_antiroll_stiffness_fraction=design[
                "front_antiroll_stiffness_fraction"
            ],
            tire_mu_scale=EXPECTED_MU_SCALE,
        )
        parameters = apply_reduced_vehicle_overrides(base_parameters, overrides)
        _validate_reconstructed(parameters, design, case_dir.name)
        model = create_model(EXPECTED_MODEL_DOF, parameters)

        traces = _load_case_traces(case_dir)
        observed_events = set(traces["event_slug"].astype(str))
        if observed_events != EXPECTED_EVENTS:
            raise ValueError(
                f"{case_dir} trace event set mismatch: {sorted(observed_events)}."
            )
        states = select_trace_states(
            traces,
            maximum_samples=int(args.max_trace_samples_per_case),
            speed_bins=int(args.speed_bins),
            lateral_bins=int(args.lateral_bins),
            longitudinal_bins=int(args.longitudinal_bins),
        )
        states = _ensure_event_coverage(
            traces,
            states,
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
                f"{case_dir.name} {sample_index + 1}/{len(states)} "
                f"{state['source_kind']}",
                flush=True,
            )
            row = audit_trim_state(
                model,
                state,
                model_dof=EXPECTED_MODEL_DOF,
                max_nfev=int(args.trim_max_nfev),
                tolerance=float(args.trim_tolerance),
            )
            row.update(
                {
                    "model_family": MODEL_FAMILY,
                    "model_key": MODEL_KEY,
                    "cg_case": case_dir.name,
                    "cg_height_m": design["cg_height_m"],
                    "cg_height_in": design["cg_height_in"],
                    "sweep_axis": CONFIGURATION_AXIS,
                    "sprung_mass_kg": float(parameters.sprung_mass_kg),
                    "total_mass_kg": float(parameters.mass_kg),
                    "fixed_unsprung_mass_kg": float(sum(parameters.unsprung_mass_kg)),
                    "mass_offset_lb": math.nan,
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
                    "tire_mu_scale": EXPECTED_MU_SCALE,
                    "tir_valid_load_min_n": float(parameters.tire.fz_min_n),
                    "tir_valid_load_max_n": float(parameters.tire.fz_max_n),
                    "sample_index": sample_index,
                }
            )
            case_rows.append(row)
        sampled_rows.extend(case_rows)
        case_summaries.append(
            {
                "model_family": MODEL_FAMILY,
                "model_key": MODEL_KEY,
                "model_dof": EXPECTED_MODEL_DOF,
                "cg_case": case_dir.name,
                "cg_height_m": design["cg_height_m"],
                "cg_height_in": design["cg_height_in"],
                "sweep_axis": CONFIGURATION_AXIS,
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
                "front_antiroll_stiffness_fraction": float(
                    parameters.front_antiroll_stiffness_fraction
                ),
                "front_elastic_roll_stiffness_fraction": float(
                    parameters.front_roll_stiffness_fraction
                ),
                "brake_distribution_front": float(
                    parameters.brake_distribution_front
                ),
                "tire_mu_scale": EXPECTED_MU_SCALE,
                "tir_valid_load_min_n": float(parameters.tire.fz_min_n),
                "tir_valid_load_max_n": float(parameters.tire.fz_max_n),
                **summarize_trim_rows(case_rows),
            }
        )

    pd.DataFrame(sampled_rows).to_csv(
        audit_dir / "sampled_trim_states.csv", index=False
    )
    pd.DataFrame(case_summaries).to_csv(
        audit_dir / "case_qss_audit_summary.csv", index=False
    )
    payload = {
        "schema": "lapsims.two-config-reduced-qss-sampled-audit.v1",
        "output_root": output_root.as_posix(),
        "bobsim_root": bobsim_root.as_posix(),
        "model_dof": EXPECTED_MODEL_DOF,
        "sweep_axis": CONFIGURATION_AXIS,
        "reference_case": EXPECTED_CASES[0][0],
        "configuration_validation": {
            "cases": [
                {
                    "name": name,
                    "cg_height_in": height,
                    "rear_static_weight_fraction": rear,
                    **tuning[name],
                }
                for name, height, rear in EXPECTED_CASES
            ],
            "categorical_not_one_axis_sensitivity": True,
            "nominal_mass_preserved": True,
            "rwd_without_lsd": True,
            "aero_balance_front": EXPECTED_AERO_BALANCE_FRONT,
            "tire_mu_scale": EXPECTED_MU_SCALE,
        },
        "sampling": {
            "maximum_binned_trace_samples_per_case": int(
                args.max_trace_samples_per_case
            ),
            "speed_bins": int(args.speed_bins),
            "lateral_bins": int(args.lateral_bins),
            "longitudinal_bins": int(args.longitudinal_bins),
            "pure_lateral_speed_mps": float(args.pure_lateral_speed_mps),
            "trim_max_nfev": int(args.trim_max_nfev),
            "trim_tolerance": float(args.trim_tolerance),
            "event_coverage_enforced": sorted(EXPECTED_EVENTS),
            "approximation_warning": (
                "Counts describe deterministic sampled QSS target states, not "
                "time/distance exposure or transient peaks."
            ),
        },
        "parameter_reconstruction": {
            "source_vehicle_yaml": str((bobsim_root / "vehicle.yml").resolve()),
            "overrides": (
                "absolute CG height, total-static rear fraction, 50/50 aero, "
                "case-tuned front ARB split and brake bias, calibrated tire mu"
            ),
            "target_sprung_mass_override": None,
            "nominal_mass_policy": True,
            "nominal_total_mass_kg": float(base_parameters.mass_kg),
            "nominal_sprung_mass_kg": float(base_parameters.sprung_mass_kg),
        },
        "case_summaries": case_summaries,
        "sampled_trim_states": sampled_rows,
    }
    _write_json(audit_dir / "reduced_qss_audit.json", payload)
    print(json.dumps({"case_summaries": case_summaries}, indent=2), flush=True)
    return payload


def main() -> None:
    run(parse_args())


if __name__ == "__main__":
    main()
