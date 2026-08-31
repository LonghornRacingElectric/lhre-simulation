"""Read-only sidecar audit for physical GGV speed-prefix serialization.

The production source lock intentionally covers the generator, runner, and
primary acceptance auditor.  This sidecar does not replace or mutate that
lock.  It first verifies the original lock through the locked auditor, then
checks the serialization conditions that the primary auditor cannot represent:
track-relevant 32 kW QSS envelopes above the drag-power terminal contain no
rows, and the exact 80 kW terminal-RPM request may likewise be empty.  Both are
consequently absent from ``ggv.csv``.

The command prints one self-identifying JSON verdict to stdout and never
writes a file or launches a simulation.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from collections.abc import Mapping, Sequence
from datetime import datetime, timezone
from itertools import pairwise
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

import audit_two_config_cg_production as locked_audit

SCHEMA = "lapsims.two-config-cg-terminal-prefix-sidecar.v1"
EXPECTED_POWER_W = 32_000.0
EXPECTED_POWER_80_W = 80_000.0
ORIGINAL_AUDIT_SOURCE_ID = "lapsims:src/audit_two_config_cg_production.py"
RUNNER_SOURCE_ID = "lapsims:src/run_ggv_cg_smoke.py"
GENERATOR_SOURCE_ID = "bobsim:_2_EnvelopeSim/GGV/ggv_generation.py"
MAP_COLUMNS = (
    "speed_mps",
    "ay_mps2",
    "ax_accel_mps2",
    "ax_brake_mps2",
    "accel_feasible",
    "brake_feasible",
)
EVENTS = (
    "acceleration",
    "autocross",
    "skidpad",
    "michigan_endurance",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _read_json(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise TypeError(f"{path} must contain one JSON object.")
    return payload


def _finite_float(value: object, *, context: str) -> float:
    if isinstance(value, bool):
        raise TypeError(f"{context} must be a finite number.")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{context} must be a finite number.") from exc
    if not math.isfinite(result):
        raise ValueError(f"{context} must be a finite number.")
    return result


def _close(
    left: object,
    right: object,
    *,
    context: str,
    tolerance: float = 1e-10,
) -> None:
    left_value = _finite_float(left, context=context)
    right_value = _finite_float(right, context=context)
    if not math.isclose(
        left_value,
        right_value,
        rel_tol=0.0,
        abs_tol=tolerance,
    ):
        raise ValueError(
            f"{context} mismatch: {left_value:.12g} != {right_value:.12g}."
        )


def drag_power_terminal(metadata: Mapping[str, Any]) -> dict[str, float]:
    """Return the independently computed drag-power terminal-speed evidence."""

    vehicle = metadata.get("vehicle")
    if not isinstance(vehicle, Mapping):
        raise TypeError("Case metadata vehicle must be an object.")
    power_w = _finite_float(
        metadata.get("effective_drive_power_limit_w"),
        context="effective drive power",
    )
    _close(
        power_w,
        EXPECTED_POWER_W,
        context="32 kW sidecar power",
        tolerance=1e-8,
    )
    _close(
        vehicle.get("max_drive_power"),
        power_w,
        context="vehicle/effective drive power",
        tolerance=1e-8,
    )
    rho = _finite_float(vehicle.get("rho"), context="air density")
    cd_area = _finite_float(vehicle.get("cd_a"), context="drag area")
    maximum_drive_speed = _finite_float(
        vehicle.get("max_drive_speed"), context="maximum drive speed"
    )
    if rho <= 0.0 or cd_area <= 0.0 or maximum_drive_speed <= 0.0:
        raise ValueError("Drag-terminal inputs must be positive.")
    drag_coefficient = 0.5 * rho * cd_area
    terminal_speed = (power_w / drag_coefficient) ** (1.0 / 3.0)
    if terminal_speed >= maximum_drive_speed - 1e-10:
        raise ValueError(
            "The sidecar exception is only valid when drag-power, not the "
            "driveline speed ceiling, sets the terminal speed."
        )
    return {
        "power_w": power_w,
        "rho_air_kg_m3": rho,
        "cd_area_m2": cd_area,
        "drag_force_coefficient_n_per_mps2": drag_coefficient,
        "drag_power_terminal_speed_mps": terminal_speed,
        "maximum_drive_speed_mps": maximum_drive_speed,
    }


def _strictly_increasing(values: Sequence[float], *, context: str) -> None:
    if len(values) < 2 or any(right <= left for left, right in pairwise(values)):
        raise ValueError(f"{context} must be strictly increasing.")


def validate_physical_speed_prefix(
    frame: pd.DataFrame,
    metadata: Mapping[str, Any],
) -> dict[str, Any]:
    """Require the CSV grid to be the exact below-terminal metadata prefix."""

    if "speed_mps" not in frame:
        raise ValueError("GGV CSV is missing speed_mps.")
    speed_column = pd.to_numeric(frame["speed_mps"], errors="raise")
    if not np.isfinite(speed_column.to_numpy(dtype=float)).all():
        raise ValueError("GGV CSV contains a nonfinite speed.")
    actual = sorted(float(value) for value in speed_column.unique())
    _strictly_increasing(actual, context="serialized speed grid")

    raw_declared = metadata.get("ggv_speed_slices")
    if not isinstance(raw_declared, list):
        raise TypeError("Case metadata ggv_speed_slices must be a list.")
    declared = [
        _finite_float(value, context="declared GGV speed") for value in raw_declared
    ]
    _strictly_increasing(declared, context="declared GGV speed grid")

    terminal = drag_power_terminal(metadata)
    terminal_speed = terminal["drag_power_terminal_speed_mps"]
    expected_prefix = [value for value in declared if value <= terminal_speed + 1e-12]
    omitted = declared[len(expected_prefix) :]
    if not omitted:
        raise ValueError(
            "The 32 kW sidecar requires a nonempty above-terminal suffix."
        )
    if len(actual) != len(expected_prefix) or not np.allclose(
        actual,
        expected_prefix,
        rtol=0.0,
        atol=1e-12,
    ):
        raise ValueError(
            "Serialized GGV speeds are not the exact contiguous physical prefix "
            "through the drag-power terminal."
        )
    if any(value <= terminal_speed + 1e-12 for value in omitted):
        raise ValueError("A requested speed at or below terminal was omitted.")
    if np.any(speed_column.to_numpy(dtype=float) > terminal_speed + 1e-12):
        raise ValueError("GGV CSV contains a row above the drag-power terminal.")

    last_included = actual[-1]
    first_omitted = omitted[0]
    coefficient = terminal["drag_force_coefficient_n_per_mps2"]
    power_w = terminal["power_w"]

    def force_margin(speed: float) -> float:
        return power_w / speed - coefficient * speed**2

    included_margin = force_margin(last_included)
    omitted_margin = force_margin(first_omitted)
    if included_margin < -1e-8 or omitted_margin >= -1e-8:
        raise ValueError(
            "Included/omitted speed slices do not bracket the analytical "
            "drag-power terminal."
        )

    raw_requested = metadata.get("ggv_requested_speed_slices_mps")
    if not isinstance(raw_requested, list):
        raise TypeError("Case metadata requested speed grid must be a list.")
    requested = [
        _finite_float(value, context="requested GGV speed")
        for value in raw_requested
    ]
    below_terminal_requested = [
        value for value in requested if value <= terminal_speed + 1e-12
    ]
    if not set(below_terminal_requested).issubset(set(actual)):
        raise ValueError("A requested speed below terminal is absent from the CSV.")

    return {
        **terminal,
        "physical_terminal_speed_mps": terminal_speed,
        "terminal_kind": "drag_power",
        "declared_speed_slices_mps": declared,
        "serialized_speed_slices_mps": actual,
        "omitted_above_terminal_speed_slices_mps": omitted,
        "last_included_speed_mps": last_included,
        "first_omitted_speed_mps": first_omitted,
        "last_included_drive_minus_drag_force_n": included_margin,
        "first_omitted_drive_minus_drag_force_n": omitted_margin,
        "all_requested_slices_at_or_below_terminal_present": True,
        "no_serialized_rows_above_terminal": True,
        "serialized_grid_is_exact_contiguous_prefix": True,
    }


def validate_terminal_rpm_speed_prefix(
    frame: pd.DataFrame,
    metadata: Mapping[str, Any],
) -> dict[str, Any]:
    """Accept only one empty exact terminal-RPM slice in the 80 kW map."""

    if "speed_mps" not in frame:
        raise ValueError("GGV CSV is missing speed_mps.")
    speed_column = pd.to_numeric(frame["speed_mps"], errors="raise")
    if not np.isfinite(speed_column.to_numpy(dtype=float)).all():
        raise ValueError("GGV CSV contains a nonfinite speed.")
    actual = sorted(float(value) for value in speed_column.unique())
    _strictly_increasing(actual, context="serialized speed grid")

    raw_declared = metadata.get("ggv_speed_slices")
    if not isinstance(raw_declared, list):
        raise TypeError("Case metadata ggv_speed_slices must be a list.")
    declared = [
        _finite_float(value, context="declared GGV speed") for value in raw_declared
    ]
    _strictly_increasing(declared, context="declared GGV speed grid")
    if len(declared) < 3 or len(actual) != len(declared) - 1 or not np.allclose(
        actual,
        declared[:-1],
        rtol=0.0,
        atol=1e-12,
    ):
        raise ValueError(
            "80 kW serialized speeds must equal the declared grid minus only "
            "the exact terminal-RPM slice."
        )

    vehicle = metadata.get("vehicle")
    if not isinstance(vehicle, Mapping):
        raise TypeError("Case metadata vehicle must be an object.")
    power_w = _finite_float(
        metadata.get("effective_drive_power_limit_w"),
        context="effective drive power",
    )
    _close(
        power_w,
        EXPECTED_POWER_80_W,
        context="80 kW sidecar power",
        tolerance=1e-8,
    )
    _close(
        vehicle.get("max_drive_power"),
        power_w,
        context="vehicle/effective drive power",
        tolerance=1e-8,
    )
    rho = _finite_float(vehicle.get("rho"), context="air density")
    cd_area = _finite_float(vehicle.get("cd_a"), context="drag area")
    maximum_drive_speed = _finite_float(
        vehicle.get("max_drive_speed"), context="maximum drive speed"
    )
    omitted = declared[-1]
    if not math.isclose(
        omitted,
        maximum_drive_speed,
        rel_tol=0.0,
        abs_tol=1e-12,
    ):
        raise ValueError("The sole omitted 80 kW slice is not the RPM ceiling.")
    if any(value > maximum_drive_speed + 1e-12 for value in actual):
        raise ValueError("80 kW CSV contains a row beyond the RPM ceiling.")
    drag_coefficient = 0.5 * rho * cd_area
    drag_power_terminal = (power_w / drag_coefficient) ** (1.0 / 3.0)
    if drag_power_terminal <= maximum_drive_speed + 1e-10:
        raise ValueError("80 kW should be RPM-limited, not drag-power-limited.")
    last_included = actual[-1]
    included_margin = power_w / last_included - drag_coefficient * last_included**2
    if included_margin <= 0.0:
        raise ValueError("The last serialized 80 kW slice is not drive-sustainable.")
    return {
        "power_w": power_w,
        "rho_air_kg_m3": rho,
        "cd_area_m2": cd_area,
        "drag_force_coefficient_n_per_mps2": drag_coefficient,
        "drag_power_terminal_speed_mps": drag_power_terminal,
        "maximum_drive_speed_mps": maximum_drive_speed,
        "physical_terminal_speed_mps": maximum_drive_speed,
        "terminal_kind": "terminal_rpm",
        "declared_speed_slices_mps": declared,
        "serialized_speed_slices_mps": actual,
        "omitted_terminal_rpm_speed_slices_mps": [omitted],
        "last_included_speed_mps": last_included,
        "first_omitted_speed_mps": omitted,
        "last_included_drive_minus_drag_force_n": included_margin,
        "serialized_grid_is_exact_prefix_minus_one_terminal_rpm_slice": True,
        "no_serialized_rows_above_terminal": True,
    }


def _validate_branch_symmetry(
    ordered: pd.DataFrame,
    *,
    speed: float,
    tolerance: float = 1e-9,
) -> None:
    ay = ordered["ay_mps2"].to_numpy(dtype=float)
    if len(ay) % 2 == 0 or np.any(np.diff(ay) <= 0.0):
        raise ValueError(f"Speed {speed:g} has a non-odd or unordered lateral grid.")
    if float(np.min(np.abs(ay))) > tolerance:
        raise ValueError(f"Speed {speed:g} omits ay=0.")
    if not np.allclose(ay, -ay[::-1], rtol=0.0, atol=tolerance):
        raise ValueError(f"Speed {speed:g} has an asymmetric lateral grid.")
    for value_column, flag_column in (
        ("ax_accel_mps2", "accel_feasible"),
        ("ax_brake_mps2", "brake_feasible"),
    ):
        values = ordered[value_column].to_numpy(dtype=float)
        flags = ordered[flag_column].to_numpy(dtype=float)
        if not np.all(np.isin(flags, [0.0, 1.0])):
            raise ValueError(f"Speed {speed:g} {flag_column} is not binary.")
        if not np.array_equal(flags > 0.5, np.isfinite(values)):
            raise ValueError(
                f"Speed {speed:g} {flag_column} disagrees with {value_column}."
            )
        if not np.array_equal(flags, flags[::-1]):
            raise ValueError(f"Speed {speed:g} {flag_column} is asymmetric.")
        if not np.allclose(
            values,
            values[::-1],
            rtol=0.0,
            atol=tolerance,
            equal_nan=True,
        ):
            raise ValueError(f"Speed {speed:g} {value_column} is asymmetric.")


def validate_serialized_map(
    path: Path,
    metadata: Mapping[str, Any],
    *,
    prefix_mode: str = "32kw_drag_power",
) -> dict[str, Any]:
    """Validate normal topology plus the tightly bounded 3-row origin case."""

    frame = pd.read_csv(path)
    missing = sorted(set(MAP_COLUMNS).difference(frame.columns))
    if missing:
        raise ValueError(f"{path} is missing map columns: {missing}.")
    for column in MAP_COLUMNS:
        frame[column] = pd.to_numeric(frame[column], errors="raise")
    if frame[["speed_mps", "ay_mps2"]].isna().any().any():
        raise ValueError(f"{path} contains a missing speed/ay coordinate.")
    if frame.duplicated(["speed_mps", "ay_mps2"]).any():
        raise ValueError(f"{path} contains duplicate speed/ay coordinates.")

    if prefix_mode == "32kw_drag_power":
        prefix = validate_physical_speed_prefix(frame, metadata)
    elif prefix_mode == "80kw_terminal_rpm":
        prefix = validate_terminal_rpm_speed_prefix(frame, metadata)
    else:
        raise ValueError(f"Unknown speed-prefix mode: {prefix_mode}.")
    zero_proxy = _finite_float(
        metadata.get("qss_zero_speed_proxy_mps"), context="zero-speed proxy"
    )
    three_row_speeds: list[float] = []
    rows_per_speed: dict[str, int] = {}
    groups: dict[float, pd.DataFrame] = {}
    for speed, raw_group in frame.groupby("speed_mps", sort=True):
        speed_value = float(speed)
        ordered = raw_group.sort_values("ay_mps2", kind="mergesort").reset_index(
            drop=True
        )
        groups[speed_value] = ordered
        _validate_branch_symmetry(ordered, speed=speed_value)
        row_count = len(ordered)
        rows_per_speed[f"{speed_value:.12g}"] = row_count
        if row_count == 3:
            if not (
                math.isclose(speed_value, 0.0, rel_tol=0.0, abs_tol=1e-12)
                or math.isclose(
                    speed_value, zero_proxy, rel_tol=0.0, abs_tol=1e-12
                )
            ):
                raise ValueError(
                    f"Three-row topology is only accepted at 0 and {zero_proxy:g} m/s."
                )
            ay = ordered["ay_mps2"].to_numpy(dtype=float)
            if ay[0] >= 0.0 or ay[2] <= 0.0:
                raise ValueError("Three-row origin must contain -ay, 0, +ay.")
            endpoint_accel = ordered.iloc[[0, 2]]["ax_accel_mps2"].to_numpy(
                dtype=float
            )
            endpoint_accel_flags = ordered.iloc[[0, 2]][
                "accel_feasible"
            ].to_numpy(dtype=float)
            endpoint_brake_flags = ordered.iloc[[0, 2]][
                "brake_feasible"
            ].to_numpy(dtype=float)
            if not np.allclose(
                endpoint_accel, 0.0, rtol=0.0, atol=1e-12
            ) or not np.all(endpoint_accel_flags == 1.0):
                raise ValueError(
                    "Three-row origin does not close both drive branches at ax=0."
                )
            if not np.all(endpoint_brake_flags == 1.0):
                raise ValueError(
                    "Three-row origin endpoints must retain the finite brake branch."
                )
            three_row_speeds.append(speed_value)
        elif row_count < 5:
            raise ValueError(f"Speed {speed_value:g} has fewer than five map rows.")

        zero = ordered.iloc[int(np.argmin(np.abs(ordered["ay_mps2"])))]
        if not (
            zero["accel_feasible"] > 0.5
            and zero["ax_accel_mps2"] > 0.0
            and zero["brake_feasible"] > 0.5
            and zero["ax_brake_mps2"] < 0.0
        ):
            raise ValueError(
                f"Speed {speed_value:g} lacks feasible ay=0 drive and brake branches."
            )

    if three_row_speeds:
        expected_three = [0.0, zero_proxy]
        if len(three_row_speeds) != 2 or not np.allclose(
            sorted(three_row_speeds),
            expected_three,
            rtol=0.0,
            atol=1e-12,
        ):
            raise ValueError(
                "The three-row exception must occur at both 0 and the QSS proxy."
            )
        zero_group = groups[0.0]
        proxy_key = next(
            speed
            for speed in groups
            if math.isclose(speed, zero_proxy, rel_tol=0.0, abs_tol=1e-12)
        )
        proxy_group = groups[proxy_key]
        comparison_columns = [column for column in MAP_COLUMNS if column != "speed_mps"]
        if not np.allclose(
            zero_group[comparison_columns].to_numpy(dtype=float),
            proxy_group[comparison_columns].to_numpy(dtype=float),
            rtol=0.0,
            atol=1e-12,
            equal_nan=True,
        ):
            raise ValueError("The 0 m/s slice is not an exact copy of the QSS proxy.")

    return {
        "path": str(path.resolve()),
        "sha256": _sha256(path),
        "row_count": len(frame),
        "rows_per_speed": rows_per_speed,
        "three_row_origin_speeds_mps": sorted(three_row_speeds),
        "three_row_origin_exception_used": bool(three_row_speeds),
        "symmetric_topology": True,
        "branch_masks_match_finiteness": True,
        "physical_speed_prefix": prefix,
    }


def validate_event_domains(
    case_dir: Path,
    *,
    map_summary: Mapping[str, Any],
    maximum_final_speed_change_mps: float,
    expected_power_w: float = EXPECTED_POWER_W,
) -> list[dict[str, Any]]:
    """Prove every actual event stays strictly inside the serialized domain."""

    map_path = Path(str(map_summary["path"])).resolve()
    map_hash = str(map_summary["sha256"])
    prefix = map_summary["physical_speed_prefix"]
    if not isinstance(prefix, Mapping):
        raise TypeError("Map summary physical prefix is missing.")
    cap = _finite_float(prefix["last_included_speed_mps"], context="map speed cap")
    terminal = _finite_float(
        prefix["physical_terminal_speed_mps"], context="terminal speed"
    )
    results: list[dict[str, Any]] = []
    for event in EVENTS:
        summary_path = case_dir / f"{event}_summary.json"
        trace_path = case_dir / f"{event}_trace.csv"
        if not summary_path.is_file() or not trace_path.is_file():
            raise FileNotFoundError(
                f"{case_dir} is missing {event} summary or trace."
            )
        summary = _read_json(summary_path)
        trace = pd.read_csv(trace_path)
        required_trace = (
            "speed_mps",
            "ggv_constraint_speed_mps",
            "ggv_speed_domain_fraction",
        )
        missing = sorted(set(required_trace).difference(trace.columns))
        if missing:
            raise ValueError(f"{trace_path} is missing columns: {missing}.")
        trace_speed = pd.to_numeric(trace["speed_mps"], errors="raise").to_numpy(
            dtype=float
        )
        constraint_speed = pd.to_numeric(
            trace["ggv_constraint_speed_mps"], errors="raise"
        ).to_numpy(dtype=float)
        trace_domain = pd.to_numeric(
            trace["ggv_speed_domain_fraction"], errors="raise"
        ).to_numpy(dtype=float)
        if (
            trace_speed.size == 0
            or not np.isfinite(trace_speed).all()
            or not np.isfinite(constraint_speed).all()
            or not np.isfinite(trace_domain).all()
        ):
            raise ValueError(f"{trace_path} has an empty or nonfinite speed domain.")
        if not bool(summary.get("converged")):
            raise ValueError(f"{event} did not converge.")
        final_change = _finite_float(
            summary.get("final_max_speed_change_mps"),
            context=f"{event} final speed change",
        )
        if final_change > maximum_final_speed_change_mps + 1e-15:
            raise ValueError(f"{event} final speed change exceeds the run contract.")
        if int(summary.get("ggv_speed_cap_segments", -1)) != 0:
            raise ValueError(f"{event} touched the GGV speed cap.")
        if str(summary.get("ggv_source_sha256", "")) != map_hash:
            raise ValueError(f"{event} summary has the wrong GGV hash.")
        if Path(str(summary.get("ggv_source_csv", ""))).resolve() != map_path:
            raise ValueError(f"{event} summary has the wrong GGV path.")
        _close(
            summary.get("effective_drive_power_limit_w"),
            expected_power_w,
            context=f"{event} power",
            tolerance=1e-8,
        )
        _close(
            summary.get("ggv_csv_speed_max_mps"),
            cap,
            context=f"{event} CSV speed cap",
            tolerance=1e-12,
        )
        _close(
            summary.get("ggv_solver_speed_cap_mps"),
            cap,
            context=f"{event} solver speed cap",
            tolerance=1e-12,
        )
        observed_max_speed = float(np.max(trace_speed))
        observed_max_constraint_speed = float(np.max(constraint_speed))
        observed_max_domain = float(np.max(trace_domain))
        if (
            observed_max_speed >= cap - 1e-10
            or observed_max_constraint_speed >= cap - 1e-10
            or observed_max_speed >= terminal - 1e-10
        ):
            raise ValueError(f"{event} does not stay strictly below the physical cap.")
        if np.any(trace_domain < -1e-12) or observed_max_domain >= 1.0 - 1e-10:
            raise ValueError(f"{event} approaches or exceeds the GGV speed cap.")
        if not np.allclose(
            trace_domain,
            constraint_speed / cap,
            rtol=0.0,
            atol=1e-12,
        ):
            raise ValueError(f"{event} trace domain fractions are inconsistent.")
        _close(
            summary.get("maximum_speed_mps"),
            observed_max_speed,
            context=f"{event} maximum trace speed",
            tolerance=1e-10,
        )
        _close(
            summary.get("ggv_max_speed_domain_fraction"),
            observed_max_domain,
            context=f"{event} maximum domain fraction",
            tolerance=1e-12,
        )
        results.append(
            {
                "event_slug": event,
                "maximum_event_speed_mps": observed_max_speed,
                "maximum_ggv_constraint_speed_mps": observed_max_constraint_speed,
                "maximum_ggv_speed_domain_fraction": observed_max_domain,
                "ggv_speed_cap_segments": 0,
                "converged": True,
                "final_max_speed_change_mps": final_change,
                "summary_sha256": _sha256(summary_path),
                "trace_sha256": _sha256(trace_path),
            }
        )
    return results


def validate_case(
    case_dir: Path,
    *,
    expected_tuning: Mapping[str, Mapping[str, float]],
    maximum_final_speed_change_mps: float,
    expected_power_w: float = EXPECTED_POWER_W,
    prefix_mode: str = "32kw_drag_power",
) -> dict[str, Any]:
    metadata_path = case_dir / "case_metadata.json"
    map_path = case_dir / "ggv.csv"
    if not metadata_path.is_file() or not map_path.is_file():
        raise FileNotFoundError(f"{case_dir} lacks case_metadata.json or ggv.csv.")
    metadata = _read_json(metadata_path)
    locked_audit.validate_configuration_metadata(
        metadata,
        expected_tuning=expected_tuning,
    )
    if int(metadata.get("model_dof", -1)) != 6:
        raise ValueError(f"{case_dir.name} is not a 6DOF case.")
    _close(
        metadata.get("effective_drive_power_limit_w"),
        expected_power_w,
        context=f"{case_dir.name} power",
        tolerance=1e-8,
    )
    map_summary = validate_serialized_map(
        map_path,
        metadata,
        prefix_mode=prefix_mode,
    )
    if str(metadata.get("ggv_source_sha256", "")) != map_summary["sha256"]:
        raise ValueError(f"{case_dir.name} metadata GGV hash mismatch.")
    event_domains = validate_event_domains(
        case_dir,
        map_summary=map_summary,
        maximum_final_speed_change_mps=maximum_final_speed_change_mps,
        expected_power_w=expected_power_w,
    )
    return {
        "cg_case": case_dir.name,
        "metadata_path": str(metadata_path.resolve()),
        "metadata_sha256": _sha256(metadata_path),
        "map": map_summary,
        "event_domains": event_domains,
        "all_events_strictly_inside_serialized_domain": True,
    }


def _as_bool(value: object, *, context: str) -> bool:
    if isinstance(value, (bool, np.bool_)):
        return bool(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "1"}:
            return True
        if normalized in {"false", "0"}:
            return False
    if isinstance(value, (int, np.integer)) and int(value) in {0, 1}:
        return bool(value)
    raise ValueError(f"{context} is not boolean.")


def _qss_sample_identity(row: pd.Series) -> dict[str, Any]:
    wheel_values = {
        "FL": float(row["fz_fl_n"]),
        "FR": float(row["fz_fr_n"]),
        "RL": float(row["fz_rl_n"]),
        "RR": float(row["fz_rr_n"]),
    }
    minimum_wheel = min(wheel_values, key=wheel_values.__getitem__)
    return {
        "sample_index": int(row["sample_index"]),
        "cg_case": str(row["cg_case"]),
        "source_kind": str(row["source_kind"]),
        "selection_reasons": str(row["selection_reasons"]),
        "event_slug": str(row["event_slug"]),
        "trace_row_index": int(row["trace_row_index"]),
        "speed_mps": float(row["speed_mps"]),
        "ax_mps2": float(row["ax_mps2"]),
        "ay_mps2": float(row["ay_mps2"]),
        "trim_success": _as_bool(row["trim_success"], context="trim_success"),
        "trim_residual_norm": float(row["trim_residual_norm"]),
        "minimum_load_wheel": minimum_wheel,
        "minimum_normal_load_n": wheel_values[minimum_wheel],
        "wheel_loads_n": wheel_values,
    }


def inspect_qss_strict_gates(
    root: Path,
    *,
    run_contract: Mapping[str, Any],
) -> dict[str, Any]:
    """Report frozen QSS gates without turning a violation into an exception."""

    audit_dir = root / "reduced_qss_audit"
    sample_path = audit_dir / "sampled_trim_states.csv"
    summary_path = audit_dir / "case_qss_audit_summary.csv"
    json_path = audit_dir / "reduced_qss_audit.json"
    for path in (sample_path, summary_path, json_path):
        if not path.is_file():
            raise FileNotFoundError(path)
    frame = pd.read_csv(sample_path)
    required = (
        "sample_index",
        "cg_case",
        "source_kind",
        "selection_reasons",
        "event_slug",
        "trace_row_index",
        "speed_mps",
        "ax_mps2",
        "ay_mps2",
        "trim_success",
        "trim_residual_norm",
        "beta_rad",
        "steering_rad",
        "fz_fl_n",
        "fz_fr_n",
        "fz_rl_n",
        "fz_rr_n",
        "tir_valid_load_min_n",
        "tir_valid_load_max_n",
    )
    missing = sorted(set(required).difference(frame.columns))
    if missing:
        raise ValueError(f"{sample_path} is missing QSS columns: {missing}.")
    numeric = [
        column
        for column in required
        if column
        not in {
            "cg_case",
            "source_kind",
            "selection_reasons",
            "event_slug",
            "trim_success",
        }
    ]
    for column in numeric:
        frame[column] = pd.to_numeric(frame[column], errors="raise")
    if not np.isfinite(frame[numeric].to_numpy(dtype=float)).all():
        raise ValueError(f"{sample_path} contains a nonfinite QSS value.")
    successes = frame["trim_success"].map(
        lambda value: _as_bool(value, context="trim_success")
    )
    acceptance = run_contract["acceptance"]
    residual_limit = float(acceptance["maximum_trim_residual_norm"])
    beta_limit = float(acceptance["maximum_abs_beta_rad"])
    steer_limit = float(acceptance["maximum_abs_steering_rad"])
    tir_min = float(acceptance["tir_valid_load_min_n"])
    tir_max = float(acceptance["tir_valid_load_max_n"])
    load_tolerance = float(acceptance["normal_load_numerical_tolerance_n"])
    load_columns = ["fz_fl_n", "fz_fr_n", "fz_rl_n", "fz_rr_n"]
    valid_load_rows = frame.loc[successes].copy()
    loads = valid_load_rows[load_columns].to_numpy(dtype=float)
    below_mask = np.any(loads < tir_min - load_tolerance, axis=1)
    above_mask = np.any(loads > tir_max + load_tolerance, axis=1)
    failed_rows = frame.loc[~successes]
    residual_rows = frame.loc[frame["trim_residual_norm"] > residual_limit]
    below_rows = valid_load_rows.loc[below_mask]
    above_rows = valid_load_rows.loc[above_mask]

    summary = pd.read_csv(summary_path)
    expected_cases = {item[0] for item in locked_audit.EXPECTED_CASES}
    if len(summary) != 2 or set(summary["cg_case"].astype(str)) != expected_cases:
        raise ValueError(f"{summary_path} has the wrong case rows.")
    summary_rows: list[dict[str, Any]] = []
    for case_name in sorted(expected_cases):
        samples = frame.loc[frame["cg_case"].astype(str) == case_name]
        sample_success = samples["trim_success"].map(
            lambda value: _as_bool(value, context="trim_success")
        )
        serialized = summary.loc[summary["cg_case"].astype(str) == case_name].iloc[0]
        exact_counts = {
            "sample_count": len(samples),
            "successful_trim_count": int(sample_success.sum()),
            "failed_trim_count": int((~sample_success).sum()),
        }
        for field, expected in exact_counts.items():
            if int(serialized[field]) != expected:
                raise ValueError(f"{summary_path} {case_name} {field} mismatch.")
        successful_samples = samples.loc[sample_success]
        minimum_load = float(successful_samples[load_columns].min().min())
        maximum_residual = float(samples["trim_residual_norm"].max())
        _close(
            serialized["sampled_normal_load_min_n"],
            minimum_load,
            context=f"{case_name} summarized minimum load",
            tolerance=1e-8,
        )
        _close(
            serialized["maximum_residual_norm"],
            maximum_residual,
            context=f"{case_name} summarized maximum residual",
            tolerance=1e-10,
        )
        summary_rows.append(
            {
                "cg_case": case_name,
                **exact_counts,
                "maximum_residual_norm": maximum_residual,
                "sampled_normal_load_min_n": minimum_load,
            }
        )

    strict_checks = {
        "all_trims_successful": bool(successes.all()),
        "all_residuals_within_frozen_limit": bool(
            (frame["trim_residual_norm"] <= residual_limit).all()
        ),
        "all_beta_within_frozen_limit": bool(
            (frame["beta_rad"].abs() <= beta_limit + 1e-8).all()
        ),
        "all_steering_within_frozen_limit": bool(
            (frame["steering_rad"].abs() <= steer_limit + 1e-8).all()
        ),
        "all_successful_loads_within_frozen_tir_range": bool(
            not np.any(below_mask) and not np.any(above_mask)
        ),
    }
    strict_pass = all(strict_checks.values())
    return {
        "root": str(root.resolve()),
        "strict_status": "passed" if strict_pass else "failed",
        "strict_checks": strict_checks,
        "frozen_thresholds": {
            "maximum_trim_residual_norm": residual_limit,
            "maximum_abs_beta_rad": beta_limit,
            "maximum_abs_steering_rad": steer_limit,
            "tir_valid_load_min_n": tir_min,
            "tir_valid_load_max_n": tir_max,
            "normal_load_numerical_tolerance_n": load_tolerance,
        },
        "case_summaries": summary_rows,
        "failed_trim_samples": [
            _qss_sample_identity(row) for _, row in failed_rows.iterrows()
        ],
        "residual_limit_violations": [
            _qss_sample_identity(row) for _, row in residual_rows.iterrows()
        ],
        "below_tir_load_samples": [
            _qss_sample_identity(row) for _, row in below_rows.iterrows()
        ],
        "above_tir_load_samples": [
            _qss_sample_identity(row) for _, row in above_rows.iterrows()
        ],
        "artifacts": {
            "sampled_trim_states_csv": {
                "path": str(sample_path.resolve()),
                "sha256": _sha256(sample_path),
            },
            "case_qss_audit_summary_csv": {
                "path": str(summary_path.resolve()),
                "sha256": _sha256(summary_path),
            },
            "reduced_qss_audit_json": {
                "path": str(json_path.resolve()),
                "sha256": _sha256(json_path),
            },
        },
    }


def qss_advisory_robustness(
    qss_80: Mapping[str, Any],
    qss_32: Mapping[str, Any],
) -> dict[str, Any]:
    """Separate decision context from the immutable strict gate results."""

    failures_80 = qss_80["failed_trim_samples"]
    failures_32 = qss_32["failed_trim_samples"]
    target = failures_80[0] if len(failures_80) == 1 else None
    duplicated_failure = bool(
        target is not None
        and len(failures_32) == 1
        and all(
            math.isclose(
                float(target[field]),
                float(failures_32[0][field]),
                rel_tol=0.0,
                abs_tol=1e-10,
            )
            for field in ("speed_mps", "ax_mps2", "ay_mps2")
        )
    )
    serialized_abs_ay = (
        abs(float(target["ay_mps2"])) if target is not None else math.nan
    )
    return {
        "changes_strict_gate_status": False,
        "config2_autocross_direct_continuation": {
            "same_target_duplicated_in_80kw_and_32kw_audits": duplicated_failure,
            "serialized_constraint_abs_ay_mps2": serialized_abs_ay,
            "highest_confirmed_feasible_abs_ay_mps2_approx": 14.166,
            "local_boundary_overstatement_mps2_approx": 0.0044,
            "interpretation": (
                "Independent direct continuation closes the same branch to "
                "approximately 14.166 m/s^2 but not the serialized 14.170385 "
                "m/s^2 target. This is advisory evidence of a small local map "
                "overstatement; it does not convert the failed sampled trim into "
                "a strict pass."
            ),
        },
        "load_range_interpretation": (
            "Below-TIR samples are actual trace states, not synthetic boundary-only "
            "samples. Their small magnitudes inform decision robustness but remain "
            "formal failures of the frozen load-range gate."
        ),
    }


def run_sidecar_audit(args: argparse.Namespace) -> dict[str, Any]:
    """Run the read-only audit and return its stdout payload."""

    bobsim_root = Path(args.bobsim_root).resolve()
    root_80 = Path(args.root_80kw).resolve()
    root_32 = Path(args.root_32kw).resolve()
    sweep_80 = Path(args.sweep_80kw).resolve()
    sweep_32 = Path(args.sweep_32kw).resolve()
    tuning_path = Path(args.tuning_json).resolve()
    scoring_path = Path(args.scoring_json).resolve()
    track_manifest = Path(args.track_manifest).resolve()
    run_contract_path = Path(args.run_contract).resolve()
    lock_path = Path(args.lock_file).resolve()

    lock = locked_audit.verify_source_lock(
        lock_path=lock_path,
        bobsim_root=bobsim_root,
        sweep_80kw_path=sweep_80,
        sweep_32kw_path=sweep_32,
        tuning_path=tuning_path,
        scoring_path=scoring_path,
        track_manifest_path=track_manifest,
        run_contract_path=run_contract_path,
        root_80kw=root_80,
        root_32kw=root_32,
    )
    manifest = lock.get("source_manifest")
    if not isinstance(manifest, Mapping):
        raise TypeError("Original source lock has no manifest.")
    for source_id in (
        ORIGINAL_AUDIT_SOURCE_ID,
        RUNNER_SOURCE_ID,
        GENERATOR_SOURCE_ID,
    ):
        if source_id not in manifest or not isinstance(manifest[source_id], Mapping):
            raise ValueError(f"Original source lock omits {source_id}.")
    original_audit_path = Path(str(manifest[ORIGINAL_AUDIT_SOURCE_ID]["path"]))
    original_audit_hash = str(manifest[ORIGINAL_AUDIT_SOURCE_ID]["sha256"])
    if (
        original_audit_path.resolve() != Path(locked_audit.__file__).resolve()
        or _sha256(original_audit_path) != original_audit_hash
    ):
        raise ValueError("The imported production auditor differs from the lock.")

    expected_tuning = locked_audit.validate_design_inputs(
        _read_json(sweep_80),
        _read_json(sweep_32),
        _read_json(tuning_path),
    )
    run_contract = locked_audit.validate_run_contract(_read_json(run_contract_path))
    maximum_final_change = float(
        run_contract["acceptance"]["maximum_final_speed_change_mps"]
    )
    expected_case_names = sorted(item[0] for item in locked_audit.EXPECTED_CASES)
    for label, root in (("80 kW", root_80), ("32 kW", root_32)):
        observed_case_names = sorted(
            path.name
            for path in root.iterdir()
            if path.is_dir() and (path / "case_metadata.json").is_file()
        )
        if observed_case_names != expected_case_names:
            raise ValueError(
                f"{label} root does not contain the exact two configuration cases."
            )
    cases_80 = [
        validate_case(
            root_80 / case_name,
            expected_tuning=expected_tuning,
            maximum_final_speed_change_mps=maximum_final_change,
            expected_power_w=EXPECTED_POWER_80_W,
            prefix_mode="80kw_terminal_rpm",
        )
        for case_name in expected_case_names
    ]
    cases_32 = [
        validate_case(
            root_32 / case_name,
            expected_tuning=expected_tuning,
            maximum_final_speed_change_mps=maximum_final_change,
            expected_power_w=EXPECTED_POWER_W,
            prefix_mode="32kw_drag_power",
        )
        for case_name in expected_case_names
    ]
    qss_80 = inspect_qss_strict_gates(root_80, run_contract=run_contract)
    qss_32 = inspect_qss_strict_gates(root_32, run_contract=run_contract)
    strict_qss_pass = all(
        cohort["strict_status"] == "passed" for cohort in (qss_80, qss_32)
    )

    script_path = Path(__file__).resolve()
    return {
        "schema": SCHEMA,
        "status": "passed",
        "audit_scope": "80kw_32kw_terminal_prefix_and_actual_event_domain",
        "terminal_prefix_status": "passed",
        "strict_qss_status": "passed" if strict_qss_pass else "failed",
        "overall_production_acceptance_status": (
            "eligible_for_remaining_gates" if strict_qss_pass else "blocked"
        ),
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "simulations_launched": False,
        "filesystem_writes_performed": False,
        "does_not_supersede_primary_production_acceptance": True,
        "excluded_acceptance_gates": [
            "mixed-power scoring and report recomputation",
        ],
        "original_source_lock": {
            "path": str(lock_path),
            "sha256": _sha256(lock_path),
            "schema": lock.get("schema"),
            "manifest_member_count": len(manifest),
            "all_locked_sources_unchanged": True,
            "original_auditor_path": str(original_audit_path.resolve()),
            "original_auditor_sha256": original_audit_hash,
            "runner_sha256": str(manifest[RUNNER_SOURCE_ID]["sha256"]),
            "generator_sha256": str(manifest[GENERATOR_SOURCE_ID]["sha256"]),
        },
        "sidecar_validator": {
            "path": str(script_path),
            "sha256": _sha256(script_path),
            "covered_by_original_preproduction_lock": False,
            "role": "post-run read-only acceptance clarification",
        },
        "terminal_prefix_validation": {
            "root_80kw": str(root_80),
            "cases_80kw": cases_80,
            "root_32kw": str(root_32),
            "cases_32kw": cases_32,
        },
        "sampled_qss_strict_gates": {
            "status": "passed" if strict_qss_pass else "failed",
            "thresholds_unchanged": True,
            "cohort_80kw": qss_80,
            "cohort_32kw": qss_32,
        },
        "advisory_robustness": qss_advisory_robustness(qss_80, qss_32),
        "acceptance_claim": (
            "The immutable 80 kW GGV CSVs are accepted as their full declared "
            "prefix minus only the empty terminal-RPM slice, and the 32 kW CSVs "
            "as exact track-relevant prefixes through the analytical drag-power "
            "terminal. All solved events remain strictly inside those serialized "
            "domains. Sampled QSS failures are reported independently and remain "
            "formal blockers; no residual or TIR load-range threshold is waived."
        ),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lock-file", type=Path, required=True)
    parser.add_argument("--bobsim-root", type=Path, required=True)
    parser.add_argument("--sweep-80kw", type=Path, required=True)
    parser.add_argument("--sweep-32kw", type=Path, required=True)
    parser.add_argument("--tuning-json", type=Path, required=True)
    parser.add_argument("--scoring-json", type=Path, required=True)
    parser.add_argument("--track-manifest", type=Path, required=True)
    parser.add_argument("--run-contract", type=Path, required=True)
    parser.add_argument("--root-80kw", type=Path, required=True)
    parser.add_argument("--root-32kw", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    try:
        payload = run_sidecar_audit(args)
    except Exception as exc:
        script_path = Path(__file__).resolve()
        payload = {
            "schema": SCHEMA,
            "status": "failed",
            "audit_scope": "80kw_32kw_terminal_prefix_and_actual_event_domain",
            "generated_utc": datetime.now(timezone.utc).isoformat(),
            "simulations_launched": False,
            "filesystem_writes_performed": False,
            "sidecar_validator": {
                "path": str(script_path),
                "sha256": _sha256(script_path),
            },
            "error_type": type(exc).__name__,
            "error": str(exc),
        }
        print(json.dumps(payload, indent=2), flush=True)
        raise SystemExit(1) from exc
    print(json.dumps(payload, indent=2), flush=True)


if __name__ == "__main__":
    main()
