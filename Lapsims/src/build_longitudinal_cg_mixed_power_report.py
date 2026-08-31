"""Build a mixed-power longitudinal-CG report without rerunning simulations.

The report retains acceleration, skidpad, and autocross rows from the completed
80 kW eight-case common field, replaces only Michigan-endurance rows with a
completed 32 kW endurance-only sweep, and then scores all eight mixed cases in
one common 2026-real-plus-simulation field.
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

from build_longitudinal_cg_points_correlation import (
    MODEL_CONTACT_PLANE_Z_M,
    POINT_DELTA_BASELINE_REAR_WEIGHT_PERCENT,
    SOURCE_NOMINAL_REAR_STATIC_WEIGHT_FRACTION,
    _as_bool,
    _build_case_totals,
    _build_event_optima,
    _build_event_points_delta_vs_50pct,
    _build_event_sensitivity,
    _fit_summary,
    _json_default,
    _plot_event_points_delta_vs_50pct,
    _validate_exact_tmin_scoring,
    _write_json,
)
from build_reduced_dof_cg_comparison import (
    DEFAULT_SCORING_REFERENCE,
    EVENT_LABELS,
    EXPECTED_EVENTS,
    _load_scoring_reference,
    _rescore_common_field,
)

DEFAULT_REAR_FRACTIONS = (0.45, 0.50, 0.52, 0.54, 0.55, 0.56, 0.58, 0.60)
DEFAULT_BASE_POWER_W = 80_000.0
DEFAULT_ENDURANCE_POWER_W = 32_000.0
EXPECTED_POWER_LIMIT_SOURCE = "sweep_json"
ENDURANCE_EVENT = "michigan_endurance"
SPRINT_EVENTS = tuple(event for event in EXPECTED_EVENTS if event != ENDURANCE_EVENT)
BASE_EVENT_RESULTS_NAME = "common_8case_event_results.csv"
BASE_METADATA_NAME = "validity_and_tuning_by_case.csv"
ENDURANCE_RESULT_CANDIDATES = (
    "event_results.csv",
    "endurance_event_results.csv",
    "michigan_endurance_event_results.csv",
)
SCORING_COLUMNS_TO_DROP = {
    "simulation_time_multiplier",
    "projected_competition_time_s",
    "fsae_2026_reference_fastest_time_s",
    "fsae_2026_reference_slowest_time_s",
    "fsae_2026_valid_result_count",
    "fsae_2026_field_results_matched_or_outperformed",
    "fsae_2026_field_percent_matched_or_outperformed",
    "fsae_2026_reference_winner_team",
    "fsae_2026_reference_pdf_page",
    "fsae_2026_reference_time_basis",
    "at_or_faster_than_fsae_2026_winner",
    "fastest_simulated_competition_time_s",
    "effective_points_tmin_s",
    "effective_points_tmax_s",
    "effective_points_tmin_source",
    "is_effective_event_fastest",
    "projected_points",
    "maximum_points",
    "point_loss_vs_max",
    "points_projection_method",
}
CONTROLLED_METADATA_PATHS = (
    "model_family",
    "model_key",
    "model_dof",
    "sweep_axis",
    "cg_case",
    "cg_height_m",
    "cg_height_in",
    "sprung_mass_kg",
    "total_mass_kg",
    "rear_static_weight_fraction",
    "front_static_weight_fraction",
    "front_antiroll_stiffness_fraction",
    "front_elastic_roll_stiffness_fraction",
    "effective_brake_distribution_front",
    "effective_aero_balance_front",
    "effective_cop_from_front_m",
    "tire_mu_scale",
    "drive_distribution_front",
    "limited_slip_differential_model",
    "ggv_ay_points",
    "ggv_ax_search_points",
    "ggv_ax_binary_iterations",
    "reduced_vehicle_parameters.mass_kg",
    "reduced_vehicle_parameters.sprung_mass_kg",
    "reduced_vehicle_parameters.center_of_gravity_m",
    "reduced_vehicle_parameters.inertia_kg_m2",
    "reduced_vehicle_parameters.sprung_inertia_kg_m2",
    "reduced_vehicle_parameters.corner_positions_m",
    "reduced_vehicle_parameters.static_wheel_loads_n",
    "reduced_vehicle_parameters.wheel_radius_m",
    "reduced_vehicle_parameters.wheel_inertia_kg_m2",
    "reduced_vehicle_parameters.unsprung_mass_kg",
    "reduced_vehicle_parameters.suspension_stiffness_n_per_m",
    "reduced_vehicle_parameters.suspension_damping_n_s_per_m",
    "reduced_vehicle_parameters.antiroll_stiffness_nm_per_rad",
    "reduced_vehicle_parameters.tire_vertical_stiffness_n_per_m",
    "reduced_vehicle_parameters.tire_vertical_damping_n_s_per_m",
    "reduced_vehicle_parameters.rho_air_kg_m3",
    "reduced_vehicle_parameters.cl_area_m2",
    "reduced_vehicle_parameters.cd_area_m2",
    "reduced_vehicle_parameters.aero_balance_front",
    "reduced_vehicle_parameters.aero_cop_m",
    "reduced_vehicle_parameters.aero_drag_application_m",
    "reduced_vehicle_parameters.peak_drive_force_n",
    "reduced_vehicle_parameters.continuous_drive_force_n",
    "reduced_vehicle_parameters.maximum_drive_speed_mps",
    "reduced_vehicle_parameters.drive_distribution_front",
    "reduced_vehicle_parameters.brake_distribution_front",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Replace endurance only in an eight-case 80 kW longitudinal-CG "
            "field with completed 32 kW results and rescore once."
        )
    )
    parser.add_argument("--base-report-root", type=Path, required=True)
    parser.add_argument("--endurance-study-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
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
        "--expected-base-power-w", type=float, default=DEFAULT_BASE_POWER_W
    )
    parser.add_argument(
        "--expected-endurance-power-w",
        type=float,
        default=DEFAULT_ENDURANCE_POWER_W,
    )
    parser.add_argument("--fraction-tolerance", type=float, default=1e-9)
    return parser.parse_args()


def _read_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(path)
    payload = json.loads(path.read_text(encoding="utf-8-sig"))
    if not isinstance(payload, dict):
        raise TypeError(f"{path} must contain a JSON object.")
    return payload


def _require_columns(frame: pd.DataFrame, columns: Sequence[str], label: str) -> None:
    missing = [column for column in columns if column not in frame.columns]
    if missing:
        raise ValueError(f"{label} is missing columns: {missing}")


def _nested(payload: Mapping[str, Any], dotted_path: str) -> Any:
    value: Any = payload
    for key in dotted_path.split("."):
        if not isinstance(value, Mapping) or key not in value:
            raise ValueError(f"Metadata is missing controlled field {dotted_path!r}.")
        value = value[key]
    return value


def _controlled_values_match(left: Any, right: Any) -> bool:
    if isinstance(left, bool) or isinstance(right, bool):
        return bool(left) is bool(right)
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        return math.isclose(float(left), float(right), rel_tol=1e-10, abs_tol=1e-10)
    if isinstance(left, Sequence) and not isinstance(left, (str, bytes)):
        try:
            left_array = np.asarray(left, dtype=float)
            right_array = np.asarray(right, dtype=float)
        except (TypeError, ValueError):
            return left == right
        return left_array.shape == right_array.shape and bool(
            np.allclose(left_array, right_array, rtol=1e-10, atol=1e-10)
        )
    return left == right


def _validate_fraction_grid(
    metadata: pd.DataFrame,
    expected_rear_fractions: Sequence[float],
    *,
    fraction_tolerance: float,
    label: str,
) -> None:
    _require_columns(
        metadata,
        ("cg_case", "rear_static_weight_fraction"),
        label,
    )
    if metadata["cg_case"].astype(str).duplicated().any():
        raise ValueError(f"{label} contains duplicate case identifiers.")
    actual = np.sort(
        pd.to_numeric(metadata["rear_static_weight_fraction"], errors="raise").to_numpy(
            dtype=float
        )
    )
    expected = np.sort(np.asarray(expected_rear_fractions, dtype=float))
    if actual.shape != expected.shape or not np.allclose(
        actual, expected, rtol=0.0, atol=fraction_tolerance
    ):
        raise ValueError(
            f"{label} rear-fraction grid {actual.tolist()} does not match "
            f"{expected.tolist()}."
        )


def _load_base_report(
    root: Path,
    *,
    expected_rear_fractions: Sequence[float],
    expected_power_w: float,
    fraction_tolerance: float,
) -> tuple[pd.DataFrame, pd.DataFrame, dict[str, dict[str, Any]], dict[str, Any]]:
    event_path = root / BASE_EVENT_RESULTS_NAME
    metadata_path = root / BASE_METADATA_NAME
    events = pd.read_csv(event_path)
    metadata = pd.read_csv(metadata_path)
    validation = _read_json(root / "validation_summary.json")
    if validation.get("status") != "passed":
        raise ValueError("The base eight-case report did not pass validation.")
    _require_columns(
        events,
        (
            "cg_case",
            "event_slug",
            "lap_time_s",
            "track_length_m",
            "rear_static_weight_fraction",
            "source_cohort",
        ),
        str(event_path),
    )
    _require_columns(
        metadata,
        (
            "cg_case",
            "rear_static_weight_fraction",
            "rear_static_weight_percent",
            "peak_drive_power_w",
            "case_metadata_path",
        ),
        str(metadata_path),
    )
    _validate_fraction_grid(
        metadata,
        expected_rear_fractions,
        fraction_tolerance=fraction_tolerance,
        label="base report metadata",
    )
    case_names = set(metadata["cg_case"].astype(str))
    if set(events["cg_case"].astype(str)) != case_names:
        raise ValueError("Base event and metadata case sets differ.")
    if len(events) != len(case_names) * len(EXPECTED_EVENTS):
        raise ValueError("Base report must contain exactly four rows per case.")
    for case_name, group in events.groupby(events["cg_case"].astype(str)):
        if set(group["event_slug"].astype(str)) != set(EXPECTED_EVENTS):
            raise ValueError(f"Base case {case_name!r} does not contain four events.")
    base_power = pd.to_numeric(metadata["peak_drive_power_w"], errors="raise")
    if not np.allclose(base_power, expected_power_w, rtol=0.0, atol=1e-9):
        raise ValueError("Base report cases are not all the expected 80 kW cases.")

    case_payloads: dict[str, dict[str, Any]] = {}
    for row in metadata.itertuples(index=False):
        case_name = str(row.cg_case)
        payload = _read_json(Path(str(row.case_metadata_path)))
        power = float(_nested(payload, "reduced_vehicle_parameters.peak_drive_power_w"))
        if not math.isclose(power, expected_power_w, rel_tol=0.0, abs_tol=1e-9):
            raise ValueError(
                f"Base metadata for {case_name} is not {expected_power_w:g} W."
            )
        case_payloads[case_name] = payload
    return events, metadata, case_payloads, validation


def _find_endurance_results(root: Path) -> Path:
    matches = [
        root / name for name in ENDURANCE_RESULT_CANDIDATES if (root / name).is_file()
    ]
    if not matches:
        raise FileNotFoundError(
            f"No endurance result CSV found below {root}; expected one of "
            f"{list(ENDURANCE_RESULT_CANDIDATES)}."
        )
    return matches[0]


def _load_endurance_study(
    root: Path,
    *,
    base_metadata: pd.DataFrame,
    base_payloads: Mapping[str, Mapping[str, Any]],
    expected_rear_fractions: Sequence[float],
    expected_source_power_w: float,
    expected_power_w: float,
    fraction_tolerance: float,
) -> tuple[pd.DataFrame, pd.DataFrame, dict[str, Any]]:
    summary_path = root / "smoke_summary.json"
    summary = _read_json(summary_path)
    sweep_json_value = summary.get("sweep_json")
    if not sweep_json_value:
        raise ValueError("Endurance root summary does not identify its sweep JSON.")
    sweep_json_path = Path(str(sweep_json_value)).resolve()
    sweep_definition = _read_json(sweep_json_path)
    summary_source_power = float(_nested(summary, "source_peak_drive_power_w"))
    summary_effective_power = float(_nested(summary, "effective_drive_power_limit_w"))
    summary_power_limit_kw = float(
        _nested(summary, "sweep_definition.drive_power_limit_kw")
    )
    power_limit_scope = str(_nested(summary, "sweep_definition.power_limit_scope"))
    if not math.isclose(
        summary_source_power, expected_source_power_w, rel_tol=0.0, abs_tol=1e-9
    ):
        raise ValueError("Endurance root summary has the wrong source peak power.")
    if not math.isclose(
        summary_effective_power, expected_power_w, rel_tol=0.0, abs_tol=1e-9
    ) or not math.isclose(
        1000.0 * summary_power_limit_kw,
        expected_power_w,
        rel_tol=0.0,
        abs_tol=1e-9,
    ):
        raise ValueError("Endurance root summary does not declare the 32 kW cap.")
    if str(summary.get("drive_power_limit_source", "")) != EXPECTED_POWER_LIMIT_SOURCE:
        raise ValueError(
            "Endurance root summary does not source power from sweep_json."
        )
    if "endurance-only replacement" not in power_limit_scope:
        raise ValueError("Endurance root summary has an unexpected power-limit scope.")
    if (
        not math.isclose(
            1000.0 * float(_nested(sweep_definition, "drive_power_limit_kw")),
            expected_power_w,
            rel_tol=0.0,
            abs_tol=1e-9,
        )
        or str(_nested(sweep_definition, "power_limit_scope")) != power_limit_scope
    ):
        raise ValueError("Endurance sweep JSON disagrees with its root summary.")
    event_path = _find_endurance_results(root)
    source_events = pd.read_csv(event_path)
    _require_columns(
        source_events,
        (
            "cg_case",
            "event_slug",
            "lap_time_s",
            "track_length_m",
            "converged",
            "effective_drive_power_limit_w",
            "ggv_source_sha256",
        ),
        str(event_path),
    )
    if source_events.duplicated(["cg_case", "event_slug"]).any():
        raise ValueError("The 32 kW source contains duplicate case/event rows.")
    if len(source_events) != len(expected_rear_fractions) * len(EXPECTED_EVENTS):
        raise ValueError("The 32 kW source must contain four provenance rows per case.")
    for case_name, group in source_events.groupby(source_events["cg_case"].astype(str)):
        if set(group["event_slug"].astype(str)) != set(EXPECTED_EVENTS):
            raise ValueError(
                f"The 32 kW provenance source for {case_name} is missing an event."
            )
    source_event_power = pd.to_numeric(
        source_events["effective_drive_power_limit_w"], errors="raise"
    )
    if not np.allclose(source_event_power, expected_power_w, rtol=0.0, atol=1e-9):
        raise ValueError("The 32 kW source event rows do not all declare the cap.")
    if "drive_power_limit_source" in source_events and set(
        source_events["drive_power_limit_source"].astype(str)
    ) != {EXPECTED_POWER_LIMIT_SOURCE}:
        raise ValueError("The 32 kW source rows do not declare sweep_json provenance.")
    events = source_events[
        source_events["event_slug"].astype(str) == ENDURANCE_EVENT
    ].copy()
    if len(events) != len(expected_rear_fractions):
        raise ValueError("The endurance-only result must contain one row per case.")
    if events["cg_case"].astype(str).duplicated().any():
        raise ValueError("The endurance-only source contains duplicate cases.")
    if not bool(events["converged"].map(_as_bool).all()):
        raise ValueError("At least one 32 kW endurance solve did not converge.")

    metadata_rows: list[dict[str, Any]] = []
    input_fingerprints: list[str] = []
    for case_name in events["cg_case"].astype(str):
        path = root / case_name / "case_metadata.json"
        payload = _read_json(path)
        if str(payload.get("cg_case")) != case_name:
            raise ValueError(f"{path} cg_case does not match its directory.")
        power = float(_nested(payload, "reduced_vehicle_parameters.peak_drive_power_w"))
        if not math.isclose(power, expected_power_w, rel_tol=0.0, abs_tol=1e-9):
            raise ValueError(
                f"Endurance metadata for {case_name} is {power:g} W, not "
                f"{expected_power_w:g} W."
            )
        required_power_fields = {
            "source_peak_drive_power_w": expected_source_power_w,
            "effective_drive_power_limit_w": expected_power_w,
            "vehicle.max_drive_power": expected_power_w,
            "reduced_vehicle_parameters.continuous_drive_power_w": expected_power_w,
            "reduced_parameter_overrides.source_peak_drive_power_w": (
                expected_source_power_w
            ),
            "reduced_parameter_overrides.effective_peak_drive_power_w": (
                expected_power_w
            ),
            "reduced_parameter_overrides.effective_continuous_drive_power_w": (
                expected_power_w
            ),
        }
        for dotted_path, expected_value in required_power_fields.items():
            observed = float(_nested(payload, dotted_path))
            if not math.isclose(observed, expected_value, rel_tol=0.0, abs_tol=1e-9):
                raise ValueError(
                    f"Endurance metadata field {dotted_path!r} for {case_name} "
                    f"is {observed:g}, not {expected_value:g}."
                )
        metadata_power_source = payload.get("drive_power_limit_source")
        if metadata_power_source is not None and str(metadata_power_source) != (
            EXPECTED_POWER_LIMIT_SOURCE
        ):
            raise ValueError(
                f"Endurance metadata for {case_name} does not source power from "
                "sweep_json."
            )
        base_payload = base_payloads.get(case_name)
        if base_payload is None:
            raise ValueError(
                f"Endurance case {case_name!r} is absent from the base field."
            )
        for dotted_path in CONTROLLED_METADATA_PATHS:
            base_value = _nested(base_payload, dotted_path)
            endurance_value = _nested(payload, dotted_path)
            if not _controlled_values_match(base_value, endurance_value):
                raise ValueError(
                    f"Controlled input {dotted_path!r} differs for {case_name}; "
                    "only endurance drive power may change."
                )
        metadata_rows.append(
            {
                "cg_case": case_name,
                "rear_static_weight_fraction": float(
                    payload["rear_static_weight_fraction"]
                ),
                "rear_static_weight_percent": 100.0
                * float(payload["rear_static_weight_fraction"]),
                "peak_drive_power_w": power,
                "case_metadata_path": str(path.resolve()),
                "ggv_source_sha256": str(payload.get("ggv_source_sha256", "")),
            }
        )
        input_fingerprints.append(str(payload.get("input_fingerprint_sha256", "")))
    metadata = pd.DataFrame(metadata_rows)
    _validate_fraction_grid(
        metadata,
        expected_rear_fractions,
        fraction_tolerance=fraction_tolerance,
        label="endurance-only metadata",
    )
    if set(metadata["cg_case"]) != set(base_metadata["cg_case"].astype(str)):
        raise ValueError("Endurance-only and base case identifiers differ.")
    metadata_by_case = metadata.set_index("cg_case")
    if "rear_static_weight_fraction" in events:
        event_fraction = pd.to_numeric(
            events["rear_static_weight_fraction"], errors="raise"
        )
        expected_fraction = (
            events["cg_case"]
            .astype(str)
            .map(metadata_by_case["rear_static_weight_fraction"])
        )
        if not np.allclose(
            event_fraction,
            expected_fraction,
            rtol=0.0,
            atol=fraction_tolerance,
        ):
            raise ValueError("Endurance event fractions disagree with case metadata.")
    if "ggv_source_sha256" in source_events:
        event_hash = source_events["ggv_source_sha256"].fillna("").astype(str)
        expected_hash = (
            source_events["cg_case"]
            .astype(str)
            .map(metadata_by_case["ggv_source_sha256"])
        )
        if not bool((event_hash == expected_hash).all()):
            raise ValueError("Endurance event GGV hashes disagree with case metadata.")
    provenance = {
        "smoke_summary_path": str(summary_path.resolve()),
        "sweep_json_path": str(sweep_json_path),
        "power_limit_scope": power_limit_scope,
        "event_results_path": str(event_path.resolve()),
        "source_event_row_count": len(source_events),
        "consumed_endurance_row_count": len(events),
        "ignored_32kw_sprint_row_count": int(len(source_events) - len(events)),
        "case_metadata_paths": metadata["case_metadata_path"].tolist(),
        "input_fingerprints": input_fingerprints,
        "all_track_solvers_converged": True,
        "total_ggv_speed_cap_segments": int(
            pd.to_numeric(
                events.get("ggv_speed_cap_segments", pd.Series(0, index=events.index)),
                errors="coerce",
            )
            .fillna(0)
            .sum()
        ),
    }
    return events, metadata, provenance


def _drop_stale_scoring_columns(events: pd.DataFrame) -> pd.DataFrame:
    columns = [
        column
        for column in events.columns
        if column.startswith("common_") or column in SCORING_COLUMNS_TO_DROP
    ]
    return events.drop(columns=columns, errors="ignore")


def _assemble_mixed_events(
    base_events: pd.DataFrame,
    endurance_events: pd.DataFrame,
    *,
    base_root: Path,
    endurance_root: Path,
    base_power_w: float,
    endurance_power_w: float,
) -> tuple[pd.DataFrame, pd.DataFrame]:
    base = _drop_stale_scoring_columns(base_events)
    endurance_by_case = endurance_events.set_index(
        endurance_events["cg_case"].astype(str)
    )
    rows: list[dict[str, Any]] = []
    replacement_rows: list[dict[str, Any]] = []
    for base_row in base.to_dict(orient="records"):
        case_name = str(base_row["cg_case"])
        event_slug = str(base_row["event_slug"])
        row = dict(base_row)
        row["base_80kw_lap_time_s"] = float(base_row["lap_time_s"])
        row["base_80kw_event_results_root"] = str(base_root)
        if event_slug == ENDURANCE_EVENT:
            source = endurance_by_case.loc[case_name]
            if isinstance(source, pd.DataFrame):
                raise ValueError(f"Duplicate endurance replacement for {case_name}.")
            for column, value in source.items():
                if column.startswith("common_") or column in SCORING_COLUMNS_TO_DROP:
                    continue
                row[column] = value
            row["source_cohort"] = str(base_row["source_cohort"])
            row["rear_static_weight_fraction"] = float(
                base_row["rear_static_weight_fraction"]
            )
            if "rear_static_weight_percent" in base_row:
                row["rear_static_weight_percent"] = float(
                    base_row["rear_static_weight_percent"]
                )
            row["mixed_power_event_source"] = "endurance_32kw"
            row["mixed_power_source_root"] = str(endurance_root)
            row["mixed_power_power_limit_w"] = endurance_power_w
            row["mixed_power_replaced_base_endurance"] = True
            replacement_rows.append(
                {
                    "cg_case": case_name,
                    "rear_static_weight_fraction": float(
                        row["rear_static_weight_fraction"]
                    ),
                    "rear_static_weight_percent": float(
                        row["rear_static_weight_fraction"]
                    )
                    * 100.0,
                    "base_80kw_endurance_lap_time_s": float(base_row["lap_time_s"]),
                    "replacement_32kw_endurance_lap_time_s": float(row["lap_time_s"]),
                    "replacement_minus_base_lap_time_s": float(row["lap_time_s"])
                    - float(base_row["lap_time_s"]),
                }
            )
        else:
            row["mixed_power_event_source"] = "base_80kw"
            row["mixed_power_source_root"] = str(base_root)
            row["mixed_power_power_limit_w"] = base_power_w
            row["mixed_power_replaced_base_endurance"] = False
        rows.append(row)
    mixed = pd.DataFrame(rows)
    provenance = mixed.groupby("event_slug", sort=False).agg(
        event_name=("event_name", "first"),
        event_row_count=("cg_case", "size"),
        unique_source_count=("mixed_power_event_source", "nunique"),
        source=("mixed_power_event_source", "first"),
        power_limit_w=("mixed_power_power_limit_w", "first"),
        source_root=("mixed_power_source_root", "first"),
    )
    expected_power = {
        **{event: base_power_w for event in SPRINT_EVENTS},
        ENDURANCE_EVENT: endurance_power_w,
    }
    for event_slug in EXPECTED_EVENTS:
        event = provenance.loc[event_slug]
        if int(event["event_row_count"]) != len(endurance_events):
            raise ValueError(f"Mixed event {event_slug} does not contain all cases.")
        if int(event["unique_source_count"]) != 1:
            raise ValueError(f"Mixed event {event_slug} has mixed row provenance.")
        if not math.isclose(
            float(event["power_limit_w"]),
            expected_power[event_slug],
            rel_tol=0.0,
            abs_tol=1e-9,
        ):
            raise ValueError(f"Mixed event {event_slug} has the wrong power source.")
    return mixed, pd.DataFrame(replacement_rows).sort_values(
        "rear_static_weight_fraction"
    )


def _plot_total_points(
    totals: pd.DataFrame,
    fit: Mapping[str, Any],
    path: Path,
) -> None:
    ordered = totals.sort_values("rear_static_weight_percent")
    x = ordered["rear_static_weight_percent"].to_numpy(dtype=float)
    mixed = ordered["timed_event_points"].to_numpy(dtype=float)
    base = ordered["all_80kw_timed_event_points"].to_numpy(dtype=float)
    grid = np.linspace(float(x.min()), float(x.max()), 200)
    fitted = float(fit["linear_intercept_points_at_50_percent_rear"]) + float(
        fit["linear_slope_points_per_rear_weight_percentage_point"]
    ) * (grid - 50.0)
    fig, axis = plt.subplots(figsize=(9.4, 6.0))
    axis.plot(
        x,
        base,
        color="#6B7280",
        linestyle="--",
        marker="o",
        markerfacecolor="white",
        linewidth=1.6,
        label="Original all-80 kW (own scoring field)",
    )
    axis.plot(
        x,
        mixed,
        color="#DC2626",
        marker="o",
        linewidth=2.2,
        label="Mixed: 32 kW endurance, 80 kW sprint",
    )
    axis.plot(grid, fitted, color="#111827", linewidth=1.4, label="Mixed-field OLS")
    axis.axvline(50.0, color="#9CA3AF", linewidth=1.0, linestyle=":")
    axis.set_xlabel("Rear static weight (%)")
    axis.set_ylabel("Timed dynamic-event points / 575")
    axis.set_title("Longitudinal-CG points with endurance limited to 32 kW")
    axis.grid(alpha=0.25)
    axis.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def _markdown_table(headers: Sequence[str], rows: Sequence[Sequence[object]]) -> str:
    def render(value: object) -> str:
        if isinstance(value, float):
            return f"{value:.6f}" if math.isfinite(value) else "—"
        return str(value)

    header = "| " + " | ".join(headers) + " |"
    divider = "| " + " | ".join("---" for _ in headers) + " |"
    body = ["| " + " | ".join(render(value) for value in row) + " |" for row in rows]
    return "\n".join([header, divider, *body])


def _write_report(
    path: Path,
    *,
    base_root: Path,
    endurance_root: Path,
    scoring_reference_path: Path,
    totals: pd.DataFrame,
    deltas: pd.DataFrame,
    replacements: pd.DataFrame,
    event_scoring: Mapping[str, Mapping[str, Any]],
    provenance: pd.DataFrame,
    endurance_provenance: Mapping[str, Any],
    design: Mapping[str, Any],
    fit: Mapping[str, Any],
    base_power_w: float,
    endurance_power_w: float,
) -> None:
    best = totals.loc[totals["timed_event_points"].idxmax()]
    baseline_rows = []
    for event_slug in EXPECTED_EVENTS:
        row = deltas[
            (deltas["event_slug"] == event_slug)
            & deltas["is_50pct_baseline_case"].map(_as_bool)
        ].iloc[0]
        baseline_rows.append(
            (
                EVENT_LABELS[event_slug],
                row["baseline_cg_case"],
                float(row["baseline_projected_points"]),
                float(row["common_field_tmin_s"]),
            )
        )
    point_rows = [
        (
            row.rear_static_weight_percent,
            row.acceleration_points,
            row.skidpad_points,
            row.autocross_points,
            row.michigan_endurance_points,
            row.timed_event_points,
            row.timed_event_points_delta_vs_50pct_rear,
            row.mixed_minus_all_80kw_points,
        )
        for row in totals.itertuples(index=False)
    ]
    raw_time_rows = [
        (
            row.rear_static_weight_percent,
            row.acceleration_time_s,
            row.skidpad_time_s,
            row.autocross_time_s,
            row.michigan_endurance_time_s,
        )
        for row in totals.itertuples(index=False)
    ]
    replacement_rows = [
        (
            row.rear_static_weight_percent,
            row.base_80kw_endurance_lap_time_s,
            row.replacement_32kw_endurance_lap_time_s,
            row.replacement_minus_base_lap_time_s,
        )
        for row in replacements.itertuples(index=False)
    ]
    scoring_rows = [
        (
            EVENT_LABELS[event_slug],
            summary["real_field_fastest_time_s"],
            summary["simulated_fastest_time_s"],
            summary["common_field_tmin_s"],
            summary["common_field_tmin_source"],
            summary["simulated_maximum_score_count"],
        )
        for event_slug, summary in event_scoring.items()
    ]
    provenance_rows = [
        (
            row.event_name,
            row.source,
            row.power_limit_w / 1000.0,
            row.row_count,
        )
        for row in provenance.itertuples(index=False)
    ]
    report = f"""# Longitudinal-CG mixed-power dynamic-event points

## Result

This is an event-specific mixed-power rescore, not a new vehicle simulation.
Acceleration, skidpad, and autocross remain the completed **{base_power_w / 1000:.0f}
kW** results. Only Michigan endurance is replaced by the completed
**{endurance_power_w / 1000:.0f} kW** endurance-only rows. All eight hybrid cases
are then rescored once with the valid 2026 Michigan EV field, using one common
Tmin per event.

All cases retain the prior controlled vehicle: {design["model_key"]} reduced
{design["model_dof"]}DOF, {design["total_mass_kg"]:.3f} kg total mass, tire mu scale
{design["tire_mu_scale"]:.16f}, {100.0 * design["aero_balance_front"]:.0f}% front
aero balance, RWD, and no LSD. Per-case ARB allocation and fixed brake bias are
unchanged from the accepted longitudinal-CG runs.

The inherited "{design["cg_height_in"]:.1f} in" convention sets model absolute
CG z={design["cg_height_m"]:.6f} m. Relative to the modeled contact plane at
z={MODEL_CONTACT_PLANE_Z_M:.6f} m, the physical height is
{design["cg_height_m"] - MODEL_CONTACT_PLANE_Z_M:.6f} m =
{(design["cg_height_m"] - MODEL_CONTACT_PLANE_Z_M) / 0.0254:.3f} in.

The best observed mixed-power case is **{best.rear_static_weight_percent:.1f}%
rear** at **{best.timed_event_points:.3f} / 575 points**. The mixed-field OLS
sensitivity is **{fit["linear_slope_points_per_rear_weight_percentage_point"]:+.6f}
points per rear-weight percentage point** (R²={fit["linear_r_squared"]:.6f}).
This remains a screening-grid result; it does not establish a continuous
physical optimum.

![Mixed-power total points](mixed_power_rear_weight_to_points.png)

The gray all-80 kW reference uses its previously published, separately scored
common field and therefore its own endurance Tmin. A red-minus-gray point
difference is a comparison between two scoring fields, not an absolute claim
that lowering endurance power gained or lost those points. The within-field
rear-weight comparisons below all use the single mixed common field.

## Event-row provenance

{_markdown_table(("Event", "Consumed source", "Power limit (kW)", "Rows"), provenance_rows)}

The separate 32 kW root contains
{endurance_provenance["source_event_row_count"]} event rows. This assembler
consumes its {endurance_provenance["consumed_endurance_row_count"]} Michigan
endurance rows and explicitly ignores its
{endurance_provenance["ignored_32kw_sprint_row_count"]} sprint rows; the accepted
80 kW sprint rows remain authoritative.

## Endurance replacement audit

No sprint row was recomputed or replaced. The endurance lap-time substitutions
are:

{_markdown_table(("Rear wt. (%)", "Base 80 kW endurance (s)", "Replacement 32 kW endurance (s)", "32-80 kW (s)"), replacement_rows)}

## Mixed-field raw event times

Acceleration, skidpad, and autocross are the retained 80 kW raw times;
endurance is the replacement 32 kW one-lap time.

{_markdown_table(("Rear wt. (%)", "Accel (s)", "Skidpad (s)", "Autocross (s)", "Endurance lap (s)"), raw_time_rows)}

## Mixed-field projected points

{_markdown_table(("Rear wt. (%)", "Accel", "Skidpad", "Autocross", "Endurance", "Mixed total / 575", "Total delta vs 50%", "Mixed - separately scored all-80"), point_rows)}

## Per-event point delta from exact 50% rear

For event `e` and rear fraction `r`, `Delta P_e(r) = P_e(r) - P_e(50%)`.
Both terms come from this same mixed common field. The baseline is the exact
simulated 50% case, not an interpolation and not the source vehicle's
{100.0 * SOURCE_NOMINAL_REAR_STATIC_WEIGHT_FRACTION:.5f}% nominal balance.

{_markdown_table(("Event", "50% case", "50% baseline points", "Common Tmin (s)"), baseline_rows)}

![Mixed-power per-event delta](mixed_power_event_points_delta_vs_50pct_rear.png)

Exact plotted values are in
[`mixed_power_event_points_delta_vs_50pct_rear.csv`](mixed_power_event_points_delta_vs_50pct_rear.csv).

## Common scoring field and exact ties

Only a simulated competition time exactly equal to the real-or-simulated common
Tmin receives event maximum points. Near ties remain below maximum.

{_markdown_table(("Event", "Real fastest (s)", "Sim fastest (s)", "Common Tmin (s)", "Tmin source", "Sim max-score count"), scoring_rows)}

## Interpretation boundary

This report answers the requested hybrid rule: sprint performance from the 80
kW maps and endurance performance from the 32 kW maps. It must not be described
as an all-event 32 kW car or as a single unchanged powertrain map across every
event.

## Provenance

- Original eight-case report: `{base_root.resolve()}`
- 32 kW endurance-only study: `{endurance_root.resolve()}`
- 2026 scoring reference: `{scoring_reference_path.resolve()}`
- The builder launches no simulation and modifies no vehicle, tire, runner,
  tuner, or bridge input.
"""
    path.write_text(report, encoding="utf-8")


def build_mixed_power_report(
    *,
    base_report_root: Path,
    endurance_study_root: Path,
    output_root: Path,
    scoring_reference_path: Path = DEFAULT_SCORING_REFERENCE,
    expected_rear_fractions: Sequence[float] = DEFAULT_REAR_FRACTIONS,
    expected_base_power_w: float = DEFAULT_BASE_POWER_W,
    expected_endurance_power_w: float = DEFAULT_ENDURANCE_POWER_W,
    fraction_tolerance: float = 1e-9,
) -> dict[str, Any]:
    base_report_root = base_report_root.resolve()
    endurance_study_root = endurance_study_root.resolve()
    output_root = output_root.resolve()
    scoring_reference_path = scoring_reference_path.resolve()
    base_events, base_metadata, base_payloads, base_validation = _load_base_report(
        base_report_root,
        expected_rear_fractions=expected_rear_fractions,
        expected_power_w=expected_base_power_w,
        fraction_tolerance=fraction_tolerance,
    )
    design_payload = next(iter(base_payloads.values()))
    design = {
        "model_key": str(design_payload["model_key"]),
        "model_dof": int(design_payload["model_dof"]),
        "cg_height_m": float(design_payload["cg_height_m"]),
        "cg_height_in": float(design_payload["cg_height_in"]),
        "total_mass_kg": float(design_payload["total_mass_kg"]),
        "sprung_mass_kg": float(design_payload["sprung_mass_kg"]),
        "tire_mu_scale": float(design_payload["tire_mu_scale"]),
        "aero_balance_front": float(design_payload["effective_aero_balance_front"]),
        "drive_distribution_front": float(design_payload["drive_distribution_front"]),
        "limited_slip_differential_model": bool(
            design_payload["limited_slip_differential_model"]
        ),
        "rear_static_weight_fractions": sorted(
            float(value) for value in expected_rear_fractions
        ),
    }
    if (
        not math.isclose(
            design["drive_distribution_front"], 0.0, rel_tol=0.0, abs_tol=1e-12
        )
        or design["limited_slip_differential_model"]
    ):
        raise ValueError("The base report is not the required RWD, no-LSD study.")
    endurance_events, _endurance_metadata, endurance_provenance = _load_endurance_study(
        endurance_study_root,
        base_metadata=base_metadata,
        base_payloads=base_payloads,
        expected_rear_fractions=expected_rear_fractions,
        expected_source_power_w=expected_base_power_w,
        expected_power_w=expected_endurance_power_w,
        fraction_tolerance=fraction_tolerance,
    )
    mixed_raw, replacements = _assemble_mixed_events(
        base_events,
        endurance_events,
        base_root=base_report_root,
        endurance_root=endurance_study_root,
        base_power_w=expected_base_power_w,
        endurance_power_w=expected_endurance_power_w,
    )
    scoring_reference = _load_scoring_reference(scoring_reference_path)
    base_rescored, _base_scoring = _rescore_common_field(
        _drop_stale_scoring_columns(base_events), scoring_reference
    )
    _validate_exact_tmin_scoring(base_rescored)
    base_scoring_reproduced = True
    for column in (
        "common_projected_competition_time_s",
        "common_field_tmin_s",
        "common_field_tmax_s",
        "common_projected_points",
        "common_maximum_points",
    ):
        if column in base_events:
            base_scoring_reproduced = base_scoring_reproduced and bool(
                np.allclose(
                    base_rescored[column].to_numpy(dtype=float),
                    base_events[column].to_numpy(dtype=float),
                    rtol=0.0,
                    atol=1e-12,
                )
            )
    if "common_is_event_fastest" in base_events:
        base_scoring_reproduced = base_scoring_reproduced and bool(
            np.array_equal(
                base_rescored["common_is_event_fastest"].map(_as_bool),
                base_events["common_is_event_fastest"].map(_as_bool),
            )
        )
    if not base_scoring_reproduced:
        raise ValueError(
            "The supplied scoring reference does not reproduce the published "
            "all-80 kW common field."
        )
    rescored, event_scoring = _rescore_common_field(mixed_raw, scoring_reference)
    _validate_exact_tmin_scoring(rescored)
    totals = _build_case_totals(rescored, base_metadata)
    base_totals = _build_case_totals(base_rescored, base_metadata)[
        ["cg_case", "timed_event_points"]
    ].rename(columns={"timed_event_points": "all_80kw_timed_event_points"})
    totals = totals.merge(base_totals, on="cg_case", validate="one_to_one")
    totals["mixed_minus_all_80kw_points"] = (
        totals["timed_event_points"] - totals["all_80kw_timed_event_points"]
    )
    total_baseline_mask = np.isclose(
        totals["rear_static_weight_percent"].to_numpy(dtype=float),
        POINT_DELTA_BASELINE_REAR_WEIGHT_PERCENT,
        rtol=0.0,
        atol=100.0 * fraction_tolerance,
    )
    if int(total_baseline_mask.sum()) != 1:
        raise ValueError("Mixed totals require exactly one simulated 50% rear case.")
    total_baseline_points = float(
        totals.loc[total_baseline_mask, "timed_event_points"].iloc[0]
    )
    totals["timed_event_points_delta_vs_50pct_rear"] = (
        totals["timed_event_points"] - total_baseline_points
    )
    fit = _fit_summary(
        totals["rear_static_weight_percent"].to_numpy(dtype=float),
        totals["timed_event_points"].to_numpy(dtype=float),
    )
    # This is a new arbitrary-grid report, so omit the legacy 45/55-named
    # compatibility alias returned by the shared fitter.
    fit.pop("endpoint_change_points_55_minus_45", None)
    event_sensitivity = _build_event_sensitivity(
        rescored, float(fit["endpoint_change_points_max_minus_min"])
    )
    event_optima = _build_event_optima(rescored)
    deltas = _build_event_points_delta_vs_50pct(
        rescored, fraction_tolerance=fraction_tolerance
    )
    provenance_rows = []
    for event_slug in EXPECTED_EVENTS:
        group = rescored[rescored["event_slug"] == event_slug]
        provenance_rows.append(
            {
                "event_slug": event_slug,
                "event_name": EVENT_LABELS[event_slug],
                "row_count": len(group),
                "source": str(group["mixed_power_event_source"].iloc[0]),
                "power_limit_w": float(group["mixed_power_power_limit_w"].iloc[0]),
                "source_root": str(group["mixed_power_source_root"].iloc[0]),
            }
        )
    provenance = pd.DataFrame(provenance_rows)

    output_root.mkdir(parents=True, exist_ok=True)
    rescored.sort_values(["rear_static_weight_fraction", "event_slug"]).to_csv(
        output_root / "mixed_power_event_results.csv", index=False
    )
    totals.to_csv(output_root / "mixed_power_case_points.csv", index=False)
    deltas.to_csv(
        output_root / "mixed_power_event_points_delta_vs_50pct_rear.csv",
        index=False,
    )
    replacements.to_csv(
        output_root / "mixed_power_endurance_replacement.csv", index=False
    )
    provenance.to_csv(output_root / "mixed_power_event_provenance.csv", index=False)
    event_sensitivity.to_csv(
        output_root / "mixed_power_event_sensitivity.csv", index=False
    )
    event_optima.to_csv(output_root / "mixed_power_event_optima.csv", index=False)
    pd.DataFrame(
        [
            {"event_slug": event_slug, **summary}
            for event_slug, summary in event_scoring.items()
        ]
    ).to_csv(output_root / "mixed_power_event_scoring.csv", index=False)
    _write_json(
        output_root / "mixed_power_correlation.json",
        {
            "fit": fit,
            "event_sensitivity": event_sensitivity.to_dict(orient="records"),
            "event_points_delta_vs_50pct": deltas.to_dict(orient="records"),
        },
    )
    _plot_total_points(
        totals, fit, output_root / "mixed_power_rear_weight_to_points.png"
    )
    _plot_event_points_delta_vs_50pct(
        deltas,
        output_root / "mixed_power_event_points_delta_vs_50pct_rear.png",
        highlight_cohort="new_sweep",
    )
    _write_report(
        output_root / "mixed_power_report.md",
        base_root=base_report_root,
        endurance_root=endurance_study_root,
        scoring_reference_path=scoring_reference_path,
        totals=totals,
        deltas=deltas,
        replacements=replacements,
        event_scoring=event_scoring,
        provenance=provenance,
        endurance_provenance=endurance_provenance,
        design=design,
        fit=fit,
        base_power_w=expected_base_power_w,
        endurance_power_w=expected_endurance_power_w,
    )
    max_score = np.isclose(
        rescored["common_projected_points"],
        rescored["common_maximum_points"],
        rtol=0.0,
        atol=0.0,
    )
    exact_tmin = rescored["common_projected_competition_time_s"].to_numpy(
        dtype=float
    ) == rescored["common_field_tmin_s"].to_numpy(dtype=float)
    baseline_delta = deltas.loc[
        deltas["is_50pct_baseline_case"].map(_as_bool),
        "projected_points_delta_vs_50pct_rear",
    ].to_numpy(dtype=float)
    sprint_audit = rescored[rescored["event_slug"].isin(SPRINT_EVENTS)].merge(
        base_rescored[
            [
                "cg_case",
                "event_slug",
                "lap_time_s",
                "track_length_m",
                "common_projected_points",
            ]
        ],
        on=["cg_case", "event_slug"],
        suffixes=("_mixed", "_base"),
        validate="one_to_one",
    )
    sprint_rows_unchanged = bool(
        (
            sprint_audit["lap_time_s_mixed"].to_numpy(dtype=float)
            == sprint_audit["lap_time_s_base"].to_numpy(dtype=float)
        ).all()
        and (
            sprint_audit["track_length_m_mixed"].to_numpy(dtype=float)
            == sprint_audit["track_length_m_base"].to_numpy(dtype=float)
        ).all()
    )
    sprint_points_unchanged = bool(
        (
            sprint_audit["common_projected_points_mixed"].to_numpy(dtype=float)
            == sprint_audit["common_projected_points_base"].to_numpy(dtype=float)
        ).all()
    )
    endurance_audit = rescored[rescored["event_slug"] == ENDURANCE_EVENT].merge(
        endurance_events[["cg_case", "lap_time_s", "track_length_m"]],
        on="cg_case",
        suffixes=("_mixed", "_32kw"),
        validate="one_to_one",
    )
    endurance_rows_match_source = bool(
        (
            endurance_audit["lap_time_s_mixed"].to_numpy(dtype=float)
            == endurance_audit["lap_time_s_32kw"].to_numpy(dtype=float)
        ).all()
        and (
            endurance_audit["track_length_m_mixed"].to_numpy(dtype=float)
            == endurance_audit["track_length_m_32kw"].to_numpy(dtype=float)
        ).all()
    )
    validation: dict[str, Any] = {
        "status": "passed",
        "case_count": len(totals),
        "event_row_count": len(rescored),
        "base_power_w": expected_base_power_w,
        "endurance_power_w": expected_endurance_power_w,
        "sprint_events_retained_from_base_80kw": list(SPRINT_EVENTS),
        "endurance_event_replaced_from_32kw": ENDURANCE_EVENT,
        "only_endurance_rows_replaced": bool(
            (rescored["mixed_power_replaced_base_endurance"])
            .map(_as_bool)
            .equals(rescored["event_slug"] == ENDURANCE_EVENT)
        ),
        "all_80kw_sprint_times_and_distances_unchanged": sprint_rows_unchanged,
        "all_80kw_sprint_projected_points_unchanged": sprint_points_unchanged,
        "all_endurance_times_and_distances_match_32kw_source": (
            endurance_rows_match_source
        ),
        "one_common_2026_real_plus_all_8_sim_scoring_field": True,
        "only_exact_common_tmin_ties_receive_maximum": bool(
            np.array_equal(np.asarray(max_score, dtype=bool), exact_tmin)
        ),
        "event_points_delta_baseline_rear_static_weight_percent": (
            POINT_DELTA_BASELINE_REAR_WEIGHT_PERCENT
        ),
        "all_event_point_deltas_exactly_zero_at_50pct": bool(
            (baseline_delta == 0.0).all()
        ),
        "total_points_delta_exactly_zero_at_50pct": bool(
            (
                totals.loc[
                    total_baseline_mask,
                    "timed_event_points_delta_vs_50pct_rear",
                ].to_numpy(dtype=float)
                == 0.0
            ).all()
        ),
        "best_observed_rear_static_weight_percent": float(
            totals.loc[
                totals["timed_event_points"].idxmax(),
                "rear_static_weight_percent",
            ]
        ),
        "best_observed_timed_event_points": float(totals["timed_event_points"].max()),
        "correlation": fit,
        "event_scoring": event_scoring,
        "event_provenance": provenance.to_dict(orient="records"),
        "controlled_design": design,
        "base_report_validation_status": base_validation.get("status"),
        "base_all_80kw_common_scoring_reproduced": base_scoring_reproduced,
        "endurance_provenance": endurance_provenance,
        "simulations_launched_by_builder": False,
        "input_paths": {
            "base_report_root": str(base_report_root),
            "endurance_study_root": str(endurance_study_root),
            "scoring_reference": str(scoring_reference_path),
        },
    }
    if not validation["only_endurance_rows_replaced"]:
        raise AssertionError("Mixed-power assembly replaced a non-endurance row.")
    if (
        not sprint_rows_unchanged
        or not sprint_points_unchanged
        or not endurance_rows_match_source
    ):
        raise AssertionError("Mixed-power row-source audit failed.")
    if not validation["only_exact_common_tmin_ties_receive_maximum"]:
        raise AssertionError("Mixed-power exact-Tmin scoring validation failed.")
    if not validation["all_event_point_deltas_exactly_zero_at_50pct"]:
        raise AssertionError("The exact 50% point-delta baseline is not zero.")
    if not validation["total_points_delta_exactly_zero_at_50pct"]:
        raise AssertionError("The exact 50% total-points baseline is not zero.")
    validation["artifacts"] = sorted(
        {artifact.name for artifact in output_root.iterdir()}
        | {"validation_summary.json"}
    )
    _write_json(output_root / "validation_summary.json", validation)
    return validation


def main() -> None:
    args = parse_args()
    validation = build_mixed_power_report(
        base_report_root=args.base_report_root,
        endurance_study_root=args.endurance_study_root,
        output_root=args.output_root,
        scoring_reference_path=args.scoring_reference,
        expected_rear_fractions=args.expected_rear_fractions,
        expected_base_power_w=args.expected_base_power_w,
        expected_endurance_power_w=args.expected_endurance_power_w,
        fraction_tolerance=args.fraction_tolerance,
    )
    print(json.dumps(validation, indent=2, default=_json_default))


if __name__ == "__main__":
    main()
