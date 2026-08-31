"""Build a locked two-configuration mixed-power event comparison.

The production roots are expected to contain exactly two configurations:
``config_nominal_54`` and ``config_plus0p25_56p5``.  Acceleration, skidpad,
and autocross are selected from the 80 kW root; Michigan endurance is selected
from the 32 kW root.  All eight selected rows are then rescored once against
the existing 2026 Michigan EV reference field.

This post-processor launches no simulation and changes no source study.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from collections.abc import Mapping, Sequence
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from build_longitudinal_cg_mixed_power_report import SCORING_COLUMNS_TO_DROP
from build_longitudinal_cg_points_correlation import _validate_exact_tmin_scoring
from build_reduced_dof_cg_comparison import (
    DEFAULT_SCORING_REFERENCE,
    EVENT_LABELS,
    EXPECTED_EVENTS,
    _load_scoring_reference,
    _rescore_common_field,
)

CONFIG_1 = "config_nominal_54"
CONFIG_2 = "config_plus0p25_56p5"
EXPECTED_CONFIGS = (CONFIG_1, CONFIG_2)
CONFIG_LABELS = {
    CONFIG_1: "Config 1: nominal 54% rear",
    CONFIG_2: "Config 2: +0.25 in / 56.5% rear",
}
SPRINT_EVENTS = ("acceleration", "skidpad", "autocross")
ENDURANCE_EVENT = "michigan_endurance"
EXPECTED_POWER_W = {"80kw": 80_000.0, "32kw": 32_000.0}
EXPECTED_MODEL_KEY = "dyn_py_6dof_qss"
PROVENANCE_SCHEMA = "lapsims.two-config-mixed-power-report.v1"
EVENT_RESULTS_NAME = "event_results.csv"
CASE_METADATA_NAME = "case_metadata.json"
SOURCE_LOCK_NAME = "source_code_lock.json"
SMOKE_SUMMARY_NAME = "smoke_summary.json"
MAXIMUM_TIMED_EVENT_POINTS = 575.0

EVENT_OUTPUT_COLUMNS = (
    "solver",
    "lap_time_s",
    "track_length_m",
    "segments",
    "track_configuration",
    "standing_start",
    "minimum_speed_mps",
    "maximum_speed_mps",
    "distance_weighted_average_speed_mps",
    "converged",
    "final_max_speed_change_mps",
    "ggv_source_csv",
    "ggv_source_sha256",
    "ggv_solver_speed_cap_mps",
    "ggv_max_lateral_utilization",
    "ggv_max_speed_domain_fraction",
    "ggv_speed_cap_segments",
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
    "effective_lltd",
    "effective_lltd_interpretation",
    "front_antiroll_stiffness_fraction",
    "front_elastic_roll_stiffness_fraction",
    "effective_brake_distribution_front",
    "effective_aero_balance_front",
    "tire_mu_scale",
    "effective_drive_power_limit_w",
    "drive_power_limit_source",
    "event_slug",
    "event_name",
    "wheel_load_min_n",
    "wheel_load_max_n",
    "wheel_load_range_method",
    "wheel_load_outside_tire_validity",
)

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
    "effective_lltd",
    "effective_lltd_interpretation",
    "front_antiroll_stiffness_fraction",
    "front_elastic_roll_stiffness_fraction",
    "effective_brake_distribution_front",
    "effective_aero_balance_front",
    "effective_cop_from_front_m",
    "tire_mu_scale",
    "drive_distribution_front",
    "limited_slip_differential_model",
    "case_tuning",
    "reduced_vehicle_parameters.center_of_gravity_m",
    "reduced_vehicle_parameters.mass_kg",
    "reduced_vehicle_parameters.sprung_mass_kg",
    "reduced_vehicle_parameters.inertia_kg_m2",
    "reduced_vehicle_parameters.sprung_inertia_kg_m2",
    "reduced_vehicle_parameters.static_wheel_loads_n",
    "reduced_vehicle_parameters.antiroll_stiffness_nm_per_rad",
    "reduced_vehicle_parameters.brake_distribution_front",
)

REQUIRED_METADATA_PATHS = (
    "model_key",
    "model_dof",
    "cg_case",
    "cg_height_in",
    "total_mass_kg",
    "rear_static_weight_fraction",
    "front_antiroll_stiffness_fraction",
    "effective_brake_distribution_front",
    "effective_aero_balance_front",
    "tire_mu_scale",
    "drive_distribution_front",
    "limited_slip_differential_model",
)


def _sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_json(path: Path, payload: object) -> None:
    path.write_text(
        json.dumps(payload, indent=2, allow_nan=True, default=_json_default) + "\n",
        encoding="utf-8",
    )


def _json_default(value: object) -> object:
    if isinstance(value, (np.bool_,)):
        return bool(value)
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    if isinstance(value, Path):
        return str(value)
    raise TypeError(f"Cannot JSON-serialize {type(value).__name__}")


def _read_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(path)
    value = json.loads(path.read_text(encoding="utf-8-sig"))
    if not isinstance(value, dict):
        raise TypeError(f"{path} must contain a JSON object.")
    return value


def _as_bool(value: object) -> bool:
    if isinstance(value, (bool, np.bool_)):
        return bool(value)
    if isinstance(value, (int, float, np.integer, np.floating)):
        if not math.isfinite(float(value)):
            raise ValueError(f"Non-finite boolean value: {value!r}")
        return bool(value)
    normalized = str(value).strip().lower()
    if normalized in {"true", "1", "yes", "y"}:
        return True
    if normalized in {"false", "0", "no", "n", ""}:
        return False
    raise ValueError(f"Cannot interpret {value!r} as boolean.")


def _nested(payload: Mapping[str, Any], dotted_path: str) -> Any:
    value: Any = payload
    for token in dotted_path.split("."):
        if not isinstance(value, Mapping) or token not in value:
            return None
        value = value[token]
    return value


def _equivalent(left: Any, right: Any) -> bool:
    if isinstance(left, Mapping) or isinstance(right, Mapping):
        if not isinstance(left, Mapping) or not isinstance(right, Mapping):
            return False
        if set(left) != set(right):
            return False
        return all(_equivalent(left[key], right[key]) for key in left)
    if isinstance(left, Sequence) and not isinstance(left, (str, bytes)):
        if not isinstance(right, Sequence) or isinstance(right, (str, bytes)):
            return False
        return len(left) == len(right) and all(
            _equivalent(a, b) for a, b in zip(left, right)
        )
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        return math.isclose(float(left), float(right), rel_tol=0.0, abs_tol=1e-10)
    return left == right


def _require_columns(frame: pd.DataFrame, columns: Sequence[str], label: str) -> None:
    missing = sorted(set(columns).difference(frame.columns))
    if missing:
        raise ValueError(f"{label} is missing required columns: {missing}")


def _inside_root(path: Path, root: Path) -> bool:
    try:
        path.resolve().relative_to(root.resolve())
    except ValueError:
        return False
    return True


def _metadata_artifact_manifest(root: Path, case_name: str) -> dict[str, Any]:
    case_dir = root / case_name
    paths = [case_dir / CASE_METADATA_NAME, case_dir / "ggv.csv"]
    for event_slug in EXPECTED_EVENTS:
        paths.extend(
            (
                case_dir / f"{event_slug}_summary.json",
                case_dir / f"{event_slug}_trace.csv",
            )
        )
    missing = [path for path in paths if not path.is_file()]
    if missing:
        raise FileNotFoundError(
            "Missing production case artifacts:\n" + "\n".join(map(str, missing))
        )
    return {
        path.name: {"path": str(path.resolve()), "sha256": _sha256_file(path)}
        for path in paths
    }


def _load_study(root: Path, *, expected_power_w: float) -> dict[str, Any]:
    root = root.resolve()
    event_path = root / EVENT_RESULTS_NAME
    if not event_path.is_file():
        raise FileNotFoundError(event_path)
    events = pd.read_csv(event_path)
    _require_columns(
        events,
        (
            "cg_case",
            "event_slug",
            "lap_time_s",
            "track_length_m",
            "converged",
            "ggv_source_csv",
            "ggv_source_sha256",
            "ggv_speed_cap_segments",
            "model_key",
            "model_dof",
            "effective_drive_power_limit_w",
        ),
        str(event_path),
    )
    if len(events) != len(EXPECTED_CONFIGS) * len(EXPECTED_EVENTS):
        raise ValueError(f"{event_path} must contain exactly eight event rows.")
    if events.duplicated(["cg_case", "event_slug"]).any():
        raise ValueError(f"{event_path} contains duplicate configuration/event rows.")
    if set(events["cg_case"].astype(str)) != set(EXPECTED_CONFIGS):
        raise ValueError(
            f"{event_path} must contain exactly {list(EXPECTED_CONFIGS)}."
        )
    for case_name, group in events.groupby(events["cg_case"].astype(str)):
        if set(group["event_slug"].astype(str)) != set(EXPECTED_EVENTS):
            raise ValueError(f"{case_name} must contain exactly four events.")
    if not events["converged"].map(_as_bool).all():
        raise ValueError(f"{event_path} contains a nonconverged event solve.")
    speed_caps = pd.to_numeric(events["ggv_speed_cap_segments"], errors="raise")
    if not bool((speed_caps == 0).all()):
        raise ValueError(f"{event_path} contains a speed-cap-limited event solve.")
    if set(events["model_key"].astype(str)) != {EXPECTED_MODEL_KEY}:
        raise ValueError(f"{event_path} is not uniformly {EXPECTED_MODEL_KEY}.")
    if set(pd.to_numeric(events["model_dof"], errors="raise").astype(int)) != {6}:
        raise ValueError(f"{event_path} is not uniformly 6DOF.")
    observed_power = pd.to_numeric(
        events["effective_drive_power_limit_w"], errors="raise"
    ).to_numpy(dtype=float)
    if not np.allclose(observed_power, expected_power_w, rtol=0.0, atol=1e-8):
        raise ValueError(f"{event_path} does not use {expected_power_w:g} W.")
    raw_time = pd.to_numeric(events["lap_time_s"], errors="raise").to_numpy(
        dtype=float
    )
    track_length = pd.to_numeric(
        events["track_length_m"], errors="raise"
    ).to_numpy(dtype=float)
    if (
        not np.all(np.isfinite(raw_time))
        or np.any(raw_time <= 0.0)
        or not np.all(np.isfinite(track_length))
        or np.any(track_length <= 0.0)
    ):
        raise ValueError(f"{event_path} contains invalid time or distance data.")

    metadata: dict[str, dict[str, Any]] = {}
    case_artifacts: dict[str, dict[str, Any]] = {}
    for case_name in EXPECTED_CONFIGS:
        case_dir = root / case_name
        metadata_path = case_dir / CASE_METADATA_NAME
        payload = _read_json(metadata_path)
        for dotted_path in REQUIRED_METADATA_PATHS:
            if _nested(payload, dotted_path) is None:
                raise ValueError(f"{metadata_path} omits {dotted_path!r}.")
        if str(payload["cg_case"]) != case_name:
            raise ValueError(f"{metadata_path} has the wrong cg_case.")
        if str(payload["model_key"]) != EXPECTED_MODEL_KEY or int(
            payload["model_dof"]
        ) != 6:
            raise ValueError(f"{metadata_path} is not the required 6DOF model.")
        if _as_bool(payload.get("limited_slip_differential_model", False)):
            raise ValueError(f"{metadata_path} unexpectedly enables an LSD model.")
        if not math.isclose(
            float(payload.get("effective_drive_power_limit_w", math.nan)),
            expected_power_w,
            rel_tol=0.0,
            abs_tol=1e-8,
        ):
            raise ValueError(f"{metadata_path} has the wrong power limit.")
        map_path = (case_dir / "ggv.csv").resolve()
        group = events[events["cg_case"].astype(str) == case_name]
        event_paths = {Path(str(value)).resolve() for value in group["ggv_source_csv"]}
        if event_paths != {map_path} or not _inside_root(map_path, root):
            raise ValueError(f"{case_name} event rows do not use their root-local GGV.")
        map_hash = _sha256_file(map_path)
        if set(group["ggv_source_sha256"].astype(str)) != {map_hash}:
            raise ValueError(f"{case_name} event rows have the wrong GGV hash.")
        if str(payload.get("ggv_source_sha256", "")) != map_hash:
            raise ValueError(f"{case_name} metadata has the wrong GGV hash.")
        event_metadata_columns = {
            "cg_height_in": "cg_height_in",
            "total_mass_kg": "total_mass_kg",
            "rear_static_weight_fraction": "rear_static_weight_fraction",
            "front_static_weight_fraction": "front_static_weight_fraction",
            "front_antiroll_stiffness_fraction": (
                "front_antiroll_stiffness_fraction"
            ),
            "effective_brake_distribution_front": (
                "effective_brake_distribution_front"
            ),
            "effective_aero_balance_front": "effective_aero_balance_front",
            "tire_mu_scale": "tire_mu_scale",
        }
        for column, metadata_field in event_metadata_columns.items():
            if column not in group.columns or payload.get(metadata_field) is None:
                continue
            observed = pd.to_numeric(group[column], errors="raise").to_numpy(
                dtype=float
            )
            if not np.allclose(
                observed,
                float(payload[metadata_field]),
                rtol=0.0,
                atol=1e-10,
            ):
                raise ValueError(
                    f"{case_name} event rows disagree with metadata field "
                    f"{metadata_field!r}."
                )
        metadata[case_name] = payload
        case_artifacts[case_name] = _metadata_artifact_manifest(root, case_name)

    optional_artifacts: dict[str, dict[str, str]] = {}
    for name in (SOURCE_LOCK_NAME, SMOKE_SUMMARY_NAME):
        path = root / name
        if path.is_file():
            optional_artifacts[name] = {
                "path": str(path.resolve()),
                "sha256": _sha256_file(path),
            }
    return {
        "root": root,
        "events": events,
        "metadata": metadata,
        "provenance": {
            "root": str(root),
            "expected_power_w": expected_power_w,
            "event_results": {
                "path": str(event_path.resolve()),
                "sha256": _sha256_file(event_path),
            },
            "root_artifacts": optional_artifacts,
            "case_artifacts": case_artifacts,
        },
    }


def _validate_controlled_vehicle_match(
    study_80kw: Mapping[str, Any], study_32kw: Mapping[str, Any]
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for case_name in EXPECTED_CONFIGS:
        left = study_80kw["metadata"][case_name]
        right = study_32kw["metadata"][case_name]
        for dotted_path in CONTROLLED_METADATA_PATHS:
            left_value = _nested(left, dotted_path)
            right_value = _nested(right, dotted_path)
            if left_value is None and right_value is None:
                continue
            if not _equivalent(left_value, right_value):
                raise ValueError(
                    f"80/32 kW controlled metadata differs for {case_name} "
                    f"at {dotted_path!r}."
                )
        rows.append(
            {
                "cg_case": case_name,
                "config_label": CONFIG_LABELS[case_name],
                "controlled_metadata_match": True,
            }
        )
    return rows


def _clean_source_events(events: pd.DataFrame) -> pd.DataFrame:
    stale = [
        column
        for column in events.columns
        if column.startswith("common_") or column in SCORING_COLUMNS_TO_DROP
    ]
    clean = events.drop(columns=stale, errors="ignore").copy()
    columns = [column for column in EVENT_OUTPUT_COLUMNS if column in clean.columns]
    return clean.loc[:, columns]


def _assemble_mixed_events(
    study_80kw: Mapping[str, Any], study_32kw: Mapping[str, Any]
) -> pd.DataFrame:
    sprint = study_80kw["events"][
        study_80kw["events"]["event_slug"].astype(str).isin(SPRINT_EVENTS)
    ].copy()
    endurance = study_32kw["events"][
        study_32kw["events"]["event_slug"].astype(str) == ENDURANCE_EVENT
    ].copy()
    sprint = _clean_source_events(sprint)
    endurance = _clean_source_events(endurance)
    sprint["mixed_power_source"] = "fresh_80kw_sprint"
    sprint["mixed_power_source_root"] = str(study_80kw["root"])
    sprint["mixed_power_power_limit_w"] = EXPECTED_POWER_W["80kw"]
    endurance["mixed_power_source"] = "fresh_32kw_endurance"
    endurance["mixed_power_source_root"] = str(study_32kw["root"])
    endurance["mixed_power_power_limit_w"] = EXPECTED_POWER_W["32kw"]
    mixed = pd.concat((sprint, endurance), ignore_index=True, sort=False)
    if len(mixed) != len(EXPECTED_CONFIGS) * len(EXPECTED_EVENTS):
        raise AssertionError("Mixed-power assembly did not produce eight rows.")
    if mixed.duplicated(["cg_case", "event_slug"]).any():
        raise AssertionError("Mixed-power assembly produced duplicate rows.")
    mixed["config_label"] = mixed["cg_case"].map(CONFIG_LABELS)
    mixed["event_display_name"] = mixed["event_slug"].map(EVENT_LABELS)
    mixed["all_track_solvers_converged"] = mixed["converged"].map(_as_bool)
    return mixed


def _build_event_deltas(rescored: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for event_slug in EXPECTED_EVENTS:
        group = rescored[rescored["event_slug"] == event_slug].set_index("cg_case")
        if set(group.index.astype(str)) != set(EXPECTED_CONFIGS):
            raise AssertionError(f"{event_slug} does not contain both configurations.")
        config_1 = group.loc[CONFIG_1]
        config_2 = group.loc[CONFIG_2]
        raw_delta = float(config_2["lap_time_s"] - config_1["lap_time_s"])
        competition_delta = float(
            config_2["common_projected_competition_time_s"]
            - config_1["common_projected_competition_time_s"]
        )
        points_delta = float(
            config_2["common_projected_points"]
            - config_1["common_projected_points"]
        )
        rows.append(
            {
                "event_slug": event_slug,
                "event_name": EVENT_LABELS[event_slug],
                "config_1_case": CONFIG_1,
                "config_2_case": CONFIG_2,
                "config_1_raw_time_s": float(config_1["lap_time_s"]),
                "config_2_raw_time_s": float(config_2["lap_time_s"]),
                "config_2_minus_config_1_raw_time_s": raw_delta,
                "config_2_minus_config_1_raw_time_percent": float(
                    100.0 * raw_delta / float(config_1["lap_time_s"])
                ),
                "negative_raw_time_delta_is_faster": True,
                "config_1_competition_time_s": float(
                    config_1["common_projected_competition_time_s"]
                ),
                "config_2_competition_time_s": float(
                    config_2["common_projected_competition_time_s"]
                ),
                "config_2_minus_config_1_competition_time_s": competition_delta,
                "config_1_points": float(config_1["common_projected_points"]),
                "config_2_points": float(config_2["common_projected_points"]),
                "config_2_minus_config_1_points": points_delta,
                "positive_points_delta_favors_config_2": True,
                "common_field_tmin_s": float(config_1["common_field_tmin_s"]),
                "common_field_tmin_source": str(
                    config_1["common_field_tmin_source"]
                ),
            }
        )
    return pd.DataFrame(rows)


def _build_case_totals(rescored: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for case_name in EXPECTED_CONFIGS:
        group = rescored[rescored["cg_case"] == case_name]
        if set(group["event_slug"].astype(str)) != set(EXPECTED_EVENTS):
            raise AssertionError(f"{case_name} does not contain four events.")
        row: dict[str, Any] = {
            "cg_case": case_name,
            "config_label": CONFIG_LABELS[case_name],
            "timed_event_points": float(group["common_projected_points"].sum()),
            "maximum_timed_event_points": MAXIMUM_TIMED_EVENT_POINTS,
            "all_selected_track_solvers_converged": bool(
                group["all_track_solvers_converged"].all()
            ),
            "selected_ggv_speed_cap_segments": int(
                pd.to_numeric(group["ggv_speed_cap_segments"], errors="raise").sum()
            ),
        }
        for event_slug in EXPECTED_EVENTS:
            event = group[group["event_slug"] == event_slug].iloc[0]
            row[f"{event_slug}_raw_time_s"] = float(event["lap_time_s"])
            row[f"{event_slug}_competition_time_s"] = float(
                event["common_projected_competition_time_s"]
            )
            row[f"{event_slug}_points"] = float(event["common_projected_points"])
        rows.append(row)
    result = pd.DataFrame(rows)
    baseline = float(
        result.loc[result["cg_case"] == CONFIG_1, "timed_event_points"].iloc[0]
    )
    result["timed_event_points_delta_vs_config_1"] = (
        result["timed_event_points"] - baseline
    )
    result["rank"] = (
        result["timed_event_points"].rank(ascending=False, method="min").astype(int)
    )
    order = pd.Categorical(result["cg_case"], EXPECTED_CONFIGS, ordered=True)
    return result.assign(_order=order).sort_values("_order").drop(columns="_order")


def _first_present(payload: Mapping[str, Any], *paths: str) -> Any:
    for path in paths:
        value = _nested(payload, path)
        if value is not None:
            return value
    return None


def _build_tuning_validity(
    study_80kw: Mapping[str, Any], study_32kw: Mapping[str, Any]
) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for case_name in EXPECTED_CONFIGS:
        metadata_80 = study_80kw["metadata"][case_name]
        metadata_32 = study_32kw["metadata"][case_name]
        event_80 = study_80kw["events"][
            study_80kw["events"]["cg_case"].astype(str) == case_name
        ].iloc[0]
        event_32 = study_32kw["events"][
            study_32kw["events"]["cg_case"].astype(str) == case_name
        ].iloc[0]
        center = _first_present(
            metadata_80, "reduced_vehicle_parameters.center_of_gravity_m"
        )
        cg_x_m = (
            float(center[0])
            if isinstance(center, Sequence) and len(center) >= 1
            else math.nan
        )
        row = {
            "cg_case": case_name,
            "config_label": CONFIG_LABELS[case_name],
            "cg_height_in": float(metadata_80["cg_height_in"]),
            "cg_height_m": float(metadata_80.get("cg_height_m", math.nan)),
            "cg_x_m": cg_x_m,
            "rear_static_weight_fraction": float(
                metadata_80["rear_static_weight_fraction"]
            ),
            "rear_static_weight_percent": 100.0
            * float(metadata_80["rear_static_weight_fraction"]),
            "front_static_weight_fraction": float(
                metadata_80.get("front_static_weight_fraction", math.nan)
            ),
            "total_mass_kg": float(metadata_80["total_mass_kg"]),
            "sprung_mass_kg": float(metadata_80.get("sprung_mass_kg", math.nan)),
            "front_antiroll_stiffness_fraction": float(
                metadata_80["front_antiroll_stiffness_fraction"]
            ),
            "front_elastic_roll_stiffness_fraction": float(
                metadata_80.get(
                    "front_elastic_roll_stiffness_fraction", math.nan
                )
            ),
            "front_brake_bias": float(
                metadata_80["effective_brake_distribution_front"]
            ),
            "front_aero_balance": float(metadata_80["effective_aero_balance_front"]),
            "tire_mu_scale": float(metadata_80["tire_mu_scale"]),
            "drive_distribution_front": float(metadata_80["drive_distribution_front"]),
            "limited_slip_differential_model": _as_bool(
                metadata_80["limited_slip_differential_model"]
            ),
            "case_tuning_json": json.dumps(
                metadata_80.get("case_tuning"), sort_keys=True, allow_nan=True
            ),
            "metadata_80kw_32kw_controlled_match": True,
            "wheel_load_min_n_80kw": float(
                metadata_80.get("wheel_load_min_n", event_80.get("wheel_load_min_n", math.nan))
            ),
            "wheel_load_max_n_80kw": float(
                metadata_80.get("wheel_load_max_n", event_80.get("wheel_load_max_n", math.nan))
            ),
            "wheel_load_outside_tire_validity_80kw": _as_bool(
                metadata_80.get(
                    "wheel_load_outside_tire_validity",
                    event_80.get("wheel_load_outside_tire_validity", False),
                )
            ),
            "wheel_load_min_n_32kw": float(
                metadata_32.get("wheel_load_min_n", event_32.get("wheel_load_min_n", math.nan))
            ),
            "wheel_load_max_n_32kw": float(
                metadata_32.get("wheel_load_max_n", event_32.get("wheel_load_max_n", math.nan))
            ),
            "wheel_load_outside_tire_validity_32kw": _as_bool(
                metadata_32.get(
                    "wheel_load_outside_tire_validity",
                    event_32.get("wheel_load_outside_tire_validity", False),
                )
            ),
            "tire_valid_load_min_n": float(
                metadata_80.get("tire_valid_load_min_n", math.nan)
            ),
            "tire_valid_load_max_n": float(
                metadata_80.get("tire_valid_load_max_n", math.nan)
            ),
        }
        rows.append(row)
    return pd.DataFrame(rows)


def _plot_delta_bars(
    event_deltas: pd.DataFrame,
    *,
    value_column: str,
    path: Path,
    title: str,
    xlabel: str,
    positive_is_better: bool,
) -> None:
    ordered = event_deltas.set_index("event_slug").loc[list(EXPECTED_EVENTS)].reset_index()
    values = ordered[value_column].to_numpy(dtype=float)
    favorable = values > 0.0 if positive_is_better else values < 0.0
    colors = np.where(favorable, "#16803a", "#b42318")
    colors = np.where(np.isclose(values, 0.0, rtol=0.0, atol=1e-15), "#6b7280", colors)
    fig, axis = plt.subplots(figsize=(9.4, 5.6), constrained_layout=True)
    y = np.arange(len(ordered))
    bars = axis.barh(y, values, color=colors, alpha=0.9)
    axis.axvline(0.0, color="#111827", linewidth=1.0)
    axis.set_yticks(y, ordered["event_name"])
    axis.invert_yaxis()
    axis.set_xlabel(xlabel)
    axis.set_title(title, fontweight="bold")
    axis.grid(axis="x", alpha=0.25)
    largest = max(float(np.max(np.abs(values))), 1e-9)
    axis.set_xlim(
        min(0.0, float(np.min(values))) - 0.18 * largest,
        max(0.0, float(np.max(values))) + 0.18 * largest,
    )
    for bar, value in zip(bars, values):
        offset = 4 if value >= 0.0 else -4
        axis.annotate(
            f"{value:+.4f}",
            (value, bar.get_y() + bar.get_height() / 2.0),
            xytext=(offset, 0),
            textcoords="offset points",
            va="center",
            ha="left" if value >= 0.0 else "right",
            fontsize=9,
        )
    fig.savefig(path, dpi=180)
    plt.close(fig)


def _markdown_table(headers: Sequence[str], rows: Sequence[Sequence[Any]]) -> str:
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    lines.extend("| " + " | ".join(str(value) for value in row) + " |" for row in rows)
    return "\n".join(lines)


def _write_report(
    path: Path,
    *,
    totals: pd.DataFrame,
    event_deltas: pd.DataFrame,
    tuning_validity: pd.DataFrame,
    event_scoring: Mapping[str, Mapping[str, Any]],
    study_80kw_root: Path,
    study_32kw_root: Path,
) -> None:
    ranked = totals.sort_values(["rank", "cg_case"])
    best = ranked.iloc[0]
    total_rows = [
        (
            row.config_label,
            f"{row.timed_event_points:.3f}",
            f"{row.timed_event_points_delta_vs_config_1:+.3f}",
            int(row.rank),
        )
        for row in totals.itertuples(index=False)
    ]
    event_rows = [
        (
            row.event_name,
            f"{row.config_1_raw_time_s:.6f}",
            f"{row.config_2_raw_time_s:.6f}",
            f"{row.config_2_minus_config_1_raw_time_s:+.6f}",
            f"{row.config_1_points:.3f}",
            f"{row.config_2_points:.3f}",
            f"{row.config_2_minus_config_1_points:+.3f}",
        )
        for row in event_deltas.itertuples(index=False)
    ]
    setup_rows = [
        (
            row.config_label,
            f"{row.cg_height_in:.3f}",
            f"{row.rear_static_weight_percent:.3f}",
            f"{row.front_antiroll_stiffness_fraction:.6f}",
            f"{row.front_brake_bias:.6f}",
            f"{row.total_mass_kg:.3f}",
            bool(row.wheel_load_outside_tire_validity_80kw),
            bool(row.wheel_load_outside_tire_validity_32kw),
        )
        for row in tuning_validity.itertuples(index=False)
    ]
    scoring_rows = [
        (
            EVENT_LABELS[event_slug],
            f"{summary['real_field_fastest_time_s']:.6f}",
            f"{summary['simulated_fastest_time_s']:.6f}",
            f"{summary['common_field_tmin_s']:.6f}",
            summary["common_field_tmin_source"],
        )
        for event_slug, summary in event_scoring.items()
    ]
    text = f"""# Two-configuration mixed-power comparison

## Result

The best observed configuration is **{best.config_label}** at
**{best.timed_event_points:.3f} / 575 timed-event points**. Acceleration,
skidpad, and autocross use the fresh 80 kW root; Michigan endurance uses the
fresh 32 kW root. Both configurations were rescored together once against the
same 2026 Michigan EV reference field.

{_markdown_table(("Configuration", "Total points", "Delta vs Config 1", "Rank"), total_rows)}

## Per-event comparison

Negative raw-time delta means Config 2 is faster. Positive point delta favors
Config 2.

{_markdown_table(("Event", "Config 1 raw s", "Config 2 raw s", "C2-C1 raw s", "Config 1 pts", "Config 2 pts", "C2-C1 pts"), event_rows)}

![Per-event point delta](two_config_event_points_delta_vs_config1.png)

![Raw event-time delta](two_config_raw_time_delta_vs_config1.png)

## Tuning and validity

{_markdown_table(("Configuration", "CG height in", "Rear wt %", "Front ARB frac", "Front brake frac", "Mass kg", "80 kW outside TIR", "32 kW outside TIR"), setup_rows)}

The source roots must contain exactly the two named configurations, all eight
event solves must converge without touching a GGV speed cap, each event must
match its root-local GGV hash, and power-independent controlled metadata must
match between the 80 and 32 kW roots.

## Common scoring field

Only a simulated time exactly equal to the real-or-simulated Tmin receives the
event maximum. Efficiency points are excluded.

{_markdown_table(("Event", "Real fastest s", "Sim fastest s", "Common Tmin s", "Tmin source"), scoring_rows)}

## Provenance

- 80 kW sprint root: `{study_80kw_root.resolve()}`
- 32 kW endurance root: `{study_32kw_root.resolve()}`
- Complete hashes and row mapping: `two_config_provenance.json`
- Machine-readable acceptance checks: `validation_summary.json`
"""
    path.write_text(text, encoding="utf-8")


def build_two_config_mixed_power_report(
    *,
    study_80kw_root: Path,
    study_32kw_root: Path,
    output_root: Path,
    scoring_reference_path: Path = DEFAULT_SCORING_REFERENCE,
) -> dict[str, Any]:
    study_80kw_root = study_80kw_root.resolve()
    study_32kw_root = study_32kw_root.resolve()
    output_root = output_root.resolve()
    scoring_reference_path = scoring_reference_path.resolve()

    study_80kw = _load_study(
        study_80kw_root, expected_power_w=EXPECTED_POWER_W["80kw"]
    )
    study_32kw = _load_study(
        study_32kw_root, expected_power_w=EXPECTED_POWER_W["32kw"]
    )
    controlled_match = _validate_controlled_vehicle_match(study_80kw, study_32kw)
    raw_mixed = _assemble_mixed_events(study_80kw, study_32kw)
    scoring_reference = _load_scoring_reference(scoring_reference_path)
    rescored, event_scoring = _rescore_common_field(raw_mixed, scoring_reference)
    _validate_exact_tmin_scoring(rescored)
    event_deltas = _build_event_deltas(rescored)
    totals = _build_case_totals(rescored)
    tuning_validity = _build_tuning_validity(study_80kw, study_32kw)

    output_root.mkdir(parents=True, exist_ok=True)
    event_results_path = output_root / "two_config_event_results.csv"
    case_points_path = output_root / "two_config_case_points.csv"
    event_deltas_path = output_root / "two_config_event_deltas_vs_config1.csv"
    tuning_path = output_root / "two_config_tuning_validity.csv"
    points_plot_path = output_root / "two_config_event_points_delta_vs_config1.png"
    time_plot_path = output_root / "two_config_raw_time_delta_vs_config1.png"
    report_path = output_root / "two_config_mixed_power_report.md"

    config_order = pd.Categorical(
        rescored["cg_case"], EXPECTED_CONFIGS, ordered=True
    )
    event_order = pd.Categorical(
        rescored["event_slug"], EXPECTED_EVENTS, ordered=True
    )
    rescored.assign(_config_order=config_order, _event_order=event_order).sort_values(
        ["_config_order", "_event_order"]
    ).drop(columns=["_config_order", "_event_order"]).to_csv(
        event_results_path, index=False
    )
    totals.to_csv(case_points_path, index=False)
    event_deltas.to_csv(event_deltas_path, index=False)
    tuning_validity.to_csv(tuning_path, index=False)
    _plot_delta_bars(
        event_deltas,
        value_column="config_2_minus_config_1_points",
        path=points_plot_path,
        title="Config 2 - Config 1 event-point change",
        xlabel="Event-point delta (points; positive favors Config 2)",
        positive_is_better=True,
    )
    _plot_delta_bars(
        event_deltas,
        value_column="config_2_minus_config_1_raw_time_s",
        path=time_plot_path,
        title="Config 2 - Config 1 raw event-time change",
        xlabel="Raw simulated time delta (s; negative is faster)",
        positive_is_better=False,
    )
    _write_report(
        report_path,
        totals=totals,
        event_deltas=event_deltas,
        tuning_validity=tuning_validity,
        event_scoring=event_scoring,
        study_80kw_root=study_80kw_root,
        study_32kw_root=study_32kw_root,
    )

    script_path = Path(__file__).resolve()
    output_artifacts = {
        path.name: {"path": str(path.resolve()), "sha256": _sha256_file(path)}
        for path in (
            event_results_path,
            case_points_path,
            event_deltas_path,
            tuning_path,
            points_plot_path,
            time_plot_path,
            report_path,
        )
    }
    provenance = {
        "schema": PROVENANCE_SCHEMA,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "configuration_order": list(EXPECTED_CONFIGS),
        "configuration_labels": CONFIG_LABELS,
        "event_source_mapping": {
            "acceleration": "fresh_80kw_sprint",
            "skidpad": "fresh_80kw_sprint",
            "autocross": "fresh_80kw_sprint",
            ENDURANCE_EVENT: "fresh_32kw_endurance",
        },
        "study_80kw": study_80kw["provenance"],
        "study_32kw": study_32kw["provenance"],
        "controlled_metadata_comparison": controlled_match,
        "scoring_reference": {
            "path": str(scoring_reference_path),
            "sha256": _sha256_file(scoring_reference_path),
        },
        "builder": {"path": str(script_path), "sha256": _sha256_file(script_path)},
        "selected_rows": rescored[
            [
                "cg_case",
                "event_slug",
                "mixed_power_source",
                "mixed_power_source_root",
                "ggv_source_csv",
                "ggv_source_sha256",
                "lap_time_s",
                "common_projected_competition_time_s",
                "common_projected_points",
            ]
        ].to_dict(orient="records"),
        "output_artifacts": output_artifacts,
    }
    provenance_path = output_root / "two_config_provenance.json"
    _write_json(provenance_path, provenance)

    exact_tmin = (
        rescored["common_projected_competition_time_s"].to_numpy(dtype=float)
        == rescored["common_field_tmin_s"].to_numpy(dtype=float)
    )
    maximum_score = (
        rescored["common_projected_points"].to_numpy(dtype=float)
        == rescored["common_maximum_points"].to_numpy(dtype=float)
    )
    baseline_delta = float(
        totals.loc[
            totals["cg_case"] == CONFIG_1,
            "timed_event_points_delta_vs_config_1",
        ].iloc[0]
    )
    validation = {
        "status": "passed",
        "case_count": len(totals),
        "event_row_count": len(rescored),
        "expected_configuration_order": list(EXPECTED_CONFIGS),
        "all_sprint_rows_from_fresh_80kw_root": bool(
            (
                rescored.loc[
                    rescored["event_slug"].isin(SPRINT_EVENTS),
                    "mixed_power_source",
                ]
                == "fresh_80kw_sprint"
            ).all()
        ),
        "all_endurance_rows_from_fresh_32kw_root": bool(
            (
                rescored.loc[
                    rescored["event_slug"] == ENDURANCE_EVENT,
                    "mixed_power_source",
                ]
                == "fresh_32kw_endurance"
            ).all()
        ),
        "all_selected_solves_converged": bool(
            rescored["all_track_solvers_converged"].all()
        ),
        "all_selected_speed_cap_segments_zero": bool(
            (pd.to_numeric(rescored["ggv_speed_cap_segments"]) == 0).all()
        ),
        "all_selected_ggv_paths_and_hashes_verified": True,
        "all_80kw_32kw_controlled_metadata_match": bool(
            all(row["controlled_metadata_match"] for row in controlled_match)
        ),
        "one_common_2026_real_plus_two_sim_scoring_field": True,
        "only_exact_common_tmin_ties_receive_maximum": bool(
            np.array_equal(exact_tmin, maximum_score)
        ),
        "config_1_total_delta_exactly_zero": baseline_delta == 0.0,
        "maximum_timed_event_points": MAXIMUM_TIMED_EVENT_POINTS,
        "efficiency_points_included": False,
        "best_observed_configuration": str(
            totals.loc[totals["timed_event_points"].idxmax(), "cg_case"]
        ),
        "best_observed_timed_event_points": float(
            totals["timed_event_points"].max()
        ),
        "event_scoring": event_scoring,
        "provenance_schema": PROVENANCE_SCHEMA,
        "provenance_sha256": _sha256_file(provenance_path),
        "artifacts": sorted(
            [*output_artifacts, provenance_path.name, "validation_summary.json"]
        ),
    }
    required_checks = (
        "all_sprint_rows_from_fresh_80kw_root",
        "all_endurance_rows_from_fresh_32kw_root",
        "all_selected_solves_converged",
        "all_selected_speed_cap_segments_zero",
        "all_selected_ggv_paths_and_hashes_verified",
        "all_80kw_32kw_controlled_metadata_match",
        "one_common_2026_real_plus_two_sim_scoring_field",
        "only_exact_common_tmin_ties_receive_maximum",
        "config_1_total_delta_exactly_zero",
    )
    if not all(bool(validation[key]) for key in required_checks):
        validation["status"] = "failed"
        raise AssertionError(f"Two-configuration validation failed: {validation}")
    _write_json(output_root / "validation_summary.json", validation)
    return validation


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--study-80kw-root", type=Path, required=True)
    parser.add_argument("--study-32kw-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument(
        "--scoring-reference",
        type=Path,
        default=DEFAULT_SCORING_REFERENCE,
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    validation = build_two_config_mixed_power_report(
        study_80kw_root=args.study_80kw_root,
        study_32kw_root=args.study_32kw_root,
        output_root=args.output_root,
        scoring_reference_path=args.scoring_reference,
    )
    print(json.dumps(validation, indent=2, default=_json_default))


if __name__ == "__main__":
    main()
