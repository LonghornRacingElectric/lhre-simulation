"""Run an EnvelopeSim-GGV dynamic-event vehicle-sensitivity study.

By default, the study keeps every projected vehicle input fixed except total
CG height. An optional analysis-only case-tuning file can prescribe one legacy
LLTD or reduced-model front anti-roll fraction plus one fixed front brake
fraction for each of the legacy three cases. A sweep definition can instead
provide arbitrary absolute CG heights, sprung masses, or longitudinal static
weight distributions with per-case tuning. It can also compare explicitly
named configurations in which CG height and longitudinal position change
together without misreporting a one-axis correlation. Each generated GGV is
propagated through the same acceleration, skidpad, autocross, and endurance
track meshes.

Event points are scored in a combined field with the official valid 2026
Formula SAE Electric Michigan results. The single fastest real or simulated
time receives the maximum score; only exact fastest-time ties share it. Raw
time deltas remain referenced to the declared reference case (or the case
nearest the projected nominal height).

SPDX-License-Identifier: GPL-3.0-or-later
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import sys
from dataclasses import asdict, replace
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from reduced_ggv_bridge import (
    DEFAULT_QSS_ZERO_PROXY_MPS,
    DEFAULT_SPEED_STEP_MPS,
    DEFAULT_TOP_SPEED_MPS,
    cache_fingerprint,
    default_output_root,
    dense_speed_slices,
    elastic_roll_stiffness,
    generate_envelopes,
    model_identity,
    qss_generation_speeds,
)

LAPSIMS_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = LAPSIMS_ROOT.parents[2]
DEFAULT_BOBSIM_ROOT = WORKSPACE_ROOT / "BobDyn" / "BobSim"
DEFAULT_OUTPUT_ROOT = LAPSIMS_ROOT / "outputs" / "ggv_cg_smoke"
CASE_CACHE_FILENAME = "raw_case_results.json"
GGV_LOOKUP_CONTRACT_VERSION = "zero-width-origin-boundary-v1"
GGV_LOOKUP_SOLVER_PATH = Path(__file__).with_name("ggv_lookup_solver.py")
EVENT_MANIFEST = (
    LAPSIMS_ROOT / "inputs" / "events" / "openlap_event_suite_manifest.json"
)
FSAE_2026_RESULTS_REFERENCE = (
    LAPSIMS_ROOT / "inputs" / "fsae_ev_michigan_2026_scoring.json"
)
LEGACY_EVENT_SUMMARY = (
    LAPSIMS_ROOT / "outputs" / "events" / "openlap_event_suite_summary.json"
)
CG_OFFSETS_M = (-0.050, 0.0, 0.050)
CASE_NAMES = ("low", "baseline", "high")
M_PER_INCH = 0.0254
SAFE_CASE_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
TIRE_MU_SCALE_FIELDS = (
    "pdx1",
    "pdx2",
    "pdy1",
    "pdy2",
    "friction_floor",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bobsim-root", type=Path, default=DEFAULT_BOBSIM_ROOT)
    parser.add_argument(
        "--output-root",
        type=Path,
        default=None,
        help=(
            "Output directory. Defaults to ggv_cg_smoke for the legacy backend "
            "or a model-specific ggv_cg_smoke_dyn_py_<N>dof directory."
        ),
    )
    parser.add_argument(
        "--model-dof",
        type=int,
        choices=(3, 6, 10, 14),
        default=None,
        help=(
            "Explicit BobSim dyn_py QSS backend. Omit to retain the legacy "
            "EnvelopeSim algebraic generator."
        ),
    )
    parser.add_argument("--ay-points", type=int, default=121)
    parser.add_argument(
        "--ay-max-g",
        type=float,
        default=3.2,
        help="Positive lateral search ceiling in g; default 3.2 g.",
    )
    parser.add_argument("--ax-search-points", type=int, default=301)
    parser.add_argument(
        "--ax-binary-iterations",
        type=int,
        default=None,
        help="Boundary-refinement iterations; omit to preserve BobSim config.",
    )
    parser.add_argument(
        "--ggv-top-speed-mps",
        type=float,
        default=DEFAULT_TOP_SPEED_MPS,
        help="Exact final GGV speed slice; defaults to the driveline rpm ceiling.",
    )
    parser.add_argument(
        "--ggv-speed-step-mps",
        type=float,
        default=DEFAULT_SPEED_STEP_MPS,
        help="Dense GGV speed spacing below the exact final slice.",
    )
    parser.add_argument(
        "--qss-zero-proxy-mps",
        type=float,
        default=DEFAULT_QSS_ZERO_PROXY_MPS,
        help=(
            "Positive QSS trim speed copied to 0 m/s for reduced-model "
            "standing starts. Ignored by the legacy backend."
        ),
    )
    parser.add_argument(
        "--aero-balance-front",
        type=float,
        default=None,
        help="Optional analysis-only front downforce fraction override.",
    )
    parser.add_argument(
        "--tire-mu-scale",
        type=float,
        default=None,
        help=(
            "Temporary study-only multiplier for PDX1/PDX2/PDY1/PDY2 and the "
            "friction floor. Defaults to a sweep JSON tire_mu_scale when present, "
            "otherwise 1.0/raw; a CLI value must match a declared sweep value."
        ),
    )
    parser.add_argument(
        "--drive-power-limit-kw",
        type=float,
        default=None,
        help=(
            "Temporary study-only peak drive-power limit in kW. Defaults to a "
            "sweep JSON drive_power_limit_kw when present, otherwise the live "
            "vehicle projection; a CLI value must match a declared sweep value."
        ),
    )
    parser.add_argument(
        "--case-tuning-json",
        type=Path,
        default=None,
        help=(
            "Optional low/baseline/high legacy LLTD or reduced-model front ARB "
            "fraction plus fixed brake-bias overrides."
        ),
    )
    parser.add_argument(
        "--sweep-json",
        type=Path,
        default=None,
        help=(
            "Optional arbitrary sweep definition with absolute CG heights and "
            "optional per-case legacy LLTD or reduced-model front ARB fraction "
            "plus fixed front brake bias. Cannot be combined with "
            "--case-tuning-json."
        ),
    )
    parser.add_argument(
        "--case",
        action="append",
        default=None,
        help=(
            "Generate only the named sweep case. Repeat for multiple cases. "
            "Filtered runs write case-local caches but intentionally skip scoring."
        ),
    )
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Reuse case-local results only when their full input hash matches.",
    )
    parser.add_argument(
        "--reuse-existing-ggv",
        action="store_true",
        help=(
            "Finish exactly one failed reduced-model --case from its existing "
            "ggv.csv without regenerating. The case metadata, full input identity, "
            "speed grid, map domain, and standing-start capability are validated "
            "before event traces and the cache are rebuilt."
        ),
    )
    parser.add_argument(
        "--expected-ggv-sha256",
        default=None,
        help=(
            "Required with --reuse-existing-ggv; binds recovery to the exact "
            "previously generated ggv.csv bytes."
        ),
    )
    return parser.parse_args()


def _positive_tire_mu_scale(value: object, *, source: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or float(value) <= 0.0
    ):
        raise ValueError(f"{source} tire mu scale must be finite and positive.")
    return float(value)


def _resolve_tire_mu_scale(
    cli_value: object,
    sweep_definition: dict[str, Any] | None,
) -> tuple[float, str]:
    """Resolve one study-wide tire scale without silently conflicting sources."""

    cli_scale = (
        None if cli_value is None else _positive_tire_mu_scale(cli_value, source="CLI")
    )
    sweep_raw = (
        None if sweep_definition is None else sweep_definition.get("tire_mu_scale")
    )
    sweep_scale = (
        None
        if sweep_raw is None
        else _positive_tire_mu_scale(sweep_raw, source="Sweep JSON")
    )
    if (
        cli_scale is not None
        and sweep_scale is not None
        and not math.isclose(cli_scale, sweep_scale, rel_tol=0.0, abs_tol=1e-15)
    ):
        raise ValueError(
            "--tire-mu-scale conflicts with sweep JSON tire_mu_scale; use one "
            "consistent study-wide value."
        )
    if cli_scale is not None:
        return cli_scale, "command_line"
    if sweep_scale is not None:
        return sweep_scale, "sweep_json"
    return 1.0, "default_raw_tir"


def _positive_drive_power_kw(value: object, *, source: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or float(value) <= 0.0
    ):
        raise ValueError(f"{source} drive power limit must be finite and positive.")
    return float(value)


def _resolve_drive_power_limit_w(
    cli_value_kw: object,
    sweep_definition: dict[str, Any] | None,
) -> tuple[float | None, str]:
    """Resolve one study-wide temporary drive-power cap without conflicts."""

    cli_kw = (
        None
        if cli_value_kw is None
        else _positive_drive_power_kw(cli_value_kw, source="CLI")
    )
    sweep_raw_kw = (
        None
        if sweep_definition is None
        else sweep_definition.get("drive_power_limit_kw")
    )
    sweep_kw = (
        None
        if sweep_raw_kw is None
        else _positive_drive_power_kw(sweep_raw_kw, source="Sweep JSON")
    )
    if (
        cli_kw is not None
        and sweep_kw is not None
        and not math.isclose(cli_kw, sweep_kw, rel_tol=0.0, abs_tol=1e-12)
    ):
        raise ValueError(
            "--drive-power-limit-kw conflicts with sweep JSON "
            "drive_power_limit_kw; use one consistent study-wide value."
        )
    if cli_kw is not None:
        return 1000.0 * cli_kw, "command_line"
    if sweep_kw is not None:
        return 1000.0 * sweep_kw, "sweep_json"
    return None, "live_vehicle_projection"


def _scale_ggv_tire_peak_parameters(vehicle: Any, scale: float) -> Any:
    """Apply the prior EnvelopeSim mu-scale convention to a GGV parameter set."""

    if scale == 1.0:
        return vehicle
    return replace(
        vehicle,
        pdx1=float(vehicle.pdx1) * scale,
        pdx2=float(vehicle.pdx2) * scale,
        pdy1=float(vehicle.pdy1) * scale,
        pdy2=float(vehicle.pdy2) * scale,
        mu_min=float(vehicle.mu_min) * scale,
    )


def _tire_mu_source(scale: float) -> str:
    if scale == 1.0:
        return "raw coefficients from the active BobSim TIR file"
    return (
        "temporary study-only scale applied to active-TIR PDX1/PDX2/PDY1/PDY2 "
        "and the projected friction floor; vehicle.yml and TIR remain unchanged"
    )


def _tire_mu_fingerprint_fields(scale: float) -> dict[str, float]:
    """Keep raw-run hashes compatible while explicitly binding scaled studies."""

    return {} if scale == 1.0 else {"tire_mu_scale": scale}


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


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_case_cache(
    case_dir: Path,
    *,
    expected_fingerprint: str,
) -> tuple[dict[str, Any], list[dict[str, Any]]] | None:
    path = case_dir / CASE_CACHE_FILENAME
    if not path.exists():
        return None
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("schema") != "lapsims.ggv-cg-case-cache.v1":
        return None
    if payload.get("input_fingerprint_sha256") != expected_fingerprint:
        return None
    metadata = payload.get("case_metadata")
    rows = payload.get("raw_event_results")
    if not isinstance(metadata, dict) or not isinstance(rows, list):
        return None
    if len(rows) != 4 or any(not isinstance(row, dict) for row in rows):
        return None
    expected_ggv_sha256 = metadata.get("ggv_source_sha256")
    if expected_ggv_sha256 is not None:
        ggv_path = case_dir / "ggv.csv"
        if (
            not isinstance(expected_ggv_sha256, str)
            or not ggv_path.is_file()
            or _sha256_file(ggv_path).lower() != expected_ggv_sha256.lower()
        ):
            return None
    expected_solver_sha256 = metadata.get("ggv_lookup_solver_sha256")
    if expected_solver_sha256 is not None and (
        not isinstance(expected_solver_sha256, str)
        or _sha256_file(GGV_LOOKUP_SOLVER_PATH).lower()
        != expected_solver_sha256.lower()
    ):
        return None
    lookup_contract = metadata.get("ggv_lookup_contract_version")
    if lookup_contract is not None and lookup_contract != GGV_LOOKUP_CONTRACT_VERSION:
        return None
    return metadata, rows


def _write_case_cache(
    case_dir: Path,
    *,
    input_fingerprint: str,
    metadata: dict[str, Any],
    rows: list[dict[str, Any]],
) -> None:
    _write_json(
        case_dir / CASE_CACHE_FILENAME,
        {
            "schema": "lapsims.ggv-cg-case-cache.v1",
            "input_fingerprint_sha256": input_fingerprint,
            "case_metadata": metadata,
            "raw_event_results": rows,
        },
    )


def _validate_existing_case_metadata(
    metadata: object,
    *,
    expected_fields: dict[str, Any],
) -> dict[str, Any]:
    """Validate that an existing GGV sidecar belongs to the requested case."""

    if not isinstance(metadata, dict):
        raise TypeError("Existing GGV case_metadata.json must contain an object.")
    missing = sorted(set(expected_fields).difference(metadata))
    if missing:
        raise ValueError(
            f"Existing GGV metadata is missing input-identity fields: {missing}."
        )

    mismatches = [
        field
        for field, expected in expected_fields.items()
        if cache_fingerprint(metadata[field]) != cache_fingerprint(expected)
    ]
    if mismatches:
        raise ValueError(
            "Existing GGV metadata does not match the requested case inputs: "
            f"{sorted(mismatches)}. Refusing to reuse or regenerate the map."
        )
    return metadata


def _validate_existing_ggv_speed_grid(
    path: Path,
    *,
    expected_speeds: tuple[float, ...],
) -> tuple[float, ...]:
    """Return a stable existing CSV speed grid after exact-coverage validation."""

    if not path.is_file():
        raise FileNotFoundError(f"Existing GGV recovery requires {path}.")
    frame = pd.read_csv(path, usecols=["speed_mps"])
    speeds = tuple(sorted(float(value) for value in frame["speed_mps"].unique()))
    full_grid_matches = len(speeds) == len(expected_speeds) and np.allclose(
        speeds, expected_speeds, rtol=0.0, atol=1e-12
    )
    # save_ggv_csv has no row to serialize when the exact terminal-rpm envelope
    # is wholly infeasible.  That one terminal slice may therefore be absent,
    # but an interior or multi-slice omission is never accepted.
    terminal_empty_grid_matches = (
        len(expected_speeds) > 1
        and len(speeds) == len(expected_speeds) - 1
        and np.allclose(speeds, expected_speeds[:-1], rtol=0.0, atol=1e-12)
    )
    if not (full_grid_matches or terminal_empty_grid_matches):
        raise ValueError(
            "Existing GGV speed grid does not match the requested dense grid; "
            "refusing to reuse or regenerate the map."
        )
    return speeds


def _finite_metadata_float(metadata: dict[str, Any], field: str) -> float:
    value = metadata.get(field)
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
    ):
        raise ValueError(f"Existing GGV metadata field {field!r} must be finite.")
    return float(value)


def _solve_case_event_rows(
    *,
    ggv_map: Any,
    solve_track: Any,
    manifest: dict[str, Any],
    case_dir: Path,
    metadata: dict[str, Any],
    legacy_times: dict[str, float],
    wheel_load_min_n: float,
    wheel_load_max_n: float,
) -> list[dict[str, Any]]:
    """Solve and serialize every event for one already-validated GGV map."""

    case_rows: list[dict[str, Any]] = []
    for event in manifest["Events"]:
        event_slug = str(event["Slug"])
        print(f"  Solving {event_slug}", flush=True)
        track = pd.read_csv(LAPSIMS_ROOT / event["OpenLAPTrackCsv"])
        trace, summary = solve_track(
            ggv_map,
            track,
            is_closed=bool(event["IsClosed"]),
        )
        summary.update(
            {
                "model_family": metadata["model_family"],
                "model_key": metadata["model_key"],
                "model_dof": metadata["model_dof"],
                "sweep_axis": metadata.get("sweep_axis", "cg_height_in"),
                "cg_case": metadata["cg_case"],
                "cg_height_m": metadata["cg_height_m"],
                "cg_height_in": metadata["cg_height_in"],
                "cg_offset_mm": metadata["cg_offset_mm"],
                "is_reference_case": metadata["is_reference_case"],
                "sprung_mass_kg": metadata.get("sprung_mass_kg"),
                "total_mass_kg": metadata.get("total_mass_kg"),
                "mass_offset_kg": metadata.get("mass_offset_kg"),
                "mass_offset_lb": metadata.get("mass_offset_lb"),
                "rear_static_weight_fraction": metadata.get(
                    "rear_static_weight_fraction"
                ),
                "front_static_weight_fraction": metadata.get(
                    "front_static_weight_fraction"
                ),
                "effective_lltd": metadata["effective_lltd"],
                "effective_lltd_interpretation": metadata[
                    "effective_lltd_interpretation"
                ],
                "front_antiroll_stiffness_fraction": metadata[
                    "front_antiroll_stiffness_fraction"
                ],
                "front_elastic_roll_stiffness_fraction": metadata[
                    "front_elastic_roll_stiffness_fraction"
                ],
                "effective_brake_distribution_front": metadata[
                    "effective_brake_distribution_front"
                ],
                "effective_aero_balance_front": metadata[
                    "effective_aero_balance_front"
                ],
                "tire_mu_scale": metadata["tire_mu_scale"],
                "tire_mu_source": metadata["tire_mu_source"],
                "effective_drive_power_limit_w": metadata[
                    "effective_drive_power_limit_w"
                ],
                "drive_power_limit_source": metadata["drive_power_limit_source"],
                "event_slug": event_slug,
                "event_name": event["Name"],
            }
        )
        trace.to_csv(case_dir / f"{event_slug}_trace.csv", index=False)
        _write_json(case_dir / f"{event_slug}_summary.json", summary)
        legacy_time = legacy_times.get(event_slug, math.nan)
        case_rows.append(
            {
                **summary,
                "legacy_openlap_time_s": legacy_time,
                "ggv_minus_legacy_openlap_s": (
                    float(summary["lap_time_s"]) - legacy_time
                    if math.isfinite(legacy_time)
                    else math.nan
                ),
                "ggv_minus_legacy_openlap_pct": (
                    100.0 * (float(summary["lap_time_s"]) / legacy_time - 1.0)
                    if math.isfinite(legacy_time)
                    else math.nan
                ),
                "wheel_load_min_n": wheel_load_min_n,
                "wheel_load_max_n": wheel_load_max_n,
                "wheel_load_range_method": metadata["wheel_load_range_method"],
                "wheel_load_outside_tire_validity": metadata[
                    "wheel_load_outside_tire_validity"
                ],
            }
        )
    return case_rows


def _load_case_tuning(path: Path | None) -> dict[str, Any] | None:
    if path is None:
        return None

    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise TypeError("Case-tuning JSON must contain an object.")
    cases = payload.get("cases")
    if not isinstance(cases, dict) or set(cases) != set(CASE_NAMES):
        raise ValueError(
            "Case-tuning JSON 'cases' must contain exactly low, baseline, and high."
        )

    for case_name in CASE_NAMES:
        case = cases[case_name]
        if not isinstance(case, dict):
            raise TypeError(f"Case-tuning entry {case_name!r} must be an object.")
        setup_keys = {
            key for key in ("lltd", "front_antiroll_stiffness_fraction") if key in case
        }
        if not setup_keys or "brake_distribution_front" not in case:
            raise ValueError(
                f"Case-tuning {case_name!r} must provide brake_distribution_front "
                "and at least one of lltd (legacy) or "
                "front_antiroll_stiffness_fraction (reduced model)."
            )
        for key in (*sorted(setup_keys), "brake_distribution_front"):
            _finite_unit_interval(
                case[key],
                field=f"Case-tuning {case_name}.{key}",
            )
    return payload


def _finite_unit_interval(value: object, *, field: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or not 0.0 <= float(value) <= 1.0
    ):
        raise ValueError(f"{field} must be finite and within [0, 1].")
    return float(value)


def _load_sweep_definition(
    path: Path,
    *,
    nominal_cg_height_m: float,
    nominal_sprung_mass_kg: float | None = None,
    nominal_rear_static_weight_fraction: float | None = None,
) -> tuple[dict[str, Any], list[dict[str, Any]], str]:
    """Load and normalize one supported physical sweep definition."""

    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise TypeError("Sweep JSON must contain an object.")
    declared_axis = payload.get("sweep_axis")
    if declared_axis is not None and declared_axis not in {
        "cg_height_in",
        "sprung_mass_kg",
        "rear_static_weight_fraction",
        "configuration",
    }:
        raise ValueError(f"Unsupported declared sweep axis {declared_axis!r}.")
    raw_cases = payload.get("cases")
    if not isinstance(raw_cases, list) or len(raw_cases) < 2:
        raise ValueError("Sweep JSON 'cases' must contain at least two entries.")

    normalized: list[dict[str, Any]] = []
    case_names: set[str] = set()
    heights: list[float] = []
    sprung_masses: list[float | None] = []
    rear_static_weight_fractions: list[float | None] = []
    tuning_presence: list[bool] = []
    tuning_field_sets: list[frozenset[str]] = []
    for index, raw_case in enumerate(raw_cases):
        if not isinstance(raw_case, dict):
            raise TypeError(f"Sweep case {index} must be an object.")
        name = raw_case.get("name")
        if not isinstance(name, str) or not SAFE_CASE_NAME.fullmatch(name):
            raise ValueError(
                f"Sweep case {index}.name must be a safe file name using only "
                "letters, digits, '.', '_', or '-'."
            )
        if name in case_names:
            raise ValueError(f"Sweep case name {name!r} is duplicated.")
        case_names.add(name)

        supplied_height_fields = [
            key for key in ("cg_height_m", "cg_height_in") if key in raw_case
        ]
        if len(supplied_height_fields) != 1:
            raise ValueError(
                f"Sweep case {name!r} must contain exactly one of cg_height_m "
                "or cg_height_in."
            )
        height_value = raw_case[supplied_height_fields[0]]
        if (
            isinstance(height_value, bool)
            or not isinstance(height_value, (int, float))
            or not math.isfinite(float(height_value))
            or float(height_value) <= 0.0
        ):
            raise ValueError(f"Sweep case {name!r} CG height must be positive.")
        height_m = float(height_value)
        if supplied_height_fields[0] == "cg_height_in":
            height_m *= M_PER_INCH
        heights.append(height_m)

        raw_sprung_mass = raw_case.get("sprung_mass_kg")
        if raw_sprung_mass is None:
            sprung_mass_kg = None
        else:
            if (
                isinstance(raw_sprung_mass, bool)
                or not isinstance(raw_sprung_mass, (int, float))
                or not math.isfinite(float(raw_sprung_mass))
                or float(raw_sprung_mass) <= 0.0
            ):
                raise ValueError(
                    f"Sweep case {name!r} sprung_mass_kg must be finite and positive."
                )
            sprung_mass_kg = float(raw_sprung_mass)
        sprung_masses.append(sprung_mass_kg)
        raw_rear_fraction = raw_case.get("rear_static_weight_fraction")
        if raw_rear_fraction is None:
            rear_static_weight_fraction = None
        else:
            rear_static_weight_fraction = _finite_unit_interval(
                raw_rear_fraction,
                field=f"Sweep case {name}.rear_static_weight_fraction",
            )
            if rear_static_weight_fraction in {0.0, 1.0}:
                raise ValueError(
                    f"Sweep case {name}.rear_static_weight_fraction must be "
                    "strictly between 0 and 1."
                )
        rear_static_weight_fractions.append(rear_static_weight_fraction)
        raw_total_mass = raw_case.get("total_mass_kg")
        if raw_total_mass is None:
            declared_total_mass_kg = None
        else:
            if (
                isinstance(raw_total_mass, bool)
                or not isinstance(raw_total_mass, (int, float))
                or not math.isfinite(float(raw_total_mass))
                or float(raw_total_mass) <= 0.0
            ):
                raise ValueError(
                    f"Sweep case {name!r} total_mass_kg must be finite and positive."
                )
            declared_total_mass_kg = float(raw_total_mass)

        setup_fields = {
            key
            for key in ("lltd", "front_antiroll_stiffness_fraction")
            if key in raw_case
        }
        has_brake_bias = "brake_distribution_front" in raw_case
        if bool(setup_fields) != has_brake_bias:
            raise ValueError(
                f"Sweep case {name!r} must provide brake_distribution_front "
                "with at least one of lltd (legacy) or "
                "front_antiroll_stiffness_fraction (reduced model), or omit all "
                "setup-tuning fields."
            )
        tuning = None
        if setup_fields:
            tuning = {
                **raw_case,
                "brake_distribution_front": _finite_unit_interval(
                    raw_case["brake_distribution_front"],
                    field=f"Sweep case {name}.brake_distribution_front",
                ),
            }
            for field in setup_fields:
                tuning[field] = _finite_unit_interval(
                    raw_case[field], field=f"Sweep case {name}.{field}"
                )
        tuning_presence.append(tuning is not None)
        tuning_field_sets.append(frozenset(setup_fields))
        normalized.append(
            {
                "name": name,
                "cg_height_m": height_m,
                "cg_height_in": height_m / M_PER_INCH,
                "cg_offset_m": height_m - nominal_cg_height_m,
                "sprung_mass_kg": sprung_mass_kg,
                "total_mass_kg": declared_total_mass_kg,
                "rear_static_weight_fraction": rear_static_weight_fraction,
                "tuning": tuning,
            }
        )

    if any(tuning_presence) and not all(tuning_presence):
        raise ValueError(
            "Sweep tuning must be supplied for every case or omitted for every case."
        )
    if all(tuning_presence) and len(set(tuning_field_sets)) != 1:
        raise ValueError(
            "Sweep cases must use the same setup-tuning fields at every height."
        )

    mass_presence = [value is not None for value in sprung_masses]
    rear_fraction_presence = [
        value is not None for value in rear_static_weight_fractions
    ]
    if any(mass_presence) and not all(mass_presence):
        raise ValueError(
            "Sweep sprung_mass_kg must be supplied for every case or omitted "
            "for every case."
        )
    if any(rear_fraction_presence) and not all(rear_fraction_presence):
        raise ValueError(
            "Sweep rear_static_weight_fraction must be supplied for every case "
            "or omitted for every case."
        )
    if (
        all(mass_presence)
        and all(rear_fraction_presence)
        and declared_axis != "configuration"
    ):
        raise ValueError(
            "A sweep cannot vary sprung_mass_kg and "
            "rear_static_weight_fraction together."
        )
    if declared_axis == "configuration":
        sweep_axis = "configuration"
        if not all(rear_fraction_presence):
            raise ValueError(
                "A configuration comparison must supply "
                "rear_static_weight_fraction for every case."
            )
        physical_inputs = {
            (
                round(float(case["cg_height_m"]), 12),
                None
                if case["sprung_mass_kg"] is None
                else round(float(case["sprung_mass_kg"]), 12),
                round(float(case["rear_static_weight_fraction"]), 12),
            )
            for case in normalized
        }
        if len(physical_inputs) != len(normalized):
            raise ValueError(
                "Configuration cases must have unique height, mass, and rear-"
                "weight inputs."
            )
        if all(mass_presence):
            if nominal_sprung_mass_kg is None:
                raise ValueError(
                    "nominal_sprung_mass_kg is required to normalize explicit "
                    "configuration masses."
                )
            for case in normalized:
                case["mass_offset_kg"] = (
                    float(case["sprung_mass_kg"]) - nominal_sprung_mass_kg
                )
                case["mass_offset_lb"] = case["mass_offset_kg"] / 0.45359237
    elif all(mass_presence):
        sweep_axis = "sprung_mass_kg"
        if len(set(heights)) != 1:
            raise ValueError(
                "A sprung-mass sweep must hold total CG height fixed in every case."
            )
        concrete_masses = [float(value) for value in sprung_masses if value is not None]
        if len(set(concrete_masses)) != len(concrete_masses):
            raise ValueError("Sweep sprung masses must be unique.")
        if nominal_sprung_mass_kg is None:
            raise ValueError(
                "nominal_sprung_mass_kg is required to normalize a mass sweep."
            )
        for case in normalized:
            case["mass_offset_kg"] = (
                float(case["sprung_mass_kg"]) - nominal_sprung_mass_kg
            )
            case["mass_offset_lb"] = case["mass_offset_kg"] / 0.45359237
    elif all(rear_fraction_presence):
        sweep_axis = "rear_static_weight_fraction"
        if len(set(heights)) != 1:
            raise ValueError(
                "A longitudinal-CG sweep must hold total CG height fixed in every case."
            )
        concrete_fractions = [
            float(value) for value in rear_static_weight_fractions if value is not None
        ]
        if len(set(concrete_fractions)) != len(concrete_fractions):
            raise ValueError("Sweep rear static weight fractions must be unique.")
    else:
        sweep_axis = "cg_height_in"
        if len(set(heights)) != len(heights):
            raise ValueError("Sweep CG heights must be unique.")

    if declared_axis is not None and declared_axis != sweep_axis:
        raise ValueError(
            f"Sweep JSON declares {declared_axis!r}, but its cases imply "
            f"{sweep_axis!r}."
        )
    for case in normalized:
        case["sweep_axis"] = sweep_axis

    reference_case = payload.get("reference_case")
    if reference_case is not None:
        if not isinstance(reference_case, str) or reference_case not in case_names:
            raise ValueError("Sweep reference_case must name one of the sweep cases.")
    else:
        if sweep_axis == "configuration":
            raise ValueError(
                "A configuration comparison requires an explicit reference_case."
            )
        if sweep_axis == "sprung_mass_kg":
            reference_case = min(
                normalized,
                key=lambda case: abs(float(case["mass_offset_kg"])),
            )["name"]
        elif sweep_axis == "rear_static_weight_fraction":
            if nominal_rear_static_weight_fraction is None:
                raise ValueError(
                    "nominal_rear_static_weight_fraction is required to choose "
                    "a longitudinal-CG sweep reference case."
                )
            reference_case = min(
                normalized,
                key=lambda case: abs(
                    float(case["rear_static_weight_fraction"])
                    - nominal_rear_static_weight_fraction
                ),
            )["name"]
        else:
            reference_case = min(
                normalized,
                key=lambda case: abs(case["cg_height_m"] - nominal_cg_height_m),
            )["name"]
    return payload, normalized, reference_case


def _legacy_sweep_cases(
    baseline_height_m: float,
    case_tuning: dict[str, Any] | None,
) -> list[dict[str, Any]]:
    return [
        {
            "name": case_name,
            "cg_height_m": baseline_height_m + offset_m,
            "cg_height_in": (baseline_height_m + offset_m) / M_PER_INCH,
            "cg_offset_m": offset_m,
            "tuning": (
                None if case_tuning is None else case_tuning["cases"][case_name]
            ),
        }
        for case_name, offset_m in zip(CASE_NAMES, CG_OFFSETS_M)
    ]


def _legacy_times() -> dict[str, float]:
    if not LEGACY_EVENT_SUMMARY.exists():
        return {}
    payload = json.loads(LEGACY_EVENT_SUMMARY.read_text(encoding="utf-8"))
    return {
        str(row["event_slug"]): float(row["lap_time_s"])
        for row in payload.get("events", [])
    }


def _wheel_load_range(vehicle: Any, envelopes: list[Any]) -> tuple[float, float]:
    from _2_EnvelopeSim.GGV.ggv_generation import wheel_loads

    minimum = math.inf
    maximum = -math.inf
    for envelope in envelopes:
        for ay, ax_accel, ax_brake in zip(
            envelope.ay,
            envelope.ax_accel,
            envelope.ax_brake,
        ):
            for ax in (ax_accel, ax_brake):
                if not np.isfinite(ax):
                    continue
                loads = wheel_loads(
                    vehicle,
                    speed=float(envelope.speed),
                    ax=float(ax),
                    ay=float(ay),
                )
                minimum = min(minimum, float(np.min(loads)))
                maximum = max(maximum, float(np.max(loads)))
    return minimum, maximum


def _load_fsae_2026_results_reference() -> dict[str, Any]:
    """Load and validate the fixed real-world time reference."""

    from battery_dynamic_points import EVENT_RULES

    payload = json.loads(FSAE_2026_RESULTS_REFERENCE.read_text(encoding="utf-8"))
    events = payload.get("events")
    if not isinstance(events, dict):
        raise TypeError("FSAE 2026 scoring reference must contain an events object.")
    required_events = {
        "acceleration",
        "skidpad",
        "autocross",
        "michigan_endurance",
    }
    if set(events) != required_events:
        raise ValueError(
            "FSAE 2026 scoring reference must contain exactly the four modeled events."
        )

    for event_slug, event in events.items():
        if not isinstance(event, dict):
            raise TypeError(f"FSAE 2026 reference {event_slug!r} must be an object.")
        times = np.asarray(event.get("valid_adjusted_times_s"), dtype=float)
        if times.ndim != 1 or times.size == 0 or not np.all(np.isfinite(times)):
            raise ValueError(f"FSAE 2026 reference {event_slug!r} has invalid times.")
        if np.any(times <= 0.0) or np.any(np.diff(times) < 0.0):
            raise ValueError(
                f"FSAE 2026 reference {event_slug!r} times must be positive and sorted."
            )
        reference_minimum = float(event["fastest_adjusted_time_s"])
        if not math.isclose(reference_minimum, float(times[0]), abs_tol=1e-12):
            raise ValueError(
                f"FSAE 2026 reference {event_slug!r} fastest time does not match "
                "the field minimum."
            )
        rule_slug = str(event["rule_slug"])
        if not math.isclose(
            reference_minimum,
            EVENT_RULES[rule_slug].tmin_s,
            abs_tol=1e-12,
        ):
            raise ValueError(
                f"FSAE 2026 reference {event_slug!r} does not match its scoring rule."
            )
        conversion = event.get("simulation_time_conversion")
        if conversion not in {"direct", "distance_scaled"}:
            raise ValueError(
                f"FSAE 2026 reference {event_slug!r} has an invalid time conversion."
            )
        if conversion == "distance_scaled":
            event_distance = float(event.get("official_event_distance_m", math.nan))
            if not math.isfinite(event_distance) or event_distance <= 0.0:
                raise ValueError(
                    f"FSAE 2026 reference {event_slug!r} has an invalid distance."
                )
    return payload


def _score_rows(
    rows: list[dict[str, Any]],
    *,
    reference_case: str = "baseline",
) -> list[dict[str, Any]]:
    from battery_dynamic_points import EVENT_RULES

    reference = _load_fsae_2026_results_reference()
    available_cases = {str(row["cg_case"]) for row in rows}
    if reference_case not in available_cases:
        raise ValueError(
            f"Reference case {reference_case!r} is absent from scoring rows; "
            f"available cases are {sorted(available_cases)}."
        )
    reference_times = {
        str(row["event_slug"]): float(row["lap_time_s"])
        for row in rows
        if row["cg_case"] == reference_case
    }

    scoring_inputs: dict[int, tuple[float, float]] = {}
    simulated_times: dict[str, list[float]] = {}
    for row in rows:
        event_slug = str(row["event_slug"])
        event_reference = reference["events"][event_slug]
        raw_simulated_time = float(row["lap_time_s"])
        conversion = str(event_reference["simulation_time_conversion"])
        if conversion == "direct":
            time_multiplier = 1.0
        else:
            modeled_distance = float(row["track_length_m"])
            if not math.isfinite(modeled_distance) or modeled_distance <= 0.0:
                raise ValueError(
                    f"Modeled track length for {event_slug!r} must be positive."
                )
            time_multiplier = (
                float(event_reference["official_event_distance_m"]) / modeled_distance
            )
        scoring_time = raw_simulated_time * time_multiplier
        scoring_inputs[id(row)] = (scoring_time, time_multiplier)
        simulated_times.setdefault(event_slug, []).append(scoring_time)

    effective_event_minimums: dict[str, tuple[float, float, str]] = {}
    for event_slug, event_simulated_times in simulated_times.items():
        event_reference = reference["events"][event_slug]
        field_fastest = float(event_reference["fastest_adjusted_time_s"])
        simulated_fastest = min(event_simulated_times)
        effective_minimum = min(field_fastest, simulated_fastest)
        winning_cases = sorted(
            str(row["cg_case"])
            for row in rows
            if row["event_slug"] == event_slug
            and math.isclose(
                scoring_inputs[id(row)][0],
                simulated_fastest,
                rel_tol=0.0,
                abs_tol=1e-12,
            )
        )
        if simulated_fastest < field_fastest - 1e-12:
            minimum_source = "simulation:" + ",".join(winning_cases)
        elif field_fastest < simulated_fastest - 1e-12:
            minimum_source = "2026_real_field"
        else:
            minimum_source = "2026_real_field_and_simulation:" + ",".join(winning_cases)
        effective_event_minimums[event_slug] = (
            effective_minimum,
            simulated_fastest,
            minimum_source,
        )

    for row in rows:
        event_slug = str(row["event_slug"])
        event_reference = reference["events"][event_slug]
        rule_slug = str(event_reference["rule_slug"])
        rule = EVENT_RULES[rule_slug]
        effective_minimum, simulated_fastest, minimum_source = effective_event_minimums[
            event_slug
        ]
        effective_rule = replace(rule, tmin_s=effective_minimum)
        baseline_time = reference_times[event_slug]
        raw_simulated_time = float(row["lap_time_s"])
        scoring_time, time_multiplier = scoring_inputs[id(row)]
        real_times = np.asarray(event_reference["valid_adjusted_times_s"], dtype=float)
        field_fastest = float(real_times[0])
        field_slowest = float(real_times[-1])
        at_or_faster_than_winner = scoring_time <= field_fastest + 1e-12
        field_outperformed_count = int(
            np.count_nonzero(scoring_time <= real_times + 1e-12)
        )
        is_effective_event_fastest = math.isclose(
            scoring_time,
            effective_minimum,
            rel_tol=0.0,
            abs_tol=1e-12,
        )
        row["baseline_lap_time_s"] = baseline_time
        row["reference_cg_case"] = reference_case
        row["reference_lap_time_s"] = baseline_time
        row["delta_time_s"] = raw_simulated_time - baseline_time
        row["delta_time_pct"] = 100.0 * (raw_simulated_time / baseline_time - 1.0)
        row["simulation_time_multiplier"] = time_multiplier
        row["projected_competition_time_s"] = scoring_time
        row["fsae_2026_reference_fastest_time_s"] = field_fastest
        row["fsae_2026_reference_slowest_time_s"] = field_slowest
        row["fsae_2026_valid_result_count"] = int(real_times.size)
        row["fsae_2026_field_results_matched_or_outperformed"] = (
            field_outperformed_count
        )
        row["fsae_2026_field_percent_matched_or_outperformed"] = (
            100.0 * field_outperformed_count / real_times.size
        )
        row["fsae_2026_reference_winner_team"] = event_reference["winner_team"]
        row["fsae_2026_reference_pdf_page"] = int(event_reference["source_pdf_page"])
        row["fsae_2026_reference_time_basis"] = event_reference["time_basis"]
        row["at_or_faster_than_fsae_2026_winner"] = at_or_faster_than_winner
        row["fastest_simulated_competition_time_s"] = simulated_fastest
        row["effective_points_tmin_s"] = effective_minimum
        row["effective_points_tmax_s"] = effective_rule.tmax_s
        row["effective_points_tmin_source"] = minimum_source
        row["is_effective_event_fastest"] = is_effective_event_fastest
        row["projected_points"] = (
            rule.maximum_points
            if is_effective_event_fastest
            else effective_rule.score(scoring_time)
        )
        row["maximum_points"] = rule.maximum_points
        row["point_loss_vs_max"] = rule.maximum_points - float(row["projected_points"])
        row["points_projection_method"] = (
            "official-formula score using the single fastest time in the combined "
            "valid 2026 real plus simulated event field as Tmin; only exact ties "
            "at that fastest time share the event maximum"
        )
    return rows


def _case_totals(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    frame = pd.DataFrame(rows)
    totals: list[dict[str, Any]] = []
    for case_name, group in frame.groupby("cg_case", sort=False):
        totals.append(
            {
                "model_family": str(group["model_family"].iloc[0]),
                "model_key": str(group["model_key"].iloc[0]),
                "model_dof": (
                    None
                    if pd.isna(group["model_dof"].iloc[0])
                    else int(group["model_dof"].iloc[0])
                ),
                "sweep_axis": str(
                    group.get("sweep_axis", pd.Series(["cg_height_in"])).iloc[0]
                ),
                "cg_case": str(case_name),
                "cg_height_m": float(group["cg_height_m"].iloc[0]),
                "cg_height_in": float(group["cg_height_in"].iloc[0]),
                "cg_offset_mm": float(group["cg_offset_mm"].iloc[0]),
                "is_reference_case": bool(
                    group["reference_cg_case"].iloc[0] == case_name
                ),
                "sprung_mass_kg": (
                    None
                    if "sprung_mass_kg" not in group
                    or pd.isna(group["sprung_mass_kg"].iloc[0])
                    else float(group["sprung_mass_kg"].iloc[0])
                ),
                "total_mass_kg": (
                    None
                    if "total_mass_kg" not in group
                    or pd.isna(group["total_mass_kg"].iloc[0])
                    else float(group["total_mass_kg"].iloc[0])
                ),
                "mass_offset_kg": (
                    None
                    if "mass_offset_kg" not in group
                    or pd.isna(group["mass_offset_kg"].iloc[0])
                    else float(group["mass_offset_kg"].iloc[0])
                ),
                "mass_offset_lb": (
                    None
                    if "mass_offset_lb" not in group
                    or pd.isna(group["mass_offset_lb"].iloc[0])
                    else float(group["mass_offset_lb"].iloc[0])
                ),
                "rear_static_weight_fraction": (
                    None
                    if "rear_static_weight_fraction" not in group
                    or pd.isna(group["rear_static_weight_fraction"].iloc[0])
                    else float(group["rear_static_weight_fraction"].iloc[0])
                ),
                "front_static_weight_fraction": (
                    None
                    if "front_static_weight_fraction" not in group
                    or pd.isna(group["front_static_weight_fraction"].iloc[0])
                    else float(group["front_static_weight_fraction"].iloc[0])
                ),
                "effective_lltd": float(group["effective_lltd"].iloc[0]),
                "front_antiroll_stiffness_fraction": (
                    None
                    if pd.isna(group["front_antiroll_stiffness_fraction"].iloc[0])
                    else float(group["front_antiroll_stiffness_fraction"].iloc[0])
                ),
                "front_elastic_roll_stiffness_fraction": (
                    None
                    if pd.isna(group["front_elastic_roll_stiffness_fraction"].iloc[0])
                    else float(group["front_elastic_roll_stiffness_fraction"].iloc[0])
                ),
                "effective_brake_distribution_front": float(
                    group["effective_brake_distribution_front"].iloc[0]
                ),
                "effective_aero_balance_front": float(
                    group["effective_aero_balance_front"].iloc[0]
                ),
                "tire_mu_scale": float(group["tire_mu_scale"].iloc[0]),
                "projected_timed_event_points": float(group["projected_points"].sum()),
                "maximum_timed_event_points": float(group["maximum_points"].sum()),
                "projected_timed_event_point_loss": float(
                    group["point_loss_vs_max"].sum()
                ),
                "all_solvers_converged": bool(group["converged"].all()),
                "maximum_lateral_utilization": float(
                    group["ggv_max_lateral_utilization"].max()
                ),
                "maximum_speed_domain_fraction": float(
                    group["ggv_max_speed_domain_fraction"].max()
                ),
                "speed_cap_segments": int(group["ggv_speed_cap_segments"].sum()),
            }
        )
    return totals


def _points_correlation(
    totals: list[dict[str, Any]],
    *,
    sweep_axis: str = "cg_height_in",
) -> dict[str, Any]:
    """Summarize linear association and descriptive curvature versus sweep axis."""

    frame = pd.DataFrame(totals)
    if sweep_axis == "configuration":
        if frame.empty or "is_reference_case" not in frame:
            raise ValueError("Configuration comparison requires case totals.")
        references = frame.loc[frame["is_reference_case"].astype(bool)]
        if len(references) != 1:
            raise ValueError(
                "Configuration comparison requires exactly one reference case."
            )
        reference = references.iloc[0]
        reference_points = float(reference["projected_timed_event_points"])
        comparisons = []
        for row in frame.itertuples(index=False):
            points = float(row.projected_timed_event_points)
            comparisons.append(
                {
                    "case": str(row.cg_case),
                    "is_reference_case": bool(row.is_reference_case),
                    "cg_height_in": float(row.cg_height_in),
                    "rear_static_weight_fraction": float(
                        row.rear_static_weight_fraction
                    ),
                    "sprung_mass_kg": (
                        None
                        if pd.isna(row.sprung_mass_kg)
                        else float(row.sprung_mass_kg)
                    ),
                    "projected_timed_event_points": points,
                    "points_delta_vs_reference": points - reference_points,
                }
            )
        return {
            "sample_count": len(frame),
            "sweep_axis": "configuration",
            "analysis_kind": "categorical_configuration_comparison",
            "reference_case": str(reference["cg_case"]),
            "reference_points": reference_points,
            "comparisons": comparisons,
            "correlation_computed": False,
            "correlation_not_computed_reason": (
                "CG height and longitudinal CG position change together, so no "
                "one-axis slope or correlation is physically identifiable."
            ),
        }
    if sweep_axis == "cg_height_in":
        axis_column = "cg_height_in"
        x = frame[axis_column].to_numpy(dtype=float)
        axis_label = "cg_height_in"
        slope_key = "linear_slope_points_per_in"
        adjacent_key = "adjacent_slopes_points_per_in"
        quadratic_a_key = "quadratic_a_points_per_in2"
        quadratic_b_key = "quadratic_b_points_per_in"
        vertex_key = "quadratic_vertex_cg_height_in"
        minimum_key = "cg_height_min_in"
        maximum_key = "cg_height_max_in"
    elif sweep_axis == "sprung_mass_kg":
        axis_column = "sprung_mass_kg"
        x = frame[axis_column].to_numpy(dtype=float) / 0.45359237
        axis_label = "sprung_mass_lb"
        slope_key = "linear_slope_points_per_lb"
        adjacent_key = "adjacent_slopes_points_per_lb"
        quadratic_a_key = "quadratic_a_points_per_lb2"
        quadratic_b_key = "quadratic_b_points_per_lb"
        vertex_key = "quadratic_vertex_sprung_mass_lb"
        minimum_key = "sprung_mass_min_lb"
        maximum_key = "sprung_mass_max_lb"
    elif sweep_axis == "rear_static_weight_fraction":
        axis_column = "rear_static_weight_fraction"
        x = 100.0 * frame[axis_column].to_numpy(dtype=float)
        axis_label = "rear_static_weight_percent"
        slope_key = "linear_slope_points_per_rear_weight_pct"
        adjacent_key = "adjacent_slopes_points_per_rear_weight_pct"
        quadratic_a_key = "quadratic_a_points_per_rear_weight_pct2"
        quadratic_b_key = "quadratic_b_points_per_rear_weight_pct"
        vertex_key = "quadratic_vertex_rear_static_weight_percent"
        minimum_key = "rear_static_weight_min_percent"
        maximum_key = "rear_static_weight_max_percent"
    else:
        raise ValueError(f"Unsupported sweep axis {sweep_axis!r}.")
    order = np.argsort(x)
    x = x[order]
    frame = frame.iloc[order]
    y = frame["projected_timed_event_points"].to_numpy(dtype=float)
    if x.size < 2 or np.allclose(x, x[0]):
        raise ValueError(f"{axis_label} correlation requires two unique values.")
    slope, intercept = np.polyfit(x, y, 1)
    pearson_r = float(np.corrcoef(x, y)[0, 1])
    fitted = slope * x + intercept
    residual_sum_squares = float(np.sum((y - fitted) ** 2))
    total_sum_squares = float(np.sum((y - np.mean(y)) ** 2))
    r_squared = (
        1.0 - residual_sum_squares / total_sum_squares
        if total_sum_squares > 0.0
        else 1.0
    )
    x_ranks = pd.Series(x).rank(method="average").to_numpy(dtype=float)
    y_ranks = pd.Series(y).rank(method="average").to_numpy(dtype=float)
    spearman_rho = float(np.corrcoef(x_ranks, y_ranks)[0, 1])
    result: dict[str, Any] = {
        "sample_count": int(x.size),
        "sweep_axis": sweep_axis,
        "axis_label": axis_label,
        minimum_key: float(np.min(x)),
        maximum_key: float(np.max(x)),
        "points_min": float(np.min(y)),
        "points_max": float(np.max(y)),
        "pearson_r": pearson_r,
        slope_key: float(slope),
        "linear_slope_points_per_axis_unit": float(slope),
        "linear_intercept_points": float(intercept),
        "linear_r_squared": float(r_squared),
        "linear_rmse_points": float(np.sqrt(np.mean((y - fitted) ** 2))),
        "spearman_rho": spearman_rho,
        adjacent_key: [
            float(delta_y / delta_x) for delta_y, delta_x in zip(np.diff(y), np.diff(x))
        ],
        "fit_definition": (
            f"projected_timed_event_points = slope * {axis_label} + intercept"
        ),
    }
    if x.size >= 3:
        quadratic_a, quadratic_b, quadratic_c = np.polyfit(x, y, 2)
        quadratic_fitted = quadratic_a * x**2 + quadratic_b * x + quadratic_c
        quadratic_residual_sum_squares = float(np.sum((y - quadratic_fitted) ** 2))
        quadratic_r_squared = (
            1.0 - quadratic_residual_sum_squares / total_sum_squares
            if total_sum_squares > 0.0
            else 1.0
        )
        quadratic_vertex = (
            -quadratic_b / (2.0 * quadratic_a)
            if abs(quadratic_a) > np.finfo(float).eps
            else math.nan
        )
        result.update(
            {
                quadratic_a_key: float(quadratic_a),
                quadratic_b_key: float(quadratic_b),
                "quadratic_c_points": float(quadratic_c),
                "quadratic_r_squared": float(quadratic_r_squared),
                "quadratic_rmse_points": float(
                    np.sqrt(np.mean((y - quadratic_fitted) ** 2))
                ),
                vertex_key: float(quadratic_vertex),
                "quadratic_vertex_inside_sweep": bool(
                    math.isfinite(quadratic_vertex)
                    and float(np.min(x)) <= quadratic_vertex <= float(np.max(x))
                ),
                "quadratic_fit_role": (
                    "descriptive curvature check only; do not extrapolate outside "
                    "the simulated height range"
                ),
            }
        )
    return result


def _make_correlation_plot(
    totals: list[dict[str, Any]],
    correlation: dict[str, Any],
    output_path: Path,
    *,
    sweep_axis: str = "cg_height_in",
) -> None:
    frame = pd.DataFrame(totals)
    if sweep_axis == "configuration":
        x = np.arange(len(frame), dtype=float)
        y = frame["projected_timed_event_points"].to_numpy(dtype=float)
        labels = frame["cg_case"].astype(str).tolist()
        reference_points = float(correlation["reference_points"])
        fig, axis = plt.subplots(figsize=(8.2, 5.2))
        axis.plot(x, y, marker="o", linewidth=2.5, color="#111827")
        axis.axhline(
            reference_points,
            linestyle="--",
            linewidth=1.5,
            color="#6b7280",
            label=f"Reference: {correlation['reference_case']}",
        )
        for index, points in enumerate(y):
            axis.annotate(
                f"{points - reference_points:+.2f} pt",
                (x[index], points),
                xytext=(0, 8),
                textcoords="offset points",
                ha="center",
                fontsize=9,
            )
        axis.set_title("Projected points by vehicle configuration")
        axis.set_xlabel("Configuration (categorical; CG z and x both vary)")
        axis.set_ylabel("Projected timed-event points (575 maximum)")
        axis.set_xticks(x, labels, rotation=12, ha="right")
        axis.grid(True, axis="y", alpha=0.25)
        axis.legend()
        fig.tight_layout()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path, dpi=180)
        plt.close(fig)
        return
    if sweep_axis == "cg_height_in":
        x = frame["cg_height_in"].to_numpy(dtype=float)
        x_label = "Total CG height (in)"
        title = "CG height to projected dynamic-event points"
        slope_key = "linear_slope_points_per_in"
        slope_unit = "points/in"
        quadratic_a_key = "quadratic_a_points_per_in2"
        quadratic_b_key = "quadratic_b_points_per_in"
    elif sweep_axis == "sprung_mass_kg":
        x = frame["sprung_mass_kg"].to_numpy(dtype=float) / 0.45359237
        x_label = "Sprung mass (lb)"
        title = "Sprung mass to projected dynamic-event points"
        slope_key = "linear_slope_points_per_lb"
        slope_unit = "points/lb"
        quadratic_a_key = "quadratic_a_points_per_lb2"
        quadratic_b_key = "quadratic_b_points_per_lb"
    elif sweep_axis == "rear_static_weight_fraction":
        x = 100.0 * frame["rear_static_weight_fraction"].to_numpy(dtype=float)
        x_label = "Static rear weight (%)"
        title = "Longitudinal CG balance to projected dynamic-event points"
        slope_key = "linear_slope_points_per_rear_weight_pct"
        slope_unit = "points/rear-%"
        quadratic_a_key = "quadratic_a_points_per_rear_weight_pct2"
        quadratic_b_key = "quadratic_b_points_per_rear_weight_pct"
    else:
        raise ValueError(f"Unsupported sweep axis {sweep_axis!r}.")
    order = np.argsort(x)
    x = x[order]
    frame = frame.iloc[order]
    y = frame["projected_timed_event_points"].to_numpy(dtype=float)
    x_fit = np.linspace(float(np.min(x)), float(np.max(x)), 200)
    linear_fitted = float(correlation[slope_key]) * x_fit + float(
        correlation["linear_intercept_points"]
    )

    fig, axis = plt.subplots(figsize=(8.2, 5.2))
    axis.plot(x, y, marker="o", linewidth=2.5, color="#111827", label="Simulation")
    axis.plot(
        x_fit,
        linear_fitted,
        linestyle="--",
        linewidth=1.8,
        color="#dc2626",
        label="Linear fit",
    )
    if quadratic_a_key in correlation:
        quadratic_fitted = (
            float(correlation[quadratic_a_key]) * x_fit**2
            + float(correlation[quadratic_b_key]) * x_fit
            + float(correlation["quadratic_c_points"])
        )
        axis.plot(
            x_fit,
            quadratic_fitted,
            linestyle=":",
            linewidth=2.0,
            color="#2563eb",
            label="Quadratic description",
        )
    axis.set_title(title)
    axis.set_xlabel(x_label)
    axis.set_ylabel("Projected timed-event points (575 maximum)")
    axis.grid(True, alpha=0.25)
    axis.legend()
    axis.text(
        0.02,
        0.03,
        (
            f"slope = {correlation[slope_key]:.3f} {slope_unit}\n"
            f"Pearson r = {correlation['pearson_r']:.4f}\n"
            f"linear R\N{SUPERSCRIPT TWO} = {correlation['linear_r_squared']:.4f}\n"
            f"Spearman rho = {correlation['spearman_rho']:.4f}\n"
            f"quadratic R\N{SUPERSCRIPT TWO} = "
            f"{correlation.get('quadratic_r_squared', math.nan):.4f}"
        ),
        transform=axis.transAxes,
        fontsize=9,
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.85},
    )
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def _make_plot(
    rows: list[dict[str, Any]],
    totals: list[dict[str, Any]],
    output_path: Path,
    *,
    setup_subtitle: str,
    sweep_axis: str = "cg_height_in",
) -> None:
    frame = pd.DataFrame(rows)
    categorical_labels: list[str] | None = None
    if sweep_axis == "configuration":
        categorical_labels = [str(row["cg_case"]) for row in totals]
        case_position = {
            case_name: float(index)
            for index, case_name in enumerate(categorical_labels)
        }
        sort_column = "configuration_order"
        x_column = "configuration_order"
        x_scale = 1.0
        x_label = "Configuration (categorical; CG z and x both vary)"
        study_title = "EnvelopeSim GGV vehicle-configuration comparison"
        frame[sort_column] = frame["cg_case"].map(case_position)
    elif sweep_axis == "cg_height_in":
        sort_column = "cg_height_m"
        x_column = "cg_height_m"
        x_scale = 1000.0
        x_label = "Total CG height (mm)"
        study_title = "EnvelopeSim GGV CG-height smoke study"
    elif sweep_axis == "sprung_mass_kg":
        sort_column = "sprung_mass_kg"
        x_column = "sprung_mass_kg"
        x_scale = 1.0 / 0.45359237
        x_label = "Sprung mass (lb)"
        study_title = "EnvelopeSim GGV sprung-mass study"
    elif sweep_axis == "rear_static_weight_fraction":
        sort_column = "rear_static_weight_fraction"
        x_column = "rear_static_weight_fraction"
        x_scale = 100.0
        x_label = "Static rear weight (%)"
        study_title = "EnvelopeSim GGV longitudinal-CG smoke study"
    else:
        raise ValueError(f"Unsupported sweep axis {sweep_axis!r}.")
    total_frame = pd.DataFrame(totals)
    if categorical_labels is not None:
        total_frame[sort_column] = total_frame["cg_case"].map(case_position)
    total_frame = total_frame.sort_values(sort_column)
    colors = {
        "acceleration": "#2563eb",
        "skidpad": "#16a34a",
        "autocross": "#ea580c",
        "michigan_endurance": "#7c3aed",
    }
    labels = {
        "acceleration": "Acceleration",
        "skidpad": "Skidpad",
        "autocross": "Autocross",
        "michigan_endurance": "Michigan lap",
    }

    fig, axes = plt.subplots(1, 3, figsize=(15.5, 4.8))
    for event_slug, group in frame.groupby("event_slug", sort=False):
        group = group.sort_values(sort_column)
        x_values = x_scale * group[x_column].to_numpy(dtype=float)
        axes[0].plot(
            x_values,
            group["delta_time_pct"],
            marker="o",
            linewidth=2.0,
            color=colors[event_slug],
            label=labels[event_slug],
        )
        axes[1].plot(
            x_values,
            100.0
            * group["projected_points"].to_numpy(dtype=float)
            / group["maximum_points"].to_numpy(dtype=float),
            marker="o",
            linewidth=2.0,
            color=colors[event_slug],
            label=labels[event_slug],
        )

    axes[0].axhline(0.0, color="#6b7280", linewidth=1.0)
    axes[0].set_title("Raw event-time sensitivity")
    axes[0].set_xlabel(x_label)
    axes[0].set_ylabel("Time change from reference case (%)")
    axes[0].grid(True, alpha=0.25)
    axes[0].legend(fontsize=8)

    axes[1].axhline(100.0, color="#6b7280", linewidth=1.0)
    axes[1].set_title("Score vs. 2026 Michigan results")
    axes[1].set_xlabel(x_label)
    axes[1].set_ylabel("Score (% of event maximum)")
    axes[1].grid(True, alpha=0.25)

    axes[2].plot(
        x_scale * total_frame[x_column],
        total_frame["projected_timed_event_points"],
        marker="o",
        linewidth=2.5,
        color="#111827",
    )
    axes[2].set_title("Projected timed-event total")
    axes[2].set_xlabel(x_label)
    axes[2].set_ylabel("Points (575 maximum)")
    axes[2].grid(True, alpha=0.25)
    for row in total_frame.itertuples(index=False):
        axes[2].annotate(
            f"{row.projected_timed_event_points:.2f}",
            (
                x_scale * float(getattr(row, x_column)),
                row.projected_timed_event_points,
            ),
            xytext=(0, 8),
            textcoords="offset points",
            ha="center",
            fontsize=8,
        )
    if categorical_labels is not None:
        tick_positions = np.arange(len(categorical_labels), dtype=float)
        for axis in axes:
            axis.set_xticks(
                tick_positions,
                categorical_labels,
                rotation=12,
                ha="right",
            )

    fig.suptitle(
        f"{study_title}\n{setup_subtitle}",
        fontsize=13,
        fontweight="bold",
    )
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.91))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def run(args: argparse.Namespace) -> dict[str, Any]:
    bobsim_root = args.bobsim_root.resolve()
    model_dof = getattr(args, "model_dof", None)
    model_family, model_key = model_identity(model_dof)
    raw_output_root = getattr(args, "output_root", None)
    output_root = (
        default_output_root(DEFAULT_OUTPUT_ROOT, model_dof)
        if raw_output_root is None
        else Path(raw_output_root)
    ).resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    for source_root in (bobsim_root, Path(__file__).resolve().parent):
        if str(source_root) not in sys.path:
            sys.path.insert(0, str(source_root))

    from _2_EnvelopeSim.GGV.ggv_generation import (
        config_to_ggv_config,
        generate_ggv,
        load_ggv_config,
        save_ggv_csv,
    )
    from _2_EnvelopeSim.vehicle_yaml import load_vehicle_yaml, project_vehicle_yaml

    if model_dof is not None:
        from _0_Utils.dyn_py import (
            ReducedVehicleOverrides,
            apply_reduced_vehicle_overrides,
            create_model,
            load_reduced_vehicle_parameters,
        )
    else:
        ReducedVehicleOverrides = None
        apply_reduced_vehicle_overrides = None
        create_model = None
        load_reduced_vehicle_parameters = None

    from ggv_lookup_solver import GGVMap, solve_track

    manifest = json.loads(EVENT_MANIFEST.read_text(encoding="utf-8"))
    legacy_times = _legacy_times()
    aero_balance_front = getattr(args, "aero_balance_front", None)
    case_tuning_path = getattr(args, "case_tuning_json", None)
    sweep_path = getattr(args, "sweep_json", None)
    if case_tuning_path is not None and sweep_path is not None:
        raise ValueError("--case-tuning-json and --sweep-json cannot be used together.")
    case_tuning = _load_case_tuning(case_tuning_path)
    projection = project_vehicle_yaml(
        load_vehicle_yaml(bobsim_root / "vehicle.yml"),
        repo_root=bobsim_root,
        aero_balance_front=aero_balance_front,
    )
    baseline_height = float(projection.ggv.cg_height)
    base_reduced_parameters = (
        load_reduced_vehicle_parameters(bobsim_root / "vehicle.yml")
        if load_reduced_vehicle_parameters is not None
        else None
    )
    sweep_definition = None
    if sweep_path is None:
        sweep_cases = _legacy_sweep_cases(baseline_height, case_tuning)
        reference_case = "baseline"
    else:
        sweep_definition, sweep_cases, reference_case = _load_sweep_definition(
            sweep_path,
            nominal_cg_height_m=baseline_height,
            nominal_sprung_mass_kg=(
                None
                if base_reduced_parameters is None
                else float(base_reduced_parameters.sprung_mass_kg)
            ),
            nominal_rear_static_weight_fraction=(
                None
                if base_reduced_parameters is None
                else float(base_reduced_parameters.static_rear_weight_fraction)
            ),
        )
    sweep_axis = str(sweep_cases[0].get("sweep_axis", "cg_height_in"))
    if (
        sweep_axis in {"sprung_mass_kg", "rear_static_weight_fraction", "configuration"}
        and model_dof is None
    ):
        raise ValueError(f"{sweep_axis} sweeps require an explicit reduced model DOF.")
    tire_mu_scale, tire_mu_scale_source = _resolve_tire_mu_scale(
        getattr(args, "tire_mu_scale", None),
        sweep_definition,
    )
    tire_mu_source = _tire_mu_source(tire_mu_scale)
    drive_power_limit_w, drive_power_limit_source = _resolve_drive_power_limit_w(
        getattr(args, "drive_power_limit_kw", None),
        sweep_definition,
    )
    source_peak_drive_power_w = float(projection.ggv.max_drive_power)
    if (
        drive_power_limit_w is not None
        and drive_power_limit_w > source_peak_drive_power_w + 1e-9
    ):
        raise ValueError(
            "Temporary drive power limit cannot exceed the live projected peak "
            f"power ({source_peak_drive_power_w / 1000.0:.6g} kW)."
        )
    effective_drive_power_limit_w = (
        source_peak_drive_power_w
        if drive_power_limit_w is None
        else drive_power_limit_w
    )
    tuning_applied = all(case["tuning"] is not None for case in sweep_cases)
    requested_cases = tuple(getattr(args, "case", None) or ())
    reuse_existing_ggv = bool(getattr(args, "reuse_existing_ggv", False))
    expected_ggv_sha256 = getattr(args, "expected_ggv_sha256", None)
    if len(set(requested_cases)) != len(requested_cases):
        raise ValueError("--case values must not be duplicated.")
    available_case_names = {str(case["name"]) for case in sweep_cases}
    unknown_cases = set(requested_cases).difference(available_case_names)
    if unknown_cases:
        raise ValueError(
            f"Unknown --case values {sorted(unknown_cases)}; available cases are "
            f"{sorted(available_case_names)}."
        )
    selected_cases = (
        [case for case in sweep_cases if str(case["name"]) in requested_cases]
        if requested_cases
        else sweep_cases
    )
    if reuse_existing_ggv and model_dof is None:
        raise ValueError("--reuse-existing-ggv requires an explicit --model-dof.")
    if reuse_existing_ggv and len(requested_cases) != 1:
        raise ValueError(
            "--reuse-existing-ggv requires exactly one --case so it cannot "
            "silently adopt maps from multiple case directories."
        )
    if reuse_existing_ggv and bool(getattr(args, "resume", False)):
        raise ValueError(
            "--reuse-existing-ggv cannot be combined with --resume; recovery must "
            "validate the supplied GGV SHA instead of bypassing through a cache."
        )
    if reuse_existing_ggv and (
        not isinstance(expected_ggv_sha256, str)
        or re.fullmatch(r"[0-9A-Fa-f]{64}", expected_ggv_sha256) is None
    ):
        raise ValueError(
            "--reuse-existing-ggv requires --expected-ggv-sha256 with the exact "
            "64-character digest of the existing case ggv.csv."
        )
    if not reuse_existing_ggv and expected_ggv_sha256 is not None:
        raise ValueError(
            "--expected-ggv-sha256 is only valid with --reuse-existing-ggv."
        )
    partial_run = len(selected_cases) != len(sweep_cases)

    configured = config_to_ggv_config(load_ggv_config())
    requested_speed_slices = dense_speed_slices(
        top_speed_mps=float(getattr(args, "ggv_top_speed_mps", DEFAULT_TOP_SPEED_MPS)),
        step_mps=float(getattr(args, "ggv_speed_step_mps", DEFAULT_SPEED_STEP_MPS)),
    )
    zero_proxy_mps = float(
        getattr(args, "qss_zero_proxy_mps", DEFAULT_QSS_ZERO_PROXY_MPS)
    )
    expected_qss_trim_speeds = (
        requested_speed_slices
        if model_dof is None
        else qss_generation_speeds(
            requested_speed_slices,
            zero_proxy_mps=zero_proxy_mps,
        )
    )
    expected_exported_speed_slices = (
        requested_speed_slices
        if model_dof is None
        else (0.0, *expected_qss_trim_speeds)
    )
    smoke_config = replace(
        configured,
        speeds=requested_speed_slices,
        model_dof=(configured.model_dof if model_dof is None else model_dof),
        ay_max_g=float(getattr(args, "ay_max_g", 3.2)),
        ay_points=int(args.ay_points),
        ax_search_points=int(args.ax_search_points),
        ax_binary_iterations=(
            configured.ax_binary_iterations
            if getattr(args, "ax_binary_iterations", None) is None
            else int(args.ax_binary_iterations)
        ),
        verbose=False,
        warn_tire_load_range=False,
        track_relevant_lateral_domain_only=model_dof is not None,
    )

    rows: list[dict[str, Any]] = []
    case_metadata: list[dict[str, Any]] = []
    for sweep_case in selected_cases:
        case_name = str(sweep_case["name"])
        height_m = float(sweep_case["cg_height_m"])
        offset_m = float(sweep_case["cg_offset_m"])
        target_sprung_mass_kg = sweep_case.get("sprung_mass_kg")
        mass_offset_kg = sweep_case.get("mass_offset_kg")
        mass_offset_lb = sweep_case.get("mass_offset_lb")
        rear_static_weight_fraction = sweep_case.get("rear_static_weight_fraction")
        vehicle = _scale_ggv_tire_peak_parameters(
            replace(projection.ggv, cg_height=height_m),
            tire_mu_scale,
        )
        vehicle = replace(
            vehicle,
            max_drive_power=effective_drive_power_limit_w,
        )
        tuning = sweep_case["tuning"]
        if model_dof is None and tuning is not None and "lltd" not in tuning:
            raise ValueError(
                f"Legacy EnvelopeSim case {case_name!r} requires explicit lltd; "
                "front_antiroll_stiffness_fraction is reduced-model-only."
            )
        if tuning is not None:
            vehicle = replace(
                vehicle,
                brake_distribution_front=float(tuning["brake_distribution_front"]),
            )
            if "lltd" in tuning:
                vehicle = replace(vehicle, lltd=float(tuning["lltd"]))
        reduced_parameters = None
        reduced_parameter_summary = None
        reduced_model = None
        effective_lltd = float(vehicle.lltd)
        front_antiroll_stiffness_fraction = None
        front_elastic_roll_stiffness_fraction = None
        if model_dof is not None:
            if (
                base_reduced_parameters is None
                or create_model is None
                or ReducedVehicleOverrides is None
                or apply_reduced_vehicle_overrides is None
            ):
                raise RuntimeError("Reduced-model imports were not initialized.")
            front_antiroll_fraction = (
                None
                if tuning is None
                else tuning.get("front_antiroll_stiffness_fraction")
            )
            if tuning is not None and front_antiroll_fraction is None:
                raise ValueError(
                    f"Reduced-model case {case_name!r} requires explicit "
                    "front_antiroll_stiffness_fraction; lltd is legacy-only."
                )
            reduced_overrides = ReducedVehicleOverrides(
                absolute_cg_height_m=height_m,
                target_sprung_mass_kg=(
                    None
                    if target_sprung_mass_kg is None
                    else float(target_sprung_mass_kg)
                ),
                static_rear_weight_fraction=(
                    None
                    if rear_static_weight_fraction is None
                    else float(rear_static_weight_fraction)
                ),
                aero_balance_front=float(vehicle.aero_balance_front),
                brake_distribution_front=float(vehicle.brake_distribution_front),
                front_antiroll_stiffness_fraction=(
                    None
                    if front_antiroll_fraction is None
                    else float(front_antiroll_fraction)
                ),
                tire_mu_scale=tire_mu_scale,
            )
            reduced_parameters = apply_reduced_vehicle_overrides(
                base_reduced_parameters,
                reduced_overrides,
            )
            reduced_parameters = replace(
                reduced_parameters,
                peak_drive_power_w=effective_drive_power_limit_w,
                continuous_drive_power_w=min(
                    float(reduced_parameters.continuous_drive_power_w),
                    effective_drive_power_limit_w,
                ),
            )
            declared_total_mass_kg = sweep_case.get("total_mass_kg")
            if declared_total_mass_kg is not None and not math.isclose(
                float(declared_total_mass_kg),
                float(reduced_parameters.mass_kg),
                rel_tol=0.0,
                abs_tol=1e-9,
            ):
                raise ValueError(
                    f"Sweep case {case_name!r} total_mass_kg does not match the "
                    "derived target sprung plus fixed unsprung mass."
                )
            vehicle = replace(
                vehicle,
                mass=float(reduced_parameters.mass_kg),
                front_static_frac=float(
                    reduced_parameters.static_front_weight_fraction
                ),
            )
            roll_summary = elastic_roll_stiffness(reduced_parameters)
            reduced_parameter_summary = {
                "override_api": (
                    "_0_Utils.dyn_py.ReducedVehicleOverrides and "
                    "apply_reduced_vehicle_overrides"
                ),
                "absolute_cg_height_m": float(reduced_parameters.absolute_cg_height_m),
                "sprung_mass_kg": float(reduced_parameters.sprung_mass_kg),
                "total_mass_kg": float(reduced_parameters.mass_kg),
                "fixed_unsprung_mass_kg": float(
                    sum(reduced_parameters.unsprung_mass_kg)
                ),
                "rear_static_weight_fraction": float(
                    reduced_parameters.static_rear_weight_fraction
                ),
                "front_static_weight_fraction": float(
                    reduced_parameters.static_front_weight_fraction
                ),
                "center_of_gravity_m": [
                    float(value) for value in reduced_parameters.center_of_gravity_m
                ],
                "aero_balance_front": float(reduced_parameters.aero_balance_front),
                "aero_cop_body_m": [
                    float(value) for value in reduced_parameters.aero_cop_m
                ],
                "brake_distribution_front": float(
                    reduced_parameters.brake_distribution_front
                ),
                "front_antiroll_stiffness_fraction": float(
                    reduced_parameters.front_antiroll_stiffness_fraction
                ),
                "front_elastic_roll_stiffness_fraction": float(
                    reduced_parameters.front_roll_stiffness_fraction
                ),
                "elastic_roll_stiffness": roll_summary,
                "antiroll_retuning_method": (
                    "preserve projected wheel spring rates and total front-plus-"
                    "rear anti-roll stiffness; redistribute ARB stiffness by the "
                    "explicit front_antiroll_stiffness_fraction"
                    if front_antiroll_fraction is not None
                    else "native projected front/rear anti-roll stiffness"
                ),
                "source_peak_drive_power_w": source_peak_drive_power_w,
                "effective_peak_drive_power_w": float(
                    reduced_parameters.peak_drive_power_w
                ),
                "effective_continuous_drive_power_w": float(
                    reduced_parameters.continuous_drive_power_w
                ),
            }
            if tire_mu_scale != 1.0:
                reduced_parameter_summary.update(
                    {
                        "tire_mu_scale": tire_mu_scale,
                        "tire_mu_scale_source": tire_mu_scale_source,
                        "tire_mu_scaled_fields": [
                            "pdx1",
                            "pdx2",
                            "pdy1",
                            "pdy2",
                            "mu_floor",
                        ],
                        "effective_tire_peak_parameters": {
                            "pdx1": float(reduced_parameters.tire.pdx1),
                            "pdx2": float(reduced_parameters.tire.pdx2),
                            "pdy1": float(reduced_parameters.tire.pdy1),
                            "pdy2": float(reduced_parameters.tire.pdy2),
                            "mu_floor": float(reduced_parameters.tire.mu_floor),
                        },
                    }
                )
            effective_lltd = float(reduced_parameters.front_roll_stiffness_fraction)
            front_antiroll_stiffness_fraction = float(
                reduced_parameters.front_antiroll_stiffness_fraction
            )
            front_elastic_roll_stiffness_fraction = float(
                reduced_parameters.front_roll_stiffness_fraction
            )
            reduced_model = create_model(model_dof, reduced_parameters)

        case_dir = output_root / case_name
        case_dir.mkdir(parents=True, exist_ok=True)
        ggv_path = case_dir / "ggv.csv"
        fingerprint = cache_fingerprint(
            {
                "schema": "lapsims.ggv-cg-inputs.v1",
                "model_family": model_family,
                "model_key": model_key,
                "model_dof": model_dof,
                "case": sweep_case,
                "vehicle": asdict(vehicle),
                "reduced_parameters": (
                    asdict(reduced_parameters)
                    if reduced_parameters is not None
                    else None
                ),
                "ggv_config": asdict(smoke_config),
                "qss_zero_proxy_mps": (
                    zero_proxy_mps if model_dof is not None else None
                ),
                "event_manifest": manifest,
                **_tire_mu_fingerprint_fields(tire_mu_scale),
            }
        )
        cached = (
            _load_case_cache(case_dir, expected_fingerprint=fingerprint)
            if bool(getattr(args, "resume", False))
            else None
        )
        if cached is not None:
            cached_metadata, cached_rows = cached
            for cached_row in cached_rows:
                cached_row.setdefault(
                    "tire_mu_scale",
                    float(cached_metadata.get("tire_mu_scale", 1.0)),
                )
                cached_row.setdefault(
                    "tire_mu_source",
                    str(
                        cached_metadata.get(
                            "tire_mu_source",
                            _tire_mu_source(
                                float(cached_metadata.get("tire_mu_scale", 1.0))
                            ),
                        )
                    ),
                )
            required_files = [ggv_path]
            required_files.extend(
                case_dir / f"{event['Slug']}_trace.csv" for event in manifest["Events"]
            )
            if all(path.exists() for path in required_files):
                print(f"Reusing validated cache for {case_name}", flush=True)
                case_metadata.append(cached_metadata)
                rows.extend(cached_rows)
                continue

        if reuse_existing_ggv:
            metadata_path = case_dir / "case_metadata.json"
            if not metadata_path.is_file():
                raise FileNotFoundError(
                    "--reuse-existing-ggv requires the case_metadata.json written "
                    f"with {ggv_path}; refusing to regenerate."
                )
            if not ggv_path.is_file():
                raise FileNotFoundError(
                    f"--reuse-existing-ggv requires {ggv_path}; refusing to regenerate."
                )
            if ggv_path.stat().st_mtime_ns > metadata_path.stat().st_mtime_ns:
                raise ValueError(
                    "Existing ggv.csv is newer than its case_metadata.json; the "
                    "sidecar may not describe this map. Refusing to reuse or regenerate."
                )

            effective_lltd_interpretation = (
                "front spring-plus-ARB elastic roll-stiffness fraction; a 3DOF "
                "algebraic LLTD proxy, not a direct 6DOF axle load-transfer result"
            )
            zero_speed_contract = (
                "0 m/s boundary copied from the near-zero positive QSS trim; "
                "the positive proxy slice is also retained"
            )
            expected_metadata_fields = {
                "model_family": model_family,
                "model_key": model_key,
                "model_dof": model_dof,
                "sweep_axis": sweep_axis,
                "cg_case": case_name,
                "cg_height_m": height_m,
                "cg_height_in": height_m / M_PER_INCH,
                "cg_offset_mm": 1000.0 * offset_m,
                "is_reference_case": case_name == reference_case,
                "sprung_mass_kg": (
                    None
                    if reduced_parameters is None
                    else float(reduced_parameters.sprung_mass_kg)
                ),
                "total_mass_kg": float(vehicle.mass),
                "mass_offset_kg": mass_offset_kg,
                "mass_offset_lb": mass_offset_lb,
                "rear_static_weight_fraction": (
                    None
                    if reduced_parameters is None
                    else float(reduced_parameters.static_rear_weight_fraction)
                ),
                "front_static_weight_fraction": (
                    None
                    if reduced_parameters is None
                    else float(reduced_parameters.static_front_weight_fraction)
                ),
                "effective_lltd": effective_lltd,
                "effective_lltd_interpretation": effective_lltd_interpretation,
                "front_antiroll_stiffness_fraction": (
                    front_antiroll_stiffness_fraction
                ),
                "front_elastic_roll_stiffness_fraction": (
                    front_elastic_roll_stiffness_fraction
                ),
                "effective_brake_distribution_front": float(
                    vehicle.brake_distribution_front
                ),
                "effective_aero_balance_front": float(vehicle.aero_balance_front),
                "effective_cop_from_front_m": float(
                    vehicle.wheelbase * (1.0 - vehicle.aero_balance_front)
                ),
                "case_tuning": tuning,
                "vehicle": asdict(vehicle),
                "reduced_vehicle_parameters": asdict(reduced_parameters),
                "reduced_parameter_overrides": reduced_parameter_summary,
                "projection_summary": projection.summary,
                "projection_summary_precedes_case_tuning_overrides": True,
                "ggv_csv": str(ggv_path),
                "ggv_requested_speed_slices_mps": list(requested_speed_slices),
                "ggv_qss_trim_speed_slices_mps": list(expected_qss_trim_speeds),
                "ggv_speed_slices": list(expected_exported_speed_slices),
                "qss_zero_speed_proxy_mps": zero_proxy_mps,
                "zero_speed_contract": zero_speed_contract,
                "ggv_ay_points": smoke_config.ay_points,
                "ggv_ay_max_g": smoke_config.ay_max_g,
                "ggv_ax_search_points": smoke_config.ax_search_points,
                "ggv_ax_binary_iterations": smoke_config.ax_binary_iterations,
                "tire_mu_scale": tire_mu_scale,
                "tire_mu_source": tire_mu_source,
                "source_peak_drive_power_w": source_peak_drive_power_w,
                "effective_drive_power_limit_w": effective_drive_power_limit_w,
                "drive_power_limit_source": drive_power_limit_source,
                "drive_distribution_front": float(vehicle.drive_distribution_front),
                "limited_slip_differential_model": False,
                "tire_valid_load_min_n": vehicle.fz_min_valid,
                "tire_valid_load_max_n": vehicle.fz_max_valid,
            }
            metadata = _validate_existing_case_metadata(
                json.loads(metadata_path.read_text(encoding="utf-8")),
                expected_fields=expected_metadata_fields,
            )

            map_stat_before = ggv_path.stat()
            exported_speed_slices = _validate_existing_ggv_speed_grid(
                ggv_path,
                expected_speeds=expected_exported_speed_slices,
            )
            ggv_map = GGVMap.from_csv(ggv_path)
            map_stat_after = ggv_path.stat()
            if (
                map_stat_before.st_size != map_stat_after.st_size
                or map_stat_before.st_mtime_ns != map_stat_after.st_mtime_ns
            ):
                raise RuntimeError(
                    "Existing ggv.csv changed during validation; wait for its "
                    "generator process to exit before recovery."
                )
            if ggv_map.source_sha256.lower() != expected_ggv_sha256.lower():
                raise ValueError(
                    "Existing ggv.csv SHA-256 does not match --expected-ggv-sha256; "
                    "refusing to recover or regenerate it."
                )
            serialized_top_speed = exported_speed_slices[-1]
            if not (
                math.isclose(ggv_map.csv_speed_min_mps, 0.0, abs_tol=1e-12)
                and math.isclose(
                    ggv_map.csv_speed_max_mps,
                    serialized_top_speed,
                    rel_tol=0.0,
                    abs_tol=1e-12,
                )
                and math.isfinite(ggv_map.maximum_speed_mps)
                and ggv_map.maximum_speed_mps > 0.0
                and ggv_map.maximum_speed_mps <= serialized_top_speed + 1e-12
                and any(
                    math.isclose(
                        ggv_map.maximum_speed_mps,
                        speed,
                        rel_tol=0.0,
                        abs_tol=1e-12,
                    )
                    for speed in exported_speed_slices
                )
            ):
                raise ValueError(
                    "Existing GGV has an invalid serialized or usable speed domain; "
                    "refusing to recover it."
                )
            standing_start_accel = ggv_map.acceleration(0.0, 0.0)
            if not math.isfinite(standing_start_accel) or standing_start_accel <= 0.0:
                raise ValueError(
                    "Existing GGV has no positive ay=0 standing-start acceleration "
                    "under the current lookup solver; refusing to recover it."
                )
            fz_min = _finite_metadata_float(metadata, "wheel_load_min_n")
            fz_max = _finite_metadata_float(metadata, "wheel_load_max_n")
            if fz_min > fz_max:
                raise ValueError(
                    "Existing GGV metadata wheel-load minimum exceeds its maximum."
                )

            metadata = {
                **metadata,
                "input_fingerprint_sha256": fingerprint,
                "ggv_source_sha256": ggv_map.source_sha256,
                "ggv_lookup_contract_version": GGV_LOOKUP_CONTRACT_VERSION,
                "ggv_lookup_solver_sha256": _sha256_file(GGV_LOOKUP_SOLVER_PATH),
                "ggv_reused_without_regeneration": True,
                "ggv_reuse_validation": {
                    "schema": "lapsims.existing-ggv-reuse.v1",
                    "input_fingerprint_sha256": fingerprint,
                    "ggv_source_sha256": ggv_map.source_sha256,
                    "validated_speed_slice_count": len(exported_speed_slices),
                    "requested_terminal_speed_mps": (
                        expected_exported_speed_slices[-1]
                    ),
                    "serialized_terminal_speed_mps": serialized_top_speed,
                    "requested_terminal_slice_had_no_serialized_rows": (
                        len(exported_speed_slices)
                        == len(expected_exported_speed_slices) - 1
                    ),
                    "validated_standing_start_accel_mps2": standing_start_accel,
                    "generation_was_skipped": True,
                },
            }
            print(
                f"Reusing validated existing GGV for {case_name}; generation skipped",
                flush=True,
            )
            case_rows = _solve_case_event_rows(
                ggv_map=ggv_map,
                solve_track=solve_track,
                manifest=manifest,
                case_dir=case_dir,
                metadata=metadata,
                legacy_times=legacy_times,
                wheel_load_min_n=fz_min,
                wheel_load_max_n=fz_max,
            )
            final_map_stat = ggv_path.stat()
            final_map_sha256 = _sha256_file(ggv_path)
            if (
                final_map_stat.st_size != map_stat_after.st_size
                or final_map_stat.st_mtime_ns != map_stat_after.st_mtime_ns
                or final_map_sha256.lower() != ggv_map.source_sha256.lower()
            ):
                raise RuntimeError(
                    "Existing ggv.csv changed while events were solved; refusing "
                    "to write a reusable case cache."
                )
            _write_json(metadata_path, metadata)
            case_metadata.append(metadata)
            rows.extend(case_rows)
            _write_case_cache(
                case_dir,
                input_fingerprint=fingerprint,
                metadata=metadata,
                rows=case_rows,
            )
            continue

        print(
            f"Generating {case_name} {model_key} GGV at h={height_m:.6f} m",
            flush=True,
        )
        envelopes, qss_trim_speeds = generate_envelopes(
            generate_ggv,
            vehicle=vehicle,
            config=smoke_config,
            reduced_model=reduced_model,
            zero_proxy_mps=zero_proxy_mps,
        )
        exported_speed_slices = [float(envelope.speed) for envelope in envelopes]
        if not math.isclose(
            exported_speed_slices[-1],
            requested_speed_slices[-1],
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise RuntimeError("GGV export did not retain the exact top-speed slice.")
        save_ggv_csv(envelopes, ggv_path)
        fz_min, fz_max = _wheel_load_range(vehicle, envelopes)
        ggv_map = GGVMap.from_csv(ggv_path)

        metadata = {
            "model_family": model_family,
            "model_key": model_key,
            "model_dof": model_dof,
            "sweep_axis": sweep_axis,
            "input_fingerprint_sha256": fingerprint,
            "ggv_source_sha256": ggv_map.source_sha256,
            "ggv_lookup_contract_version": GGV_LOOKUP_CONTRACT_VERSION,
            "ggv_lookup_solver_sha256": _sha256_file(GGV_LOOKUP_SOLVER_PATH),
            "cg_case": case_name,
            "cg_height_m": height_m,
            "cg_height_in": height_m / M_PER_INCH,
            "cg_offset_mm": 1000.0 * offset_m,
            "is_reference_case": case_name == reference_case,
            "sprung_mass_kg": (
                None
                if reduced_parameters is None
                else float(reduced_parameters.sprung_mass_kg)
            ),
            "total_mass_kg": float(vehicle.mass),
            "mass_offset_kg": mass_offset_kg,
            "mass_offset_lb": mass_offset_lb,
            "rear_static_weight_fraction": (
                None
                if reduced_parameters is None
                else float(reduced_parameters.static_rear_weight_fraction)
            ),
            "front_static_weight_fraction": (
                None
                if reduced_parameters is None
                else float(reduced_parameters.static_front_weight_fraction)
            ),
            "effective_lltd": effective_lltd,
            "effective_lltd_interpretation": (
                "front spring-plus-ARB elastic roll-stiffness fraction; a 3DOF "
                "algebraic LLTD proxy, not a direct 6DOF axle load-transfer result"
                if model_dof is not None
                else "legacy EnvelopeSim prescribed lateral load-transfer distribution"
            ),
            "front_antiroll_stiffness_fraction": (front_antiroll_stiffness_fraction),
            "front_elastic_roll_stiffness_fraction": (
                front_elastic_roll_stiffness_fraction
            ),
            "effective_brake_distribution_front": float(
                vehicle.brake_distribution_front
            ),
            "effective_aero_balance_front": float(vehicle.aero_balance_front),
            "effective_cop_from_front_m": float(
                vehicle.wheelbase * (1.0 - vehicle.aero_balance_front)
            ),
            "case_tuning": tuning,
            "vehicle": asdict(vehicle),
            "reduced_vehicle_parameters": (
                asdict(reduced_parameters) if reduced_parameters is not None else None
            ),
            "reduced_parameter_overrides": reduced_parameter_summary,
            "projection_summary": projection.summary,
            "projection_summary_precedes_case_tuning_overrides": True,
            "ggv_csv": str(ggv_path),
            "ggv_requested_speed_slices_mps": list(requested_speed_slices),
            "ggv_qss_trim_speed_slices_mps": list(qss_trim_speeds),
            "ggv_speed_slices": exported_speed_slices,
            "qss_zero_speed_proxy_mps": (
                zero_proxy_mps if model_dof is not None else None
            ),
            "zero_speed_contract": (
                "0 m/s boundary copied from the near-zero positive QSS trim; "
                "the positive proxy slice is also retained"
                if model_dof is not None
                else "legacy algebraic EnvelopeSim evaluates 0 m/s directly"
            ),
            "ggv_ay_points": smoke_config.ay_points,
            "ggv_ay_max_g": smoke_config.ay_max_g,
            "ggv_ax_search_points": smoke_config.ax_search_points,
            "ggv_ax_binary_iterations": smoke_config.ax_binary_iterations,
            "tire_mu_scale": tire_mu_scale,
            "tire_mu_scale_source": tire_mu_scale_source,
            "tire_mu_scaled_fields": list(TIRE_MU_SCALE_FIELDS),
            "tire_mu_source": tire_mu_source,
            "source_peak_drive_power_w": source_peak_drive_power_w,
            "effective_drive_power_limit_w": effective_drive_power_limit_w,
            "drive_power_limit_source": drive_power_limit_source,
            "drive_distribution_front": float(vehicle.drive_distribution_front),
            "limited_slip_differential_model": False,
            "wheel_load_min_n": fz_min,
            "wheel_load_max_n": fz_max,
            "wheel_load_range_method": (
                "legacy VehicleParams algebraic diagnostic over the exported "
                "boundary; reduced-QSS trim loads are not stored in GGVEnvelope"
                if model_dof is not None
                else "native legacy VehicleParams algebraic wheel loads"
            ),
            "tire_valid_load_min_n": vehicle.fz_min_valid,
            "tire_valid_load_max_n": vehicle.fz_max_valid,
            "wheel_load_outside_tire_validity": bool(
                fz_min < vehicle.fz_min_valid or fz_max > vehicle.fz_max_valid
            ),
        }
        _write_json(case_dir / "case_metadata.json", metadata)
        case_metadata.append(metadata)

        case_rows = _solve_case_event_rows(
            ggv_map=ggv_map,
            solve_track=solve_track,
            manifest=manifest,
            case_dir=case_dir,
            metadata=metadata,
            legacy_times=legacy_times,
            wheel_load_min_n=fz_min,
            wheel_load_max_n=fz_max,
        )

        rows.extend(case_rows)
        _write_case_cache(
            case_dir,
            input_fingerprint=fingerprint,
            metadata=metadata,
            rows=case_rows,
        )

    if partial_run:
        partial_payload = {
            "schema": "lapsims.ggv-cg-partial-generation.v1",
            "partial_generation": True,
            "scoring_performed": False,
            "model_family": model_family,
            "model_key": model_key,
            "model_dof": model_dof,
            "sweep_axis": sweep_axis,
            "tire_mu_scale": tire_mu_scale,
            "tire_mu_scale_source": tire_mu_scale_source,
            "source_peak_drive_power_w": source_peak_drive_power_w,
            "effective_drive_power_limit_w": effective_drive_power_limit_w,
            "drive_power_limit_source": drive_power_limit_source,
            "selected_cases": [str(case["name"]) for case in selected_cases],
            "output_root": str(output_root),
            "case_metadata": case_metadata,
            "raw_event_results": rows,
            "completion_instruction": (
                "After every case cache exists, rerun without --case and with "
                "--resume to assemble one model-wide scoring field and report."
            ),
        }
        partial_name = "partial_" + "__".join(
            str(case["name"]) for case in selected_cases
        )
        _write_json(output_root / f"{partial_name}.json", partial_payload)
        return partial_payload

    rows = _score_rows(rows, reference_case=reference_case)
    totals = _case_totals(rows)
    correlation = _points_correlation(totals, sweep_axis=sweep_axis)
    results = pd.DataFrame(rows)
    results.to_csv(output_root / "event_results.csv", index=False)
    pd.DataFrame(totals).to_csv(output_root / "case_totals.csv", index=False)
    tuned_subtitles = {
        "cg_height_in": "Per-height physical ARB split and fixed brake bias tuned",
        "sprung_mass_kg": "Per-mass physical ARB split and fixed brake bias tuned",
        "rear_static_weight_fraction": (
            "Per-balance physical ARB split and fixed brake bias tuned"
        ),
        "configuration": (
            "Per-configuration physical ARB split and fixed brake bias tuned"
        ),
    }
    untuned_subtitles = {
        "cg_height_in": "All vehicle inputs fixed except projected total CG height",
        "sprung_mass_kg": "All vehicle inputs fixed except sprung and total mass",
        "rear_static_weight_fraction": (
            "All vehicle inputs fixed except longitudinal total-CG position"
        ),
        "configuration": (
            "Explicit vehicle configurations; CG height and longitudinal CG "
            "position both change"
        ),
    }
    setup_subtitle = (
        tuned_subtitles[sweep_axis] if tuning_applied else untuned_subtitles[sweep_axis]
    )
    event_plot_name = {
        "cg_height_in": "cg_height_event_sensitivity.png",
        "sprung_mass_kg": "mass_event_sensitivity.png",
        "rear_static_weight_fraction": "longitudinal_cg_event_sensitivity.png",
        "configuration": "configuration_event_comparison.png",
    }[sweep_axis]
    correlation_json_name = {
        "cg_height_in": "cg_height_points_correlation.json",
        "sprung_mass_kg": "mass_points_correlation.json",
        "rear_static_weight_fraction": "longitudinal_cg_points_correlation.json",
        "configuration": "configuration_points_comparison.json",
    }[sweep_axis]
    correlation_plot_name = {
        "cg_height_in": "cg_height_points_correlation.png",
        "sprung_mass_kg": "mass_points_correlation.png",
        "rear_static_weight_fraction": "longitudinal_cg_points_correlation.png",
        "configuration": "configuration_points_comparison.png",
    }[sweep_axis]
    _make_plot(
        rows,
        totals,
        output_root / event_plot_name,
        setup_subtitle=setup_subtitle,
        sweep_axis=sweep_axis,
    )
    _write_json(output_root / correlation_json_name, correlation)
    _make_correlation_plot(
        totals,
        correlation,
        output_root / correlation_plot_name,
        sweep_axis=sweep_axis,
    )

    payload = {
        "study": {
            "cg_height_in": "EnvelopeSim GGV CG-height smoke study",
            "sprung_mass_kg": "EnvelopeSim GGV sprung-mass study",
            "rear_static_weight_fraction": (
                "EnvelopeSim GGV longitudinal-CG smoke study"
            ),
            "configuration": "EnvelopeSim GGV vehicle-configuration comparison",
        }[sweep_axis],
        "sweep_axis": sweep_axis,
        "model_family": model_family,
        "model_key": model_key,
        "model_dof": model_dof,
        "baseline_cg_height_m": baseline_height,
        "reference_cg_case": reference_case,
        "cg_offsets_m": [float(case["cg_offset_m"]) for case in sweep_cases],
        "sweep_cases": sweep_cases,
        "controlled_inputs": {
            "cg_height_in": (
                "Projected total CG height changes between cases. Mass, CG x, tire "
                "parameters and their study-wide mu scale, total aero load, powertrain, "
                "and tracks remain fixed. The front aero balance may be supplied as a "
                "sweep-level override; LLTD and one fixed front brake fraction per "
                "legacy case, or an explicit front anti-roll stiffness fraction and "
                "brake fraction per reduced-model case, may be supplied by either "
                "analysis-only tuning interface."
            ),
            "sprung_mass_kg": (
                "Sprung mass changes between cases while total CG x/y/z, explicit "
                "unsprung masses, spring and damper rates, total ARB stiffness, aero, "
                "powertrain, tires, and tracks remain fixed. Sprung inertia scales "
                "linearly with sprung mass. ARB distribution and one fixed front "
                "brake fraction are retuned per mass; ride/roll frequencies are not."
            ),
            "rear_static_weight_fraction": (
                "Total CG x changes to impose the declared rear static weight "
                "fraction while total and sprung mass, inertia about the CG, CG y/z, "
                "global axle/contact locations, 50/50 aero balance and total aero "
                "load, tire model and study-wide mu scale, powertrain, and tracks "
                "remain fixed. ARB distribution and one fixed front brake fraction "
                "are retuned per longitudinal-CG case."
            ),
            "configuration": (
                "Each named case may change total CG height and longitudinal total-"
                "CG position/static rear-weight fraction together. Total and sprung "
                "mass remain nominal when no explicit sprung mass is supplied. CG y, "
                "inertia about the CG, global axle/contact locations, 50/50 aero "
                "balance and total aero load, tire model and study-wide mu scale, "
                "powertrain, and tracks remain fixed. ARB distribution and one fixed "
                "front brake fraction are retuned per configuration. Results are a "
                "categorical comparison, not a one-variable sensitivity."
            ),
        }[sweep_axis],
        "aero_balance_front_override": aero_balance_front,
        "case_tuning_json": (
            str(case_tuning_path.resolve()) if case_tuning_path is not None else None
        ),
        "case_tuning": case_tuning,
        "sweep_json": (str(sweep_path.resolve()) if sweep_path is not None else None),
        "sweep_definition": sweep_definition,
        "ggv_resolution": {
            "requested_speed_slices_mps": list(requested_speed_slices),
            "exact_top_speed_mps": requested_speed_slices[-1],
            "qss_zero_speed_proxy_mps": (
                zero_proxy_mps if model_dof is not None else None
            ),
            "ay_points": smoke_config.ay_points,
            "ay_max_g": smoke_config.ay_max_g,
            "ax_search_points_per_branch": smoke_config.ax_search_points,
            "ax_binary_iterations": smoke_config.ax_binary_iterations,
        },
        "tire_mu_scale": tire_mu_scale,
        "tire_mu_scale_source": tire_mu_scale_source,
        "tire_mu_scaled_fields": list(TIRE_MU_SCALE_FIELDS),
        "tire_mu_scale_policy": tire_mu_source,
        "source_peak_drive_power_w": source_peak_drive_power_w,
        "effective_drive_power_limit_w": effective_drive_power_limit_w,
        "drive_power_limit_source": drive_power_limit_source,
        "drive_model": (
            "fixed RWD distribution with equal rear-wheel torque; no LSD model"
        ),
        "points_projection": (
            "Raw acceleration, skidpad, and autocross times are scored directly "
            "in a combined field with the official valid 2026 FSAE Electric "
            "Michigan results. The single fastest real or simulated result defines "
            "Tmin; only exact fastest-time ties share maximum points. The "
            "Michigan representative-lap pace is extended to the official 22 km "
            "distance and treated as a zero-penalty endurance time. Efficiency "
            "points are excluded."
        ),
        "fsae_2026_results_reference": _load_fsae_2026_results_reference(),
        "case_metadata": case_metadata,
        "event_results": rows,
        "case_totals": totals,
        "cg_height_points_correlation": (
            correlation if sweep_axis == "cg_height_in" else None
        ),
        "mass_points_correlation": (
            correlation if sweep_axis == "sprung_mass_kg" else None
        ),
        "longitudinal_cg_points_correlation": (
            correlation if sweep_axis == "rear_static_weight_fraction" else None
        ),
        "configuration_points_comparison": (
            correlation if sweep_axis == "configuration" else None
        ),
        "all_solvers_converged": all(bool(row["converged"]) for row in rows),
        "all_track_speeds_inside_ggv_domain": all(
            float(row["ggv_max_speed_domain_fraction"]) <= 1.0 + 1e-12 for row in rows
        ),
    }
    _write_json(output_root / "smoke_summary.json", payload)
    print(json.dumps({"case_totals": totals}, indent=2), flush=True)
    return payload


def main() -> None:
    run(parse_args())


if __name__ == "__main__":
    main()
