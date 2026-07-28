"""Refresh endurance and efficiency scoring while reusing sprint results.

The battery trade study and the short-event sweep intentionally have different
execution costs and state models.  This module provides the narrow bridge
between them: it accepts a new one-row-per-pack endurance result, preserves
every sprint-derived value from an existing dynamic-points output, and
recomputes only the shared endurance, efficiency, total, and rank columns.
"""

from __future__ import annotations

import hashlib
import json
import math
import shutil
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from battery_dynamic_points import (
    BASELINE_CANDIDATE_ID,
    EFFICIENCY_INPUTS,
    EVENT_RULES,
    _format_top_table,
    _global_mass_slopes,
    _headlines,
    _json_default,
    _plot_results,
    endurance_and_efficiency,
)


REQUIRED_BATTERY_RESULT_COLUMNS = {
    "candidate_id",
    "cell_id",
    "series_cells",
    "parallel_cells",
    "total_cells",
    "pack_mass_kg",
    "vehicle_mass_kg",
    "nominal_pack_energy_kwh",
    "model_usable_energy_kwh",
    "power_limit_kw",
    "elapsed_time_s",
    "equivalent_average_lap_time_s",
    "terminal_energy_kwh",
}

STRING_METADATA_COLUMNS = (
    "cell_id",
    "manufacturer",
    "cell_model",
)

INTEGER_METADATA_COLUMNS = (
    "series_cells",
    "parallel_cells",
    "total_cells",
)

FLOAT_METADATA_COLUMNS = (
    "pack_mass_kg",
    "vehicle_mass_kg",
    "nominal_pack_energy_kwh",
    "model_usable_energy_kwh",
)

SHARED_COLUMNS = (
    "raw_endurance_time_s",
    "projected_endurance_time_s",
    "endurance_points",
    "projected_average_lap_s",
    "projected_terminal_energy_kwh_per_lap",
    "efficiency_factor",
    "efficiency_points",
)

TOTAL_COLUMNS = (
    "pack_aware_performance_points_excluding_efficiency",
    "pack_aware_full_dynamic_points",
    "mass_isolated_sprint_hybrid_performance_points_excluding_efficiency",
    "mass_isolated_sprint_hybrid_full_dynamic_points",
    "pack_aware_minus_mass_isolated_full_dynamic_points",
)

OPTIONAL_BATTERY_METRIC_COLUMNS = (
    "terminal_discharge_energy_kwh",
    "terminal_regenerated_energy_kwh",
    "pack_resistive_heat_kwh",
    "regen_pack_resistive_heat_kwh",
    "regen_active_rms_terminal_power_kw",
    "regen_whole_event_rms_terminal_power_kw",
    "regen_active_rms_error_kw",
    "peak_pack_charge_current_a",
    "maximum_terminal_voltage_v",
    "adiabatic_cell_temperature_rise_c",
)

AFFECTED_RANK_BASE_COLUMNS = (
    "pack_aware_performance_points_excluding_efficiency",
    "pack_aware_full_dynamic_points",
    "mass_isolated_sprint_hybrid_performance_points_excluding_efficiency",
    "mass_isolated_sprint_hybrid_full_dynamic_points",
)

REFRESHED_SUMMARY_COLUMNS = {
    "endurance_terminal_power_limit_kw",
    "terminal_energy_kwh",
    *SHARED_COLUMNS,
    *TOTAL_COLUMNS,
    *OPTIONAL_BATTERY_METRIC_COLUMNS,
    *(f"{column}_rank" for column in AFFECTED_RANK_BASE_COLUMNS),
}

TRACEABILITY_COLUMNS = (
    "candidate_id",
    "cell_id",
    "manufacturer",
    "cell_model",
    "series_cells",
    "parallel_cells",
    "total_cells",
    "pack_mass_kg",
    "vehicle_mass_kg",
    "nominal_pack_energy_kwh",
    "model_usable_energy_kwh",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sprint_invariant_columns(source: pd.DataFrame) -> list[str]:
    """Return source-summary columns that an endurance refresh may not change."""

    return [
        column
        for column in source.columns
        if column not in REFRESHED_SUMMARY_COLUMNS
    ]


def _truthy(series: pd.Series) -> pd.Series:
    if pd.api.types.is_bool_dtype(series):
        return series.astype(bool)
    normalized = series.astype(str).str.strip().str.lower()
    return normalized.isin({"true", "1", "yes"})


def _validate_inputs(
    source: pd.DataFrame, battery_results: pd.DataFrame
) -> pd.DataFrame:
    if "candidate_id" not in source.columns:
        raise ValueError("Source summary is missing candidate_id")
    if source.empty:
        raise ValueError("Source summary is empty")
    if source["candidate_id"].duplicated().any():
        raise ValueError("Source summary contains duplicate candidate IDs")

    missing = REQUIRED_BATTERY_RESULT_COLUMNS - set(battery_results.columns)
    if missing:
        raise ValueError(
            f"Battery results are missing required columns: {sorted(missing)}"
        )
    if battery_results["candidate_id"].duplicated().any():
        raise ValueError("Battery results contain duplicate candidate IDs")

    source_ids = set(source["candidate_id"].astype(str))
    battery_ids = set(battery_results["candidate_id"].astype(str))
    if source_ids != battery_ids:
        missing_ids = sorted(source_ids - battery_ids)
        extra_ids = sorted(battery_ids - source_ids)
        raise ValueError(
            "Battery/source candidate sets differ: "
            f"missing={missing_ids}, extra={extra_ids}"
        )
    if BASELINE_CANDIDATE_ID not in source_ids:
        raise ValueError(f"Missing baseline {BASELINE_CANDIDATE_ID}")

    indexed = battery_results.copy()
    indexed["candidate_id"] = indexed["candidate_id"].astype(str)
    indexed = indexed.set_index("candidate_id", drop=False)
    source_indexed = source.copy()
    source_indexed["candidate_id"] = source_indexed["candidate_id"].astype(str)
    source_indexed = source_indexed.set_index("candidate_id", drop=False)
    indexed = indexed.loc[source_indexed.index]

    for column in STRING_METADATA_COLUMNS:
        if column not in source.columns or column not in indexed.columns:
            continue
        left = source_indexed[column].astype(str)
        right = indexed[column].astype(str)
        if not left.equals(right):
            raise ValueError(f"Battery results changed topology metadata: {column}")

    for column in INTEGER_METADATA_COLUMNS:
        if column not in source.columns:
            continue
        left = pd.to_numeric(source_indexed[column], errors="raise").astype(int)
        right = pd.to_numeric(indexed[column], errors="raise").astype(int)
        if not left.equals(right):
            raise ValueError(f"Battery results changed topology metadata: {column}")

    for column in FLOAT_METADATA_COLUMNS:
        if column not in source.columns:
            continue
        left = pd.to_numeric(source_indexed[column], errors="raise").to_numpy(
            dtype=float
        )
        right = pd.to_numeric(indexed[column], errors="raise").to_numpy(
            dtype=float
        )
        if not np.allclose(left, right, rtol=0.0, atol=1e-9):
            raise ValueError(f"Battery results changed topology metadata: {column}")

    positive_columns = (
        "power_limit_kw",
        "elapsed_time_s",
        "equivalent_average_lap_time_s",
        "terminal_energy_kwh",
    )
    for column in positive_columns:
        values = pd.to_numeric(indexed[column], errors="raise").to_numpy(
            dtype=float
        )
        if not np.isfinite(values).all() or np.any(values <= 0.0):
            raise ValueError(
                f"Battery result {column} must be finite and positive"
            )

    for completion_column in ("completed_target_distance", "constraint_valid"):
        if completion_column in indexed.columns and not _truthy(
            indexed[completion_column]
        ).all():
            failed = indexed.loc[
                ~_truthy(indexed[completion_column]), "candidate_id"
            ].tolist()
            raise ValueError(
                f"Battery results contain {completion_column}=False: {failed}"
            )
    return indexed


def refresh_dynamic_summary(
    source: pd.DataFrame, battery_results: pd.DataFrame
) -> tuple[pd.DataFrame, pd.DataFrame]:
    """Replace endurance results and recompute shared dynamic-point columns."""

    indexed = _validate_inputs(source, battery_results)
    refreshed = source.copy(deep=True)
    refreshed["candidate_id"] = refreshed["candidate_id"].astype(str)
    baseline = indexed.loc[BASELINE_CANDIDATE_ID]
    shared_rows: list[dict[str, Any]] = []

    for row_index, candidate_id in refreshed["candidate_id"].items():
        endurance_row = indexed.loc[candidate_id]
        shared = endurance_and_efficiency(endurance_row, baseline)
        refreshed.loc[row_index, "endurance_terminal_power_limit_kw"] = float(
            endurance_row["power_limit_kw"]
        )
        refreshed.loc[row_index, "terminal_energy_kwh"] = float(
            endurance_row["terminal_energy_kwh"]
        )
        optional_metrics = {
            column: float(endurance_row[column])
            for column in OPTIONAL_BATTERY_METRIC_COLUMNS
            if column in indexed.columns
        }
        for column, value in optional_metrics.items():
            refreshed.loc[row_index, column] = value
        for column, value in shared.items():
            refreshed.loc[row_index, column] = value
        shared_rows.append(
            {
                **{
                    column: refreshed.loc[row_index, column]
                    for column in TRACEABILITY_COLUMNS
                    if column in refreshed.columns
                },
                "endurance_terminal_power_limit_kw": float(
                    endurance_row["power_limit_kw"]
                ),
                "terminal_energy_kwh": float(
                    endurance_row["terminal_energy_kwh"]
                ),
                **optional_metrics,
                **shared,
            }
        )

    refreshed["pack_aware_performance_points_excluding_efficiency"] = (
        refreshed["pack_aware_nonenergy_points"]
        + refreshed["endurance_points"]
    )
    refreshed["pack_aware_full_dynamic_points"] = (
        refreshed["pack_aware_performance_points_excluding_efficiency"]
        + refreshed["efficiency_points"]
    )
    refreshed[
        "mass_isolated_sprint_hybrid_performance_points_excluding_efficiency"
    ] = (
        refreshed["mass_isolated_sprint_nonenergy_points"]
        + refreshed["endurance_points"]
    )
    refreshed["mass_isolated_sprint_hybrid_full_dynamic_points"] = (
        refreshed[
            "mass_isolated_sprint_hybrid_performance_points_excluding_efficiency"
        ]
        + refreshed["efficiency_points"]
    )
    refreshed["pack_aware_minus_mass_isolated_full_dynamic_points"] = (
        refreshed["pack_aware_full_dynamic_points"]
        - refreshed["mass_isolated_sprint_hybrid_full_dynamic_points"]
    )
    for column in AFFECTED_RANK_BASE_COLUMNS:
        refreshed[f"{column}_rank"] = refreshed[column].rank(
            method="min", ascending=False
        )

    # Reassert exact preservation after all vectorized calculations.  This also
    # guards against accidentally adding a refreshed field to the invariant set.
    invariant_columns = sprint_invariant_columns(source)
    pd.testing.assert_frame_equal(
        source[invariant_columns],
        refreshed[invariant_columns],
        check_exact=True,
        check_dtype=True,
        check_names=True,
    )
    return refreshed, pd.DataFrame(shared_rows)


def validate_refresh(
    source: pd.DataFrame,
    refreshed: pd.DataFrame,
    battery_results: pd.DataFrame,
) -> dict[str, Any]:
    """Return scenario-independent execution checks for a refreshed summary."""

    indexed = _validate_inputs(source, battery_results)
    source_indexed = source.set_index(source["candidate_id"].astype(str))
    refreshed_indexed = refreshed.set_index(
        refreshed["candidate_id"].astype(str)
    )
    invariant_columns = sprint_invariant_columns(source)

    try:
        pd.testing.assert_frame_equal(
            source_indexed[invariant_columns],
            refreshed_indexed[invariant_columns],
            check_exact=True,
            check_dtype=True,
            check_names=True,
        )
        sprint_columns_exact = True
    except AssertionError:
        sprint_columns_exact = False

    checks = {
        "all_source_candidates_present_once": bool(
            len(refreshed) == len(source)
            and refreshed["candidate_id"].nunique() == len(source)
            and set(refreshed["candidate_id"]) == set(source["candidate_id"])
        ),
        "all_sprint_invariant_columns_exact": sprint_columns_exact,
        "raw_endurance_time_matches_battery_results": bool(
            np.allclose(
                refreshed_indexed.loc[indexed.index, "raw_endurance_time_s"],
                indexed["elapsed_time_s"],
                rtol=0.0,
                atol=0.0,
            )
        ),
        "net_terminal_energy_matches_battery_results": bool(
            np.allclose(
                refreshed_indexed.loc[indexed.index, "terminal_energy_kwh"],
                indexed["terminal_energy_kwh"],
                rtol=0.0,
                atol=0.0,
            )
        ),
        "endurance_power_limit_matches_battery_results": bool(
            np.allclose(
                refreshed_indexed.loc[
                    indexed.index, "endurance_terminal_power_limit_kw"
                ],
                indexed["power_limit_kw"],
                rtol=0.0,
                atol=0.0,
            )
        ),
        "baseline_endurance_maps_to_tmin": bool(
            math.isclose(
                float(
                    refreshed_indexed.loc[
                        BASELINE_CANDIDATE_ID, "projected_endurance_time_s"
                    ]
                ),
                EVENT_RULES["endurance"].tmin_s,
                rel_tol=0.0,
                abs_tol=1e-9,
            )
        ),
        "baseline_efficiency_maps_to_maximum": bool(
            math.isclose(
                float(
                    refreshed_indexed.loc[
                        BASELINE_CANDIDATE_ID, "efficiency_points"
                    ]
                ),
                EFFICIENCY_INPUTS["maximum_points"],
                rel_tol=0.0,
                abs_tol=1e-9,
            )
        ),
        "pack_aware_performance_total_arithmetic": bool(
            np.allclose(
                refreshed["pack_aware_nonenergy_points"]
                + refreshed["endurance_points"],
                refreshed[
                    "pack_aware_performance_points_excluding_efficiency"
                ],
                rtol=0.0,
                atol=1e-9,
            )
        ),
        "pack_aware_full_total_arithmetic": bool(
            np.allclose(
                refreshed[
                    "pack_aware_performance_points_excluding_efficiency"
                ]
                + refreshed["efficiency_points"],
                refreshed["pack_aware_full_dynamic_points"],
                rtol=0.0,
                atol=1e-9,
            )
        ),
        "mass_isolated_hybrid_total_arithmetic": bool(
            np.allclose(
                refreshed[
                    "mass_isolated_sprint_hybrid_performance_points_excluding_efficiency"
                ]
                + refreshed["efficiency_points"],
                refreshed[
                    "mass_isolated_sprint_hybrid_full_dynamic_points"
                ],
                rtol=0.0,
                atol=1e-9,
            )
        ),
        "full_dynamic_scores_in_0_to_675": bool(
            refreshed[
                [
                    "pack_aware_full_dynamic_points",
                    "mass_isolated_sprint_hybrid_full_dynamic_points",
                ]
            ]
            .apply(lambda column: column.between(0.0, 675.0 + 1e-9))
            .all()
            .all()
        ),
    }
    for column in OPTIONAL_BATTERY_METRIC_COLUMNS:
        if column not in indexed.columns:
            continue
        checks[f"{column}_matches_battery_results"] = bool(
            np.allclose(
                refreshed_indexed.loc[indexed.index, column],
                indexed[column],
                rtol=0.0,
                atol=0.0,
            )
        )
    return {
        "all_checks_passed": bool(all(checks.values())),
        "checks": checks,
        "candidate_count": int(len(refreshed)),
        "sprint_invariant_column_count": len(invariant_columns),
    }


def copy_sprint_long_file(source: Path, destination: Path) -> str:
    """Copy the sprint long-form CSV and assert byte-for-byte identity."""

    source = source.resolve()
    destination = destination.resolve()
    if source == destination:
        raise ValueError("Sprint source and destination must differ")
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    source_hash = _sha256(source)
    if _sha256(destination) != source_hash:
        raise RuntimeError("Sprint long-form CSV changed during copy")
    return source_hash


def _comparison_frame(
    source: pd.DataFrame, refreshed: pd.DataFrame
) -> pd.DataFrame:
    columns = [
        "candidate_id",
        "raw_endurance_time_s",
        "endurance_points",
        "efficiency_points",
        "pack_aware_performance_points_excluding_efficiency",
        "pack_aware_full_dynamic_points",
        "pack_aware_full_dynamic_points_rank",
    ]
    before = source[columns].copy()
    after = refreshed[
        [
            *columns,
            "endurance_terminal_power_limit_kw",
            "terminal_energy_kwh",
            *[
                column
                for column in OPTIONAL_BATTERY_METRIC_COLUMNS
                if column in refreshed.columns
            ],
        ]
    ].copy()
    before = before.add_prefix("previous_").rename(
        columns={"previous_candidate_id": "candidate_id"}
    )
    after = after.add_prefix("refreshed_").rename(
        columns={"refreshed_candidate_id": "candidate_id"}
    )
    comparison = before.merge(
        after, on="candidate_id", how="inner", validate="one_to_one"
    )
    for column in (
        "raw_endurance_time_s",
        "endurance_points",
        "efficiency_points",
        "pack_aware_performance_points_excluding_efficiency",
        "pack_aware_full_dynamic_points",
        "pack_aware_full_dynamic_points_rank",
    ):
        comparison[f"{column}_change"] = (
            comparison[f"refreshed_{column}"]
            - comparison[f"previous_{column}"]
        )
    return comparison


def _write_report(
    output_dir: Path,
    refreshed: pd.DataFrame,
    scenario_name: str,
    battery_results_csv: Path,
    sprint_source_dir: Path,
    sprint_hash: str,
) -> None:
    best = refreshed.sort_values(
        [
            "pack_aware_full_dynamic_points",
            "pack_aware_performance_points_excluding_efficiency",
            "pack_mass_kg",
        ],
        ascending=[False, False, True],
    ).iloc[0]
    baseline = refreshed.loc[
        refreshed["candidate_id"] == BASELINE_CANDIDATE_ID
    ].iloc[0]
    regen_headline = ""
    if "terminal_regenerated_energy_kwh" in refreshed.columns:
        regen_headline = (
            "- Winning pack recovered terminal energy: "
            f"**{best.terminal_regenerated_energy_kwh:.6f} kWh**; "
            "regen-created pack heat: "
            f"**{best.regen_pack_resistive_heat_kwh:.6f} kWh**; achieved "
            "active regen RMS: "
            f"**{best.regen_active_rms_terminal_power_kw:.3f} kW**.\n"
        )
    report = f"""# Battery dynamic points: endurance-only refresh

## Scope

Scenario: `{scenario_name}`.

Acceleration, skidpad, autocross, pack-aware full-SOC diagnostics, and the
mass-isolated sprint model are reused from `{sprint_source_dir}`. The long-form
sprint CSV is copied byte-for-byte (SHA-256 `{sprint_hash}`), and all
sprint-invariant summary columns are required to remain exactly equal.

Only endurance raw time, net accumulator-terminal energy, the sustainable
endurance power limit, inherited endurance/efficiency scores, combined totals,
and their affected ranks are refreshed from `{battery_results_csv}`.

## Headline results

- Highest refreshed pack-aware full dynamic score: **{best.candidate_id}** at
  **{best.pack_aware_full_dynamic_points:.3f} / 675**.
- Baseline **{BASELINE_CANDIDATE_ID}**: **{baseline.raw_endurance_time_s:.3f} s**,
  **{baseline.terminal_energy_kwh:.6f} kWh net terminal energy**, and
  **{baseline.endurance_terminal_power_limit_kw:.3f} kW** sustainable terminal
  power.
- The P30B baseline is remapped to the established 2026 endurance `Tmin`, and
  every candidate retains its refreshed endurance-time ratio to that baseline.
- The inherited efficiency projection is also recalculated relative to the
  refreshed P30B time and net terminal energy.
{regen_headline}

## Top refreshed configurations

{_format_top_table(refreshed)}

## Artifacts

- `battery_dynamic_points_summary.csv`: refreshed one-row-per-pack results.
- `battery_dynamic_event_results_long.csv`: byte-identical sprint-event source.
- `inherited_endurance_efficiency_scores.csv`: refreshed shared scoring inputs.
- `endurance_efficiency_delta.csv`: previous-to-refreshed score and rank deltas.
- `scoring_inputs.json`: scenario, source paths, formulas, and hashes.
- `validation_report.json`: preservation and arithmetic checks.
- `headline_summary.json`, `plots/`, and `artifact_manifest.json`: decision
  outputs.
"""
    (output_dir / "battery_dynamic_points_report.md").write_text(
        report, encoding="utf-8"
    )


def run_refresh(
    *,
    sprint_source_dir: Path,
    battery_results_csv: Path,
    output_dir: Path,
    scenario_name: str,
    write_plots: bool = True,
    overwrite: bool = False,
) -> dict[str, Any]:
    """Write a versioned dynamic-points output without rerunning sprints."""

    sprint_source_dir = sprint_source_dir.resolve()
    battery_results_csv = battery_results_csv.resolve()
    output_dir = output_dir.resolve()
    if output_dir == sprint_source_dir:
        raise ValueError("Output directory must not overwrite the sprint source")
    if not sprint_source_dir.is_dir():
        raise FileNotFoundError(sprint_source_dir)
    if not battery_results_csv.is_file():
        raise FileNotFoundError(battery_results_csv)
    if output_dir.exists() and any(output_dir.iterdir()) and not overwrite:
        raise FileExistsError(
            f"Output directory is not empty: {output_dir}; pass overwrite=True"
        )
    output_dir.mkdir(parents=True, exist_ok=True)

    source_summary_csv = (
        sprint_source_dir / "battery_dynamic_points_summary.csv"
    )
    source_long_csv = (
        sprint_source_dir / "battery_dynamic_event_results_long.csv"
    )
    if not source_summary_csv.is_file() or not source_long_csv.is_file():
        raise FileNotFoundError(
            "Sprint source must contain the summary and long-form CSVs"
        )

    source = pd.read_csv(source_summary_csv)
    battery_results = pd.read_csv(battery_results_csv)
    refreshed, shared = refresh_dynamic_summary(source, battery_results)
    validation = validate_refresh(source, refreshed, battery_results)

    refreshed.to_csv(
        output_dir / "battery_dynamic_points_summary.csv", index=False
    )
    shared.to_csv(
        output_dir / "inherited_endurance_efficiency_scores.csv", index=False
    )
    _comparison_frame(source, refreshed).to_csv(
        output_dir / "endurance_efficiency_delta.csv", index=False
    )
    sprint_hash = copy_sprint_long_file(
        source_long_csv,
        output_dir / "battery_dynamic_event_results_long.csv",
    )
    validation["checks"]["sprint_long_file_is_byte_identical"] = bool(
        _sha256(output_dir / "battery_dynamic_event_results_long.csv")
        == sprint_hash
    )
    validation["all_checks_passed"] = bool(
        all(validation["checks"].values())
    )

    old_headline_path = sprint_source_dir / "headline_summary.json"
    if old_headline_path.is_file():
        old_headline = json.loads(old_headline_path.read_text(encoding="utf-8"))
        local_sensitivity = old_headline.get("local_mass_sensitivity", {})
        global_slopes = old_headline.get("global_mass_slopes", {})
    else:
        local_sensitivity = {}
        global_slopes = {}
    if not global_slopes:
        global_slopes = _global_mass_slopes(refreshed)
    headlines = _headlines(
        refreshed,
        local_sensitivity=local_sensitivity,
        global_slopes=global_slopes,
    )
    headlines["scenario_name"] = scenario_name
    headlines["sprint_source_directory"] = str(sprint_source_dir)
    headlines["battery_results_csv"] = str(battery_results_csv)
    headlines["sprint_long_sha256"] = sprint_hash
    (output_dir / "headline_summary.json").write_text(
        json.dumps(headlines, indent=2, default=_json_default),
        encoding="utf-8",
    )

    old_inputs_path = sprint_source_dir / "scoring_inputs.json"
    if old_inputs_path.is_file():
        scoring_inputs = json.loads(old_inputs_path.read_text(encoding="utf-8"))
    else:
        scoring_inputs = {}
    scoring_inputs["scenario"] = {
        "name": scenario_name,
        "scope": "endurance_and_efficiency_only",
        "terminal_energy_interpretation": (
            "net accumulator-terminal energy from the refreshed battery results"
        ),
        "sprint_reuse_policy": (
            "No sprint simulation is rerun; source sprint columns are exact and "
            "the long-form sprint CSV is copied byte-for-byte."
        ),
    }
    scoring_inputs["refresh_sources"] = {
        "source_summary_csv": str(source_summary_csv),
        "source_summary_sha256": _sha256(source_summary_csv),
        "source_sprint_long_csv": str(source_long_csv),
        "source_sprint_long_sha256": sprint_hash,
        "battery_results_csv": str(battery_results_csv),
        "battery_results_sha256": _sha256(battery_results_csv),
    }
    scoring_inputs.setdefault("input_paths", {})["candidates"] = str(
        battery_results_csv
    )
    (output_dir / "scoring_inputs.json").write_text(
        json.dumps(scoring_inputs, indent=2), encoding="utf-8"
    )
    (output_dir / "validation_report.json").write_text(
        json.dumps(validation, indent=2), encoding="utf-8"
    )
    if write_plots:
        _plot_results(refreshed, output_dir / "plots")
    _write_report(
        output_dir,
        refreshed,
        scenario_name,
        battery_results_csv,
        sprint_source_dir,
        sprint_hash,
    )

    manifest = {
        "output_directory": str(output_dir),
        "scenario_name": scenario_name,
        "source_summary_sha256": _sha256(source_summary_csv),
        "source_sprint_long_sha256": sprint_hash,
        "battery_results_sha256": _sha256(battery_results_csv),
        "validation_passed": validation["all_checks_passed"],
        "files": sorted(
            str(path.relative_to(output_dir))
            for path in output_dir.rglob("*")
            if path.is_file() and path.name != "artifact_manifest.json"
        ),
    }
    (output_dir / "artifact_manifest.json").write_text(
        json.dumps(manifest, indent=2), encoding="utf-8"
    )
    return {
        "output_directory": str(output_dir),
        "scenario_name": scenario_name,
        "validation": validation,
        "headlines": headlines,
        "artifact_count": len(manifest["files"]) + 1,
    }
