"""Re-score the battery event sweep against the fastest simulated candidate.

This module deliberately leaves the competition-anchored study untouched. It
reads that study's raw OpenLAP and endurance times, chooses a separate cohort
``Tmin`` for every timed event and sprint model, and applies the official 2026
Formula SAE event equations directly to the raw simulated times.

Both useful efficiency interpretations are retained:

* the existing competition-calibrated efficiency projection; and
* a fully cohort-normalized 2026 EV efficiency score with strict energy and
  endurance-time eligibility.

The latter is the recommended internal design-ranking total because every
dynamic-event component then uses the same simulated cohort as its reference.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import replace
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from battery_dynamic_points import (
    BASELINE_CANDIDATE_ID,
    EVENT_RULES,
    MASS_ISOLATED_MODE,
    OFFICIAL_2026_RULES_URL,
    PACK_AWARE_MODE,
    TimeScoreRule,
)


SPRINT_MODES = {
    "pack_aware": {
        "display_name": "Pack-aware, full SOC, 80 kW terminal ceiling",
        "source_mode": PACK_AWARE_MODE,
    },
    "mass_isolated": {
        "display_name": "Mass-only, common 80 kW tractive curve",
        "source_mode": MASS_ISOLATED_MODE,
    },
}
SPRINT_EVENTS = ("acceleration", "skidpad", "autocross")
TIMED_EVENT_MAXIMUM = sum(
    EVENT_RULES[slug].maximum_points
    for slug in (*SPRINT_EVENTS, "endurance")
)
FULL_DYNAMIC_MAXIMUM = TIMED_EVENT_MAXIMUM + 100.0
EV_EFFICIENCY_CO2_LIMIT_KG_PER_100_KM = 20.02
EV_CO2_CONVERSION_KG_PER_KWH = 0.65
ENDURANCE_DISTANCE_KM = 22.0
EFFICIENCY_ENDURANCE_TIME_FACTOR = 1.45
EV_EFFICIENCY_ENERGY_LIMIT_KWH = (
    EV_EFFICIENCY_CO2_LIMIT_KG_PER_100_KM
    * ENDURANCE_DISTANCE_KM
    / 100.0
    / EV_CO2_CONVERSION_KG_PER_KWH
)


def cohort_rule(rule: TimeScoreRule, tmin_s: float) -> TimeScoreRule:
    """Return an event rule anchored to a simulated-cohort minimum time."""

    if not math.isfinite(tmin_s) or tmin_s <= 0.0:
        raise ValueError("Cohort Tmin must be finite and positive")
    return replace(rule, tmin_s=float(tmin_s))


def score_time_series(
    times: pd.Series, rule: TimeScoreRule
) -> tuple[pd.Series, TimeScoreRule]:
    """Score a positive time series with its fastest value assigned max points."""

    numeric = pd.to_numeric(times, errors="raise").astype(float)
    if numeric.empty:
        raise ValueError("Cannot score an empty cohort")
    if not np.isfinite(numeric.to_numpy()).all() or (numeric <= 0.0).any():
        raise ValueError("Timed-event results must be finite and positive")
    anchored_rule = cohort_rule(rule, float(numeric.min()))
    points = numeric.map(anchored_rule.score).astype(float)
    return points, anchored_rule


def _source_columns(source: pd.DataFrame) -> list[str]:
    """Select traceability, raw-result, and inherited-efficiency columns."""

    exact = [
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
        "endurance_terminal_power_limit_kw",
        "raw_endurance_time_s",
        "terminal_energy_kwh",
        "projected_average_lap_s",
        "projected_terminal_energy_kwh_per_lap",
        "efficiency_factor",
        "efficiency_points",
    ]
    prefixes = (
        "pack_aware_curve_",
        "pack_aware_full_soc",
        "pack_aware_terminal_power_limit_",
        "pack_aware_maximum_",
        "pack_aware_active_limiter_",
        "pack_aware_reaches_",
    )
    suffixes = ("_raw_time_s", "_converged")
    selected = []
    for column in source.columns:
        if (
            column in exact
            or any(column.startswith(prefix) for prefix in prefixes)
            or (
                column.startswith(("pack_aware_", "mass_isolated_"))
                and column.endswith(suffixes)
            )
        ):
            selected.append(column)
    missing = sorted(set(exact) - set(selected))
    if missing:
        raise ValueError(f"Source summary is missing required columns: {missing}")
    return selected


def join_terminal_energy(
    source: pd.DataFrame, battery_results: pd.DataFrame
) -> pd.DataFrame:
    """Join one authoritative terminal-energy value to every candidate."""

    required = {"candidate_id", "terminal_energy_kwh"}
    missing = required - set(battery_results.columns)
    if missing:
        raise ValueError(
            f"Battery results are missing required columns: {sorted(missing)}"
        )
    energy = battery_results[["candidate_id", "terminal_energy_kwh"]].copy()
    if energy["candidate_id"].duplicated().any():
        raise ValueError("Battery results contain duplicate candidate IDs")
    if "terminal_energy_kwh" in source.columns:
        source = source.drop(columns=["terminal_energy_kwh"])
    joined = source.merge(
        energy, on="candidate_id", how="left", validate="one_to_one"
    )
    if joined["terminal_energy_kwh"].isna().any():
        missing_ids = joined.loc[
            joined["terminal_energy_kwh"].isna(), "candidate_id"
        ].tolist()
        raise ValueError(
            f"Terminal energy is unavailable for candidates: {missing_ids}"
        )
    if set(joined["candidate_id"]) != set(energy["candidate_id"]):
        extra_ids = sorted(set(energy["candidate_id"]) - set(joined["candidate_id"]))
        raise ValueError(
            f"Battery results contain candidates absent from event sweep: {extra_ids}"
        )
    return joined


def cohort_efficiency_projection(
    endurance_times_s: pd.Series,
    terminal_energies_kwh: pd.Series,
) -> tuple[pd.DataFrame, dict[str, Any]]:
    """Apply strict 2026 EV eligibility and cohort-normalized efficiency.

    ``Tmin`` is the fastest *eligible* configuration. ``Emin`` is the minimum
    energy among eligible configurations. The fastest overall endurance time is
    retained separately because it establishes the 1.45-times time-eligibility
    threshold even when that configuration is energy-ineligible.
    """

    times = pd.to_numeric(endurance_times_s, errors="raise").astype(float)
    energies = pd.to_numeric(
        terminal_energies_kwh, errors="raise"
    ).astype(float)
    if len(times) != len(energies) or times.empty:
        raise ValueError("Efficiency inputs must be nonempty and equal-length")
    if (
        not np.isfinite(times.to_numpy()).all()
        or not np.isfinite(energies.to_numpy()).all()
        or (times <= 0.0).any()
        or (energies <= 0.0).any()
    ):
        raise ValueError("Efficiency times and energies must be positive")

    overall_tmin_s = float(times.min())
    endurance_time_limit_s = (
        EFFICIENCY_ENDURANCE_TIME_FACTOR * overall_tmin_s
    )
    energy_eligible = energies <= EV_EFFICIENCY_ENERGY_LIMIT_KWH
    time_eligible = times <= endurance_time_limit_s
    eligible = energy_eligible & time_eligible
    if not eligible.any():
        raise ValueError("No configuration satisfies EV efficiency eligibility")

    eligible_tmin_s = float(times[eligible].min())
    eligible_emin_kwh = float(energies[eligible].min())
    factors = (eligible_tmin_s / times) * (eligible_emin_kwh / energies)
    efmax = float(factors[eligible].max())
    efmin = (
        (1.0 / EFFICIENCY_ENDURANCE_TIME_FACTOR)
        * (eligible_emin_kwh / EV_EFFICIENCY_ENERGY_LIMIT_KWH)
    )
    if efmax <= efmin:
        raise ValueError("Efficiency EFmax must exceed EFmin")
    unclipped_points = 100.0 * (factors - efmin) / (efmax - efmin)
    points = unclipped_points.clip(lower=0.0, upper=100.0)
    points = points.where(eligible, 0.0)
    results = pd.DataFrame(
        {
            "cohort_efficiency_energy_eligible": energy_eligible.astype(bool),
            "cohort_efficiency_time_eligible": time_eligible.astype(bool),
            "cohort_efficiency_eligible": eligible.astype(bool),
            "cohort_efficiency_factor": factors.astype(float),
            "cohort_efficiency_unclipped_points": unclipped_points.astype(float),
            "cohort_efficiency_points": points.astype(float),
        },
        index=times.index,
    )
    metadata = {
        "formula": "EF=(eligible_Tmin/T)*(eligible_Emin/E)",
        "points_formula": (
            "clip(100*(EF-EFmin)/(EFmax-EFmin),0,100); ineligible=0"
        ),
        "overall_fastest_endurance_time_s": overall_tmin_s,
        "endurance_time_eligibility_factor": EFFICIENCY_ENDURANCE_TIME_FACTOR,
        "endurance_time_limit_s": endurance_time_limit_s,
        "energy_limit_derivation": (
            "20.02 kgCO2/100km * 22km / 100 / 0.65 kgCO2/kWh"
        ),
        "energy_limit_kwh": EV_EFFICIENCY_ENERGY_LIMIT_KWH,
        "eligible_tmin_s": eligible_tmin_s,
        "eligible_emin_kwh": eligible_emin_kwh,
        "efmin": float(efmin),
        "efmax": efmax,
        "eligible_count": int(eligible.sum()),
        "energy_ineligible_count": int((~energy_eligible).sum()),
        "time_ineligible_count": int((~time_eligible).sum()),
    }
    return results, metadata


def score_cohort(source: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame, dict]:
    """Return cohort-scored wide/long tables and the event anchor metadata."""

    if source["candidate_id"].duplicated().any():
        raise ValueError("Source summary contains duplicate candidate IDs")
    summary = source[_source_columns(source)].copy()
    anchors: dict[str, Any] = {
        "sprint_modes": {},
        "endurance": {},
        "efficiency": {},
    }

    for mode in SPRINT_MODES:
        anchors["sprint_modes"][mode] = {}
        for slug in SPRINT_EVENTS:
            raw_column = f"{mode}_{slug}_raw_time_s"
            points, anchored_rule = score_time_series(
                summary[raw_column], EVENT_RULES[slug]
            )
            points_column = f"{mode}_{slug}_cohort_points"
            summary[points_column] = points
            summary[f"{mode}_{slug}_cohort_point_loss"] = (
                anchored_rule.maximum_points - points
            )
            summary[f"{mode}_{slug}_cohort_time_rank"] = (
                summary[raw_column].rank(method="min", ascending=True).astype(int)
            )
            fastest = summary.loc[
                np.isclose(
                    summary[raw_column],
                    anchored_rule.tmin_s,
                    rtol=0.0,
                    atol=1e-12,
                ),
                "candidate_id",
            ].tolist()
            anchors["sprint_modes"][mode][slug] = {
                **anchored_rule.as_dict(),
                "source_raw_time_column": raw_column,
                "fastest_candidate_ids": fastest,
            }

        nonenergy_column = f"{mode}_cohort_nonenergy_points"
        performance_column = (
            f"{mode}_cohort_performance_points_excluding_efficiency"
        )
        full_column = f"{mode}_cohort_full_dynamic_points"
        summary[nonenergy_column] = sum(
            summary[f"{mode}_{slug}_cohort_points"]
            for slug in SPRINT_EVENTS
        )
        summary[performance_column] = (
            summary[nonenergy_column]
        )
        summary[full_column] = summary[nonenergy_column]

    endurance_points, endurance_rule = score_time_series(
        summary["raw_endurance_time_s"], EVENT_RULES["endurance"]
    )
    summary["endurance_cohort_points"] = endurance_points
    summary["endurance_cohort_point_loss"] = (
        endurance_rule.maximum_points - endurance_points
    )
    summary["endurance_cohort_time_rank"] = (
        summary["raw_endurance_time_s"]
        .rank(method="min", ascending=True)
        .astype(int)
    )
    endurance_fastest = summary.loc[
        np.isclose(
            summary["raw_endurance_time_s"],
            endurance_rule.tmin_s,
            rtol=0.0,
            atol=1e-12,
        ),
        "candidate_id",
    ].tolist()
    anchors["endurance"] = {
        **endurance_rule.as_dict(),
        "source_raw_time_column": "raw_endurance_time_s",
        "fastest_candidate_ids": endurance_fastest,
    }

    efficiency_results, efficiency_metadata = cohort_efficiency_projection(
        summary["raw_endurance_time_s"], summary["terminal_energy_kwh"]
    )
    for column in efficiency_results:
        summary[column] = efficiency_results[column]
    eligible = summary["cohort_efficiency_eligible"]
    efficiency_metadata.update(
        {
            "overall_fastest_candidate_ids": summary.loc[
                np.isclose(
                    summary["raw_endurance_time_s"],
                    efficiency_metadata["overall_fastest_endurance_time_s"],
                    rtol=0.0,
                    atol=1e-12,
                ),
                "candidate_id",
            ].tolist(),
            "eligible_tmin_candidate_ids": summary.loc[
                eligible
                & np.isclose(
                    summary["raw_endurance_time_s"],
                    efficiency_metadata["eligible_tmin_s"],
                    rtol=0.0,
                    atol=1e-12,
                ),
                "candidate_id",
            ].tolist(),
            "eligible_emin_candidate_ids": summary.loc[
                eligible
                & np.isclose(
                    summary["terminal_energy_kwh"],
                    efficiency_metadata["eligible_emin_kwh"],
                    rtol=0.0,
                    atol=1e-12,
                ),
                "candidate_id",
            ].tolist(),
            "efmax_candidate_ids": summary.loc[
                eligible
                & np.isclose(
                    summary["cohort_efficiency_factor"],
                    efficiency_metadata["efmax"],
                    rtol=0.0,
                    atol=1e-12,
                ),
                "candidate_id",
            ].tolist(),
            "energy_ineligible_candidate_ids": summary.loc[
                ~summary["cohort_efficiency_energy_eligible"], "candidate_id"
            ].tolist(),
            "time_ineligible_candidate_ids": summary.loc[
                ~summary["cohort_efficiency_time_eligible"], "candidate_id"
            ].tolist(),
        }
    )
    anchors["efficiency"] = efficiency_metadata
    summary["inherited_competition_efficiency_factor"] = summary[
        "efficiency_factor"
    ]
    summary["inherited_competition_efficiency_points"] = summary[
        "efficiency_points"
    ]

    for mode in SPRINT_MODES:
        nonenergy_column = f"{mode}_cohort_nonenergy_points"
        performance_column = (
            f"{mode}_cohort_performance_points_excluding_efficiency"
        )
        full_column = f"{mode}_cohort_full_dynamic_points"
        inherited_explicit_column = (
            f"{mode}_cohort_full_dynamic_points_with_inherited_efficiency"
        )
        fully_cohort_column = (
            f"{mode}_fully_cohort_normalized_full_dynamic_points"
        )
        summary[performance_column] = (
            summary[nonenergy_column] + summary["endurance_cohort_points"]
        )
        summary[full_column] = (
            summary[performance_column] + summary["efficiency_points"]
        )
        summary[inherited_explicit_column] = summary[full_column]
        summary[fully_cohort_column] = (
            summary[performance_column] + summary["cohort_efficiency_points"]
        )
        for column in (
            nonenergy_column,
            performance_column,
            full_column,
            inherited_explicit_column,
            fully_cohort_column,
        ):
            summary[f"{column}_rank"] = (
                summary[column].rank(method="min", ascending=False).astype(int)
            )

    summary["pack_aware_minus_mass_isolated_cohort_nonenergy_points"] = (
        summary["pack_aware_cohort_nonenergy_points"]
        - summary["mass_isolated_cohort_nonenergy_points"]
    )
    summary[
        "pack_aware_minus_mass_isolated_cohort_full_dynamic_points"
    ] = (
        summary["pack_aware_cohort_full_dynamic_points"]
        - summary["mass_isolated_cohort_full_dynamic_points"]
    )
    summary[
        "pack_aware_minus_mass_isolated_fully_cohort_normalized_points"
    ] = (
        summary["pack_aware_fully_cohort_normalized_full_dynamic_points"]
        - summary["mass_isolated_fully_cohort_normalized_full_dynamic_points"]
    )

    long_rows = []
    metadata = [
        "candidate_id",
        "cell_id",
        "manufacturer",
        "cell_model",
        "series_cells",
        "parallel_cells",
        "pack_mass_kg",
        "vehicle_mass_kg",
    ]
    for row in summary.itertuples(index=False):
        row_dict = row._asdict()
        base = {column: row_dict[column] for column in metadata}
        for mode, mode_inputs in SPRINT_MODES.items():
            for slug in SPRINT_EVENTS:
                anchor = anchors["sprint_modes"][mode][slug]
                raw_time = float(row_dict[f"{mode}_{slug}_raw_time_s"])
                points = float(row_dict[f"{mode}_{slug}_cohort_points"])
                long_rows.append(
                    {
                        **base,
                        "simulation_mode": mode_inputs["source_mode"],
                        "simulation_mode_short": mode,
                        "event_slug": slug,
                        "event_name": EVENT_RULES[slug].display_name,
                        "raw_simulated_time_s": raw_time,
                        "cohort_tmin_s": anchor["tmin_s"],
                        "tmax_s": anchor["tmax_s"],
                        "points": points,
                        "maximum_points": EVENT_RULES[slug].maximum_points,
                        "point_loss_vs_event_max": (
                            EVENT_RULES[slug].maximum_points - points
                        ),
                        "is_cohort_fastest": bool(
                            math.isclose(
                                raw_time,
                                anchor["tmin_s"],
                                rel_tol=0.0,
                                abs_tol=1e-12,
                            )
                        ),
                        "time_rank": int(
                            row_dict[f"{mode}_{slug}_cohort_time_rank"]
                        ),
                    }
                )
        raw_endurance = float(row_dict["raw_endurance_time_s"])
        endurance_score = float(row_dict["endurance_cohort_points"])
        long_rows.append(
            {
                **base,
                "simulation_mode": "shared_chronological_endurance",
                "simulation_mode_short": "shared_endurance",
                "event_slug": "endurance",
                "event_name": EVENT_RULES["endurance"].display_name,
                "raw_simulated_time_s": raw_endurance,
                "cohort_tmin_s": endurance_rule.tmin_s,
                "tmax_s": endurance_rule.tmax_s,
                "points": endurance_score,
                "maximum_points": endurance_rule.maximum_points,
                "point_loss_vs_event_max": (
                    endurance_rule.maximum_points - endurance_score
                ),
                "is_cohort_fastest": bool(
                    math.isclose(
                        raw_endurance,
                        endurance_rule.tmin_s,
                        rel_tol=0.0,
                        abs_tol=1e-12,
                    )
                ),
                "time_rank": int(row_dict["endurance_cohort_time_rank"]),
            }
        )
    long = pd.DataFrame(long_rows)
    return summary, long, anchors


def _row_payload(row: pd.Series) -> dict[str, Any]:
    return {
        "candidate_id": str(row["candidate_id"]),
        "cell_id": str(row["cell_id"]),
        "series_cells": int(row["series_cells"]),
        "parallel_cells": int(row["parallel_cells"]),
        "pack_mass_kg": float(row["pack_mass_kg"]),
        "vehicle_mass_kg": float(row["vehicle_mass_kg"]),
        "acceleration_points": float(
            row["pack_aware_acceleration_cohort_points"]
        ),
        "skidpad_points": float(row["pack_aware_skidpad_cohort_points"]),
        "autocross_points": float(row["pack_aware_autocross_cohort_points"]),
        "endurance_points": float(row["endurance_cohort_points"]),
        "terminal_energy_kwh": float(row["terminal_energy_kwh"]),
        "cohort_efficiency_eligible": bool(
            row["cohort_efficiency_eligible"]
        ),
        "inherited_competition_efficiency_points": float(
            row["inherited_competition_efficiency_points"]
        ),
        "cohort_efficiency_points": float(
            row["cohort_efficiency_points"]
        ),
        "performance_points_excluding_efficiency": float(
            row["pack_aware_cohort_performance_points_excluding_efficiency"]
        ),
        "full_dynamic_points_with_inherited_efficiency": float(
            row[
                "pack_aware_cohort_full_dynamic_points_with_inherited_efficiency"
            ]
        ),
        "full_dynamic_rank_with_inherited_efficiency": int(
            row[
                "pack_aware_cohort_full_dynamic_points_with_inherited_efficiency_rank"
            ]
        ),
        "fully_cohort_normalized_full_dynamic_points": float(
            row[
                "pack_aware_fully_cohort_normalized_full_dynamic_points"
            ]
        ),
        "fully_cohort_normalized_full_dynamic_rank": int(
            row[
                "pack_aware_fully_cohort_normalized_full_dynamic_points_rank"
            ]
        ),
    }


def _headline_summary(
    summary: pd.DataFrame, anchors: dict[str, Any]
) -> dict[str, Any]:
    baseline = summary.loc[
        summary["candidate_id"] == BASELINE_CANDIDATE_ID
    ].iloc[0]
    pack_best = summary.sort_values(
        [
            "pack_aware_cohort_full_dynamic_points",
            "pack_aware_cohort_performance_points_excluding_efficiency",
            "pack_mass_kg",
        ],
        ascending=[False, False, True],
    ).iloc[0]
    recommended_best = summary.sort_values(
        [
            "pack_aware_fully_cohort_normalized_full_dynamic_points",
            "pack_aware_cohort_performance_points_excluding_efficiency",
            "pack_mass_kg",
        ],
        ascending=[False, False, True],
    ).iloc[0]
    pack_performance_best = summary.sort_values(
        [
            "pack_aware_cohort_performance_points_excluding_efficiency",
            "pack_aware_cohort_full_dynamic_points",
            "pack_mass_kg",
        ],
        ascending=[False, False, True],
    ).iloc[0]
    mass_best = summary.sort_values(
        [
            "mass_isolated_cohort_full_dynamic_points",
            "mass_isolated_cohort_performance_points_excluding_efficiency",
            "pack_mass_kg",
        ],
        ascending=[False, False, True],
    ).iloc[0]
    mass_recommended_best = summary.sort_values(
        [
            "mass_isolated_fully_cohort_normalized_full_dynamic_points",
            "mass_isolated_cohort_performance_points_excluding_efficiency",
            "pack_mass_kg",
        ],
        ascending=[False, False, True],
    ).iloc[0]
    return {
        "candidate_count": int(len(summary)),
        "scoring_basis": (
            "fastest raw simulated configuration defines Tmin separately "
            "for every timed event and sprint mode"
        ),
        "maximum_points": {
            "nonenergy_sprint": 300.0,
            "performance_excluding_efficiency": TIMED_EVENT_MAXIMUM,
            "full_dynamic_including_efficiency": FULL_DYNAMIC_MAXIMUM,
        },
        "recommended_internal_design_total": (
            "fully cohort-normalized timed events plus cohort-normalized "
            "strict-eligibility EV efficiency"
        ),
        "efficiency_policy": {
            "recommended": (
                "Strict-eligibility cohort efficiency using cohort Tmin, Emin, "
                "EFmin, and EFmax."
            ),
            "comparison": (
                "The inherited competition-calibrated efficiency projection "
                "and its total remain unchanged under explicit names."
            ),
        },
        "cohort_anchors": anchors,
        "recommended_pack_aware_best_fully_cohort_normalized": _row_payload(
            recommended_best
        ),
        "pack_aware_best_full_dynamic": _row_payload(pack_best),
        "pack_aware_best_performance_excluding_efficiency": _row_payload(
            pack_performance_best
        ),
        "mass_isolated_best_full_dynamic": {
            "candidate_id": str(mass_best["candidate_id"]),
            "pack_mass_kg": float(mass_best["pack_mass_kg"]),
            "performance_points_excluding_efficiency": float(
                mass_best[
                    "mass_isolated_cohort_performance_points_excluding_efficiency"
                ]
            ),
            "full_dynamic_points": float(
                mass_best["mass_isolated_cohort_full_dynamic_points"]
            ),
            "full_dynamic_rank": int(
                mass_best["mass_isolated_cohort_full_dynamic_points_rank"]
            ),
        },
        "recommended_mass_isolated_best_fully_cohort_normalized": {
            "candidate_id": str(mass_recommended_best["candidate_id"]),
            "pack_mass_kg": float(mass_recommended_best["pack_mass_kg"]),
            "performance_points_excluding_efficiency": float(
                mass_recommended_best[
                    "mass_isolated_cohort_performance_points_excluding_efficiency"
                ]
            ),
            "cohort_efficiency_points": float(
                mass_recommended_best["cohort_efficiency_points"]
            ),
            "fully_cohort_normalized_full_dynamic_points": float(
                mass_recommended_best[
                    "mass_isolated_fully_cohort_normalized_full_dynamic_points"
                ]
            ),
            "fully_cohort_normalized_full_dynamic_rank": int(
                mass_recommended_best[
                    "mass_isolated_fully_cohort_normalized_full_dynamic_points_rank"
                ]
            ),
        },
        "baseline": _row_payload(baseline),
    }


def _validation_report(
    source: pd.DataFrame,
    summary: pd.DataFrame,
    long: pd.DataFrame,
    anchors: dict[str, Any],
) -> dict[str, Any]:
    event_point_columns = [
        f"{mode}_{slug}_cohort_points"
        for mode in SPRINT_MODES
        for slug in SPRINT_EVENTS
    ] + ["endurance_cohort_points"]
    endurance_times = summary["raw_endurance_time_s"].astype(float)
    terminal_energies = summary["terminal_energy_kwh"].astype(float)
    expected_overall_tmin = float(endurance_times.min())
    expected_time_limit = (
        EFFICIENCY_ENDURANCE_TIME_FACTOR * expected_overall_tmin
    )
    expected_energy_eligible = (
        terminal_energies <= EV_EFFICIENCY_ENERGY_LIMIT_KWH
    )
    expected_time_eligible = endurance_times <= expected_time_limit
    expected_eligible = expected_energy_eligible & expected_time_eligible
    eligible_times = endurance_times[expected_eligible]
    eligible_energies = terminal_energies[expected_eligible]
    expected_eligible_tmin = float(eligible_times.min())
    expected_eligible_emin = float(eligible_energies.min())
    expected_factors = (
        expected_eligible_tmin / endurance_times
    ) * (expected_eligible_emin / terminal_energies)
    expected_efmax = float(expected_factors[expected_eligible].max())
    expected_efmin = (
        (1.0 / EFFICIENCY_ENDURANCE_TIME_FACTOR)
        * (
            expected_eligible_emin
            / EV_EFFICIENCY_ENERGY_LIMIT_KWH
        )
    )
    checks: dict[str, bool] = {
        "all_source_candidates_present_once": bool(
            len(summary) == len(source)
            and summary["candidate_id"].nunique() == len(source)
            and set(summary["candidate_id"]) == set(source["candidate_id"])
        ),
        "seven_timed_rows_per_candidate": bool(
            len(long) == len(source) * 7
            and long.groupby("candidate_id").size().eq(7).all()
        ),
        "all_timed_scores_finite": bool(
            np.isfinite(summary[event_point_columns].to_numpy(dtype=float)).all()
        ),
        "efficiency_scores_inherited_exactly": bool(
            np.allclose(
                summary.sort_values("candidate_id")["efficiency_points"],
                source.sort_values("candidate_id")["efficiency_points"],
                rtol=0.0,
                atol=0.0,
            )
        ),
        "strict_efficiency_energy_limit_is_6p776_kwh": bool(
            math.isclose(
                anchors["efficiency"]["energy_limit_kwh"],
                6.776,
                rel_tol=0.0,
                abs_tol=1e-12,
            )
        ),
        "cohort_efficiency_has_eligible_configuration": bool(
            expected_eligible.any()
        ),
        "energy_eligibility_matches_current_scenario": bool(
            summary["cohort_efficiency_energy_eligible"].equals(
                expected_energy_eligible
            )
        ),
        "time_eligibility_matches_current_scenario": bool(
            summary["cohort_efficiency_time_eligible"].equals(
                expected_time_eligible
            )
        ),
        "combined_efficiency_eligibility_matches_current_scenario": bool(
            summary["cohort_efficiency_eligible"].equals(expected_eligible)
        ),
        "cohort_efficiency_has_at_least_one_maximum": bool(
            np.isclose(
                summary["cohort_efficiency_points"],
                100.0,
                rtol=0.0,
                atol=1e-9,
            ).any()
        ),
        "all_strictly_ineligible_efficiency_scores_are_zero": bool(
            np.allclose(
                summary.loc[
                    ~summary["cohort_efficiency_eligible"],
                    "cohort_efficiency_points",
                ],
                0.0,
                rtol=0.0,
                atol=0.0,
            )
        ),
        "efficiency_eligibility_counts_match_current_scenario": bool(
            anchors["efficiency"]["eligible_count"]
            == int(expected_eligible.sum())
            and anchors["efficiency"]["energy_ineligible_count"]
            == int((~expected_energy_eligible).sum())
            and anchors["efficiency"]["time_ineligible_count"]
            == int((~expected_time_eligible).sum())
        ),
        "overall_efficiency_time_anchor_matches_current_scenario": bool(
            math.isclose(
                anchors["efficiency"]["overall_fastest_endurance_time_s"],
                expected_overall_tmin,
                rel_tol=0.0,
                abs_tol=1e-12,
            )
            and math.isclose(
                anchors["efficiency"]["endurance_time_limit_s"],
                expected_time_limit,
                rel_tol=0.0,
                abs_tol=1e-12,
            )
        ),
        "eligible_efficiency_anchors_match_current_scenario": bool(
            math.isclose(
                anchors["efficiency"]["eligible_tmin_s"],
                expected_eligible_tmin,
                rel_tol=0.0,
                abs_tol=1e-12,
            )
            and math.isclose(
                anchors["efficiency"]["eligible_emin_kwh"],
                expected_eligible_emin,
                rel_tol=0.0,
                abs_tol=1e-12,
            )
            and math.isclose(
                anchors["efficiency"]["efmin"],
                expected_efmin,
                rel_tol=0.0,
                abs_tol=1e-12,
            )
            and math.isclose(
                anchors["efficiency"]["efmax"],
                expected_efmax,
                rel_tol=0.0,
                abs_tol=1e-12,
            )
        ),
        "performance_totals_in_0_to_575": bool(
            (
                summary[
                    [
                        "pack_aware_cohort_performance_points_excluding_efficiency",
                        "mass_isolated_cohort_performance_points_excluding_efficiency",
                    ]
                ]
                >= 0.0
            ).all().all()
            and (
                summary[
                    [
                        "pack_aware_cohort_performance_points_excluding_efficiency",
                        "mass_isolated_cohort_performance_points_excluding_efficiency",
                    ]
                ]
                <= TIMED_EVENT_MAXIMUM + 1e-9
            ).all().all()
        ),
        "full_dynamic_totals_in_0_to_675": bool(
            (
                summary[
                    [
                        "pack_aware_cohort_full_dynamic_points",
                        "mass_isolated_cohort_full_dynamic_points",
                        "pack_aware_fully_cohort_normalized_full_dynamic_points",
                        "mass_isolated_fully_cohort_normalized_full_dynamic_points",
                    ]
                ]
                >= 0.0
            ).all().all()
            and (
                summary[
                    [
                        "pack_aware_cohort_full_dynamic_points",
                        "mass_isolated_cohort_full_dynamic_points",
                        "pack_aware_fully_cohort_normalized_full_dynamic_points",
                        "mass_isolated_fully_cohort_normalized_full_dynamic_points",
                    ]
                ]
                <= FULL_DYNAMIC_MAXIMUM + 1e-9
            ).all().all()
        ),
    }
    eligible_efficiency = summary.loc[
        summary["cohort_efficiency_eligible"]
    ].sort_values("cohort_efficiency_factor")
    checks["eligible_efficiency_score_monotonic_with_factor"] = bool(
        eligible_efficiency["cohort_efficiency_points"]
        .diff()
        .fillna(0.0)
        .ge(-1e-9)
        .all()
    )

    for mode in SPRINT_MODES:
        for slug in SPRINT_EVENTS:
            anchor = anchors["sprint_modes"][mode][slug]
            raw = summary[f"{mode}_{slug}_raw_time_s"]
            points = summary[f"{mode}_{slug}_cohort_points"]
            maximum = EVENT_RULES[slug].maximum_points
            prefix = f"{mode}_{slug}"
            checks[f"{prefix}_fastest_gets_maximum"] = bool(
                np.allclose(
                    points[np.isclose(raw, raw.min(), rtol=0.0, atol=1e-12)],
                    maximum,
                    rtol=0.0,
                    atol=1e-9,
                )
            )
            checks[f"{prefix}_anchor_matches_raw_minimum"] = bool(
                math.isclose(
                    anchor["tmin_s"],
                    float(raw.min()),
                    rel_tol=0.0,
                    abs_tol=1e-12,
                )
            )
            ordered = summary.sort_values(raw.name)
            checks[f"{prefix}_points_nonincreasing_with_time"] = bool(
                ordered[points.name].diff().fillna(0.0).le(1e-9).all()
            )

    endurance_fastest = np.isclose(
        summary["raw_endurance_time_s"],
        summary["raw_endurance_time_s"].min(),
        rtol=0.0,
        atol=1e-12,
    )
    checks["endurance_fastest_gets_275"] = bool(
        np.allclose(
            summary.loc[endurance_fastest, "endurance_cohort_points"],
            EVENT_RULES["endurance"].maximum_points,
            rtol=0.0,
            atol=1e-9,
        )
    )
    checks["endurance_anchor_matches_raw_minimum"] = bool(
        math.isclose(
            anchors["endurance"]["tmin_s"],
            float(summary["raw_endurance_time_s"].min()),
            rel_tol=0.0,
            abs_tol=1e-12,
        )
    )
    ordered_endurance = summary.sort_values("raw_endurance_time_s")
    checks["endurance_points_nonincreasing_with_time"] = bool(
        ordered_endurance["endurance_cohort_points"]
        .diff()
        .fillna(0.0)
        .le(1e-9)
        .all()
    )
    for mode in SPRINT_MODES:
        arithmetic = (
            summary[f"{mode}_cohort_nonenergy_points"]
            + summary["endurance_cohort_points"]
        )
        checks[f"{mode}_performance_total_arithmetic"] = bool(
            np.allclose(
                arithmetic,
                summary[
                    f"{mode}_cohort_performance_points_excluding_efficiency"
                ],
                rtol=0.0,
                atol=1e-9,
            )
        )
        checks[f"{mode}_full_total_arithmetic"] = bool(
            np.allclose(
                arithmetic + summary["efficiency_points"],
                summary[f"{mode}_cohort_full_dynamic_points"],
                rtol=0.0,
                atol=1e-9,
            )
        )
        checks[f"{mode}_fully_cohort_total_arithmetic"] = bool(
            np.allclose(
                arithmetic + summary["cohort_efficiency_points"],
                summary[
                    f"{mode}_fully_cohort_normalized_full_dynamic_points"
                ],
                rtol=0.0,
                atol=1e-9,
            )
        )
    return {
        "all_checks_passed": bool(all(checks.values())),
        "checks": checks,
        "candidate_count": int(len(summary)),
        "timed_event_row_count": int(len(long)),
        "check_count": int(len(checks)),
    }


def _format_top_table(summary: pd.DataFrame, count: int = 15) -> str:
    top = summary.sort_values(
        [
            "pack_aware_fully_cohort_normalized_full_dynamic_points",
            "pack_aware_cohort_performance_points_excluding_efficiency",
            "pack_mass_kg",
        ],
        ascending=[False, False, True],
    ).head(count)
    header = (
        "| Rank | Candidate | Pack kg | Accel | Skidpad | Autox | "
        "Endurance | Cohort eff. | Recommended total | Inherited-eff. total |\n"
        "|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|"
    )
    rows = []
    for rank, row in enumerate(top.itertuples(index=False), start=1):
        rows.append(
            f"| {rank} | {row.candidate_id} | {row.pack_mass_kg:.3f} | "
            f"{row.pack_aware_acceleration_cohort_points:.2f} | "
            f"{row.pack_aware_skidpad_cohort_points:.2f} | "
            f"{row.pack_aware_autocross_cohort_points:.2f} | "
            f"{row.endurance_cohort_points:.2f} | "
            f"{row.cohort_efficiency_points:.2f} | "
            f"{row.pack_aware_fully_cohort_normalized_full_dynamic_points:.2f} | "
            f"{row.pack_aware_cohort_full_dynamic_points_with_inherited_efficiency:.2f} |"
        )
    return "\n".join([header, *rows])


def _fastest_table(anchors: dict[str, Any]) -> str:
    header = (
        "| Model | Event | Cohort Tmin | Max points | Fastest config |\n"
        "|---|---|---:|---:|---|"
    )
    rows = []
    for mode, mode_data in anchors["sprint_modes"].items():
        for slug in SPRINT_EVENTS:
            anchor = mode_data[slug]
            rows.append(
                f"| {mode} | {EVENT_RULES[slug].display_name} | "
                f"{anchor['tmin_s']:.6f} s | "
                f"{anchor['maximum_points']:.1f} | "
                f"{', '.join(anchor['fastest_candidate_ids'])} |"
            )
    anchor = anchors["endurance"]
    rows.append(
        f"| shared | Endurance | {anchor['tmin_s']:.6f} s | "
        f"{anchor['maximum_points']:.1f} | "
        f"{', '.join(anchor['fastest_candidate_ids'])} |"
    )
    return "\n".join([header, *rows])


def _write_report(
    output_dir: Path,
    summary: pd.DataFrame,
    headlines: dict[str, Any],
    source_csv: Path,
) -> None:
    recommended_best = headlines[
        "recommended_pack_aware_best_fully_cohort_normalized"
    ]
    inherited_best = headlines["pack_aware_best_full_dynamic"]
    performance_best = headlines[
        "pack_aware_best_performance_excluding_efficiency"
    ]
    baseline = headlines["baseline"]
    efficiency = headlines["cohort_anchors"]["efficiency"]
    report = f"""# Battery dynamic points: simulated-cohort normalization

## What changed

This scoring step reads and re-scores {len(summary)} completed source rows; it
does not run a lap solver. The source may be the original dynamic study or a
versioned endurance-only refresh that reused its sprint results. For every
timed event, the fastest raw simulated configuration now defines that event's
`Tmin` and receives the official maximum score. All slower configurations are
scored directly from that raw time using the 2026 Formula SAE equation and the
event's official `Tmax/Tmin` factor.

Acceleration, skidpad, and autocross receive separate cohort anchors for the
pack-aware and mass-isolated sprint models. Endurance uses the fastest raw 22 km
time across the cohort.

Two efficiency interpretations are included:

1. **Recommended internal-design score:** strict 2026 EV eligibility followed
   by cohort normalization of efficiency factor. This makes every component of
   the 675-point total relative to the same simulated design cohort.
2. **Inherited comparison score:** the prior competition-calibrated efficiency
   projection is preserved unchanged, along with its original total and rank.

Source: `{source_csv}`

## Cohort anchors

{_fastest_table(headlines["cohort_anchors"])}

### Efficiency anchors and eligibility

- Maximum permitted terminal energy:
  **{efficiency["energy_limit_kwh"]:.6f} kWh**, from
  20.02 kgCO2/100 km * 22 km / 100 / 0.65 kgCO2/kWh.
- Overall endurance `Tmin`: **{efficiency["overall_fastest_endurance_time_s"]:.6f} s**
  ({", ".join(efficiency["overall_fastest_candidate_ids"])}).
- Efficiency time limit: **{efficiency["endurance_time_limit_s"]:.6f} s**
  (1.45 times overall endurance `Tmin`).
- Fastest eligible `Tmin`: **{efficiency["eligible_tmin_s"]:.6f} s**
  ({", ".join(efficiency["eligible_tmin_candidate_ids"])}).
- Eligible minimum `Emin`: **{efficiency["eligible_emin_kwh"]:.9f} kWh**
  ({", ".join(efficiency["eligible_emin_candidate_ids"])}).
- `EFmin = {efficiency["efmin"]:.9f}` and
  `EFmax = {efficiency["efmax"]:.9f}`.
- {efficiency["eligible_count"]} eligible configurations;
  {efficiency["energy_ineligible_count"]} energy-ineligible and
  {efficiency["time_ineligible_count"]} time-ineligible.
- Energy-ineligible:
  {", ".join(efficiency["energy_ineligible_candidate_ids"])}.

## Headline rankings

- **Recommended fully cohort-normalized winner:**
  **{recommended_best["candidate_id"]}**,
  **{recommended_best["fully_cohort_normalized_full_dynamic_points"]:.3f} /
  {FULL_DYNAMIC_MAXIMUM:.0f}**.
- Winner when retaining inherited competition-calibrated efficiency:
  **{inherited_best["candidate_id"]}**,
  **{inherited_best["full_dynamic_points_with_inherited_efficiency"]:.3f} /
  {FULL_DYNAMIC_MAXIMUM:.0f}**.

- Best pack-aware timed-event performance subtotal (efficiency excluded):
  **{performance_best["candidate_id"]}**,
  **{performance_best["performance_points_excluding_efficiency"]:.3f} /
  {TIMED_EVENT_MAXIMUM:.0f}**.
- Baseline **{BASELINE_CANDIDATE_ID}** scores
  **{baseline["performance_points_excluding_efficiency"]:.3f} /
  {TIMED_EVENT_MAXIMUM:.0f}** timed-event points and
  **{baseline["fully_cohort_normalized_full_dynamic_points"]:.3f} /
  {FULL_DYNAMIC_MAXIMUM:.0f}** with cohort efficiency versus
  **{baseline["full_dynamic_points_with_inherited_efficiency"]:.3f} /
  {FULL_DYNAMIC_MAXIMUM:.0f}** with inherited efficiency.

## Recommended top 15 pack-aware configurations

{_format_top_table(summary)}

## Scoring details

- Acceleration: 100 maximum, 4.5 completion, `Tmax = 1.50 * Tmin`.
- Skidpad: 75 maximum, 3.5 completion, `Tmax = 1.25 * Tmin`, squared
  time-ratio equation.
- Autocross: 125 maximum, 6.5 completion, `Tmax = 1.45 * Tmin`.
- Endurance: 275 maximum, 25 completion, `Tmax = 1.45 * Tmin`.
- Timed-event performance subtotal: {TIMED_EVENT_MAXIMUM:.0f} points.
- Cohort efficiency:
  `EF=(eligible_Tmin/T)*(eligible_Emin/E)` and
  `points=clip(100*(EF-EFmin)/(EFmax-EFmin),0,100)`. Ineligible
  configurations receive zero efficiency points.
- Inherited competition-calibrated efficiency: retained as a comparison,
  up to 100 points.
- Full dynamic total: {FULL_DYNAMIC_MAXIMUM:.0f} points.

The pack-aware result is the decision model: it includes pack mass plus the
pack's full-SOC voltage, resistance, continuous-current limit, and the common
80 kW battery-terminal ceiling. The mass-isolated result is a diagnostic that
holds the 80 kW tractive curve constant and changes mass only. Because each
sprint model has its own fastest candidate, compare ranks and point losses
within a model; do not interpret their absolute cross-model point difference as
a physical powertrain penalty.

## Files

- `battery_cohort_points_summary.csv`: one row per configuration with all
  cohort scores, totals, and ranks.
- `battery_cohort_event_results_long.csv`: seven timed-event rows per
  configuration (three events in each sprint model plus shared endurance).
- `inherited_efficiency_scores.csv`: explicit unchanged efficiency inputs.
- `cohort_efficiency_scores.csv`: terminal energy, strict eligibility, cohort
  factor, and cohort efficiency points.
- `cohort_scoring_inputs.json`: formulas, cohort anchors, and policy.
- `headline_summary.json`: decision-ready winners and baseline.
- `validation_report.json`: automated score and arithmetic checks.
- `plots/`: cohort-normalized decision plots.
"""
    (output_dir / "battery_cohort_points_report.md").write_text(
        report, encoding="utf-8"
    )


def _plot_results(summary: pd.DataFrame, plot_dir: Path) -> None:
    plot_dir.mkdir(parents=True, exist_ok=True)
    colors = {
        "pack": "#0072B2",
        "mass": "#D55E00",
        "acceleration": "#56B4E9",
        "skidpad": "#009E73",
        "autocross": "#E69F00",
        "endurance": "#CC79A7",
        "efficiency": "#6A3D9A",
    }
    mass = summary["pack_mass_kg"]
    order = np.argsort(mass.to_numpy())

    fig, axes = plt.subplots(2, 2, figsize=(13.5, 9.0), sharex=True)
    for ax, slug in zip(
        axes.ravel(), (*SPRINT_EVENTS, "endurance")
    ):
        if slug == "endurance":
            ax.scatter(
                mass,
                summary["endurance_cohort_points"],
                s=22,
                alpha=0.72,
                color=colors[slug],
                label="Shared endurance",
            )
        else:
            ax.scatter(
                mass,
                summary[f"pack_aware_{slug}_cohort_points"],
                s=22,
                alpha=0.72,
                color=colors["pack"],
                label="Pack-aware",
            )
            ax.plot(
                mass.iloc[order],
                summary[f"mass_isolated_{slug}_cohort_points"].iloc[order],
                linewidth=1.7,
                color=colors["mass"],
                label="Mass-only",
            )
        ax.set_title(EVENT_RULES[slug].display_name)
        ax.set_xlabel("Pack mass (kg)")
        ax.set_ylabel("Cohort-normalized points")
        ax.grid(alpha=0.25)
        ax.legend()
    fig.suptitle("Event scores with fastest simulated configuration at max")
    fig.tight_layout()
    fig.savefig(plot_dir / "cohort_event_points_vs_pack_mass.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9.8, 6.2))
    ax.scatter(
        mass,
        summary["pack_aware_fully_cohort_normalized_full_dynamic_points"],
        s=25,
        alpha=0.75,
        color=colors["pack"],
        label="Pack-aware, fully cohort-normalized (recommended)",
    )
    ax.scatter(
        mass,
        summary[
            "pack_aware_cohort_full_dynamic_points_with_inherited_efficiency"
        ],
        s=18,
        alpha=0.45,
        color=colors["efficiency"],
        label="Pack-aware with inherited efficiency",
    )
    ax.set(
        xlabel="Pack mass (kg)",
        ylabel=f"Dynamic points ({FULL_DYNAMIC_MAXIMUM:.0f} max)",
        title="Fully cohort-normalized versus inherited-efficiency total",
    )
    ax.grid(alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(plot_dir / "cohort_full_dynamic_points_vs_pack_mass.png", dpi=180)
    plt.close(fig)

    top = summary.nlargest(
        15, "pack_aware_fully_cohort_normalized_full_dynamic_points"
    ).sort_values("pack_aware_fully_cohort_normalized_full_dynamic_points")
    components = [
        (
            "pack_aware_acceleration_cohort_points",
            "Acceleration",
            colors["acceleration"],
        ),
        ("pack_aware_skidpad_cohort_points", "Skidpad", colors["skidpad"]),
        (
            "pack_aware_autocross_cohort_points",
            "Autocross",
            colors["autocross"],
        ),
        ("endurance_cohort_points", "Endurance", colors["endurance"]),
        (
            "cohort_efficiency_points",
            "Cohort efficiency",
            colors["efficiency"],
        ),
    ]
    fig, ax = plt.subplots(figsize=(11.2, 7.6))
    left = np.zeros(len(top))
    for column, label, color in components:
        values = top[column].to_numpy(dtype=float)
        ax.barh(top["candidate_id"], values, left=left, label=label, color=color)
        left += values
    ax.set(
        xlabel=f"Dynamic points ({FULL_DYNAMIC_MAXIMUM:.0f} max)",
        title="Top 15 pack-aware configurations: fully cohort-normalized",
    )
    ax.grid(axis="x", alpha=0.25)
    ax.legend(ncol=3, loc="lower right")
    fig.tight_layout()
    fig.savefig(
        plot_dir / "top15_cohort_dynamic_points_stacked.png", dpi=180
    )
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9.8, 6.2))
    point_loss = (
        300.0 - summary["pack_aware_cohort_nonenergy_points"]
    )
    scatter = ax.scatter(
        mass,
        point_loss,
        c=summary["pack_aware_maximum_achievable_terminal_power_kw"],
        cmap="viridis",
        s=35,
        alpha=0.82,
    )
    ax.set(
        xlabel="Pack mass (kg)",
        ylabel="Sprint point loss versus event maxima",
        title="Pack-aware sprint loss: mass and electrical capability",
    )
    ax.grid(alpha=0.25)
    colorbar = fig.colorbar(scatter, ax=ax)
    colorbar.set_label("Maximum achievable terminal power at full SOC (kW)")
    fig.tight_layout()
    fig.savefig(
        plot_dir / "pack_aware_sprint_point_loss_vs_pack_mass.png", dpi=180
    )
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9.8, 6.2))
    eligible = summary["cohort_efficiency_eligible"]
    scatter = ax.scatter(
        summary.loc[eligible, "terminal_energy_kwh"],
        summary.loc[eligible, "raw_endurance_time_s"],
        c=summary.loc[eligible, "cohort_efficiency_points"],
        cmap="plasma",
        vmin=0.0,
        vmax=100.0,
        s=38,
        alpha=0.82,
        label="Eligible",
    )
    ax.scatter(
        summary.loc[~eligible, "terminal_energy_kwh"],
        summary.loc[~eligible, "raw_endurance_time_s"],
        marker="x",
        s=80,
        linewidth=2.0,
        color="#D55E00",
        label="Ineligible (0 points)",
    )
    ax.axvline(
        EV_EFFICIENCY_ENERGY_LIMIT_KWH,
        color="black",
        linestyle="--",
        linewidth=1.3,
        label="6.776 kWh energy limit",
    )
    ax.set(
        xlabel="22 km terminal energy (kWh)",
        ylabel="22 km endurance time (s)",
        title="Strict EV efficiency eligibility and cohort score",
    )
    ax.grid(alpha=0.25)
    ax.legend()
    colorbar = fig.colorbar(scatter, ax=ax)
    colorbar.set_label("Cohort efficiency points")
    fig.tight_layout()
    fig.savefig(
        plot_dir / "cohort_efficiency_eligibility_and_points.png", dpi=180
    )
    plt.close(fig)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def run_cohort_scoring(
    *,
    source_summary_csv: Path,
    battery_results_csv: Path,
    output_dir: Path,
) -> dict[str, Any]:
    """Generate the complete cohort-normalized scoring artifact set."""

    source_summary_csv = source_summary_csv.resolve()
    battery_results_csv = battery_results_csv.resolve()
    output_dir = output_dir.resolve()
    if not source_summary_csv.is_file():
        raise FileNotFoundError(source_summary_csv)
    if not battery_results_csv.is_file():
        raise FileNotFoundError(battery_results_csv)
    output_dir.mkdir(parents=True, exist_ok=True)

    source = join_terminal_energy(
        pd.read_csv(source_summary_csv), pd.read_csv(battery_results_csv)
    )
    summary, long, anchors = score_cohort(source)
    headlines = _headline_summary(summary, anchors)
    validation = _validation_report(source, summary, long, anchors)

    summary.to_csv(
        output_dir / "battery_cohort_points_summary.csv", index=False
    )
    long.to_csv(
        output_dir / "battery_cohort_event_results_long.csv", index=False
    )
    efficiency_columns = [
        "candidate_id",
        "projected_average_lap_s",
        "projected_terminal_energy_kwh_per_lap",
        "efficiency_factor",
        "efficiency_points",
        "inherited_competition_efficiency_factor",
        "inherited_competition_efficiency_points",
        "pack_aware_cohort_full_dynamic_points_with_inherited_efficiency",
        "pack_aware_cohort_full_dynamic_points_with_inherited_efficiency_rank",
    ]
    summary[efficiency_columns].to_csv(
        output_dir / "inherited_efficiency_scores.csv", index=False
    )
    cohort_efficiency_columns = [
        "candidate_id",
        "raw_endurance_time_s",
        "terminal_energy_kwh",
        "cohort_efficiency_energy_eligible",
        "cohort_efficiency_time_eligible",
        "cohort_efficiency_eligible",
        "cohort_efficiency_factor",
        "cohort_efficiency_unclipped_points",
        "cohort_efficiency_points",
    ]
    summary[cohort_efficiency_columns].to_csv(
        output_dir / "cohort_efficiency_scores.csv", index=False
    )

    scoring_inputs = {
        "study_name": "battery_dynamic_points_cohort_normalized",
        "official_rules_url": OFFICIAL_2026_RULES_URL,
        "source_summary_csv": str(source_summary_csv),
        "source_summary_sha256": _sha256(source_summary_csv),
        "battery_results_csv": str(battery_results_csv),
        "battery_results_sha256": _sha256(battery_results_csv),
        "candidate_count": int(len(summary)),
        "normalization": (
            "For each timed event and sprint mode, Tmin is the minimum raw "
            "simulated time among all accepted configurations."
        ),
        "sprint_modes": SPRINT_MODES,
        "event_rules_and_cohort_anchors": anchors,
        "efficiency": {
            "recommended_internal_design_method": anchors["efficiency"],
            "recommended_reason": (
                "Strict eligibility plus cohort Tmin/Emin/EFmin/EFmax makes "
                "efficiency consistent with the cohort-normalized timed events."
            ),
            "inherited_comparison_method": {
                "policy": (
                    "Preserve the source competition-calibrated efficiency "
                    "score and total unchanged under explicit column names."
                ),
                "maximum_points": 100.0,
            },
        },
        "maximum_points": {
            "timed_event_performance": TIMED_EVENT_MAXIMUM,
            "full_dynamic_including_efficiency": FULL_DYNAMIC_MAXIMUM,
        },
    }
    (output_dir / "cohort_scoring_inputs.json").write_text(
        json.dumps(scoring_inputs, indent=2), encoding="utf-8"
    )
    (output_dir / "headline_summary.json").write_text(
        json.dumps(headlines, indent=2), encoding="utf-8"
    )
    (output_dir / "validation_report.json").write_text(
        json.dumps(validation, indent=2), encoding="utf-8"
    )
    _write_report(output_dir, summary, headlines, source_summary_csv)
    _plot_results(summary, output_dir / "plots")

    manifest = {
        "output_directory": str(output_dir),
        "source_summary_csv": str(source_summary_csv),
        "source_summary_sha256": _sha256(source_summary_csv),
        "battery_results_csv": str(battery_results_csv),
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
        "validation": validation,
        "headlines": headlines,
        "artifact_count": len(manifest["files"]) + 1,
    }
