"""Build one common-field comparison for legacy, 3DOF, and 6DOF CG sweeps.

The three source studies may have scored themselves independently.  This
post-processor deliberately ignores those source scores when making the model
comparison: all eighteen simulated cases are rescored together with the valid
2026 Michigan EV field, so each event has one common ``Tmin``.  Existing legacy
totals are retained separately for historical traceability.

The script only reads completed study roots and writes a new report bundle.  It
does not modify a vehicle definition, GGV, tuning input, or source result.
"""

from __future__ import annotations

import argparse
import json
import math
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from battery_dynamic_points import EVENT_RULES

LAPSIMS_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCORING_REFERENCE = (
    LAPSIMS_ROOT / "inputs" / "fsae_ev_michigan_2026_scoring.json"
)
EXPECTED_EVENTS = (
    "acceleration",
    "skidpad",
    "autocross",
    "michigan_endurance",
)
EXPECTED_CASE_COUNT_PER_MODEL = 6
MODEL_ORDER = ("legacy", "3dof", "6dof")
MODEL_LABELS = {
    "legacy": "Legacy analytical EnvelopeSim",
    "3dof": "Reduced 3DOF",
    "6dof": "Reduced 6DOF",
}
MODEL_COLORS = {
    "legacy": "#6B7280",
    "3dof": "#2563EB",
    "6dof": "#DC2626",
}
EVENT_LABELS = {
    "acceleration": "Acceleration",
    "skidpad": "Skidpad",
    "autocross": "Autocross",
    "michigan_endurance": "Endurance",
}
TUNING_COLUMN_CANDIDATES = {
    "front_antiroll_stiffness_fraction": (
        "effective_front_antiroll_stiffness_fraction",
        "front_antiroll_stiffness_fraction",
        "effective_arb_distribution_front",
        "arb_distribution_front",
    ),
    "front_elastic_roll_stiffness_fraction": (
        "effective_front_roll_stiffness_fraction",
        "front_roll_stiffness_fraction",
        "effective_elastic_roll_stiffness_front",
        "elastic_roll_stiffness_front",
        "effective_lltd",
    ),
    "front_brake_bias": (
        "effective_brake_distribution_front",
        "brake_distribution_front",
        "fixed_front_brake_bias",
    ),
    "legacy_lltd": (
        "effective_lltd",
        "lltd",
    ),
}


@dataclass(frozen=True)
class StudySource:
    """One completed source sweep."""

    model_key: str
    root: Path

    @property
    def event_results_path(self) -> Path:
        return self.root / "event_results.csv"

    @property
    def case_totals_path(self) -> Path:
        return self.root / "case_totals.csv"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Rescore completed legacy/3DOF/6DOF CG sweeps in one common "
            "2026 Michigan real-plus-simulation field."
        )
    )
    parser.add_argument("--three-dof-root", type=Path, required=True)
    parser.add_argument("--six-dof-root", type=Path, required=True)
    parser.add_argument("--legacy-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument(
        "--scoring-reference",
        type=Path,
        default=DEFAULT_SCORING_REFERENCE,
    )
    parser.add_argument("--reference-height-in", type=float, default=11.6)
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


def _required_source_files(
    sources: Sequence[StudySource],
    scoring_reference: Path,
) -> list[Path]:
    return [source.event_results_path for source in sources] + [scoring_reference]


def _check_inputs_exist(
    sources: Sequence[StudySource],
    scoring_reference: Path,
) -> None:
    missing = [
        path.resolve()
        for path in _required_source_files(sources, scoring_reference)
        if not path.is_file()
    ]
    if missing:
        rendered = "\n".join(f"  - {path}" for path in missing)
        raise FileNotFoundError(
            "Comparison inputs are not complete yet. Missing required files:\n"
            + rendered
        )


def _load_scoring_reference(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    events = payload.get("events")
    if not isinstance(events, dict) or set(events) != set(EXPECTED_EVENTS):
        raise ValueError(
            "Scoring reference must contain exactly acceleration, skidpad, "
            "autocross, and michigan_endurance."
        )
    for slug, event in events.items():
        if not isinstance(event, dict):
            raise TypeError(f"Scoring event {slug!r} must be an object.")
        times = np.asarray(event.get("valid_adjusted_times_s"), dtype=float)
        if (
            times.ndim != 1
            or times.size == 0
            or not np.all(np.isfinite(times))
            or np.any(times <= 0.0)
            or np.any(np.diff(times) < 0.0)
        ):
            raise ValueError(f"Scoring event {slug!r} has invalid field times.")
        fastest = float(event.get("fastest_adjusted_time_s", math.nan))
        if fastest != float(times[0]):
            raise ValueError(
                f"Scoring event {slug!r} fastest time does not equal its field minimum."
            )
        rule_slug = str(event.get("rule_slug"))
        if rule_slug not in EVENT_RULES:
            raise ValueError(f"Scoring event {slug!r} has unknown rule {rule_slug!r}.")
        conversion = event.get("simulation_time_conversion")
        if conversion not in {"direct", "distance_scaled"}:
            raise ValueError(
                f"Scoring event {slug!r} has invalid simulation conversion."
            )
    return payload


def _require_columns(frame: pd.DataFrame, columns: Iterable[str], label: str) -> None:
    missing = sorted(set(columns).difference(frame.columns))
    if missing:
        raise ValueError(f"{label} is missing required columns: {missing}")


def _load_source_events(source: StudySource) -> pd.DataFrame:
    frame = pd.read_csv(source.event_results_path)
    _require_columns(
        frame,
        (
            "cg_case",
            "cg_height_in",
            "event_slug",
            "lap_time_s",
            "track_length_m",
        ),
        str(source.event_results_path),
    )
    if frame.empty:
        raise ValueError(f"{source.event_results_path} is empty.")

    normalized = frame.copy()
    normalized.rename(columns={"cg_case": "source_cg_case"}, inplace=True)
    for column in ("cg_height_in", "lap_time_s", "track_length_m"):
        normalized[column] = pd.to_numeric(normalized[column], errors="coerce")
    if "cg_height_m" in normalized:
        normalized["cg_height_m"] = pd.to_numeric(
            normalized["cg_height_m"], errors="coerce"
        )
    else:
        normalized["cg_height_m"] = normalized["cg_height_in"] * 0.0254

    finite_positive = (
        np.isfinite(normalized["cg_height_in"])
        & np.isfinite(normalized["lap_time_s"])
        & np.isfinite(normalized["track_length_m"])
        & (normalized["cg_height_in"] > 0.0)
        & (normalized["lap_time_s"] > 0.0)
        & (normalized["track_length_m"] > 0.0)
    )
    if not bool(finite_positive.all()):
        raise ValueError(
            f"{source.event_results_path} contains invalid height, time, or distance."
        )
    if set(normalized["event_slug"].astype(str)) != set(EXPECTED_EVENTS):
        raise ValueError(
            f"{source.event_results_path} does not contain the expected event set."
        )
    if normalized["source_cg_case"].nunique() != EXPECTED_CASE_COUNT_PER_MODEL:
        raise ValueError(
            f"{source.event_results_path} must contain exactly "
            f"{EXPECTED_CASE_COUNT_PER_MODEL} CG cases."
        )
    if normalized.duplicated(["source_cg_case", "event_slug"]).any():
        raise ValueError(f"{source.event_results_path} has duplicate case/event rows.")
    counts = normalized.groupby("source_cg_case")["event_slug"].nunique()
    if not bool((counts == len(EXPECTED_EVENTS)).all()):
        raise ValueError(
            f"Every case in {source.event_results_path} must contain four events."
        )
    height_counts = normalized.groupby("source_cg_case")["cg_height_in"].nunique()
    if not bool((height_counts == 1).all()):
        raise ValueError(
            f"Each case in {source.event_results_path} must have one CG height."
        )

    # Reduced-QSS source files carry their native solver identity in
    # ``model_key`` (for example, ``dyn_py_3dof_qss``).  The comparison also
    # needs a short, common grouping key (``legacy``/``3dof``/``6dof``).
    # Preserve the native value explicitly before installing the normalized
    # comparison key so provenance is not lost or silently overwritten.
    if "model_key" in normalized:
        if "source_model_key" in normalized:
            raise ValueError(
                f"{source.event_results_path} contains both model_key and "
                "source_model_key; native model identity is ambiguous."
            )
        normalized.rename(columns={"model_key": "source_model_key"}, inplace=True)

    normalized.insert(0, "model_key", source.model_key)
    normalized.insert(1, "model_name", MODEL_LABELS[source.model_key])
    normalized.insert(2, "source_root", str(source.root.resolve()))
    normalized.insert(
        3,
        "cg_case",
        source.model_key + ":" + normalized["source_cg_case"].astype(str),
    )
    normalized["source_projected_points"] = pd.to_numeric(
        normalized.get("projected_points", math.nan), errors="coerce"
    )
    return normalized


def _validate_shared_design(events: pd.DataFrame) -> dict[str, list[float]]:
    heights_by_model: dict[str, list[float]] = {}
    for model_key in MODEL_ORDER:
        heights = sorted(
            events.loc[events["model_key"] == model_key, "cg_height_in"]
            .drop_duplicates()
            .astype(float)
            .tolist()
        )
        heights_by_model[model_key] = heights
    reference = np.asarray(heights_by_model[MODEL_ORDER[0]], dtype=float)
    for model_key in MODEL_ORDER[1:]:
        candidate = np.asarray(heights_by_model[model_key], dtype=float)
        if candidate.shape != reference.shape or not np.allclose(
            candidate, reference, rtol=0.0, atol=1e-9
        ):
            raise ValueError(
                "Legacy, 3DOF, and 6DOF sweeps must use matching CG heights; "
                f"got {heights_by_model}."
            )
    return heights_by_model


def _rescore_common_field(
    events: pd.DataFrame,
    scoring_reference: Mapping[str, Any],
) -> tuple[pd.DataFrame, dict[str, dict[str, Any]]]:
    rescored = events.copy()
    scoring_times: list[float] = []
    time_multipliers: list[float] = []
    for row in rescored.itertuples(index=False):
        reference = scoring_reference["events"][str(row.event_slug)]
        conversion = str(reference["simulation_time_conversion"])
        multiplier = 1.0
        if conversion == "distance_scaled":
            multiplier = float(reference["official_event_distance_m"]) / float(
                row.track_length_m
            )
        time_multipliers.append(multiplier)
        scoring_times.append(float(row.lap_time_s) * multiplier)
    rescored["common_simulation_time_multiplier"] = time_multipliers
    rescored["common_projected_competition_time_s"] = scoring_times
    rescored["common_field_tmin_s"] = np.nan
    rescored["common_field_tmax_s"] = np.nan
    rescored["common_field_tmin_source"] = ""
    rescored["common_is_event_fastest"] = False
    rescored["common_projected_points"] = np.nan
    rescored["common_maximum_points"] = np.nan
    rescored["common_point_loss_vs_max"] = np.nan
    rescored["common_simulated_rank"] = np.nan

    event_summary: dict[str, dict[str, Any]] = {}
    for event_slug in EXPECTED_EVENTS:
        event_mask = rescored["event_slug"] == event_slug
        event_rows = rescored.loc[event_mask]
        simulated_times = event_rows["common_projected_competition_time_s"].to_numpy(
            dtype=float
        )
        simulated_fastest = float(np.min(simulated_times))
        reference = scoring_reference["events"][event_slug]
        real_fastest = float(reference["fastest_adjusted_time_s"])
        common_tmin = min(real_fastest, simulated_fastest)
        winning_ids = sorted(
            event_rows.loc[
                event_rows["common_projected_competition_time_s"] == simulated_fastest,
                "cg_case",
            ].astype(str)
        )
        if simulated_fastest < real_fastest:
            source = "simulation:" + ",".join(winning_ids)
        elif real_fastest < simulated_fastest:
            source = "2026_real_field"
        else:
            source = "2026_real_field_and_simulation:" + ",".join(winning_ids)

        rule_slug = str(reference["rule_slug"])
        base_rule = EVENT_RULES[rule_slug]
        common_rule = replace(base_rule, tmin_s=common_tmin)
        common_points: list[float] = []
        exact_fastest: list[bool] = []
        for scoring_time in simulated_times:
            is_fastest = bool(scoring_time == common_tmin)
            exact_fastest.append(is_fastest)
            if is_fastest:
                score = base_rule.maximum_points
            else:
                score = min(
                    common_rule.score(float(scoring_time)),
                    float(np.nextafter(base_rule.maximum_points, -math.inf)),
                )
            common_points.append(score)
        rescored.loc[event_mask, "common_field_tmin_s"] = common_tmin
        rescored.loc[event_mask, "common_field_tmax_s"] = common_rule.tmax_s
        rescored.loc[event_mask, "common_field_tmin_source"] = source
        rescored.loc[event_mask, "common_is_event_fastest"] = exact_fastest
        rescored.loc[event_mask, "common_projected_points"] = common_points
        rescored.loc[event_mask, "common_maximum_points"] = base_rule.maximum_points
        rescored.loc[event_mask, "common_point_loss_vs_max"] = (
            base_rule.maximum_points - np.asarray(common_points)
        )
        rescored.loc[event_mask, "common_simulated_rank"] = (
            event_rows["common_projected_competition_time_s"]
            .rank(method="min", ascending=True)
            .to_numpy(dtype=float)
        )
        event_summary[event_slug] = {
            "real_field_fastest_time_s": real_fastest,
            "simulated_fastest_time_s": simulated_fastest,
            "common_field_tmin_s": common_tmin,
            "common_field_tmax_s": common_rule.tmax_s,
            "common_field_tmin_source": source,
            "exact_fastest_simulation_case_ids": winning_ids,
            "simulated_maximum_score_count": int(sum(exact_fastest)),
            "real_field_valid_result_count": len(reference["valid_adjusted_times_s"]),
        }
    return rescored, event_summary


def _truthy_all(values: pd.Series) -> bool:
    if values.empty:
        return True
    return bool(
        values.map(
            lambda value: (
                value
                if isinstance(value, (bool, np.bool_))
                else str(value).strip().lower() in {"true", "1", "yes"}
            )
        ).all()
    )


def _constant_numeric(
    group: pd.DataFrame,
    candidates: Sequence[str],
) -> float:
    for column in candidates:
        if column not in group:
            continue
        values = pd.to_numeric(group[column], errors="coerce").dropna()
        if not values.empty:
            reference = float(values.iloc[0])
            if not np.allclose(
                values.to_numpy(dtype=float),
                reference,
                rtol=1e-10,
                atol=1e-12,
            ):
                case = str(group["cg_case"].iloc[0])
                raise ValueError(
                    f"Tuning column {column!r} is not constant within {case}."
                )
            return reference
    return math.nan


def _numeric_sum(group: pd.DataFrame, column: str) -> float:
    """Return a numeric column sum, treating an absent column as zero."""

    if column not in group:
        return 0.0
    return float(pd.to_numeric(group[column], errors="coerce").fillna(0).sum())


def _load_published_legacy_totals(source: StudySource) -> pd.DataFrame:
    if not source.case_totals_path.is_file():
        return pd.DataFrame(
            columns=(
                "source_cg_case",
                "legacy_published_timed_event_points",
            )
        )
    totals = pd.read_csv(source.case_totals_path)
    _require_columns(
        totals,
        ("cg_case", "projected_timed_event_points"),
        str(source.case_totals_path),
    )
    result = totals[["cg_case", "projected_timed_event_points"]].copy()
    result.rename(
        columns={
            "cg_case": "source_cg_case",
            "projected_timed_event_points": ("legacy_published_timed_event_points"),
        },
        inplace=True,
    )
    result["legacy_published_timed_event_points"] = pd.to_numeric(
        result["legacy_published_timed_event_points"], errors="coerce"
    )
    return result


def _case_totals(
    events: pd.DataFrame,
    published_legacy: pd.DataFrame,
) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for cg_case, group in events.groupby("cg_case", sort=False):
        row: dict[str, Any] = {
            "model_key": str(group["model_key"].iloc[0]),
            "model_name": str(group["model_name"].iloc[0]),
            "cg_case": str(cg_case),
            "source_cg_case": str(group["source_cg_case"].iloc[0]),
            "cg_height_in": float(group["cg_height_in"].iloc[0]),
            "cg_height_m": float(group["cg_height_m"].iloc[0]),
            "common_field_timed_event_points": float(
                group["common_projected_points"].sum()
            ),
            "maximum_timed_event_points": float(group["common_maximum_points"].sum()),
            "common_field_point_loss": float(group["common_point_loss_vs_max"].sum()),
            "source_field_timed_event_points": float(
                group["source_projected_points"].sum(min_count=1)
            ),
            "all_track_solvers_converged": (
                _truthy_all(group["converged"]) if "converged" in group else True
            ),
            "speed_cap_segments": int(_numeric_sum(group, "ggv_speed_cap_segments")),
        }
        for target, candidates in TUNING_COLUMN_CANDIDATES.items():
            row[target] = _constant_numeric(group, candidates)
        rows.append(row)

    totals = pd.DataFrame(rows)
    totals["legacy_published_timed_event_points"] = math.nan
    if not published_legacy.empty:
        lookup = published_legacy.set_index("source_cg_case")[
            "legacy_published_timed_event_points"
        ]
        legacy_mask = totals["model_key"] == "legacy"
        totals.loc[legacy_mask, "legacy_published_timed_event_points"] = totals.loc[
            legacy_mask, "source_cg_case"
        ].map(lookup)
    return totals.sort_values(["model_key", "cg_height_in"]).reset_index(drop=True)


def _correlation(values: pd.DataFrame) -> dict[str, Any]:
    ordered = values.sort_values("cg_height_in")
    x = ordered["cg_height_in"].to_numpy(dtype=float)
    y = ordered["common_field_timed_event_points"].to_numpy(dtype=float)
    slope, intercept = np.polyfit(x, y, 1)
    fitted = slope * x + intercept
    total_sum_squares = float(np.sum((y - np.mean(y)) ** 2))
    residual_sum_squares = float(np.sum((y - fitted) ** 2))
    r_squared = (
        1.0 - residual_sum_squares / total_sum_squares
        if total_sum_squares > 0.0
        else 1.0
    )
    x_rank = pd.Series(x).rank(method="average").to_numpy(dtype=float)
    y_rank = pd.Series(y).rank(method="average").to_numpy(dtype=float)
    result: dict[str, Any] = {
        "sample_count": int(x.size),
        "cg_height_min_in": float(np.min(x)),
        "cg_height_max_in": float(np.max(x)),
        "points_at_min_height": float(y[0]),
        "points_at_max_height": float(y[-1]),
        "high_minus_low_points": float(y[-1] - y[0]),
        "linear_slope_points_per_in": float(slope),
        "linear_intercept_points": float(intercept),
        "pearson_r": float(np.corrcoef(x, y)[0, 1]),
        "linear_r_squared": float(r_squared),
        "linear_rmse_points": float(np.sqrt(np.mean((y - fitted) ** 2))),
        "spearman_rho": float(np.corrcoef(x_rank, y_rank)[0, 1]),
        "adjacent_slopes_points_per_in": [
            float(delta_y / delta_x) for delta_y, delta_x in zip(np.diff(y), np.diff(x))
        ],
    }
    if x.size >= 3:
        qa, qb, qc = np.polyfit(x, y, 2)
        qfit = qa * x**2 + qb * x + qc
        qresidual = float(np.sum((y - qfit) ** 2))
        result.update(
            {
                "quadratic_a_points_per_in2": float(qa),
                "quadratic_b_points_per_in": float(qb),
                "quadratic_c_points": float(qc),
                "quadratic_r_squared": float(
                    1.0 - qresidual / total_sum_squares
                    if total_sum_squares > 0.0
                    else 1.0
                ),
                "quadratic_vertex_cg_height_in": float(
                    -qb / (2.0 * qa) if abs(qa) > np.finfo(float).eps else math.nan
                ),
            }
        )
    return result


def _correlations_by_model(
    totals: pd.DataFrame,
) -> tuple[pd.DataFrame, dict[str, Any]]:
    details: dict[str, Any] = {}
    rows: list[dict[str, Any]] = []
    for model_key in MODEL_ORDER:
        result = _correlation(totals[totals["model_key"] == model_key])
        result["model_key"] = model_key
        result["model_name"] = MODEL_LABELS[model_key]
        details[model_key] = result
        rows.append(
            {
                key: value
                for key, value in result.items()
                if key != "adjacent_slopes_points_per_in"
            }
        )
    return pd.DataFrame(rows), details


def _model_comparison_by_height(
    events: pd.DataFrame,
    totals: pd.DataFrame,
) -> pd.DataFrame:
    heights = sorted(totals["cg_height_in"].unique())
    rows: list[dict[str, Any]] = []
    for height in heights:
        row: dict[str, Any] = {"cg_height_in": float(height)}
        height_totals = totals[np.isclose(totals["cg_height_in"], height)]
        for model_key in MODEL_ORDER:
            model_total = height_totals[height_totals["model_key"] == model_key]
            row[f"{model_key}_common_field_points"] = float(
                model_total["common_field_timed_event_points"].iloc[0]
            )
            if model_key == "legacy":
                row["legacy_published_points"] = float(
                    model_total["legacy_published_timed_event_points"].iloc[0]
                )
        row["6dof_minus_3dof_points"] = (
            row["6dof_common_field_points"] - row["3dof_common_field_points"]
        )
        row["6dof_minus_3dof_points_pct"] = 100.0 * (
            row["6dof_common_field_points"] / row["3dof_common_field_points"] - 1.0
        )

        height_events = events[np.isclose(events["cg_height_in"], height)]
        for event_slug in EXPECTED_EVENTS:
            event_rows = height_events[height_events["event_slug"] == event_slug]
            for model_key in MODEL_ORDER:
                model_row = event_rows[event_rows["model_key"] == model_key]
                row[f"{event_slug}_{model_key}_time_s"] = float(
                    model_row["lap_time_s"].iloc[0]
                )
            time_3dof = row[f"{event_slug}_3dof_time_s"]
            time_6dof = row[f"{event_slug}_6dof_time_s"]
            legacy_time = row[f"{event_slug}_legacy_time_s"]
            row[f"{event_slug}_6dof_minus_3dof_pct"] = 100.0 * (
                time_6dof / time_3dof - 1.0
            )
            row[f"{event_slug}_3dof_minus_legacy_pct"] = 100.0 * (
                time_3dof / legacy_time - 1.0
            )
            row[f"{event_slug}_6dof_minus_legacy_pct"] = 100.0 * (
                time_6dof / legacy_time - 1.0
            )
        rows.append(row)
    return pd.DataFrame(rows)


def _plot_points_overlay(totals: pd.DataFrame, output_path: Path) -> None:
    fig, axis = plt.subplots(figsize=(9.0, 5.8))
    markers = {"legacy": "^", "3dof": "o", "6dof": "s"}
    for model_key in MODEL_ORDER:
        group = totals[totals["model_key"] == model_key].sort_values("cg_height_in")
        axis.plot(
            group["cg_height_in"],
            group["common_field_timed_event_points"],
            marker=markers[model_key],
            linewidth=2.2,
            linestyle="--" if model_key == "legacy" else "-",
            color=MODEL_COLORS[model_key],
            label=f"{MODEL_LABELS[model_key]} (common-field score)",
        )
    legacy = totals[totals["model_key"] == "legacy"].sort_values("cg_height_in")
    if legacy["legacy_published_timed_event_points"].notna().all():
        axis.plot(
            legacy["cg_height_in"],
            legacy["legacy_published_timed_event_points"],
            marker="x",
            linewidth=1.5,
            linestyle=":",
            color="#111827",
            label="Legacy originally published score",
        )
    axis.set_xlabel("CG height (in)")
    axis.set_ylabel("Timed-event points / 575")
    axis.set_title("CG height vs points: one common 2026 real + simulated field")
    axis.grid(True, linestyle="--", alpha=0.35)
    axis.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def _plot_event_time_percent_change(
    events: pd.DataFrame,
    output_path: Path,
    reference_height_in: float,
) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(12.2, 8.0), sharex=True)
    for axis, event_slug in zip(axes.flat, EXPECTED_EVENTS):
        for model_key in MODEL_ORDER:
            group = events[
                (events["model_key"] == model_key)
                & (events["event_slug"] == event_slug)
            ].sort_values("cg_height_in")
            heights = group["cg_height_in"].to_numpy(dtype=float)
            times = group["lap_time_s"].to_numpy(dtype=float)
            reference_index = int(np.argmin(np.abs(heights - reference_height_in)))
            percent = 100.0 * (times / times[reference_index] - 1.0)
            axis.plot(
                heights,
                percent,
                marker="o",
                linewidth=2.0,
                linestyle="--" if model_key == "legacy" else "-",
                color=MODEL_COLORS[model_key],
                label=MODEL_LABELS[model_key],
            )
        axis.axhline(0.0, color="#111827", linewidth=0.8)
        axis.set_title(EVENT_LABELS[event_slug])
        axis.set_ylabel(f"Time change from {reference_height_in:.1f} in (%)")
        axis.grid(True, linestyle="--", alpha=0.3)
    for axis in axes[-1, :]:
        axis.set_xlabel("CG height (in)")
    axes[0, 0].legend(fontsize=8)
    fig.suptitle("Raw event-time sensitivity within each model", fontweight="bold")
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def _plot_dof_delta(comparison: pd.DataFrame, output_path: Path) -> None:
    fig, axis = plt.subplots(figsize=(9.0, 5.8))
    colors = ("#7C3AED", "#0891B2", "#D97706", "#16A34A")
    for event_slug, color in zip(EXPECTED_EVENTS, colors):
        axis.plot(
            comparison["cg_height_in"],
            comparison[f"{event_slug}_6dof_minus_3dof_pct"],
            marker="o",
            linewidth=2.0,
            color=color,
            label=EVENT_LABELS[event_slug],
        )
    axis.axhline(0.0, color="#111827", linewidth=1.0)
    axis.set_xlabel("CG height (in)")
    axis.set_ylabel("6DOF minus 3DOF time (%)")
    axis.set_title("Separately retuned design outcome: 6DOF vs 3DOF")
    axis.grid(True, linestyle="--", alpha=0.35)
    axis.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def _plot_tuning_setup(totals: pd.DataFrame, output_path: Path) -> bool:
    reduced = totals[totals["model_key"].isin(("3dof", "6dof"))]
    available = [
        column
        for column in (
            "front_antiroll_stiffness_fraction",
            "front_elastic_roll_stiffness_fraction",
            "front_brake_bias",
        )
        if reduced[column].notna().any()
    ]
    if not available:
        return False
    fig, axes = plt.subplots(
        len(available),
        1,
        figsize=(9.0, 3.2 * len(available)),
        sharex=True,
        squeeze=False,
    )
    labels = {
        "front_antiroll_stiffness_fraction": "Front ARB fraction (%)",
        "front_elastic_roll_stiffness_fraction": (
            "Front elastic roll-stiffness fraction (%)"
        ),
        "front_brake_bias": "Fixed front brake bias (%)",
    }
    for axis, column in zip(axes[:, 0], available):
        for model_key in ("3dof", "6dof"):
            group = reduced[reduced["model_key"] == model_key].sort_values(
                "cg_height_in"
            )
            axis.plot(
                group["cg_height_in"],
                100.0 * group[column],
                marker="o",
                linewidth=2.0,
                color=MODEL_COLORS[model_key],
                label=MODEL_LABELS[model_key],
            )
        axis.set_ylabel(labels[column])
        axis.grid(True, linestyle="--", alpha=0.35)
    axes[0, 0].legend(fontsize=8)
    axes[-1, 0].set_xlabel("CG height (in)")
    fig.suptitle("Per-model, per-height tuned setup", fontweight="bold")
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
    fig.savefig(output_path, dpi=200)
    plt.close(fig)
    return True


def _markdown_table(headers: Sequence[str], rows: Sequence[Sequence[object]]) -> str:
    def render(value: object) -> str:
        if isinstance(value, float):
            return "" if not math.isfinite(value) else f"{value:.4f}"
        return str(value)

    lines = [
        "| " + " | ".join(headers) + " |",
        "|" + "|".join("---" for _ in headers) + "|",
    ]
    lines.extend(
        "| " + " | ".join(render(value) for value in row) + " |" for row in rows
    )
    return "\n".join(lines)


def _write_study_report(
    path: Path,
    *,
    sources: Sequence[StudySource],
    totals: pd.DataFrame,
    correlations: pd.DataFrame,
    comparison: pd.DataFrame,
    event_scoring: Mapping[str, Mapping[str, Any]],
    tuning_plot_written: bool,
    reference_height_in: float,
) -> None:
    correlation_rows = [
        (
            row.model_name,
            row.points_at_min_height,
            row.points_at_max_height,
            row.high_minus_low_points,
            row.linear_slope_points_per_in,
            row.pearson_r,
            row.linear_r_squared,
        )
        for row in correlations.itertuples(index=False)
    ]
    total_rows = [
        (
            row["cg_height_in"],
            row["legacy_common_field_points"],
            row["legacy_published_points"],
            row["3dof_common_field_points"],
            row["6dof_common_field_points"],
            row["6dof_minus_3dof_points"],
        )
        for row in comparison.to_dict(orient="records")
    ]
    scoring_rows = [
        (
            EVENT_LABELS[event_slug],
            values["real_field_fastest_time_s"],
            values["simulated_fastest_time_s"],
            values["common_field_tmin_s"],
            values["common_field_tmin_source"],
        )
        for event_slug, values in event_scoring.items()
    ]
    legacy_rows = [
        (
            row.cg_height_in,
            row.legacy_published_timed_event_points,
            row.common_field_timed_event_points,
            row.common_field_timed_event_points
            - row.legacy_published_timed_event_points,
        )
        for row in totals[totals["model_key"] == "legacy"].itertuples(index=False)
    ]
    source_rows = [
        (MODEL_LABELS[source.model_key], str(source.root.resolve()))
        for source in sources
    ]
    raw_time_rows: list[tuple[object, ...]] = []
    for row in comparison.to_dict(orient="records"):
        for model_key in MODEL_ORDER:
            raw_time_rows.append(
                (
                    row["cg_height_in"],
                    MODEL_LABELS[model_key],
                    row[f"acceleration_{model_key}_time_s"],
                    row[f"skidpad_{model_key}_time_s"],
                    row[f"autocross_{model_key}_time_s"],
                    row[f"michigan_endurance_{model_key}_time_s"],
                    row[f"{model_key}_common_field_points"],
                )
            )

    ordered_comparison = comparison.sort_values("cg_height_in")
    low_height_row = ordered_comparison.iloc[0]
    high_height_row = ordered_comparison.iloc[-1]
    time_trend_rows: list[tuple[object, ...]] = []
    for model_key in MODEL_ORDER:
        for event_slug in EXPECTED_EVENTS:
            low_time = float(low_height_row[f"{event_slug}_{model_key}_time_s"])
            high_time = float(high_height_row[f"{event_slug}_{model_key}_time_s"])
            time_trend_rows.append(
                (
                    MODEL_LABELS[model_key],
                    EVENT_LABELS[event_slug],
                    low_time,
                    high_time,
                    high_time - low_time,
                    100.0 * (high_time / low_time - 1.0),
                )
            )

    reduced_totals = totals[totals["model_key"].isin(("3dof", "6dof"))]
    tuning_columns = (
        "front_antiroll_stiffness_fraction",
        "front_elastic_roll_stiffness_fraction",
        "front_brake_bias",
    )
    tuning_values_available = any(
        reduced_totals[column].notna().any() for column in tuning_columns
    )
    tuning_rows: list[tuple[object, ...]] = []
    for model_key in ("3dof", "6dof"):
        group = reduced_totals[reduced_totals["model_key"] == model_key].sort_values(
            "cg_height_in"
        )
        for row in group.itertuples(index=False):
            values = [float(getattr(row, column)) for column in tuning_columns]
            tuning_rows.append(
                (
                    row.cg_height_in,
                    MODEL_LABELS[model_key],
                    *(100.0 * value for value in values),
                )
            )

    tuning_section = (
        _markdown_table(
            (
                "CG (in)",
                "Model",
                "Front ARB (%)",
                "Front elastic roll stiffness (%)",
                "Fixed front brake bias (%)",
            ),
            tuning_rows,
        )
        if tuning_values_available
        else "No recognized tuning fields were present in the reduced-model event rows."
    )

    report = f"""# Raw-tire reduced-DOF CG-height comparison

## Outcome

All 18 simulated configurations were rescored in one combined field with the
valid 2026 Michigan EV results. Each event therefore has one common fastest
time; only simulations with a numerically identical fastest time receive the
event maximum.

The 3DOF-versus-6DOF comparison is a **separately retuned design-outcome
comparison**. It does not isolate the added DOFs at one identical physical
setup. The legacy comparison is **confounded by both the tire-mu change and the
model-backend change**; use it to compare trend shape and historical context,
not to attribute an absolute time delta to DOF alone.

The common-field point values are cohort-dependent: changing any real or
simulated fastest time can rescore every configuration in that event. Raw
event times and within-model time trends do not have that dependency.

## Model validity / interpretation

- The reduced 3DOF and 6DOF sweeps use the unscaled raw TIR (`mu = 1.0`),
  whereas the legacy analytical sweep used `fitted_tire_mu_scale =
  0.622543713`. Absolute legacy-to-reduced differences therefore combine the
  tire-mu change with the model-backend change.
- Both reduced sweeps use 50/50 front/rear aero balance, no LSD model, and a
  fixed front brake bias plus front/rear ARB allocation retuned independently
  for every CG-height case. The 3DOF-versus-6DOF result is consequently a
  comparison of separately optimized configurations, not a same-setup DOF
  isolation.
- The pure-lateral tuning optimum is constraint-active: the 8.0, 9.2, and
  10.4 in cases reach the +/-0.25 rad sideslip guard, while the 11.6, 12.8,
  and 14.0 in cases converge near the wheel-lift boundary. These points should
  be treated as screening estimates rather than unconstrained interior optima.
- Reduced-QSS load audits found no negative sampled tire normal load. However,
  many sampled higher-CG states fall below the TIR's 100 N minimum and/or
  above its 1800 N maximum, so tire-load extrapolation contributes to the
  high-CG sensitivity.
- The production GGV grid uses 1 m/s speed spacing and approximately 0.1 g
  lateral-acceleration spacing; no fine-grid rerun was performed. The populated
  top speed slice is 42.0 m/s. The requested 42.053998 m/s endpoint is empty
  because it lies just above that slice, but all event traces remained within
  the populated domain and reported zero speed-cap segments.

## Points correlation

{
        _markdown_table(
            (
                "Model",
                "Points @ low",
                "Points @ high",
                "High-low",
                "Slope (pt/in)",
                "r",
                "R2",
            ),
            correlation_rows,
        )
    }

## Common-field totals by CG height

{
        _markdown_table(
            (
                "CG (in)",
                "Legacy common",
                "Legacy published",
                "3DOF",
                "6DOF",
                "6-3 (pt)",
            ),
            total_rows,
        )
    }

## Raw simulated event times by CG height

{
        _markdown_table(
            (
                "CG (in)",
                "Model",
                "Acceleration (s)",
                "Skidpad (s)",
                "Autocross (s)",
                "Endurance modeled lap (s)",
                "Common points",
            ),
            raw_time_rows,
        )
    }

These are unscaled solver times. In particular, the endurance column is the
modeled-track lap time, not the 22 km projected competition time used for
scoring.

## High-to-low raw event-time change

{
        _markdown_table(
            (
                "Model",
                "Event",
                "Time @ low (s)",
                "Time @ high (s)",
                "High-low (s)",
                "High-low (%)",
            ),
            time_trend_rows,
        )
    }

A positive high-minus-low time means the 14 in CG case is slower than the 8 in
case; a negative value means it is faster.

## Per-height tuned setup

{tuning_section}

Values are front-allocation percentages extracted from the source event rows.
Each reported value is required to be constant across all four event rows for
that configuration. The brake entry is the fixed per-configuration bias, not
an ideal speed- or deceleration-varying schedule.

## Common event scoring field

{
        _markdown_table(
            ("Event", "2026 real Tmin", "Sim Tmin", "Common Tmin", "Source"),
            scoring_rows,
        )
    }

Endurance simulation times are scaled from the modeled lap distance to the
official 22 km event distance before scoring. Acceleration, skidpad, and
autocross are scored from their raw simulated times.

## Legacy published versus common-field score

{
        _markdown_table(
            ("CG (in)", "Published", "Common-field rescored", "Difference"),
            legacy_rows,
        )
    }

The published legacy values remain unchanged in the table above. Their
common-field counterparts use the faster of the real field and all 18
simulated cases as each event's Tmin, so the two columns answer different
questions.

## Inputs

{_markdown_table(("Model", "Completed output root"), source_rows)}

Reference height for within-model event-time plots: {reference_height_in:.1f} in.

## Report bundle

- `combined_event_results.csv`: all source event rows with prefixed case IDs and common scoring.
- `combined_case_totals.csv`: one common-field total per model and CG case.
- `correlations_by_model.csv` and `.json`: linear and descriptive correlation metrics.
- `model_comparison_by_height.csv`: points and raw event-time comparisons at each height.
- `cg_points_overlay.png`: common-field points, plus the original legacy score.
- `event_time_percent_change.png`: raw time change relative to the reference height within each model.
- `dof6_minus_3dof_delta.png`: raw 6DOF-minus-3DOF event-time differences.
- `tuning_setup.png`: {
        "written"
        if tuning_plot_written
        else "omitted because no recognized tuning fields were available"
    }.
- `validation_summary.json`: input, solver, scoring, and bundle checks.
"""
    path.write_text(report, encoding="utf-8")


def build_report_bundle(
    *,
    three_dof_root: Path,
    six_dof_root: Path,
    legacy_root: Path,
    output_root: Path,
    scoring_reference_path: Path = DEFAULT_SCORING_REFERENCE,
    reference_height_in: float = 11.6,
) -> dict[str, Any]:
    """Build the common-field report bundle from three completed sweep roots."""

    sources = (
        StudySource("legacy", legacy_root.resolve()),
        StudySource("3dof", three_dof_root.resolve()),
        StudySource("6dof", six_dof_root.resolve()),
    )
    scoring_reference_path = scoring_reference_path.resolve()
    _check_inputs_exist(sources, scoring_reference_path)
    scoring_reference = _load_scoring_reference(scoring_reference_path)

    source_frames = [_load_source_events(source) for source in sources]
    events = pd.concat(source_frames, ignore_index=True, sort=False)
    heights_by_model = _validate_shared_design(events)
    rescored, event_scoring = _rescore_common_field(events, scoring_reference)
    published_legacy = _load_published_legacy_totals(sources[0])
    totals = _case_totals(rescored, published_legacy)
    correlations, correlation_details = _correlations_by_model(totals)
    comparison = _model_comparison_by_height(rescored, totals)

    awarded_maximum = rescored["common_projected_points"].to_numpy(
        dtype=float
    ) == rescored["common_maximum_points"].to_numpy(dtype=float)
    exact_common_fastest = rescored["common_projected_competition_time_s"].to_numpy(
        dtype=float
    ) == rescored["common_field_tmin_s"].to_numpy(dtype=float)
    if not np.array_equal(awarded_maximum, exact_common_fastest):
        raise RuntimeError(
            "Common scoring awarded maximum points to a non-fastest simulation."
        )

    output_root = output_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    event_results_path = output_root / "combined_event_results.csv"
    case_totals_path = output_root / "combined_case_totals.csv"
    correlation_csv_path = output_root / "correlations_by_model.csv"
    correlation_json_path = output_root / "correlations_by_model.json"
    comparison_path = output_root / "model_comparison_by_height.csv"
    points_plot_path = output_root / "cg_points_overlay.png"
    time_plot_path = output_root / "event_time_percent_change.png"
    delta_plot_path = output_root / "dof6_minus_3dof_delta.png"
    tuning_plot_path = output_root / "tuning_setup.png"
    validation_path = output_root / "validation_summary.json"
    report_path = output_root / "study_report.md"

    rescored.to_csv(event_results_path, index=False)
    totals.to_csv(case_totals_path, index=False)
    correlations.to_csv(correlation_csv_path, index=False)
    _write_json(correlation_json_path, correlation_details)
    comparison.to_csv(comparison_path, index=False)
    _plot_points_overlay(totals, points_plot_path)
    _plot_event_time_percent_change(
        rescored,
        time_plot_path,
        reference_height_in=reference_height_in,
    )
    _plot_dof_delta(comparison, delta_plot_path)
    tuning_plot_written = _plot_tuning_setup(totals, tuning_plot_path)

    point_sum_residual = float(
        np.max(
            np.abs(
                totals.set_index("cg_case")["common_field_timed_event_points"]
                - rescored.groupby("cg_case")["common_projected_points"].sum()
            )
        )
    )
    exact_fastest_counts = {
        slug: int(
            rescored.loc[
                rescored["event_slug"] == slug,
                "common_is_event_fastest",
            ].sum()
        )
        for slug in EXPECTED_EVENTS
    }
    solver_convergence = {
        model_key: (_truthy_all(group["converged"]) if "converged" in group else True)
        for model_key, group in rescored.groupby("model_key")
    }
    speed_caps = {
        model_key: int(_numeric_sum(group, "ggv_speed_cap_segments"))
        for model_key, group in rescored.groupby("model_key")
    }
    validation: dict[str, Any] = {
        "status": "passed",
        "comparison_scope": {
            "legacy": "mu-plus-backend confounded historical comparison",
            "3dof_vs_6dof": "separately retuned design outcomes",
        },
        "sources": {
            source.model_key: {
                "root": str(source.root),
                "event_results_csv": str(source.event_results_path),
                "event_row_count": len(source_frames[index]),
                "case_count": int(source_frames[index]["cg_case"].nunique()),
            }
            for index, source in enumerate(sources)
        },
        "scoring_reference": str(scoring_reference_path),
        "combined_event_row_count": len(rescored),
        "combined_case_count": int(totals["cg_case"].nunique()),
        "heights_by_model_in": heights_by_model,
        "matching_height_grids": True,
        "event_scoring": event_scoring,
        "exact_fastest_simulation_count_by_event": exact_fastest_counts,
        "only_numerically_identical_fastest_times_share_maximum": True,
        "maximum_score_award_rule_passed": True,
        "maximum_case_point_sum_residual": point_sum_residual,
        "all_track_solvers_converged_by_model": solver_convergence,
        "speed_cap_segments_by_model": speed_caps,
        "legacy_published_total_count": int(
            totals["legacy_published_timed_event_points"].notna().sum()
        ),
        "tuning_plot_written": tuning_plot_written,
        "artifacts": {
            "combined_event_results_csv": str(event_results_path),
            "combined_case_totals_csv": str(case_totals_path),
            "correlations_csv": str(correlation_csv_path),
            "correlations_json": str(correlation_json_path),
            "model_comparison_csv": str(comparison_path),
            "points_plot": str(points_plot_path),
            "event_time_plot": str(time_plot_path),
            "dof_delta_plot": str(delta_plot_path),
            "tuning_plot": str(tuning_plot_path) if tuning_plot_written else None,
            "study_report": str(report_path),
        },
    }
    _write_json(validation_path, validation)
    _write_study_report(
        report_path,
        sources=sources,
        totals=totals,
        correlations=correlations,
        comparison=comparison,
        event_scoring=event_scoring,
        tuning_plot_written=tuning_plot_written,
        reference_height_in=reference_height_in,
    )
    return validation


def main() -> int:
    args = parse_args()
    validation = build_report_bundle(
        three_dof_root=args.three_dof_root,
        six_dof_root=args.six_dof_root,
        legacy_root=args.legacy_root,
        output_root=args.output_root,
        scoring_reference_path=args.scoring_reference,
        reference_height_in=float(args.reference_height_in),
    )
    print(json.dumps(validation, indent=2, default=_json_default))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
