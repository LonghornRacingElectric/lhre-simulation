"""Matched-candidate comparison of versioned no-regen and regen studies.

The comparison deliberately operates on published ``all_configuration_results``
artifacts rather than rerunning either study.  Candidate IDs are the primary
key, topology metadata must remain unchanged, and every reported delta uses a
documented sign convention.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import pandas as pd


@dataclass(frozen=True)
class MetricSpec:
    """Canonical metric definition and accepted source-column aliases."""

    canonical: str
    aliases: tuple[str, ...]
    prior_fallback: str | float | None = None


METRIC_SPECS = (
    MetricSpec(
        "sustainable_power_kw",
        ("power_limit_kw", "sustainable_power_kw"),
    ),
    MetricSpec(
        "endurance_time_s",
        ("elapsed_time_s", "endurance_time_s"),
    ),
    MetricSpec(
        "gross_discharge_kwh",
        (
            "terminal_discharge_energy_kwh",
            "gross_discharge_terminal_energy_kwh",
            "gross_terminal_discharge_energy_kwh",
        ),
        prior_fallback="terminal_energy_kwh",
    ),
    MetricSpec(
        "recovered_energy_kwh",
        (
            "terminal_regenerated_energy_kwh",
            "recovered_terminal_energy_kwh",
            "terminal_recovered_energy_kwh",
        ),
        prior_fallback=0.0,
    ),
    MetricSpec(
        "net_terminal_energy_kwh",
        ("terminal_energy_kwh", "net_terminal_energy_kwh"),
    ),
    MetricSpec(
        "total_pack_heat_kwh",
        ("pack_resistive_heat_kwh", "total_pack_heat_kwh"),
    ),
    MetricSpec(
        "regen_pack_heat_kwh",
        ("regen_pack_resistive_heat_kwh", "regen_pack_heat_kwh"),
        prior_fallback=0.0,
    ),
    MetricSpec(
        "final_reserve_kwh",
        ("remaining_usable_chemical_kwh", "final_reserve_kwh"),
    ),
    MetricSpec(
        "active_rms_kw",
        (
            "regen_active_rms_terminal_power_kw",
            "active_rms_terminal_power_kw",
            "regen_active_rms_power_kw",
        ),
        prior_fallback=0.0,
    ),
    MetricSpec("overall_rank", ("overall_rank",)),
    MetricSpec("cell_rank", ("cell_rank",)),
)

IDENTITY_COLUMNS = (
    "cell_id",
    "manufacturer",
    "cell_model",
)

INTEGER_TOPOLOGY_COLUMNS = (
    "series_cells",
    "parallel_cells",
    "total_cells",
)

FLOAT_TOPOLOGY_COLUMNS = (
    "pack_mass_kg",
    "vehicle_mass_kg",
    "nominal_pack_energy_kwh",
    "model_usable_energy_kwh",
)

BENEFIT_COLUMNS = (
    "sustainable_power_gain_kw",
    "endurance_time_reduction_s",
    "gross_discharge_change_kwh",
    "recovered_energy_gain_kwh",
    "net_terminal_energy_reduction_kwh",
    "total_pack_heat_change_kwh",
    "regen_pack_heat_increase_kwh",
    "final_reserve_change_kwh",
    "active_rms_change_kw",
    "overall_rank_improvement",
    "cell_rank_improvement",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _json_default(value: Any) -> Any:
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return None if not math.isfinite(float(value)) else float(value)
    if isinstance(value, np.bool_):
        return bool(value)
    if isinstance(value, Path):
        return str(value)
    raise TypeError(f"Object of type {type(value).__name__} is not serializable")


def _resolve_metric(
    frame: pd.DataFrame,
    spec: MetricSpec,
    role: str,
) -> tuple[pd.Series, dict[str, Any]]:
    for alias in spec.aliases:
        if alias in frame.columns:
            return (
                pd.to_numeric(frame[alias], errors="coerce"),
                {"source_column": alias, "fallback": False},
            )

    if role == "prior" and spec.prior_fallback is not None:
        if isinstance(spec.prior_fallback, str):
            fallback_column = spec.prior_fallback
            if fallback_column not in frame.columns:
                raise ValueError(
                    f"Prior results cannot resolve {spec.canonical!r}: "
                    f"fallback column {fallback_column!r} is absent"
                )
            return (
                pd.to_numeric(frame[fallback_column], errors="coerce"),
                {
                    "source_column": fallback_column,
                    "fallback": True,
                    "reason": (
                        "No-regen gross discharge equals net terminal energy"
                    ),
                },
            )
        return (
            pd.Series(
                float(spec.prior_fallback),
                index=frame.index,
                dtype=float,
            ),
            {
                "source_column": None,
                "fallback": True,
                "constant": float(spec.prior_fallback),
                "reason": "No-regen source has no regenerative contribution",
            },
        )

    accepted = ", ".join(spec.aliases)
    raise ValueError(
        f"{role.capitalize()} results are missing {spec.canonical!r}; "
        f"accepted columns: {accepted}"
    )


def _validate_source_frame(frame: pd.DataFrame, role: str) -> None:
    if "candidate_id" not in frame.columns:
        raise ValueError(f"{role.capitalize()} results lack candidate_id")
    candidate_ids = frame["candidate_id"].astype(str)
    if candidate_ids.str.strip().eq("").any():
        raise ValueError(f"{role.capitalize()} results contain blank candidate_id")
    duplicates = candidate_ids[candidate_ids.duplicated()].unique().tolist()
    if duplicates:
        raise ValueError(
            f"{role.capitalize()} results contain duplicate candidate_id values: "
            + ", ".join(sorted(duplicates))
        )

    required_metadata = {
        *IDENTITY_COLUMNS,
        *INTEGER_TOPOLOGY_COLUMNS,
        *FLOAT_TOPOLOGY_COLUMNS,
    }
    missing = sorted(required_metadata.difference(frame.columns))
    if missing:
        raise ValueError(
            f"{role.capitalize()} results are missing metadata columns: "
            + ", ".join(missing)
        )


def _topology_mismatches(
    prior: pd.DataFrame,
    regen: pd.DataFrame,
) -> list[dict[str, Any]]:
    mismatches: list[dict[str, Any]] = []
    for column in (*IDENTITY_COLUMNS, *INTEGER_TOPOLOGY_COLUMNS):
        lhs = prior[column]
        rhs = regen[column]
        unequal = lhs.astype(str) != rhs.astype(str)
        for candidate_id in prior.index[unequal]:
            mismatches.append(
                {
                    "candidate_id": candidate_id,
                    "column": column,
                    "prior": lhs.loc[candidate_id],
                    "regen": rhs.loc[candidate_id],
                }
            )

    for column in FLOAT_TOPOLOGY_COLUMNS:
        lhs = pd.to_numeric(prior[column], errors="coerce")
        rhs = pd.to_numeric(regen[column], errors="coerce")
        unequal = ~np.isclose(
            lhs.to_numpy(dtype=float),
            rhs.to_numpy(dtype=float),
            rtol=1e-10,
            atol=1e-10,
            equal_nan=False,
        )
        for candidate_id in prior.index[unequal]:
            mismatches.append(
                {
                    "candidate_id": candidate_id,
                    "column": column,
                    "prior": lhs.loc[candidate_id],
                    "regen": rhs.loc[candidate_id],
                }
            )
    return mismatches


def build_comparison(
    prior_results: pd.DataFrame,
    regen_results: pd.DataFrame,
) -> tuple[pd.DataFrame, dict[str, Any]]:
    """Join two result versions and compute canonical metrics and deltas."""

    prior = prior_results.copy()
    regen = regen_results.copy()
    _validate_source_frame(prior, "prior")
    _validate_source_frame(regen, "regen")
    prior["candidate_id"] = prior["candidate_id"].astype(str)
    regen["candidate_id"] = regen["candidate_id"].astype(str)

    prior_ids = set(prior["candidate_id"])
    regen_ids = set(regen["candidate_id"])
    if prior_ids != regen_ids:
        missing_from_regen = sorted(prior_ids - regen_ids)
        new_in_regen = sorted(regen_ids - prior_ids)
        raise ValueError(
            "Prior and regen candidate sets differ; "
            f"missing from regen={missing_from_regen}, "
            f"new in regen={new_in_regen}"
        )

    prior = prior.set_index("candidate_id").sort_index()
    regen = regen.set_index("candidate_id").sort_index()
    mismatches = _topology_mismatches(prior, regen)
    if mismatches:
        preview = "; ".join(
            f"{row['candidate_id']}:{row['column']} "
            f"{row['prior']} != {row['regen']}"
            for row in mismatches[:5]
        )
        raise ValueError(
            "Matched candidates changed topology or identity metadata: " + preview
        )

    comparison = pd.DataFrame(index=prior.index)
    comparison.index.name = "candidate_id"
    for column in (
        *IDENTITY_COLUMNS,
        *INTEGER_TOPOLOGY_COLUMNS,
        *FLOAT_TOPOLOGY_COLUMNS,
    ):
        comparison[column] = prior[column]

    if "simulation_fidelity" in prior.columns:
        comparison["prior_simulation_fidelity"] = prior["simulation_fidelity"]
    if "simulation_fidelity" in regen.columns:
        comparison["regen_simulation_fidelity"] = regen["simulation_fidelity"]
    for column in ("constraint_valid", "equal_reserve_eligible"):
        if column in prior.columns:
            comparison[f"prior_{column}"] = prior[column]
        if column in regen.columns:
            comparison[f"regen_{column}"] = regen[column]

    source_map: dict[str, dict[str, Any]] = {"prior": {}, "regen": {}}
    for spec in METRIC_SPECS:
        prior_values, prior_source = _resolve_metric(prior, spec, "prior")
        regen_values, regen_source = _resolve_metric(regen, spec, "regen")
        if spec.canonical not in {"overall_rank", "cell_rank"}:
            if not np.isfinite(prior_values.to_numpy(dtype=float)).all():
                raise ValueError(
                    f"Prior metric {spec.canonical!r} contains non-finite values"
                )
            if not np.isfinite(regen_values.to_numpy(dtype=float)).all():
                raise ValueError(
                    f"Regen metric {spec.canonical!r} contains non-finite values"
                )
        comparison[f"prior_{spec.canonical}"] = prior_values
        comparison[f"regen_{spec.canonical}"] = regen_values
        comparison[f"delta_{spec.canonical}"] = regen_values - prior_values
        source_map["prior"][spec.canonical] = prior_source
        source_map["regen"][spec.canonical] = regen_source

    comparison["sustainable_power_gain_kw"] = comparison[
        "delta_sustainable_power_kw"
    ]
    comparison["endurance_time_reduction_s"] = -comparison[
        "delta_endurance_time_s"
    ]
    comparison["gross_discharge_change_kwh"] = comparison[
        "delta_gross_discharge_kwh"
    ]
    comparison["recovered_energy_gain_kwh"] = comparison[
        "delta_recovered_energy_kwh"
    ]
    comparison["net_terminal_energy_reduction_kwh"] = -comparison[
        "delta_net_terminal_energy_kwh"
    ]
    comparison["total_pack_heat_change_kwh"] = comparison[
        "delta_total_pack_heat_kwh"
    ]
    comparison["regen_pack_heat_increase_kwh"] = comparison[
        "delta_regen_pack_heat_kwh"
    ]
    comparison["final_reserve_change_kwh"] = comparison[
        "delta_final_reserve_kwh"
    ]
    comparison["active_rms_change_kw"] = comparison["delta_active_rms_kw"]
    comparison["overall_rank_improvement"] = -comparison[
        "delta_overall_rank"
    ]
    comparison["cell_rank_improvement"] = -comparison["delta_cell_rank"]

    comparison = comparison.reset_index()
    provenance = {
        "candidate_count": int(len(comparison)),
        "join_key": "candidate_id",
        "delta_convention": "delta_* = regen - prior",
        "benefit_conventions": {
            "sustainable_power_gain_kw": "regen - prior",
            "endurance_time_reduction_s": "prior - regen",
            "gross_discharge_change_kwh": "regen - prior",
            "recovered_energy_gain_kwh": "regen - prior",
            "net_terminal_energy_reduction_kwh": "prior - regen",
            "total_pack_heat_change_kwh": "regen - prior",
            "regen_pack_heat_increase_kwh": "regen - prior",
            "final_reserve_change_kwh": "regen - prior",
            "active_rms_change_kw": "regen - prior",
            "overall_rank_improvement": "prior - regen",
            "cell_rank_improvement": "prior - regen",
        },
        "metric_sources": source_map,
    }
    return comparison, provenance


def validate_comparison(
    comparison: pd.DataFrame,
    expected_candidate_count: int,
) -> dict[str, Any]:
    """Validate row identity, numeric deltas, and benefit sign conventions."""

    duplicate_count = int(comparison["candidate_id"].duplicated().sum())
    finite_counts = {
        column: int(
            (~np.isfinite(pd.to_numeric(comparison[column], errors="coerce"))).sum()
        )
        for column in BENEFIT_COLUMNS[:-2]
    }
    delta_checks: dict[str, bool] = {}
    for spec in METRIC_SPECS:
        prior = pd.to_numeric(
            comparison[f"prior_{spec.canonical}"], errors="coerce"
        )
        regen = pd.to_numeric(
            comparison[f"regen_{spec.canonical}"], errors="coerce"
        )
        delta = pd.to_numeric(
            comparison[f"delta_{spec.canonical}"], errors="coerce"
        )
        delta_checks[spec.canonical] = bool(
            np.allclose(
                delta.to_numpy(dtype=float),
                (regen - prior).to_numpy(dtype=float),
                rtol=1e-12,
                atol=1e-12,
                equal_nan=True,
            )
        )

    benefit_checks = {
        "sustainable_power_gain": bool(
            np.allclose(
                comparison["sustainable_power_gain_kw"],
                comparison["delta_sustainable_power_kw"],
                equal_nan=True,
            )
        ),
        "endurance_time_reduction": bool(
            np.allclose(
                comparison["endurance_time_reduction_s"],
                -comparison["delta_endurance_time_s"],
                equal_nan=True,
            )
        ),
        "net_terminal_energy_reduction": bool(
            np.allclose(
                comparison["net_terminal_energy_reduction_kwh"],
                -comparison["delta_net_terminal_energy_kwh"],
                equal_nan=True,
            )
        ),
        "rank_improvement": bool(
            np.allclose(
                comparison["overall_rank_improvement"],
                -comparison["delta_overall_rank"],
                equal_nan=True,
            )
        ),
    }
    checks = {
        "candidate_count_matches": len(comparison) == expected_candidate_count,
        "candidate_ids_unique": duplicate_count == 0,
        "all_core_benefit_metrics_finite": all(
            count == 0 for count in finite_counts.values()
        ),
        "all_delta_arithmetic_exact": all(delta_checks.values()),
        "all_benefit_signs_consistent": all(benefit_checks.values()),
    }
    return {
        "all_checks_passed": all(checks.values()),
        "checks": checks,
        "row_count": int(len(comparison)),
        "expected_candidate_count": int(expected_candidate_count),
        "duplicate_candidate_count": duplicate_count,
        "non_finite_counts": finite_counts,
        "delta_checks": delta_checks,
        "benefit_checks": benefit_checks,
    }


def summarize_comparison(comparison: pd.DataFrame) -> dict[str, Any]:
    """Build machine-readable headline statistics and candidate leaders."""

    aggregates: dict[str, dict[str, float]] = {}
    for column in BENEFIT_COLUMNS:
        values = pd.to_numeric(comparison[column], errors="coerce").dropna()
        aggregates[column] = {
            "minimum": float(values.min()) if not values.empty else math.nan,
            "median": float(values.median()) if not values.empty else math.nan,
            "maximum": float(values.max()) if not values.empty else math.nan,
        }

    def leader(column: str, ascending: bool = False) -> dict[str, Any] | None:
        valid = comparison.dropna(subset=[column])
        if valid.empty:
            return None
        row = valid.sort_values(
            [column, "candidate_id"],
            ascending=[ascending, True],
        ).iloc[0]
        return {
            "candidate_id": str(row["candidate_id"]),
            "cell_id": str(row["cell_id"]),
            "value": float(row[column]),
        }

    ranked = comparison.dropna(subset=["regen_overall_rank"]).sort_values(
        ["regen_overall_rank", "candidate_id"]
    )
    return {
        "candidate_count": int(len(comparison)),
        "aggregates": aggregates,
        "leaders": {
            "sustainable_power_gain": leader("sustainable_power_gain_kw"),
            "endurance_time_reduction": leader("endurance_time_reduction_s"),
            "net_terminal_energy_reduction": leader(
                "net_terminal_energy_reduction_kwh"
            ),
            "overall_rank_improvement": leader("overall_rank_improvement"),
        },
        "regen_top_five": [
            {
                "candidate_id": str(row["candidate_id"]),
                "regen_overall_rank": float(row["regen_overall_rank"]),
                "prior_overall_rank": float(row["prior_overall_rank"]),
                "rank_improvement": float(row["overall_rank_improvement"]),
                "sustainable_power_gain_kw": float(
                    row["sustainable_power_gain_kw"]
                ),
                "endurance_time_reduction_s": float(
                    row["endurance_time_reduction_s"]
                ),
            }
            for _, row in ranked.head(5).iterrows()
        ],
    }


def _markdown_table(
    frame: pd.DataFrame,
    columns: Iterable[tuple[str, str, str]],
) -> list[str]:
    definitions = list(columns)
    lines = [
        "| " + " | ".join(label for _, label, _ in definitions) + " |",
        "|" + "|".join("---" for _ in definitions) + "|",
    ]
    for _, row in frame.iterrows():
        values: list[str] = []
        for column, _, formatter in definitions:
            value = row[column]
            if pd.isna(value):
                values.append("n/a")
            elif formatter == "s":
                values.append(str(value))
            else:
                values.append(format(float(value), formatter))
        lines.append("| " + " | ".join(values) + " |")
    return lines


def render_report(
    comparison: pd.DataFrame,
    summary: dict[str, Any],
    provenance: dict[str, Any],
    scenario_name: str,
) -> str:
    """Render a concise but auditable Markdown comparison report."""

    top_ranked = comparison.dropna(subset=["regen_overall_rank"]).sort_values(
        ["regen_overall_rank", "candidate_id"]
    ).head(10)
    top_gain = comparison.sort_values(
        ["sustainable_power_gain_kw", "candidate_id"],
        ascending=[False, True],
    ).head(10)
    agg = summary["aggregates"]
    lines = [
        f"# Battery Regen Comparison: {scenario_name}",
        "",
        "## Scope and conventions",
        "",
        (
            f"This report compares {len(comparison)} matched battery candidates "
            "by `candidate_id`. It reads versioned result artifacts and does not "
            "rerun either endurance study."
        ),
        "",
        "- Every `delta_*` column is `regen - prior`.",
        (
            "- Positive power gain, time reduction, recovered-energy gain, "
            "net-energy reduction, reserve change, and rank improvement are "
            "beneficial by definition."
        ),
        (
            "- Positive gross-discharge, total-heat, regen-heat, and active-RMS "
            "changes indicate increases, not automatically benefits."
        ),
        (
            "- Missing no-regen recovered energy, regen heat, and regen-active "
            "RMS are physically reconstructed as zero; missing no-regen gross "
            "discharge is reconstructed from its net terminal energy."
        ),
        "",
        "## Cohort-level result",
        "",
        (
            "- Sustainable power gain: "
            f"median {agg['sustainable_power_gain_kw']['median']:.3f} kW, "
            f"range {agg['sustainable_power_gain_kw']['minimum']:.3f} to "
            f"{agg['sustainable_power_gain_kw']['maximum']:.3f} kW."
        ),
        (
            "- Endurance time reduction: "
            f"median {agg['endurance_time_reduction_s']['median']:.3f} s, "
            f"range {agg['endurance_time_reduction_s']['minimum']:.3f} to "
            f"{agg['endurance_time_reduction_s']['maximum']:.3f} s."
        ),
        (
            "- Recovered terminal energy gain: "
            f"median {agg['recovered_energy_gain_kwh']['median']:.4f} kWh."
        ),
        (
            "- Net terminal energy reduction: "
            f"median {agg['net_terminal_energy_reduction_kwh']['median']:.4f} "
            "kWh."
        ),
        (
            "- Total pack heat change: "
            f"median {agg['total_pack_heat_change_kwh']['median']:.4f} kWh; "
            "positive means more heat."
        ),
        (
            "- Final reserve change: "
            f"median {agg['final_reserve_change_kwh']['median']:.4f} kWh."
        ),
        "",
        "## Regen ranking",
        "",
        *_markdown_table(
            top_ranked,
            (
                ("regen_overall_rank", "Regen rank", ".0f"),
                ("candidate_id", "Candidate", "s"),
                ("prior_overall_rank", "Prior rank", ".0f"),
                ("overall_rank_improvement", "Rank improvement", "+.0f"),
                ("sustainable_power_gain_kw", "Power gain (kW)", "+.3f"),
                ("endurance_time_reduction_s", "Time reduction (s)", "+.3f"),
            ),
        ),
        "",
        "## Largest sustainable-power gains",
        "",
        *_markdown_table(
            top_gain,
            (
                ("candidate_id", "Candidate", "s"),
                ("sustainable_power_gain_kw", "Power gain (kW)", "+.3f"),
                ("endurance_time_reduction_s", "Time reduction (s)", "+.3f"),
                (
                    "net_terminal_energy_reduction_kwh",
                    "Net energy reduction (kWh)",
                    "+.4f",
                ),
                (
                    "total_pack_heat_change_kwh",
                    "Pack heat change (kWh)",
                    "+.4f",
                ),
            ),
        ),
        "",
        "## Interpretation cautions",
        "",
        (
            "- Rank changes combine the regen model with the study's existing "
            "equal-reserve constraint and ranking rules; they are not a "
            "standalone cell-selection recommendation."
        ),
        (
            "- Compare `prior_simulation_fidelity` and "
            "`regen_simulation_fidelity` in the CSV if the study contains mixed "
            "native and resampled runs."
        ),
        (
            "- Active RMS is computed only over the model's declared regen-active "
            "window, so it should not be compared with a whole-event RMS value."
        ),
        "",
        "## Metric source resolution",
        "",
        "```json",
        json.dumps(provenance["metric_sources"], indent=2, default=_json_default),
        "```",
        "",
    ]
    return "\n".join(lines)


def write_plots(comparison: pd.DataFrame, plot_dir: Path) -> list[Path]:
    """Create deterministic diagnostic plots for the matched comparison."""

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plot_dir.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []

    def scatter(
        x: str,
        y: str,
        xlabel: str,
        ylabel: str,
        title: str,
        filename: str,
    ) -> None:
        fig, ax = plt.subplots(figsize=(8.5, 5.5))
        ax.scatter(
            comparison[x],
            comparison[y],
            c=comparison["regen_sustainable_power_kw"],
            cmap="viridis",
            edgecolors="black",
            linewidths=0.35,
            alpha=0.85,
        )
        ax.axhline(0.0, color="0.35", linewidth=0.9)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(True, alpha=0.25)
        fig.tight_layout()
        path = plot_dir / filename
        fig.savefig(path, dpi=180)
        plt.close(fig)
        written.append(path)

    scatter(
        "pack_mass_kg",
        "sustainable_power_gain_kw",
        "Pack mass (kg)",
        "Sustainable power gain (kW)",
        "Regen effect on sustainable endurance power",
        "sustainable_power_gain_vs_pack_mass.png",
    )
    scatter(
        "pack_mass_kg",
        "endurance_time_reduction_s",
        "Pack mass (kg)",
        "Endurance time reduction (s)",
        "Regen effect on 22 km endurance time",
        "endurance_time_reduction_vs_pack_mass.png",
    )
    scatter(
        "recovered_energy_gain_kwh",
        "net_terminal_energy_reduction_kwh",
        "Recovered terminal energy (kWh)",
        "Net terminal energy reduction (kWh)",
        "Recovered energy versus net energy reduction",
        "recovered_vs_net_energy_reduction.png",
    )
    scatter(
        "regen_pack_heat_increase_kwh",
        "total_pack_heat_change_kwh",
        "Regen pack heat (kWh)",
        "Total pack heat change (kWh)",
        "Regen-specific heat versus total pack heat change",
        "regen_heat_vs_total_heat_change.png",
    )

    ranked = comparison.dropna(subset=["overall_rank_improvement"]).copy()
    ranked = ranked.reindex(
        ranked["overall_rank_improvement"]
        .abs()
        .sort_values(ascending=False)
        .head(20)
        .index
    ).sort_values("overall_rank_improvement")
    if not ranked.empty:
        fig, ax = plt.subplots(figsize=(9.5, max(4.5, 0.3 * len(ranked) + 1.5)))
        colors = np.where(
            ranked["overall_rank_improvement"] >= 0.0,
            "#2f855a",
            "#c53030",
        )
        ax.barh(
            ranked["candidate_id"],
            ranked["overall_rank_improvement"],
            color=colors,
        )
        ax.axvline(0.0, color="0.25", linewidth=0.9)
        ax.set_xlabel("Overall rank improvement (prior rank - regen rank)")
        ax.set_title("Largest absolute overall-rank changes")
        ax.grid(True, axis="x", alpha=0.25)
        fig.tight_layout()
        path = plot_dir / "largest_rank_changes.png"
        fig.savefig(path, dpi=180)
        plt.close(fig)
        written.append(path)

    return written


def _prepare_output_directory(output_dir: Path, overwrite: bool) -> None:
    if output_dir.exists():
        contents = list(output_dir.iterdir())
        if contents and not overwrite:
            raise FileExistsError(
                f"Output directory is not empty: {output_dir}. "
                "Use --overwrite only when replacement is intentional."
            )
    output_dir.mkdir(parents=True, exist_ok=True)


def run_comparison(
    prior_results_csv: Path,
    regen_results_csv: Path,
    output_dir: Path,
    scenario_name: str,
    *,
    write_plot_files: bool = True,
    overwrite: bool = False,
) -> dict[str, Any]:
    """Run the artifact-only comparison and write a versioned report bundle."""

    prior_results_csv = Path(prior_results_csv).resolve()
    regen_results_csv = Path(regen_results_csv).resolve()
    output_dir = Path(output_dir).resolve()
    for path, label in (
        (prior_results_csv, "Prior"),
        (regen_results_csv, "Regen"),
    ):
        if not path.is_file():
            raise FileNotFoundError(f"{label} result CSV does not exist: {path}")
        if output_dir == path.parent:
            raise ValueError(
                f"Output directory cannot be the {label.lower()} source directory"
            )
    if prior_results_csv == regen_results_csv:
        raise ValueError("Prior and regen result CSVs must be different files")

    _prepare_output_directory(output_dir, overwrite)
    prior = pd.read_csv(prior_results_csv)
    regen = pd.read_csv(regen_results_csv)
    comparison, provenance = build_comparison(prior, regen)
    validation = validate_comparison(comparison, len(prior))
    if not validation["all_checks_passed"]:
        raise RuntimeError(f"Comparison validation failed: {validation}")
    summary = summarize_comparison(comparison)

    input_record = {
        "scenario_name": scenario_name,
        "prior_results_csv": str(prior_results_csv),
        "prior_results_sha256": _sha256(prior_results_csv),
        "regen_results_csv": str(regen_results_csv),
        "regen_results_sha256": _sha256(regen_results_csv),
        **provenance,
    }
    comparison_path = output_dir / "battery_regen_comparison.csv"
    comparison.to_csv(comparison_path, index=False)

    top_rank_changes = comparison.dropna(
        subset=["overall_rank_improvement"]
    ).copy()
    top_rank_changes["_absolute_rank_change"] = top_rank_changes[
        "overall_rank_improvement"
    ].abs()
    top_rank_changes = top_rank_changes.sort_values(
        ["_absolute_rank_change", "candidate_id"],
        ascending=[False, True],
    ).drop(columns="_absolute_rank_change")
    rank_path = output_dir / "top_rank_changes.csv"
    top_rank_changes.to_csv(rank_path, index=False)

    summary_path = output_dir / "headline_summary.json"
    summary_path.write_text(
        json.dumps(summary, indent=2, default=_json_default) + "\n",
        encoding="utf-8",
    )
    validation_path = output_dir / "validation_report.json"
    validation_path.write_text(
        json.dumps(validation, indent=2, default=_json_default) + "\n",
        encoding="utf-8",
    )
    inputs_path = output_dir / "comparison_inputs.json"
    inputs_path.write_text(
        json.dumps(input_record, indent=2, default=_json_default) + "\n",
        encoding="utf-8",
    )
    report_path = output_dir / "battery_regen_comparison_report.md"
    report_path.write_text(
        render_report(comparison, summary, provenance, scenario_name),
        encoding="utf-8",
    )

    plot_paths = (
        write_plots(comparison, output_dir / "plots")
        if write_plot_files
        else []
    )
    artifact_paths = [
        comparison_path,
        rank_path,
        summary_path,
        validation_path,
        inputs_path,
        report_path,
        *plot_paths,
    ]
    manifest = {
        "scenario_name": scenario_name,
        "candidate_count": int(len(comparison)),
        "artifacts": [
            {
                "path": str(path.relative_to(output_dir)).replace("\\", "/"),
                "sha256": _sha256(path),
                "bytes": path.stat().st_size,
            }
            for path in artifact_paths
        ],
    }
    manifest_path = output_dir / "artifact_manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, default=_json_default) + "\n",
        encoding="utf-8",
    )
    return {
        "output_dir": str(output_dir),
        "candidate_count": int(len(comparison)),
        "validation": validation,
        "summary": summary,
        "artifacts": [
            str(path)
            for path in (
                *artifact_paths,
                manifest_path,
            )
        ],
    }
