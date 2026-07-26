"""Battery-configuration dynamic-event scoring study.

The primary result couples each candidate pack to the existing motor, inverter,
and drivetrain at full SOC with an 80 kW battery-terminal power ceiling.  A
second result holds the original 80 kW OpenLAP tractive curve constant and
changes only vehicle mass, making the isolated mass penalty visible.

Endurance and efficiency are inherited from the chronological 22 km battery
trade study.  Their competition projections use the same ratio normalization
as the preceding vehicle-mass analysis.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from battery_trade_study import build_powertrain_config
from openlap_solver import Vehicle, load_vehicle, solve_track
from powertrain_model import PowertrainConfig, PowertrainModel, load_powertrain_config


OFFICIAL_2026_RULES_URL = (
    "https://www.fsaeonline.com/cdsweb/gen/DownloadDocument.aspx?"
    "DocumentID=278fd4d7-aa27-4e33-bc4a-090148e662a0"
)
PACK_AWARE_MODE = "pack_aware_full_soc_80kw_terminal"
MASS_ISOLATED_MODE = "mass_isolated_common_80kw_curve"
BASELINE_CANDIDATE_ID = "molicel_p30b_130s5p"
COMMON_TERMINAL_POWER_LIMIT_KW = 80.0


@dataclass(frozen=True)
class TimeScoreRule:
    slug: str
    display_name: str
    maximum_points: float
    performance_points: float
    completion_points: float
    tmin_s: float
    tmax_factor: float
    time_ratio_exponent: float = 1.0

    @property
    def tmax_s(self) -> float:
        return self.tmin_s * self.tmax_factor

    def score(self, time_s: float) -> float:
        """Return the official time score, clamped to the event point range."""

        if not math.isfinite(time_s) or time_s <= 0.0:
            raise ValueError(f"{self.slug} time must be finite and positive")
        if time_s >= self.tmax_s:
            return self.completion_points
        numerator = (
            (self.tmax_s / time_s) ** self.time_ratio_exponent - 1.0
        )
        denominator = (
            (self.tmax_s / self.tmin_s) ** self.time_ratio_exponent - 1.0
        )
        score = (
            self.performance_points * numerator / denominator
            + self.completion_points
        )
        return float(np.clip(score, self.completion_points, self.maximum_points))

    def as_dict(self) -> dict[str, Any]:
        return {
            "slug": self.slug,
            "display_name": self.display_name,
            "maximum_points": self.maximum_points,
            "performance_points": self.performance_points,
            "completion_points": self.completion_points,
            "tmin_s": self.tmin_s,
            "tmax_factor": self.tmax_factor,
            "tmax_s": self.tmax_s,
            "time_ratio_exponent": self.time_ratio_exponent,
        }


EVENT_RULES: dict[str, TimeScoreRule] = {
    "acceleration": TimeScoreRule(
        slug="acceleration",
        display_name="Acceleration",
        maximum_points=100.0,
        performance_points=95.5,
        completion_points=4.5,
        tmin_s=3.697,
        tmax_factor=1.50,
    ),
    "skidpad": TimeScoreRule(
        slug="skidpad",
        display_name="Skidpad",
        maximum_points=75.0,
        performance_points=71.5,
        completion_points=3.5,
        tmin_s=4.782,
        tmax_factor=1.25,
        time_ratio_exponent=2.0,
    ),
    "autocross": TimeScoreRule(
        slug="autocross",
        display_name="Autocross",
        maximum_points=125.0,
        performance_points=118.5,
        completion_points=6.5,
        tmin_s=43.937,
        tmax_factor=1.45,
    ),
    "endurance": TimeScoreRule(
        slug="endurance",
        display_name="Endurance",
        maximum_points=275.0,
        performance_points=250.0,
        completion_points=25.0,
        tmin_s=1312.281,
        tmax_factor=1.45,
    ),
}


EFFICIENCY_INPUTS = {
    "tmin_average_lap_s": 59.649,
    "minimum_adjusted_energy_per_lap_kg_co2": 0.0840,
    "ef_min": 0.289,
    "ef_max": 0.797,
    "winner_average_lap_s": 65.192,
    "winner_total_energy_kwh": 3.263,
    "winner_lap_count": 22,
    "ev_conversion_kg_co2_per_kwh": 0.65,
    "maximum_points": 100.0,
}


def efficiency_projection(
    candidate_average_lap_s: float,
    candidate_terminal_energy_kwh: float,
    baseline_average_lap_s: float,
    baseline_terminal_energy_kwh: float,
) -> dict[str, float]:
    """Apply the inherited competition-ratio efficiency normalization."""

    values = (
        candidate_average_lap_s,
        candidate_terminal_energy_kwh,
        baseline_average_lap_s,
        baseline_terminal_energy_kwh,
    )
    if any(not math.isfinite(value) or value <= 0.0 for value in values):
        raise ValueError("Efficiency projection inputs must be finite and positive")
    inputs = EFFICIENCY_INPUTS
    projected_lap_s = inputs["winner_average_lap_s"] * (
        candidate_average_lap_s / baseline_average_lap_s
    )
    projected_kwh_per_lap = (
        inputs["winner_total_energy_kwh"]
        / inputs["winner_lap_count"]
        * (candidate_terminal_energy_kwh / baseline_terminal_energy_kwh)
    )
    efficiency_factor = (
        inputs["tmin_average_lap_s"]
        * inputs["minimum_adjusted_energy_per_lap_kg_co2"]
        / (
            projected_lap_s
            * projected_kwh_per_lap
            * inputs["ev_conversion_kg_co2_per_kwh"]
        )
    )
    score = inputs["maximum_points"] * (
        efficiency_factor - inputs["ef_min"]
    ) / (inputs["ef_max"] - inputs["ef_min"])
    return {
        "projected_average_lap_s": float(projected_lap_s),
        "projected_terminal_energy_kwh_per_lap": float(
            projected_kwh_per_lap
        ),
        "efficiency_factor": float(efficiency_factor),
        "efficiency_points": float(
            np.clip(score, 0.0, inputs["maximum_points"])
        ),
    }


def load_cells(path: Path) -> dict[str, dict[str, Any]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    cells = {str(cell["cell_id"]): cell for cell in data["cells"]}
    if len(cells) != len(data["cells"]):
        raise ValueError("Cell input contains duplicate cell_id values")
    return cells


def build_pack_aware_vehicle(
    base_vehicle: Vehicle,
    base_powertrain: PowertrainConfig,
    cell: dict[str, Any],
    candidate: pd.Series,
    *,
    curve_points: int = 121,
) -> tuple[Vehicle, dict[str, Any]]:
    """Return a full-SOC candidate vehicle and electrical-limit diagnostics."""

    if curve_points < 3:
        raise ValueError("curve_points must be at least 3")
    config = build_powertrain_config(
        base_powertrain,
        cell,
        int(candidate["series_cells"]),
        int(candidate["parallel_cells"]),
        COMMON_TERMINAL_POWER_LIMIT_KW,
    )
    model = PowertrainModel(config)
    speeds = np.linspace(0.0, base_vehicle.v_max, curve_points)
    points = [
        model.maximum_available_torque(
            config.drivetrain.motor_speed_rpm(float(speed)),
            config.pack.initial_soc,
        )
        for speed in speeds
    ]
    torques = np.asarray([point.motor_torque_nm for point in points], dtype=float)
    wheel_forces = np.asarray(
        [
            config.drivetrain.force_for_motor_torque(float(torque))
            for torque in torques
        ],
        dtype=float,
    )
    terminal_powers_kw = np.asarray(
        [point.battery_terminal_power_w / 1000.0 for point in points],
        dtype=float,
    )
    wheel_powers_kw = wheel_forces * speeds / 1000.0
    peak_index = int(np.argmax(terminal_powers_kw))
    vehicle = replace(
        base_vehicle,
        mass=float(candidate["vehicle_mass_kg"]),
        vehicle_speed=speeds,
        fx_engine=wheel_forces,
    )
    diagnostics = {
        "curve_points": int(curve_points),
        "full_soc": float(config.pack.initial_soc),
        "terminal_power_limit_kw": COMMON_TERMINAL_POWER_LIMIT_KW,
        "maximum_achievable_terminal_power_kw": float(
            terminal_powers_kw.max()
        ),
        "maximum_achievable_wheel_power_kw": float(wheel_powers_kw.max()),
        "maximum_tractive_force_n": float(wheel_forces.max()),
        "active_limiter_at_peak_terminal_power": points[
            peak_index
        ].active_limiter,
        "reaches_79p9kw_terminal": bool(terminal_powers_kw.max() >= 79.9),
    }
    return vehicle, diagnostics


def build_mass_isolated_vehicle(
    base_vehicle: Vehicle, vehicle_mass_kg: float
) -> Vehicle:
    """Change mass only while retaining the shared 80 kW vehicle curve."""

    if not math.isfinite(vehicle_mass_kg) or vehicle_mass_kg <= 0.0:
        raise ValueError("vehicle mass must be finite and positive")
    return replace(base_vehicle, mass=float(vehicle_mass_kg))


def run_nonenergy_events(
    vehicle: Vehicle,
    event_inputs: dict[str, tuple[pd.DataFrame, bool]],
) -> dict[str, dict[str, Any]]:
    results: dict[str, dict[str, Any]] = {}
    for slug in ("acceleration", "skidpad", "autocross"):
        track, is_closed = event_inputs[slug]
        _, summary = solve_track(vehicle, track, is_closed=is_closed)
        time_s = float(summary["lap_time_s"])
        results[slug] = {
            "raw_time_s": time_s,
            "converged": bool(summary["converged"]),
            "iterations": int(summary["iterations"]),
            "maximum_speed_mps": float(summary["maximum_speed_mps"]),
            "minimum_speed_mps": float(summary["minimum_speed_mps"]),
        }
    return results


def project_nonenergy_scores(
    event_results: dict[str, dict[str, Any]],
    baseline_results: dict[str, dict[str, Any]],
) -> dict[str, dict[str, Any]]:
    """Map converted-track time ratios onto each 2026 event winner."""

    projected: dict[str, dict[str, Any]] = {}
    for slug in ("acceleration", "skidpad", "autocross"):
        raw_time_s = float(event_results[slug]["raw_time_s"])
        baseline_time_s = float(baseline_results[slug]["raw_time_s"])
        if baseline_time_s <= 0.0:
            raise ValueError(f"{slug} baseline time must be positive")
        projected_time_s = EVENT_RULES[slug].tmin_s * (
            raw_time_s / baseline_time_s
        )
        projected[slug] = {
            **event_results[slug],
            "projected_time_s": float(projected_time_s),
            "points": EVENT_RULES[slug].score(projected_time_s),
        }
    return projected


def endurance_and_efficiency(
    candidate: pd.Series, baseline: pd.Series
) -> dict[str, float]:
    projected_endurance_time_s = EVENT_RULES["endurance"].tmin_s * (
        float(candidate["elapsed_time_s"]) / float(baseline["elapsed_time_s"])
    )
    efficiency = efficiency_projection(
        float(candidate["equivalent_average_lap_time_s"]),
        float(candidate["terminal_energy_kwh"]),
        float(baseline["equivalent_average_lap_time_s"]),
        float(baseline["terminal_energy_kwh"]),
    )
    return {
        "raw_endurance_time_s": float(candidate["elapsed_time_s"]),
        "projected_endurance_time_s": float(projected_endurance_time_s),
        "endurance_points": EVENT_RULES["endurance"].score(
            projected_endurance_time_s
        ),
        **efficiency,
    }


def _candidate_metadata(candidate: pd.Series) -> dict[str, Any]:
    return {
        "candidate_id": str(candidate["candidate_id"]),
        "cell_id": str(candidate["cell_id"]),
        "manufacturer": str(candidate["manufacturer"]),
        "cell_model": str(candidate["cell_model"]),
        "series_cells": int(candidate["series_cells"]),
        "parallel_cells": int(candidate["parallel_cells"]),
        "total_cells": int(candidate["total_cells"]),
        "pack_mass_kg": float(candidate["pack_mass_kg"]),
        "vehicle_mass_kg": float(candidate["vehicle_mass_kg"]),
        "nominal_pack_energy_kwh": float(
            candidate["nominal_pack_energy_kwh"]
        ),
        "model_usable_energy_kwh": float(
            candidate["model_usable_energy_kwh"]
        ),
        "endurance_terminal_power_limit_kw": float(
            candidate["power_limit_kw"]
        ),
    }


def _wide_row(
    candidate: pd.Series,
    pack_aware: dict[str, dict[str, Any]],
    mass_isolated: dict[str, dict[str, Any]],
    pack_diagnostics: dict[str, Any],
    shared: dict[str, float],
) -> dict[str, Any]:
    row = {
        **_candidate_metadata(candidate),
        **{
            f"pack_aware_{key}": value
            for key, value in pack_diagnostics.items()
        },
        **shared,
    }
    for mode_prefix, event_results in (
        ("pack_aware", pack_aware),
        ("mass_isolated", mass_isolated),
    ):
        for slug, result in event_results.items():
            row[f"{mode_prefix}_{slug}_raw_time_s"] = result["raw_time_s"]
            row[f"{mode_prefix}_{slug}_projected_time_s"] = result[
                "projected_time_s"
            ]
            row[f"{mode_prefix}_{slug}_points"] = result["points"]
            row[f"{mode_prefix}_{slug}_converged"] = result["converged"]
        nonenergy = sum(
            event_results[slug]["points"]
            for slug in ("acceleration", "skidpad", "autocross")
        )
        performance = nonenergy + shared["endurance_points"]
        full_dynamic = performance + shared["efficiency_points"]
        if mode_prefix == "pack_aware":
            row["pack_aware_nonenergy_points"] = float(nonenergy)
            row[
                "pack_aware_performance_points_excluding_efficiency"
            ] = float(performance)
            row["pack_aware_full_dynamic_points"] = float(full_dynamic)
        else:
            row["mass_isolated_sprint_nonenergy_points"] = float(nonenergy)
            row[
                "mass_isolated_sprint_hybrid_performance_points_excluding_efficiency"
            ] = float(performance)
            row[
                "mass_isolated_sprint_hybrid_full_dynamic_points"
            ] = float(full_dynamic)
    row["pack_aware_minus_mass_isolated_nonenergy_points"] = (
        row["pack_aware_nonenergy_points"]
        - row["mass_isolated_sprint_nonenergy_points"]
    )
    row["pack_aware_minus_mass_isolated_full_dynamic_points"] = (
        row["pack_aware_full_dynamic_points"]
        - row["mass_isolated_sprint_hybrid_full_dynamic_points"]
    )
    return row


def _long_rows(
    candidate: pd.Series,
    mode: str,
    event_results: dict[str, dict[str, Any]],
    pack_diagnostics: dict[str, Any] | None,
) -> list[dict[str, Any]]:
    rows = []
    for slug in ("acceleration", "skidpad", "autocross"):
        rule = EVENT_RULES[slug]
        result = event_results[slug]
        rows.append(
            {
                **_candidate_metadata(candidate),
                "simulation_mode": mode,
                "event_slug": slug,
                "event_name": rule.display_name,
                "raw_simulated_time_s": result["raw_time_s"],
                "projected_competition_time_s": result["projected_time_s"],
                "event_winner_tmin_s": rule.tmin_s,
                "tmax_s": rule.tmax_s,
                "points": result["points"],
                "maximum_points": rule.maximum_points,
                "solver_converged": result["converged"],
                "solver_iterations": result["iterations"],
                "maximum_speed_mps": result["maximum_speed_mps"],
                "maximum_achievable_terminal_power_kw": (
                    pack_diagnostics["maximum_achievable_terminal_power_kw"]
                    if pack_diagnostics is not None
                    else np.nan
                ),
                "maximum_achievable_wheel_power_kw": (
                    pack_diagnostics["maximum_achievable_wheel_power_kw"]
                    if pack_diagnostics is not None
                    else 80.0
                ),
                "active_limiter_at_peak_terminal_power": (
                    pack_diagnostics[
                        "active_limiter_at_peak_terminal_power"
                    ]
                    if pack_diagnostics is not None
                    else "common_vehicle_curve"
                ),
            }
        )
    return rows


def local_mass_sensitivity(
    base_vehicle: Vehicle,
    event_inputs: dict[str, tuple[pd.DataFrame, bool]],
    *,
    step_kg: float = 0.5,
) -> dict[str, float]:
    """One-sided adverse mass sensitivity at baseline mass.

    The lighter side clips at the event maximum after ratio normalization, so a
    central difference would artificially halve the useful added-mass
    sensitivity.
    """

    baseline_raw = run_nonenergy_events(base_vehicle, event_inputs)
    upper_raw = run_nonenergy_events(
        build_mass_isolated_vehicle(base_vehicle, base_vehicle.mass + step_kg),
        event_inputs,
    )
    baseline = project_nonenergy_scores(baseline_raw, baseline_raw)
    upper = project_nonenergy_scores(upper_raw, baseline_raw)
    sensitivity = {}
    for slug in ("acceleration", "skidpad", "autocross"):
        signed = (
            upper[slug]["points"] - baseline[slug]["points"]
        ) / step_kg
        sensitivity[f"{slug}_points_change_per_added_kg"] = signed
        sensitivity[f"{slug}_point_loss_per_added_kg"] = -signed
        sensitivity[f"{slug}_raw_seconds_per_added_kg"] = (
            upper[slug]["raw_time_s"] - baseline[slug]["raw_time_s"]
        ) / step_kg
    sensitivity["nonenergy_points_change_per_added_kg"] = sum(
        sensitivity[f"{slug}_points_change_per_added_kg"]
        for slug in ("acceleration", "skidpad", "autocross")
    )
    sensitivity["nonenergy_point_loss_per_added_kg"] = -sensitivity[
        "nonenergy_points_change_per_added_kg"
    ]
    sensitivity["reference_vehicle_mass_kg"] = float(base_vehicle.mass)
    sensitivity["one_sided_added_mass_step_kg"] = float(step_kg)
    return sensitivity


def _rank_columns(summary: pd.DataFrame) -> pd.DataFrame:
    ranked = summary.copy()
    lightest_index = ranked["vehicle_mass_kg"].idxmin()
    for slug in ("acceleration", "skidpad", "autocross"):
        raw_column = f"mass_isolated_{slug}_raw_time_s"
        ranked[f"{raw_column}_delta_vs_lightest_s"] = (
            ranked[raw_column] - ranked.loc[lightest_index, raw_column]
        )
    for column in (
        "pack_aware_nonenergy_points",
        "pack_aware_performance_points_excluding_efficiency",
        "pack_aware_full_dynamic_points",
        "mass_isolated_sprint_nonenergy_points",
        "mass_isolated_sprint_hybrid_performance_points_excluding_efficiency",
        "mass_isolated_sprint_hybrid_full_dynamic_points",
    ):
        ranked[f"{column}_rank"] = ranked[column].rank(
            method="min", ascending=False
        )
    return ranked


def _global_mass_slopes(summary: pd.DataFrame) -> dict[str, float]:
    mass = summary["vehicle_mass_kg"].to_numpy(dtype=float)
    outputs = {}
    for column in (
        "mass_isolated_acceleration_points",
        "mass_isolated_skidpad_points",
        "mass_isolated_autocross_points",
        "mass_isolated_sprint_nonenergy_points",
        "mass_isolated_acceleration_raw_time_s",
        "mass_isolated_skidpad_raw_time_s",
        "mass_isolated_autocross_raw_time_s",
    ):
        slope, intercept = np.polyfit(
            mass, summary[column].to_numpy(dtype=float), 1
        )
        predicted = slope * mass + intercept
        residual = summary[column].to_numpy(dtype=float) - predicted
        total = summary[column].to_numpy(dtype=float) - summary[column].mean()
        r_squared = 1.0 - float(np.sum(residual**2)) / float(np.sum(total**2))
        is_time = column.endswith("_raw_time_s")
        unit = "seconds_per_added_kg" if is_time else "points_per_added_kg"
        if is_time:
            metric = column.removesuffix("_raw_time_s") + "_raw_time"
        else:
            metric = column.removesuffix("_points") + "_points"
        outputs[f"{metric}_{unit}"] = float(slope)
        outputs[f"{metric}_linear_r_squared"] = float(r_squared)
    return outputs


def _headlines(
    summary: pd.DataFrame,
    local_sensitivity: dict[str, float],
    global_slopes: dict[str, float],
) -> dict[str, Any]:
    baseline = summary.loc[
        summary["candidate_id"] == BASELINE_CANDIDATE_ID
    ].iloc[0]
    primary_best = summary.sort_values(
        [
            "pack_aware_full_dynamic_points",
            "pack_aware_performance_points_excluding_efficiency",
        ],
        ascending=False,
    ).iloc[0]
    performance_best = summary.sort_values(
        "pack_aware_performance_points_excluding_efficiency", ascending=False
    ).iloc[0]
    lightest = summary.sort_values("vehicle_mass_kg").iloc[0]
    heaviest = summary.sort_values("vehicle_mass_kg", ascending=False).iloc[0]
    return {
        "candidate_count": int(len(summary)),
        "primary_model": PACK_AWARE_MODE,
        "primary_best_full_dynamic": primary_best.to_dict(),
        "primary_best_performance_excluding_efficiency": (
            performance_best.to_dict()
        ),
        "baseline": baseline.to_dict(),
        "lightest_candidate": lightest.to_dict(),
        "heaviest_candidate": heaviest.to_dict(),
        "mass_only_lightest_to_heaviest_nonenergy_point_change": float(
            heaviest["mass_isolated_sprint_nonenergy_points"]
            - lightest["mass_isolated_sprint_nonenergy_points"]
        ),
        "mass_only_lightest_to_heaviest_raw_time_changes_s": {
            slug: float(
                heaviest[f"mass_isolated_{slug}_raw_time_s"]
                - lightest[f"mass_isolated_{slug}_raw_time_s"]
            )
            for slug in ("acceleration", "skidpad", "autocross")
        },
        "mass_only_lightest_to_heaviest_mass_change_kg": float(
            heaviest["vehicle_mass_kg"] - lightest["vehicle_mass_kg"]
        ),
        "pack_aware_79p9kw_reaching_count": int(
            summary["pack_aware_reaches_79p9kw_terminal"].sum()
        ),
        "local_mass_sensitivity": local_sensitivity,
        "global_mass_slopes": global_slopes,
    }


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

    fig, ax = plt.subplots(figsize=(9.5, 6.0))
    ax.scatter(
        mass,
        summary["pack_aware_nonenergy_points"],
        s=24,
        alpha=0.75,
        color=colors["pack"],
        label="Pack-aware, full SOC, 80 kW terminal cap",
    )
    order = np.argsort(mass.to_numpy())
    ax.plot(
        mass.iloc[order],
        summary["mass_isolated_sprint_nonenergy_points"].iloc[order],
        color=colors["mass"],
        linewidth=2.0,
        label="Mass-only, common 80 kW tractive curve",
    )
    ax.set(
        xlabel="Pack mass (kg)",
        ylabel="Accel + skidpad + autocross points (300 max)",
        title="Non-energy event points versus pack mass",
    )
    ax.grid(alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(plot_dir / "nonenergy_points_vs_pack_mass.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9.5, 6.0))
    ax.scatter(
        mass,
        summary["pack_aware_full_dynamic_points"],
        s=24,
        alpha=0.75,
        color=colors["pack"],
        label="Pack-aware full dynamic total",
    )
    ax.scatter(
        mass,
        summary["mass_isolated_sprint_hybrid_full_dynamic_points"],
        s=18,
        alpha=0.45,
        color=colors["mass"],
        label="Mass-isolated sprint hybrid total",
    )
    ax.set(
        xlabel="Pack mass (kg)",
        ylabel="Dynamic points including efficiency (675 max)",
        title="Full dynamic points versus pack mass",
    )
    ax.grid(alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(plot_dir / "full_dynamic_points_vs_pack_mass.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(1, 3, figsize=(15.0, 4.7), sharex=True)
    for ax, slug in zip(
        axes, ("acceleration", "skidpad", "autocross")
    ):
        ax.scatter(
            mass,
            summary[f"pack_aware_{slug}_points"],
            s=16,
            alpha=0.65,
            color=colors["pack"],
            label="Pack-aware",
        )
        ax.plot(
            mass.iloc[order],
            summary[f"mass_isolated_{slug}_points"].iloc[order],
            color=colors["mass"],
            linewidth=1.7,
            label="Mass-only",
        )
        ax.set_title(EVENT_RULES[slug].display_name)
        ax.set_xlabel("Pack mass (kg)")
        ax.grid(alpha=0.25)
    axes[0].set_ylabel("Event points")
    axes[0].legend()
    fig.suptitle("Battery configuration effect by non-energy event")
    fig.tight_layout()
    fig.savefig(plot_dir / "event_points_vs_pack_mass.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(1, 3, figsize=(15.0, 4.7), sharex=True)
    for ax, slug in zip(
        axes, ("acceleration", "skidpad", "autocross")
    ):
        delta_column = (
            f"mass_isolated_{slug}_raw_time_s_delta_vs_lightest_s"
        )
        ax.plot(
            mass.iloc[order],
            1000.0 * summary[delta_column].iloc[order],
            color=colors[slug],
            linewidth=2.0,
        )
        ax.set_title(EVENT_RULES[slug].display_name)
        ax.set_xlabel("Pack mass (kg)")
        ax.grid(alpha=0.25)
    axes[0].set_ylabel("Raw time loss versus lightest pack (ms)")
    fig.suptitle("Uncapped mass-only event-time penalty")
    fig.tight_layout()
    fig.savefig(
        plot_dir / "raw_event_time_penalty_vs_pack_mass.png", dpi=180
    )
    plt.close(fig)

    top = summary.nlargest(15, "pack_aware_full_dynamic_points").copy()
    top = top.sort_values("pack_aware_full_dynamic_points")
    components = [
        ("pack_aware_acceleration_points", "Acceleration", colors["acceleration"]),
        ("pack_aware_skidpad_points", "Skidpad", colors["skidpad"]),
        ("pack_aware_autocross_points", "Autocross", colors["autocross"]),
        ("endurance_points", "Endurance", colors["endurance"]),
        ("efficiency_points", "Efficiency", colors["efficiency"]),
    ]
    fig, ax = plt.subplots(figsize=(11.0, 7.5))
    left = np.zeros(len(top))
    for column, label, color in components:
        values = top[column].to_numpy(dtype=float)
        ax.barh(
            top["candidate_id"],
            values,
            left=left,
            label=label,
            color=color,
        )
        left += values
    ax.set(
        xlabel="Dynamic points (675 max)",
        title="Top 15 pack-aware battery configurations",
    )
    ax.grid(axis="x", alpha=0.25)
    ax.legend(ncol=3, loc="lower right")
    fig.tight_layout()
    fig.savefig(plot_dir / "top15_dynamic_points_stacked.png", dpi=180)
    plt.close(fig)


def _format_top_table(frame: pd.DataFrame, count: int = 10) -> str:
    columns = [
        "candidate_id",
        "pack_mass_kg",
        "pack_aware_acceleration_points",
        "pack_aware_skidpad_points",
        "pack_aware_autocross_points",
        "endurance_points",
        "efficiency_points",
        "pack_aware_full_dynamic_points",
    ]
    top = frame.nlargest(count, "pack_aware_full_dynamic_points")[columns]
    header = (
        "| Candidate | Pack kg | Accel | Skidpad | Autox | Endurance | "
        "Efficiency | Total |\n"
        "|---|---:|---:|---:|---:|---:|---:|---:|"
    )
    rows = [
        (
            f"| {row.candidate_id} | {row.pack_mass_kg:.3f} | "
            f"{row.pack_aware_acceleration_points:.2f} | "
            f"{row.pack_aware_skidpad_points:.2f} | "
            f"{row.pack_aware_autocross_points:.2f} | "
            f"{row.endurance_points:.2f} | {row.efficiency_points:.2f} | "
            f"{row.pack_aware_full_dynamic_points:.2f} |"
        )
        for row in top.itertuples(index=False)
    ]
    return "\n".join([header, *rows])


def _write_report(
    output_dir: Path,
    summary: pd.DataFrame,
    headlines: dict[str, Any],
) -> None:
    baseline = summary.loc[
        summary["candidate_id"] == BASELINE_CANDIDATE_ID
    ].iloc[0]
    lightest = summary.sort_values("vehicle_mass_kg").iloc[0]
    heaviest = summary.sort_values("vehicle_mass_kg", ascending=False).iloc[0]
    best = summary.nlargest(1, "pack_aware_full_dynamic_points").iloc[0]
    local = headlines["local_mass_sensitivity"]
    report = f"""# Battery configuration dynamic-points study

## Scope

All {len(summary)} accepted battery configurations from the 22 km trade study
were run through acceleration, skidpad, and autocross. The primary result uses
each pack's full-SOC voltage, resistance, continuous-current limit, and the
common 80 kW battery-terminal ceiling. The diagnostic result changes vehicle
mass only while keeping one common 80 kW OpenLAP tractive curve.

The performance subtotal is 575 points maximum (acceleration 100, skidpad 75,
autocross 125, endurance 275). The full dynamic total adds the inherited
100-point efficiency projection for a 675-point maximum.

## Headline results

- Highest pack-aware full dynamic score: **{best.candidate_id}** at
  **{best.pack_aware_full_dynamic_points:.2f} / 675**.
- Baseline **{BASELINE_CANDIDATE_ID}**:
  **{baseline.pack_aware_full_dynamic_points:.2f} / 675** pack-aware and
  **{baseline.mass_isolated_sprint_hybrid_full_dynamic_points:.2f} / 675** in
  the hybrid that combines mass-isolated sprint scores with actual
  pack-specific endurance and efficiency.
- {headlines["pack_aware_79p9kw_reaching_count"]} of {len(summary)} packs reach
  at least 79.9 kW battery-terminal power at full SOC somewhere on the speed
  curve. The others are limited first by their electrical envelope.
- The mass-only non-energy subtotal changes
  **{headlines["mass_only_lightest_to_heaviest_nonenergy_point_change"]:.2f} points**
  from the lightest pack ({lightest.pack_mass_kg:.2f} kg) to the
  heaviest pack ({heaviest.pack_mass_kg:.2f} kg). Because lighter-than-baseline
  candidates clip at the event maxima, use the uncapped raw-time columns for
  comparisons within that clipped region.
- Lightest-to-heaviest mass-only raw time changes are
  **{headlines["mass_only_lightest_to_heaviest_raw_time_changes_s"]["acceleration"]:.4f} s acceleration**,
  **{headlines["mass_only_lightest_to_heaviest_raw_time_changes_s"]["skidpad"]:.4f} s skidpad**,
  and **{headlines["mass_only_lightest_to_heaviest_raw_time_changes_s"]["autocross"]:.4f} s autocross**.
- At the baseline vehicle mass, the one-sided adverse mass-only sensitivity is
  **{local["nonenergy_point_loss_per_added_kg"]:.3f} non-energy points lost per added kg**:
  acceleration
  {local["acceleration_point_loss_per_added_kg"]:.3f}, skidpad
  {local["skidpad_point_loss_per_added_kg"]:.3f}, and autocross
  {local["autocross_point_loss_per_added_kg"]:.3f} point/kg.

## Primary ranking

{_format_top_table(summary)}

## Scoring and normalization

- Formula SAE 2026 time-score equations are used with event-winner anchors:
  acceleration 3.697 s, skidpad 4.782 s, autocross 43.937 s, and endurance
  1312.281 s.
- For each sprint model and event, the P30B 130s5p baseline's converted-track
  time is assigned that event's 2026 winner time. Every candidate retains its
  simulated time ratio to the P30B baseline before the official formula is
  applied. Raw converted-track and projected competition times are both saved.
- The 22 km endurance simulation is mapped by time ratio: the P30B 130s5p
  baseline is assigned the 1312.281 s winner time, and every candidate keeps
  its simulated time ratio to that baseline. A completed candidate receives
  the 25 lap/completion points plus up to 250 time points.
- Efficiency is explicitly an inherited ratio-normalized projection, not a
  fresh competition energy-meter reconstruction. Candidate average-lap-time
  and terminal-energy ratios to P30B are mapped onto the established winner
  inputs (65.192 s average lap and 3.263 kWh over 22 laps), then scored with
  EFmin 0.289 and EFmax 0.797.

## Interpretation

Use the pack-aware result to compare real candidate concepts because it includes
the full-SOC electrical power constraint. Use the common-curve result to answer
the narrower question, "what does battery mass alone cost?" The signed
`pack_aware_minus_mass_isolated` difference reports how a candidate's
electrical envelope changes its score *relative to P30B*, because each model is
independently ratio-normalized to the P30B baseline. A negative value means the
candidate is electrically disadvantaged relative to P30B; a positive value
means it is less constrained than P30B. It is not an absolute point penalty.

Only the 300-point short-event subtotal is cleanly mass-isolated. Any 575- or
675-point column carrying the `mass_isolated_sprint_hybrid` prefix combines
mass-only sprint scores with the actual pack-specific chronological endurance
and inherited efficiency results.

The short-event calculation starts each run at full SOC and does not integrate
SOC or temperature over the few-second event. Endurance remains the prior
chronological, equal-reserve simulation, where each candidate's terminal power
ceiling was tuned independently up to 80 kW to finish 22 km with 0.500 kWh
remaining.

## Artifacts

- `battery_dynamic_points_summary.csv`: one row per pack, both models, all event
  scores, subtotals, totals, and ranks.
- `battery_dynamic_event_results_long.csv`: one row per pack/model/short event.
- `inherited_endurance_efficiency_scores.csv`: shared endurance and efficiency
  projections.
- `scoring_inputs.json`: formulas, anchors, and normalization inputs.
- `headline_summary.json`: best configurations and mass sensitivities.
- `validation_report.json`: automated execution checks.
- `plots/`: decision plots.
"""
    (output_dir / "battery_dynamic_points_report.md").write_text(
        report, encoding="utf-8"
    )


def _validation_report(
    summary: pd.DataFrame,
    long: pd.DataFrame,
    input_candidate_count: int,
) -> dict[str, Any]:
    baseline = summary.loc[
        summary["candidate_id"] == BASELINE_CANDIDATE_ID
    ].iloc[0]
    checks = {
        "all_input_candidates_present": (
            len(summary) == input_candidate_count
            and summary["candidate_id"].nunique() == input_candidate_count
        ),
        "two_modes_three_events_per_candidate": (
            len(long) == input_candidate_count * 2 * 3
            and set(long["simulation_mode"])
            == {PACK_AWARE_MODE, MASS_ISOLATED_MODE}
            and set(long["event_slug"])
            == {"acceleration", "skidpad", "autocross"}
        ),
        "all_event_solvers_converged": bool(long["solver_converged"].all()),
        "all_scores_finite": bool(
            np.isfinite(
                summary[
                    [
                        "pack_aware_full_dynamic_points",
                        "mass_isolated_sprint_hybrid_full_dynamic_points",
                    ]
                ].to_numpy(dtype=float)
            ).all()
        ),
        "full_dynamic_scores_in_0_to_675": bool(
            (
                summary[
                    [
                        "pack_aware_full_dynamic_points",
                        "mass_isolated_sprint_hybrid_full_dynamic_points",
                    ]
                ]
                >= 0.0
            ).all().all()
            and (
                summary[
                    [
                        "pack_aware_full_dynamic_points",
                        "mass_isolated_sprint_hybrid_full_dynamic_points",
                    ]
                ]
                <= 675.0 + 1e-9
            ).all().all()
        ),
        "p30b_pack_aware_acceleration_regression": bool(
            abs(baseline["pack_aware_acceleration_raw_time_s"] - 4.284205)
            < 5e-5
        ),
        "p30b_pack_aware_autocross_regression": bool(
            abs(baseline["pack_aware_autocross_raw_time_s"] - 52.259586)
            < 5e-5
        ),
        "p30b_pack_aware_skidpad_regression": bool(
            abs(baseline["pack_aware_skidpad_raw_time_s"] - 4.800188)
            < 5e-5
        ),
        "p30b_is_max_score_for_both_sprint_models": bool(
            all(
                abs(
                    baseline[f"{prefix}_{slug}_points"]
                    - EVENT_RULES[slug].maximum_points
                )
                < 1e-9
                for prefix in ("pack_aware", "mass_isolated")
                for slug in ("acceleration", "skidpad", "autocross")
            )
        ),
        "baseline_endurance_maps_to_tmin": bool(
            abs(
                baseline["projected_endurance_time_s"]
                - EVENT_RULES["endurance"].tmin_s
            )
            < 1e-9
        ),
        "baseline_efficiency_maps_to_100_points": bool(
            abs(baseline["efficiency_points"] - 100.0) < 1e-9
        ),
        "mass_isolated_nonenergy_points_nonincreasing_with_mass": bool(
            (
                summary.sort_values("vehicle_mass_kg")[
                    "mass_isolated_sprint_nonenergy_points"
                ]
                .diff()
                .fillna(0.0)
                <= 1e-7
            ).all()
        ),
    }
    return {
        "all_checks_passed": bool(all(checks.values())),
        "checks": checks,
        "candidate_count": int(len(summary)),
        "long_result_count": int(len(long)),
    }


def run_study(
    *,
    root: Path,
    candidate_csv: Path,
    cells_json: Path,
    output_dir: Path,
    curve_points: int = 121,
    progress_every: int = 10,
) -> dict[str, Any]:
    root = root.resolve()
    output_dir = output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    candidates = pd.read_csv(candidate_csv.resolve())
    if candidates["candidate_id"].duplicated().any():
        raise ValueError("Candidate input contains duplicate candidate_id values")
    if BASELINE_CANDIDATE_ID not in set(candidates["candidate_id"]):
        raise ValueError(f"Missing baseline {BASELINE_CANDIDATE_ID}")
    cells = load_cells(cells_json.resolve())
    missing_cells = sorted(set(candidates["cell_id"]) - set(cells))
    if missing_cells:
        raise ValueError(f"Missing cell inputs for: {missing_cells}")

    base_vehicle = load_vehicle(root / "inputs" / "openlap_vehicle.json")
    base_powertrain = load_powertrain_config(
        root / "inputs" / "powertrain_130s5p_p30b_provisional.json"
    )
    manifest = json.loads(
        (
            root / "inputs" / "events" / "openlap_event_suite_manifest.json"
        ).read_text(encoding="utf-8")
    )
    event_by_slug = {event["Slug"]: event for event in manifest["Events"]}
    event_inputs = {
        slug: (
            pd.read_csv(root / event_by_slug[slug]["OpenLAPTrackCsv"]),
            bool(event_by_slug[slug]["IsClosed"]),
        )
        for slug in ("acceleration", "skidpad", "autocross")
    }
    baseline = candidates.loc[
        candidates["candidate_id"] == BASELINE_CANDIDATE_ID
    ].iloc[0]
    baseline_pack_vehicle, baseline_pack_diagnostics = (
        build_pack_aware_vehicle(
            base_vehicle,
            base_powertrain,
            cells[str(baseline["cell_id"])],
            baseline,
            curve_points=curve_points,
        )
    )
    baseline_pack_raw = run_nonenergy_events(
        baseline_pack_vehicle, event_inputs
    )
    baseline_mass_raw = run_nonenergy_events(
        build_mass_isolated_vehicle(
            base_vehicle, float(baseline["vehicle_mass_kg"])
        ),
        event_inputs,
    )

    wide_rows: list[dict[str, Any]] = []
    long_rows: list[dict[str, Any]] = []
    shared_rows: list[dict[str, Any]] = []
    mass_cache: dict[float, dict[str, dict[str, Any]]] = {
        round(float(baseline["vehicle_mass_kg"]), 9): baseline_mass_raw
    }
    for index, candidate in candidates.iterrows():
        if candidate["candidate_id"] == BASELINE_CANDIDATE_ID:
            pack_raw = baseline_pack_raw
            diagnostics = baseline_pack_diagnostics
        else:
            pack_vehicle, diagnostics = build_pack_aware_vehicle(
                base_vehicle,
                base_powertrain,
                cells[str(candidate["cell_id"])],
                candidate,
                curve_points=curve_points,
            )
            pack_raw = run_nonenergy_events(pack_vehicle, event_inputs)
        pack_results = project_nonenergy_scores(
            pack_raw, baseline_pack_raw
        )
        mass_key = round(float(candidate["vehicle_mass_kg"]), 9)
        if mass_key not in mass_cache:
            mass_cache[mass_key] = run_nonenergy_events(
                build_mass_isolated_vehicle(base_vehicle, mass_key),
                event_inputs,
            )
        mass_results = project_nonenergy_scores(
            mass_cache[mass_key], baseline_mass_raw
        )
        shared = endurance_and_efficiency(candidate, baseline)
        wide_rows.append(
            _wide_row(
                candidate,
                pack_results,
                mass_results,
                diagnostics,
                shared,
            )
        )
        long_rows.extend(
            _long_rows(candidate, PACK_AWARE_MODE, pack_results, diagnostics)
        )
        long_rows.extend(
            _long_rows(
                candidate,
                MASS_ISOLATED_MODE,
                mass_results,
                pack_diagnostics=None,
            )
        )
        shared_rows.append({**_candidate_metadata(candidate), **shared})
        completed = index + 1
        if progress_every > 0 and (
            completed % progress_every == 0 or completed == len(candidates)
        ):
            print(
                f"Completed {completed}/{len(candidates)} battery "
                "configurations",
                flush=True,
            )

    summary = _rank_columns(pd.DataFrame(wide_rows))
    long = pd.DataFrame(long_rows)
    shared_frame = pd.DataFrame(shared_rows)
    local_sensitivity = local_mass_sensitivity(base_vehicle, event_inputs)
    global_slopes = _global_mass_slopes(summary)
    headlines = _headlines(
        summary, local_sensitivity=local_sensitivity, global_slopes=global_slopes
    )

    summary.to_csv(output_dir / "battery_dynamic_points_summary.csv", index=False)
    long.to_csv(
        output_dir / "battery_dynamic_event_results_long.csv", index=False
    )
    shared_frame.to_csv(
        output_dir / "inherited_endurance_efficiency_scores.csv", index=False
    )
    scoring_inputs = {
        "official_rules_source": OFFICIAL_2026_RULES_URL,
        "rules_version": "Formula SAE Rules 2026 Version 1.0",
        "time_score_rules": {
            slug: rule.as_dict() for slug, rule in EVENT_RULES.items()
        },
        "event_winner_anchor_policy": (
            "For every event and sprint model, the P30B baseline simulation "
            "maps to that event's Tmin and every candidate retains its "
            "simulated time ratio to the same-mode baseline. Endurance uses "
            "the same P30B time-ratio policy."
        ),
        "primary_model": {
            "name": PACK_AWARE_MODE,
            "terminal_power_limit_kw": COMMON_TERMINAL_POWER_LIMIT_KW,
            "soc": "full SOC",
            "soc_interpretation": (
                "Best-case short-event power; SOC and temperature are not "
                "integrated during the sprint."
            ),
            "pack_constraints": (
                "series/parallel voltage, DC resistance, continuous cell "
                "current, inverter bus current, motor limits, and losses"
            ),
        },
        "mass_isolated_model": {
            "name": MASS_ISOLATED_MODE,
            "tractive_curve": "inputs/openlap_vehicle.json",
            "changed_input": "vehicle mass only",
            "subtotal_interpretation": (
                "Only the 300-point sprint subtotal is cleanly mass-isolated. "
                "The explicitly named hybrid totals retain pack-specific "
                "endurance and efficiency."
            ),
        },
        "efficiency_normalization": {
            **EFFICIENCY_INPUTS,
            "description": (
                "Inherited ratio-normalized model from the prior vehicle-mass "
                "study; not a fresh competition energy-meter reconstruction."
            ),
            "formula": (
                "projected_lap=65.192*(candidate_avg_lap/baseline_avg_lap); "
                "projected_kWh_per_lap=(3.263/22)*"
                "(candidate_terminal_energy/baseline_terminal_energy); "
                "EF=(59.649*0.0840)/(projected_lap*projected_kWh_per_lap*0.65); "
                "score=clamp(100*(EF-0.289)/(0.797-0.289),0,100)"
            ),
        },
        "maximum_points": {
            "nonenergy_short_events": 300.0,
            "performance_excluding_efficiency": 575.0,
            "full_dynamic_including_efficiency": 675.0,
        },
        "input_paths": {
            "candidates": str(candidate_csv.resolve()),
            "cells": str(cells_json.resolve()),
            "vehicle": str((root / "inputs" / "openlap_vehicle.json").resolve()),
            "powertrain": str(
                (
                    root / "inputs" / "powertrain_130s5p_p30b_provisional.json"
                ).resolve()
            ),
        },
    }
    (output_dir / "scoring_inputs.json").write_text(
        json.dumps(scoring_inputs, indent=2), encoding="utf-8"
    )
    (output_dir / "headline_summary.json").write_text(
        json.dumps(headlines, indent=2, default=_json_default),
        encoding="utf-8",
    )
    _plot_results(summary, output_dir / "plots")
    _write_report(output_dir, summary, headlines)
    validation = _validation_report(summary, long, len(candidates))
    (output_dir / "validation_report.json").write_text(
        json.dumps(validation, indent=2), encoding="utf-8"
    )
    artifact_manifest = {
        "output_directory": str(output_dir),
        "artifacts": sorted(
            {
                *(
                    str(path.relative_to(output_dir))
                    for path in output_dir.rglob("*")
                    if path.is_file()
                ),
                "artifact_manifest.json",
            }
        ),
    }
    (output_dir / "artifact_manifest.json").write_text(
        json.dumps(artifact_manifest, indent=2), encoding="utf-8"
    )
    return {
        "output_directory": str(output_dir),
        "validation": validation,
        "headlines": headlines,
    }


def _json_default(value: Any) -> Any:
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        if np.isnan(value):
            return None
        return float(value)
    raise TypeError(f"Cannot JSON serialize {type(value).__name__}")
