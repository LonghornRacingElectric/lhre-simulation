"""Battery-pack generation and exact-distance endurance metrics.

This module keeps the trade-study policy separate from the chronological
OpenLAP solver. Candidate packs share one vehicle, motor, inverter, cooling
assumption, and track; only cell data and series/parallel topology change.
"""

from __future__ import annotations

import math
from dataclasses import replace
from typing import Any, Iterable

import numpy as np
import pandas as pd

from powertrain_model import Curve, PowertrainConfig


BASELINE_SERIES_CELLS = 130
BASELINE_PARALLEL_CELLS = 5
BASELINE_CELL_MASS_KG = 0.047
BASELINE_CELL_DIAMETER_M = 0.0186
BASELINE_CELL_HEIGHT_M = 0.0652
PACK_MASS_MULTIPLIER = 1.25
MAXIMUM_PACK_VOLTAGE_V = 600.0
MAXIMUM_USABLE_ENERGY_KWH = 8.0
MINIMUM_PACK_CAPACITY_AH = 9.0
CELL_SPECIFIC_HEAT_J_PER_KGK = 1000.0


def cylindrical_cell_volume_m3(diameter_m: float, height_m: float) -> float:
    return math.pi * (diameter_m / 2.0) ** 2 * height_m


BASELINE_CELL_ENVELOPE_M3 = (
    BASELINE_SERIES_CELLS
    * BASELINE_PARALLEL_CELLS
    * cylindrical_cell_volume_m3(
        BASELINE_CELL_DIAMETER_M, BASELINE_CELL_HEIGHT_M
    )
)


def assumed_ocv_curve(
    minimum_voltage_v: float, maximum_voltage_v: float
) -> Curve:
    """Return one shared normalized NMC OCV shape.

    Public cell sheets in the shortlist do not publish machine-readable OCV
    maps on a common test basis. The shape is therefore held fixed so the
    study compares topology, resistance, mass, and current capability without
    inventing cell-specific discharge-curve detail.
    """

    normalized_pairs = [
        [0.0, 0.0],
        [0.05, 0.4],
        [0.1, 0.4823529411764706],
        [0.2, 0.5588235294117647],
        [0.3, 0.6176470588235294],
        [0.4, 0.6588235294117647],
        [0.5, 0.7058823529411765],
        [0.6, 0.7529411764705882],
        [0.7, 0.7941176470588235],
        [0.8, 0.8411764705882353],
        [0.9, 0.8941176470588236],
        [1.0, 1.0],
    ]
    span = maximum_voltage_v - minimum_voltage_v
    return Curve.from_pairs(
        [
            [soc, minimum_voltage_v + normalized_voltage * span]
            for soc, normalized_voltage in normalized_pairs
        ]
    )


def assumed_resistance_curve(dcir_ohm: float) -> Curve:
    """Apply one common SOC multiplier to the room-temperature DCIR value."""

    return Curve.from_pairs(
        [
            [0.0, 1.35 * dcir_ohm],
            [0.05, 1.25 * dcir_ohm],
            [0.2, 1.10 * dcir_ohm],
            [0.5, dcir_ohm],
            [1.0, 1.05 * dcir_ohm],
        ]
    )


def usable_cell_energy_wh(cell: dict[str, Any]) -> float:
    curve = assumed_ocv_curve(
        float(cell["minimum_voltage_v"]), float(cell["maximum_voltage_v"])
    )
    return float(cell["capacity_ah"]) * curve.integral(0.05, 1.0)


def generate_candidates(
    cells: Iterable[dict[str, Any]],
    series_counts: Iterable[int],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Generate topology candidates and record every rejected topology."""

    accepted: list[dict[str, Any]] = []
    rejected: list[dict[str, Any]] = []
    for cell in cells:
        cell_volume = cylindrical_cell_volume_m3(
            float(cell["diameter_m"]), float(cell["height_m"])
        )
        minimum_parallel = max(
            1,
            int(
                math.ceil(
                    MINIMUM_PACK_CAPACITY_AH / float(cell["capacity_ah"])
                )
            ),
        )
        for series in series_counts:
            for parallel in range(minimum_parallel, 7):
                total_cells = int(series * parallel)
                maximum_voltage = (
                    float(cell["maximum_voltage_v"]) * series
                )
                nominal_energy = (
                    series
                    * parallel
                    * float(cell["capacity_ah"])
                    * float(cell["nominal_voltage_v"])
                    / 1000.0
                )
                usable_energy = (
                    series
                    * parallel
                    * usable_cell_energy_wh(cell)
                    / 1000.0
                )
                envelope = total_cells * cell_volume
                reasons: list[str] = []
                if maximum_voltage >= MAXIMUM_PACK_VOLTAGE_V:
                    reasons.append("maximum_voltage_not_below_600v")
                if usable_energy > MAXIMUM_USABLE_ENERGY_KWH + 1e-12:
                    reasons.append("usable_energy_above_8kwh")
                if envelope > BASELINE_CELL_ENVELOPE_M3 + 1e-12:
                    reasons.append("cell_envelope_above_130s5p_p30b")
                row = {
                    "candidate_id": (
                        f"{cell['cell_id']}_{series}s{parallel}p"
                    ),
                    "cell_id": cell["cell_id"],
                    "manufacturer": cell["manufacturer"],
                    "cell_model": cell["model"],
                    "form_factor": cell["form_factor"],
                    "series_cells": int(series),
                    "parallel_cells": int(parallel),
                    "total_cells": total_cells,
                    "maximum_pack_voltage_v": maximum_voltage,
                    "nominal_pack_voltage_v": (
                        float(cell["nominal_voltage_v"]) * series
                    ),
                    "nominal_pack_energy_kwh": nominal_energy,
                    "model_usable_energy_kwh": usable_energy,
                    "cell_envelope_l": envelope * 1000.0,
                    "cell_envelope_vs_baseline_pct": (
                        envelope / BASELINE_CELL_ENVELOPE_M3 * 100.0
                    ),
                    "cell_mass_kg": float(cell["mass_kg"]),
                    "cell_capacity_ah": float(cell["capacity_ah"]),
                    "cell_dcir_mohm": float(cell["dcir_ohm"]) * 1000.0,
                    "cell_continuous_current_a": float(
                        cell["continuous_current_a"]
                    ),
                    "pack_current_limit_a": (
                        parallel * float(cell["continuous_current_a"])
                    ),
                    "pack_mass_kg": (
                        total_cells
                        * float(cell["mass_kg"])
                        * PACK_MASS_MULTIPLIER
                    ),
                    "source_url": cell["source_url"],
                    "dcir_basis": cell["dcir_basis"],
                }
                if reasons:
                    rejected.append({**row, "rejection_reasons": ";".join(reasons)})
                else:
                    accepted.append(row)
    return accepted, rejected


def build_powertrain_config(
    base_config: PowertrainConfig,
    cell: dict[str, Any],
    series_cells: int,
    parallel_cells: int,
    terminal_power_limit_kw: float,
) -> PowertrainConfig:
    """Create one pack while retaining all non-pack powertrain settings."""

    pack = replace(
        base_config.pack,
        series_cells=int(series_cells),
        parallel_cells=int(parallel_cells),
        base_series_cells=BASELINE_SERIES_CELLS,
        base_parallel_cells=BASELINE_PARALLEL_CELLS,
        cell_capacity_ah=float(cell["capacity_ah"]),
        cell_mass_kg=float(cell["mass_kg"]),
        base_cell_mass_kg=BASELINE_CELL_MASS_KG,
        pack_mass_multiplier=PACK_MASS_MULTIPLIER,
        minimum_cell_voltage_v=float(cell["minimum_voltage_v"]),
        maximum_cell_voltage_v=float(cell["maximum_voltage_v"]),
        maximum_cell_discharge_current_a=float(
            cell["continuous_current_a"]
        ),
        maximum_cell_charge_current_a=(
            float(cell["maximum_charge_current_a"])
            if cell.get("maximum_charge_current_a") is not None
            else None
        ),
        ocv_v=assumed_ocv_curve(
            float(cell["minimum_voltage_v"]),
            float(cell["maximum_voltage_v"]),
        ),
        resistance_ohm=assumed_resistance_curve(float(cell["dcir_ohm"])),
    )
    config = replace(
        base_config,
        pack=pack,
        terminal_power_limit_w=float(terminal_power_limit_kw) * 1000.0,
    )
    config.validate()
    return config


def downsample_track(track: pd.DataFrame, factor: int) -> pd.DataFrame:
    if factor <= 1:
        return track.copy()
    rows: list[dict[str, float]] = []
    cumulative_distance = 0.0
    for start in range(0, len(track), factor):
        group = track.iloc[start : start + factor]
        dx = group["dx_m"].to_numpy(dtype=float)
        group_dx = float(dx.sum())
        cumulative_distance += group_dx
        rows.append(
            {
                "distance_m": cumulative_distance,
                "dx_m": group_dx,
                "curvature_1pm": float(
                    np.average(
                        group["curvature_1pm"].to_numpy(dtype=float),
                        weights=dx,
                    )
                ),
            }
        )
    return pd.DataFrame(rows)


def _partial_segment_time_s(row: pd.Series, partial_dx_m: float) -> float:
    start_speed = float(row["speed_mps"])
    acceleration = float(row["longitudinal_accel_mps2"])
    end_speed = math.sqrt(
        max(0.0, start_speed**2 + 2.0 * acceleration * partial_dx_m)
    )
    denominator = start_speed + end_speed
    if denominator > 1e-12:
        return 2.0 * partial_dx_m / denominator
    full_dx = float(row["dx_m"])
    return float(row["time_in_segment_s"]) * partial_dx_m / max(
        full_dx, 1e-12
    )


def truncate_trace_at_distance(
    trace: pd.DataFrame, target_distance_m: float
) -> pd.DataFrame:
    """Return a trace whose final row ends at the exact requested distance."""

    if trace.empty:
        raise ValueError("Cannot truncate an empty trace")
    distances = trace["cumulative_distance_m"].to_numpy(dtype=float)
    if distances[-1] < target_distance_m - 1e-9:
        raise ValueError("Trace does not reach the requested distance")
    index = int(np.searchsorted(distances, target_distance_m, side="left"))
    truncated = trace.iloc[: index + 1].copy().reset_index(drop=True)
    row = truncated.iloc[-1].copy()
    end_distance = float(row["cumulative_distance_m"])
    full_dx = float(row["dx_m"])
    start_distance = end_distance - full_dx
    partial_dx = target_distance_m - start_distance
    if abs(partial_dx - full_dx) <= 1e-9:
        truncated.loc[len(truncated) - 1, "cumulative_distance_m"] = (
            target_distance_m
        )
        return truncated

    partial_dt = _partial_segment_time_s(row, partial_dx)
    full_dt = float(row["time_in_segment_s"])
    time_fraction = float(np.clip(partial_dt / max(full_dt, 1e-12), 0.0, 1.0))
    start_elapsed = float(row["elapsed_time_s"]) - full_dt
    start_lap_distance = float(row["lap_distance_m"]) - full_dx
    start_speed = float(row["speed_mps"])
    acceleration = float(row["longitudinal_accel_mps2"])
    partial_end_speed = math.sqrt(
        max(0.0, start_speed**2 + 2.0 * acceleration * partial_dx)
    )
    last = len(truncated) - 1
    truncated.loc[last, "dx_m"] = partial_dx
    truncated.loc[last, "lap_distance_m"] = start_lap_distance + partial_dx
    truncated.loc[last, "cumulative_distance_m"] = target_distance_m
    truncated.loc[last, "time_in_segment_s"] = partial_dt
    truncated.loc[last, "elapsed_time_s"] = start_elapsed + partial_dt
    truncated.loc[last, "next_speed_mps"] = partial_end_speed
    truncated.loc[last, "next_soc"] = float(row["soc"]) + time_fraction * (
        float(row["next_soc"]) - float(row["soc"])
    )

    for cumulative_column, power_column in (
        ("cumulative_terminal_energy_wh", "battery_terminal_power_w"),
        (
            "cumulative_chemical_energy_wh",
            "battery_terminal_power_w",
        ),
    ):
        end_value = float(row[cumulative_column])
        if cumulative_column == "cumulative_chemical_energy_wh":
            segment_power = (
                float(row["battery_ocv_v"])
                * float(row["battery_current_a"])
            )
        else:
            segment_power = float(row[power_column])
        start_value = end_value - segment_power * full_dt / 3600.0
        truncated.loc[last, cumulative_column] = (
            start_value + segment_power * partial_dt / 3600.0
        )
    return truncated


def aggregate_exact_distance(
    trace: pd.DataFrame,
    target_distance_m: float,
    lap_length_m: float,
    config: PowertrainConfig,
    *,
    specific_heat_j_per_kgk: float = CELL_SPECIFIC_HEAT_J_PER_KGK,
    regen_active_power_threshold_kw: float = 1.0,
) -> tuple[pd.DataFrame, dict[str, Any]]:
    """Compute all trade-study metrics from an exact-distance trace."""

    exact = truncate_trace_at_distance(trace, target_distance_m)
    dt = exact["time_in_segment_s"].to_numpy(dtype=float)
    duration = float(dt.sum())

    def energy_kwh(power_column: str) -> float:
        return float(
            np.sum(exact[power_column].to_numpy(dtype=float) * dt)
            / 3.6e6
        )

    chemical_power = (
        exact["battery_ocv_v"].to_numpy(dtype=float)
        * exact["battery_current_a"].to_numpy(dtype=float)
    )
    terminal_power = exact["battery_terminal_power_w"].to_numpy(dtype=float)
    battery_current = exact["battery_current_a"].to_numpy(dtype=float)
    terminal_discharge_power = np.maximum(terminal_power, 0.0)
    terminal_regen_power = np.maximum(-terminal_power, 0.0)
    terminal_energy = float(np.sum(terminal_power * dt) / 3.6e6)
    terminal_discharge_energy = float(
        np.sum(terminal_discharge_power * dt) / 3.6e6
    )
    terminal_regenerated_energy = float(
        np.sum(terminal_regen_power * dt) / 3.6e6
    )
    chemical_energy = float(np.sum(chemical_power * dt) / 3.6e6)
    chemical_discharge_energy = float(
        np.sum(np.maximum(chemical_power, 0.0) * dt) / 3.6e6
    )
    chemical_regenerated_energy = float(
        np.sum(np.maximum(-chemical_power, 0.0) * dt) / 3.6e6
    )
    shaft_energy = energy_kwh("motor_shaft_power_w")
    shaft_discharge_energy = float(
        np.sum(
            np.maximum(
                exact["motor_shaft_power_w"].to_numpy(dtype=float), 0.0
            )
            * dt
        )
        / 3.6e6
    )
    pack_heat = energy_kwh("pack_resistive_loss_w")
    regenerative_wheel_energy = (
        float(
            np.sum(
                np.maximum(
                    -exact["wheel_power_w"].to_numpy(dtype=float), 0.0
                )
                * dt
            )
            / 3.6e6
        )
        if "wheel_power_w" in exact
        else 0.0
    )
    mechanical_braking_energy = (
        energy_kwh("mechanical_braking_power_w")
        if "mechanical_braking_power_w" in exact
        else 0.0
    )
    if regen_active_power_threshold_kw < 0.0:
        raise ValueError("Regen active-power threshold must be nonnegative")
    regen_active = (
        terminal_regen_power > regen_active_power_threshold_kw * 1000.0
    )
    charge_active = battery_current < -1e-9
    regen_active_duration = float(dt[regen_active].sum())
    regen_pack_heat = float(
        np.sum(
            exact.loc[charge_active, "pack_resistive_loss_w"].to_numpy(
                dtype=float
            )
            * dt[charge_active]
        )
        / 3.6e6
    )
    regen_active_rms_power = (
        float(
            math.sqrt(
                np.sum(terminal_regen_power[regen_active] ** 2 * dt[regen_active])
                / regen_active_duration
            )
        )
        if regen_active_duration > 0.0
        else 0.0
    )
    regen_whole_event_rms_power = float(
        math.sqrt(
            np.sum(terminal_regen_power**2 * dt) / max(duration, 1e-12)
        )
    )
    cell_current = (
        battery_current / config.pack.parallel_cells
    )
    cell_mass_total = (
        config.pack.series_cells
        * config.pack.parallel_cells
        * config.pack.cell_mass_kg
    )
    temperature_rise = (
        pack_heat * 3.6e6
        / max(cell_mass_total * specific_heat_j_per_kgk, 1e-12)
    )
    final_soc = float(exact["next_soc"].iloc[-1])
    completed_lap_times: list[float] = []
    for _, lap_frame in exact.groupby("lap", sort=True):
        if (
            float(lap_frame["lap_distance_m"].max())
            >= lap_length_m - 1e-6
        ):
            completed_lap_times.append(
                float(lap_frame["time_in_segment_s"].sum())
            )
    equivalent_laps = target_distance_m / lap_length_m
    metrics = {
        "completed_target_distance": True,
        "target_distance_m": target_distance_m,
        "elapsed_time_s": duration,
        "equivalent_average_lap_time_s": duration / equivalent_laps,
        "fastest_completed_lap_time_s": (
            min(completed_lap_times) if completed_lap_times else math.nan
        ),
        "slowest_completed_lap_time_s": (
            max(completed_lap_times) if completed_lap_times else math.nan
        ),
        "completed_full_laps_before_target": len(completed_lap_times),
        "final_soc": final_soc,
        "remaining_usable_chemical_kwh": (
            config.pack.chemical_energy_wh(
                config.pack.minimum_soc, final_soc
            )
            / 1000.0
        ),
        "terminal_energy_kwh": terminal_energy,
        "terminal_discharge_energy_kwh": terminal_discharge_energy,
        "terminal_regenerated_energy_kwh": terminal_regenerated_energy,
        "chemical_energy_kwh": chemical_energy,
        "chemical_discharge_energy_kwh": chemical_discharge_energy,
        "chemical_regenerated_energy_kwh": (
            chemical_regenerated_energy
        ),
        "mechanical_energy_kwh": shaft_energy,
        "positive_motor_shaft_energy_kwh": shaft_discharge_energy,
        "regenerative_wheel_energy_kwh": regenerative_wheel_energy,
        "mechanical_braking_energy_kwh": mechanical_braking_energy,
        "pack_resistive_heat_kwh": pack_heat,
        "regen_pack_resistive_heat_kwh": regen_pack_heat,
        "motor_loss_kwh": (
            energy_kwh("motor_copper_loss_w")
            + energy_kwh("motor_iron_loss_w")
        ),
        "inverter_loss_kwh": energy_kwh("inverter_loss_w"),
        "drivetrain_loss_kwh": energy_kwh("drivetrain_loss_w"),
        "average_battery_terminal_power_kw": (
            terminal_energy * 3600.0 / duration
        ),
        "peak_battery_terminal_power_kw": (
            float(terminal_power.max()) / 1000.0
        ),
        "peak_regen_terminal_power_kw": (
            float(terminal_regen_power.max()) / 1000.0
        ),
        "regen_active_rms_terminal_power_kw": (
            regen_active_rms_power / 1000.0
        ),
        "regen_whole_event_rms_terminal_power_kw": (
            regen_whole_event_rms_power / 1000.0
        ),
        "regen_active_duration_s": regen_active_duration,
        "regen_active_power_threshold_kw": (
            regen_active_power_threshold_kw
        ),
        "regen_active_duty_fraction": (
            regen_active_duration / max(duration, 1e-12)
        ),
        "average_motor_shaft_power_kw": (
            shaft_energy * 3600.0 / duration
        ),
        "peak_motor_shaft_power_kw": (
            float(exact["motor_shaft_power_w"].max()) / 1000.0
        ),
        "peak_pack_current_a": float(exact["battery_current_a"].max()),
        "peak_pack_charge_current_a": float(
            np.maximum(-battery_current, 0.0).max()
        ),
        "peak_cell_current_a": float(np.max(cell_current)),
        "peak_cell_charge_current_a": float(
            np.maximum(-cell_current, 0.0).max()
        ),
        "rms_cell_current_a": float(
            math.sqrt(np.sum(cell_current**2 * dt) / duration)
        ),
        "minimum_terminal_voltage_v": float(
            exact["battery_terminal_voltage_v"].min()
        ),
        "maximum_terminal_voltage_v": float(
            exact["battery_terminal_voltage_v"].max()
        ),
        "battery_discharge_efficiency_pct": (
            100.0
            * terminal_discharge_energy
            / max(chemical_discharge_energy, 1e-12)
        ),
        "battery_charge_storage_efficiency_pct": (
            100.0
            * chemical_regenerated_energy
            / max(terminal_regenerated_energy, 1e-12)
            if terminal_regenerated_energy > 0.0
            else math.nan
        ),
        "chemical_to_motor_efficiency_pct": (
            100.0
            * shaft_discharge_energy
            / max(chemical_discharge_energy, 1e-12)
        ),
        "adiabatic_cell_temperature_rise_c": temperature_rise,
    }
    return exact, metrics


def pareto_frontier_mask(
    frame: pd.DataFrame,
    columns: tuple[str, str, str] = (
        "elapsed_time_s",
        "pack_mass_kg",
        "pack_resistive_heat_kwh",
    ),
) -> pd.Series:
    """Return non-dominated rows for three objectives that are minimized."""

    values = frame.loc[:, list(columns)].to_numpy(dtype=float)
    mask = np.ones(len(frame), dtype=bool)
    for index, value in enumerate(values):
        if not np.all(np.isfinite(value)):
            mask[index] = False
            continue
        dominated = np.all(values <= value, axis=1) & np.any(
            values < value, axis=1
        )
        dominated[index] = False
        if bool(np.any(dominated)):
            mask[index] = False
    return pd.Series(mask, index=frame.index)


def fit_linear_sensitivities(frame: pd.DataFrame) -> dict[str, float]:
    """Fit time against pack energy, mass, and initial pack resistance."""

    clean = frame.dropna(
        subset=[
            "elapsed_time_s",
            "model_usable_energy_kwh",
            "pack_mass_kg",
            "pack_resistance_initial_mohm",
        ]
    )
    if len(clean) < 5:
        return {}
    matrix = np.column_stack(
        [
            np.ones(len(clean)),
            clean["model_usable_energy_kwh"].to_numpy(dtype=float),
            clean["pack_mass_kg"].to_numpy(dtype=float),
            clean["pack_resistance_initial_mohm"].to_numpy(dtype=float),
        ]
    )
    target = clean["elapsed_time_s"].to_numpy(dtype=float)
    coefficients, *_ = np.linalg.lstsq(matrix, target, rcond=None)
    prediction = matrix @ coefficients
    residual = float(np.sum((target - prediction) ** 2))
    total = float(np.sum((target - np.mean(target)) ** 2))
    return {
        "seconds_per_usable_kwh": float(coefficients[1]),
        "seconds_per_pack_kg": float(coefficients[2]),
        "seconds_per_pack_mohm": float(coefficients[3]),
        "seconds_per_10_pack_mohm": float(coefficients[3] * 10.0),
        "r_squared": 1.0 - residual / total if total > 0.0 else math.nan,
        "sample_count": int(len(clean)),
    }
