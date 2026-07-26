"""Chronological multi-lap endurance simulation with battery state."""

from __future__ import annotations

import math
from dataclasses import replace
from typing import Any

import numpy as np
import pandas as pd

from openlap_solver import (
    Vehicle,
    aero_and_loads,
    deceleration,
    ellipse_multiplier,
    lateral_speed_limit,
    load_sensitive_mu,
)
from powertrain_model import MaximumTorqueSurface, PowertrainModel


def braking_speed_envelope(
    vehicle: Vehicle,
    track: pd.DataFrame,
    tolerance: float = 1e-9,
    max_iterations: int = 2000,
) -> np.ndarray:
    """Closed-track lateral and mechanical-braking speed ceiling."""

    dx = track["dx_m"].to_numpy(dtype=float)
    curvature = track["curvature_1pm"].to_numpy(dtype=float)
    count = len(track)
    lateral_limit = np.array(
        [lateral_speed_limit(vehicle, value) for value in curvature],
        dtype=float,
    )
    speed = lateral_limit.copy()
    for _ in range(max_iterations):
        old_speed = speed.copy()
        for index in range(count - 1, -1, -1):
            previous_index = (index - 1) % count
            proposed = speed[index]
            for _ in range(12):
                decel = deceleration(
                    vehicle, curvature[previous_index], proposed
                )
                updated = math.sqrt(
                    max(
                        0.0,
                        speed[index] ** 2
                        + 2.0 * decel * dx[previous_index],
                    )
                )
                updated = min(updated, lateral_limit[previous_index])
                if abs(updated - proposed) < 1e-11:
                    proposed = updated
                    break
                proposed = updated
            speed[previous_index] = min(speed[previous_index], proposed)
        if float(np.max(np.abs(speed - old_speed))) < tolerance:
            break
    return speed


def _traction_force_limit(
    vehicle: Vehicle, curvature: float, speed_mps: float
) -> tuple[float, float, float, float]:
    downforce, drag, total_load = aero_and_loads(vehicle, speed_mps)
    ellipse = ellipse_multiplier(vehicle, curvature, speed_mps, total_load)
    driven_load_per_tire = (
        vehicle.factor_drive * vehicle.mass * 9.81
        + vehicle.factor_aero * downforce
    ) / vehicle.driven_wheels
    mu_x = load_sensitive_mu(
        vehicle.mu_x,
        vehicle.sens_x,
        vehicle.ref_mass_x,
        driven_load_per_tire,
    )
    tyre_force = max(
        0.0,
        mu_x * driven_load_per_tire * vehicle.driven_wheels * ellipse,
    )
    resistance = drag + vehicle.cr * total_load
    return tyre_force, resistance, drag, total_load


def simulate_endurance(
    base_vehicle: Vehicle,
    track: pd.DataFrame,
    powertrain: PowertrainModel,
    laps: int,
    *,
    initial_speed_mps: float = 0.0,
    surface_soc_count: int = 25,
    surface_speed_count: int = 121,
    torque_surface: MaximumTorqueSurface | None = None,
    regen_terminal_power_target_w: float = 0.0,
    regen_active_power_threshold_w: float = 1000.0,
) -> tuple[pd.DataFrame, dict[str, Any]]:
    if laps <= 0:
        raise ValueError("laps must be positive")
    if len(track) == 0 or np.any(track["dx_m"].to_numpy(dtype=float) <= 0.0):
        raise ValueError("Track must contain positive-length segments")
    if regen_terminal_power_target_w < 0.0:
        raise ValueError("Regen terminal-power target must be nonnegative")
    if regen_active_power_threshold_w < 0.0:
        raise ValueError("Regen active-power threshold must be nonnegative")

    mass = base_vehicle.mass + powertrain.config.pack.vehicle_mass_delta_kg()
    vehicle = replace(base_vehicle, mass=mass)
    pack = powertrain.config.pack
    drivetrain = powertrain.config.drivetrain
    full_usable_chemical_energy_wh = pack.chemical_energy_wh(
        pack.minimum_soc, 1.0
    )
    braking_limit = braking_speed_envelope(vehicle, track)
    if torque_surface is None:
        torque_surface = MaximumTorqueSurface(
            powertrain,
            maximum_vehicle_speed_mps=vehicle.v_max,
            soc_count=surface_soc_count,
            speed_count=surface_speed_count,
        )

    dx = track["dx_m"].to_numpy(dtype=float)
    curvature = track["curvature_1pm"].to_numpy(dtype=float)
    distance = track["distance_m"].to_numpy(dtype=float)
    count = len(track)
    soc = pack.initial_soc
    speed = min(max(initial_speed_mps, 0.0), braking_limit[0])
    elapsed_time = 0.0
    cumulative_distance = 0.0
    cumulative_terminal_energy_wh = 0.0
    cumulative_chemical_energy_wh = 0.0
    cumulative_pack_loss_wh = 0.0
    cumulative_motor_loss_wh = 0.0
    cumulative_inverter_loss_wh = 0.0
    cumulative_drivetrain_loss_wh = 0.0
    rows: list[dict[str, Any]] = []
    per_lap: list[dict[str, Any]] = []
    completed = True
    stop_reason = "completed_requested_laps"

    for lap in range(1, laps + 1):
        lap_start_time = elapsed_time
        lap_start_soc = soc
        lap_start_energy = cumulative_terminal_energy_wh
        lap_completed = True
        for index in range(count):
            if soc <= pack.minimum_soc + 1e-12:
                completed = False
                lap_completed = False
                stop_reason = "minimum_soc"
                break

            next_index = (index + 1) % count
            segment_dx = dx[index]
            evaluation_speed = speed
            for _ in range(12):
                tyre_force, resistance, drag, total_load = (
                    _traction_force_limit(
                        vehicle, curvature[index], evaluation_speed
                    )
                )
                motor_rpm = drivetrain.motor_speed_rpm(evaluation_speed)
                available_torque = torque_surface.maximum_torque_nm(
                    evaluation_speed, soc
                )
                powertrain_force = drivetrain.force_for_motor_torque(
                    available_torque
                )
                available_drive_force = min(tyre_force, powertrain_force)
                acceleration_max = (
                    available_drive_force - resistance
                ) / vehicle.mass
                possible_next_speed = math.sqrt(
                    max(
                        0.0,
                        speed**2 + 2.0 * acceleration_max * segment_dx,
                    )
                )
                next_speed = min(
                    possible_next_speed,
                    braking_limit[next_index],
                    vehicle.v_max,
                )
                updated_evaluation_speed = 0.5 * (speed + next_speed)
                if abs(updated_evaluation_speed - evaluation_speed) < 1e-8:
                    evaluation_speed = updated_evaluation_speed
                    break
                evaluation_speed = updated_evaluation_speed

            motor_rpm = drivetrain.motor_speed_rpm(evaluation_speed)
            available_torque = torque_surface.maximum_torque_nm(
                evaluation_speed, soc
            )
            available_point = powertrain.operating_point(
                available_torque,
                motor_rpm,
                soc,
                classify_limiter=True,
            )
            powertrain_force = drivetrain.force_for_motor_torque(
                available_torque
            )
            available_drive_force = min(tyre_force, powertrain_force)
            actual_acceleration = (
                next_speed**2 - speed**2
            ) / (2.0 * segment_dx)
            required_drive_force = (
                vehicle.mass * actual_acceleration + resistance
            )
            used_drive_force = min(
                available_drive_force, max(0.0, required_drive_force)
            )
            braking_force = max(
                0.0, -(vehicle.mass * actual_acceleration + resistance)
            )
            requested_torque = drivetrain.motor_torque_for_force(tyre_force)

            average_speed = 0.5 * (speed + next_speed)
            if average_speed <= 1e-12:
                completed = False
                lap_completed = False
                stop_reason = "vehicle_stalled"
                break
            dt = segment_dx / average_speed
            regenerative_braking_force = 0.0
            mechanical_braking_force = braking_force
            regen_tire_force_limit = min(braking_force, tyre_force)
            used_torque = drivetrain.motor_torque_for_force(used_drive_force)
            if (
                braking_force > 1e-6
                and regen_terminal_power_target_w > 0.0
            ):
                maximum_regen_torque = abs(
                    drivetrain.motor_torque_for_regenerative_force(
                        regen_tire_force_limit
                    )
                )
                charge_headroom_current = (
                    max(1.0 - soc, 0.0)
                    * pack.capacity_ah
                    * 3600.0
                    / dt
                )
                actual_point = (
                    powertrain.regenerative_point_for_terminal_power(
                        regen_terminal_power_target_w,
                        motor_rpm,
                        soc,
                        maximum_regen_torque,
                        maximum_charge_current_a=charge_headroom_current,
                    )
                )
                if actual_point.battery_current_a < -1e-9:
                    used_torque = actual_point.motor_torque_nm
                    regenerative_braking_force = min(
                        braking_force,
                        regen_tire_force_limit,
                        drivetrain.regenerative_force_for_motor_torque(
                            used_torque
                        ),
                    )
                    mechanical_braking_force = max(
                        0.0, braking_force - regenerative_braking_force
                    )
                else:
                    actual_point = powertrain.operating_point(
                        0.0, motor_rpm, soc
                    )
                    used_torque = 0.0
            else:
                actual_point = powertrain.operating_point(
                    used_torque, motor_rpm, soc
                )
            if not actual_point.feasible:
                if used_torque >= 0.0:
                    used_torque = min(used_torque, available_torque) * (
                        1.0 - 1e-8
                    )
                    actual_point = powertrain.operating_point(
                        used_torque, motor_rpm, soc
                    )
                if not actual_point.feasible:
                    raise RuntimeError(
                        "Used powertrain operating point is infeasible"
                    )

            chemical_power_w = (
                actual_point.battery_ocv_v * actual_point.battery_current_a
            )
            charge_delta_ah = actual_point.battery_current_a * dt / 3600.0
            hit_soc_limit = False
            if (
                actual_point.battery_current_a > 0.0
                and charge_delta_ah
                > (soc - pack.minimum_soc) * pack.capacity_ah
            ):
                hit_soc_limit = True
                dt = (
                    (soc - pack.minimum_soc)
                    * pack.capacity_ah
                    * 3600.0
                    / actual_point.battery_current_a
                )
                segment_dx = max(
                    0.0,
                    speed * dt + 0.5 * actual_acceleration * dt**2,
                )
                next_speed = max(0.0, speed + actual_acceleration * dt)
                next_soc = pack.minimum_soc
            else:
                next_soc = float(
                    np.clip(
                        soc - charge_delta_ah / pack.capacity_ah,
                        pack.minimum_soc,
                        1.0,
                    )
                )
            terminal_energy_wh = (
                actual_point.battery_terminal_power_w * dt / 3600.0
            )
            chemical_energy_wh = chemical_power_w * dt / 3600.0

            if braking_force > 1e-6:
                if regen_terminal_power_target_w <= 0.0:
                    active_limiter = "braking_envelope"
                else:
                    active_limiter = (
                        f"braking_envelope+{actual_point.active_limiter}"
                        if regenerative_braking_force > 0.0
                        else "braking_envelope+mechanical"
                    )
            elif tyre_force < powertrain_force - 1e-6:
                active_limiter = "tire_traction"
            else:
                active_limiter = available_point.active_limiter

            elapsed_time += dt
            cumulative_distance += segment_dx
            cumulative_terminal_energy_wh += terminal_energy_wh
            cumulative_chemical_energy_wh += chemical_energy_wh
            cumulative_pack_loss_wh += (
                actual_point.battery_resistive_loss_w * dt / 3600.0
            )
            cumulative_motor_loss_wh += (
                (
                    actual_point.motor_copper_loss_w
                    + actual_point.motor_iron_loss_w
                )
                * dt
                / 3600.0
            )
            cumulative_inverter_loss_wh += (
                actual_point.inverter_loss_w * dt / 3600.0
            )
            if regenerative_braking_force > 0.0:
                drivetrain_loss_w = (
                    regenerative_braking_force
                    * evaluation_speed
                    * (1.0 - drivetrain.mechanical_efficiency)
                )
            else:
                drivetrain_loss_w = (
                    actual_point.motor_shaft_power_w
                    * (1.0 - drivetrain.mechanical_efficiency)
                )
            wheel_power_w = (
                -regenerative_braking_force * evaluation_speed
                if regenerative_braking_force > 0.0
                else used_drive_force * evaluation_speed
            )
            mechanical_braking_power_w = (
                mechanical_braking_force * evaluation_speed
            )
            cumulative_drivetrain_loss_wh += drivetrain_loss_w * dt / 3600.0
            rows.append(
                {
                    "lap": lap,
                    "segment": index,
                    "lap_distance_m": (
                        distance[index] - dx[index] + segment_dx
                    ),
                    "cumulative_distance_m": cumulative_distance,
                    "dx_m": segment_dx,
                    "curvature_1pm": curvature[index],
                    "speed_mps": speed,
                    "next_speed_mps": next_speed,
                    "powertrain_evaluation_speed_mps": evaluation_speed,
                    "braking_speed_limit_mps": braking_limit[index],
                    "time_in_segment_s": dt,
                    "elapsed_time_s": elapsed_time,
                    "longitudinal_accel_mps2": actual_acceleration,
                    "drag_n": drag,
                    "total_normal_load_n": total_load,
                    "requested_motor_torque_nm": requested_torque,
                    "available_motor_torque_nm": available_torque,
                    "used_motor_torque_nm": used_torque,
                    "braking_force_n": braking_force,
                    "regenerative_braking_force_n": (
                        regenerative_braking_force
                    ),
                    "mechanical_braking_force_n": mechanical_braking_force,
                    "regen_tire_force_limit_n": regen_tire_force_limit,
                    "regen_terminal_power_target_w": (
                        regen_terminal_power_target_w
                    ),
                    "wheel_power_w": wheel_power_w,
                    "mechanical_braking_power_w": (
                        mechanical_braking_power_w
                    ),
                    "motor_speed_rpm": motor_rpm,
                    "phase_current_arms": actual_point.phase_current_arms,
                    "id_arms": actual_point.id_arms,
                    "iq_arms": actual_point.iq_arms,
                    "active_limiter": active_limiter,
                    "soc": soc,
                    "next_soc": next_soc,
                    "soe": (
                        pack.chemical_energy_wh(pack.minimum_soc, soc)
                        / full_usable_chemical_energy_wh
                    ),
                    "cell_ocv_v": pack.cell_ocv_v(soc),
                    "battery_ocv_v": actual_point.battery_ocv_v,
                    "battery_terminal_voltage_v": (
                        actual_point.battery_terminal_voltage_v
                    ),
                    "battery_current_a": actual_point.battery_current_a,
                    "battery_terminal_power_w": (
                        actual_point.battery_terminal_power_w
                    ),
                    "battery_terminal_power_from_vi_w": (
                        actual_point.battery_terminal_voltage_v
                        * actual_point.battery_current_a
                    ),
                    "motor_shaft_power_w": actual_point.motor_shaft_power_w,
                    "motor_copper_loss_w": actual_point.motor_copper_loss_w,
                    "motor_iron_loss_w": actual_point.motor_iron_loss_w,
                    "inverter_loss_w": actual_point.inverter_loss_w,
                    "drivetrain_loss_w": drivetrain_loss_w,
                    "pack_resistive_loss_w": (
                        actual_point.battery_resistive_loss_w
                    ),
                    "cumulative_terminal_energy_wh": (
                        cumulative_terminal_energy_wh
                    ),
                    "cumulative_chemical_energy_wh": (
                        cumulative_chemical_energy_wh
                    ),
                }
            )
            speed = next_speed
            soc = next_soc
            if hit_soc_limit:
                completed = False
                lap_completed = False
                stop_reason = "minimum_soc"
                break

        per_lap.append(
            {
                "lap": lap,
                "completed": lap_completed,
                "lap_time_s": elapsed_time - lap_start_time,
                "start_soc": lap_start_soc,
                "end_soc": soc,
                "terminal_energy_wh": (
                    cumulative_terminal_energy_wh - lap_start_energy
                ),
            }
        )
        if not completed:
            break

    trace = pd.DataFrame(rows)
    if trace.empty:
        raise RuntimeError("Endurance simulation produced no samples")
    energy_balance_error_wh = (
        cumulative_chemical_energy_wh
        - cumulative_terminal_energy_wh
        - cumulative_pack_loss_wh
    )
    shaft_energy_wh = float(
        np.sum(
            trace["motor_shaft_power_w"]
            * trace["time_in_segment_s"]
            / 3600.0
        )
    )
    chain_balance_error_wh = (
        cumulative_terminal_energy_wh
        - shaft_energy_wh
        - cumulative_motor_loss_wh
        - cumulative_inverter_loss_wh
    )
    trace_dt = trace["time_in_segment_s"].to_numpy(dtype=float)
    terminal_power_values = trace["battery_terminal_power_w"].to_numpy(
        dtype=float
    )
    chemical_power_values = (
        trace["battery_ocv_v"].to_numpy(dtype=float)
        * trace["battery_current_a"].to_numpy(dtype=float)
    )
    regen_power_values = np.maximum(-terminal_power_values, 0.0)
    discharge_power_values = np.maximum(terminal_power_values, 0.0)
    regen_active = (
        regen_power_values > regen_active_power_threshold_w
    )
    charge_active = (
        trace["battery_current_a"].to_numpy(dtype=float) < -1e-9
    )
    terminal_discharge_energy_wh = float(
        np.sum(discharge_power_values * trace_dt) / 3600.0
    )
    terminal_regenerated_energy_wh = float(
        np.sum(regen_power_values * trace_dt) / 3600.0
    )
    chemical_discharge_energy_wh = float(
        np.sum(np.maximum(chemical_power_values, 0.0) * trace_dt) / 3600.0
    )
    chemical_regenerated_energy_wh = float(
        np.sum(np.maximum(-chemical_power_values, 0.0) * trace_dt) / 3600.0
    )
    regen_pack_loss_wh = float(
        np.sum(
            trace.loc[charge_active, "pack_resistive_loss_w"].to_numpy(
                dtype=float
            )
            * trace_dt[charge_active]
        )
        / 3600.0
    )
    regen_active_duration_s = float(trace_dt[regen_active].sum())
    regen_active_rms_power_w = (
        float(
            math.sqrt(
                np.sum(regen_power_values[regen_active] ** 2 * trace_dt[regen_active])
                / regen_active_duration_s
            )
        )
        if regen_active_duration_s > 0.0
        else 0.0
    )
    regen_whole_event_rms_power_w = float(
        math.sqrt(
            np.sum(regen_power_values**2 * trace_dt)
            / max(float(trace_dt.sum()), 1e-12)
        )
    )
    regenerative_wheel_energy_wh = float(
        np.sum(
            np.maximum(-trace["wheel_power_w"].to_numpy(dtype=float), 0.0)
            * trace_dt
        )
        / 3600.0
    )
    mechanical_braking_energy_wh = float(
        np.sum(
            trace["mechanical_braking_power_w"].to_numpy(dtype=float)
            * trace_dt
        )
        / 3600.0
    )
    braking_force_balance_residual = float(
        np.max(
            np.abs(
                trace["braking_force_n"].to_numpy(dtype=float)
                - trace["regenerative_braking_force_n"].to_numpy(dtype=float)
                - trace["mechanical_braking_force_n"].to_numpy(dtype=float)
            )
        )
    )
    expected_next_soc = (
        trace["soc"].to_numpy(dtype=float)
        - trace["battery_current_a"].to_numpy(dtype=float)
        * trace_dt
        / (3600.0 * pack.capacity_ah)
    )
    soc_transition_residual = float(
        np.max(
            np.abs(
                trace["next_soc"].to_numpy(dtype=float)
                - expected_next_soc
            )
        )
    )
    maximum_terminal_power = float(
        trace["battery_terminal_power_from_vi_w"].max()
    )
    maximum_terminal_power_residual = float(
        np.max(
            np.abs(
                trace["battery_terminal_power_w"]
                - trace["battery_terminal_power_from_vi_w"]
            )
        )
    )
    limiter_counts = {
        str(name): int(count)
        for name, count in trace["active_limiter"].value_counts().items()
    }
    summary: dict[str, Any] = {
        "solver": (
            "Chronological OpenLAP point-mass endurance with coupled "
            "accumulator/inverter/EMRAX torque limit"
        ),
        "completed": completed,
        "stop_reason": stop_reason,
        "requested_laps": laps,
        "completed_laps": sum(lap["completed"] for lap in per_lap),
        "segments_simulated": len(trace),
        "distance_m": cumulative_distance,
        "elapsed_time_s": elapsed_time,
        "vehicle_mass_kg": mass,
        "pack_series_cells": pack.series_cells,
        "pack_parallel_cells": pack.parallel_cells,
        "pack_capacity_ah": pack.capacity_ah,
        "pack_resistance_at_initial_soc_ohm": (
            pack.pack_resistance_ohm(pack.initial_soc)
        ),
        "initial_soc": pack.initial_soc,
        "final_soc": soc,
        "initial_soe": pack.usable_soe(pack.initial_soc),
        "final_soe": pack.usable_soe(soc),
        "terminal_energy_kwh": cumulative_terminal_energy_wh / 1000.0,
        "terminal_discharge_energy_kwh": (
            terminal_discharge_energy_wh / 1000.0
        ),
        "terminal_regenerated_energy_kwh": (
            terminal_regenerated_energy_wh / 1000.0
        ),
        "chemical_energy_kwh": cumulative_chemical_energy_wh / 1000.0,
        "chemical_discharge_energy_kwh": (
            chemical_discharge_energy_wh / 1000.0
        ),
        "chemical_regenerated_energy_kwh": (
            chemical_regenerated_energy_wh / 1000.0
        ),
        "pack_resistive_loss_kwh": cumulative_pack_loss_wh / 1000.0,
        "regen_pack_resistive_loss_kwh": regen_pack_loss_wh / 1000.0,
        "motor_loss_kwh": cumulative_motor_loss_wh / 1000.0,
        "inverter_loss_kwh": cumulative_inverter_loss_wh / 1000.0,
        "drivetrain_loss_kwh": cumulative_drivetrain_loss_wh / 1000.0,
        "maximum_terminal_power_w": maximum_terminal_power,
        "minimum_terminal_power_w": float(terminal_power_values.min()),
        "regen_terminal_power_target_w": regen_terminal_power_target_w,
        "regen_active_power_threshold_w": (
            regen_active_power_threshold_w
        ),
        "regen_active_duration_s": regen_active_duration_s,
        "regen_active_duty_fraction": (
            regen_active_duration_s / max(elapsed_time, 1e-12)
        ),
        "regen_active_rms_terminal_power_w": regen_active_rms_power_w,
        "regen_whole_event_rms_terminal_power_w": (
            regen_whole_event_rms_power_w
        ),
        "peak_regen_terminal_power_w": float(regen_power_values.max()),
        "regenerative_wheel_energy_kwh": (
            regenerative_wheel_energy_wh / 1000.0
        ),
        "mechanical_braking_energy_kwh": (
            mechanical_braking_energy_wh / 1000.0
        ),
        "maximum_terminal_power_residual_w": (
            maximum_terminal_power_residual
        ),
        "minimum_terminal_voltage_v": float(
            trace["battery_terminal_voltage_v"].min()
        ),
        "maximum_terminal_voltage_v": float(
            trace["battery_terminal_voltage_v"].max()
        ),
        "maximum_bus_current_a": float(trace["battery_current_a"].max()),
        "maximum_charge_current_a": float(
            np.maximum(-trace["battery_current_a"].to_numpy(dtype=float), 0.0).max()
        ),
        "checks": {
            "terminal_power_never_exceeds_limit": (
                maximum_terminal_power
                <= powertrain.config.terminal_power_limit_w + 1e-4
            ),
            "terminal_power_matches_voltage_times_current": (
                maximum_terminal_power_residual <= 1e-5
            ),
            "soc_monotonic_nonincreasing": bool(
                np.all(np.diff(trace["soc"].to_numpy(dtype=float)) <= 1e-12)
            ),
            "soc_within_bounds": bool(
                (
                    trace[["soc", "next_soc"]].to_numpy(dtype=float)
                    >= pack.minimum_soc - 1e-10
                ).all()
                and (
                    trace[["soc", "next_soc"]].to_numpy(dtype=float)
                    <= 1.0 + 1e-10
                ).all()
            ),
            "soc_transition_maximum_residual": soc_transition_residual,
            "braking_force_split_maximum_residual_n": (
                braking_force_balance_residual
            ),
            "regenerative_force_never_exceeds_braking_force": bool(
                (
                    trace["regenerative_braking_force_n"]
                    <= trace["braking_force_n"] + 1e-8
                ).all()
            ),
            "regen_only_during_braking": bool(
                (
                    trace.loc[
                        trace["battery_current_a"] < -1e-9,
                        "braking_force_n",
                    ]
                    > 1e-6
                ).all()
            ),
            "terminal_net_equals_discharge_minus_regenerated": bool(
                abs(
                    cumulative_terminal_energy_wh
                    - terminal_discharge_energy_wh
                    + terminal_regenerated_energy_wh
                )
                <= 1e-8 * max(terminal_discharge_energy_wh, 1.0)
            ),
            "maximum_terminal_voltage_respected": bool(
                trace["battery_terminal_voltage_v"].max()
                <= pack.maximum_terminal_voltage_v + 1e-6
            ),
            "regen_target_not_exceeded": bool(
                regen_terminal_power_target_w <= 0.0
                or regen_power_values.max()
                <= regen_terminal_power_target_w + 1e-4
            ),
            "pack_energy_balance_error_wh": energy_balance_error_wh,
            "pack_energy_balance_relative": (
                energy_balance_error_wh
                / max(abs(cumulative_chemical_energy_wh), 1e-12)
            ),
            "powertrain_chain_balance_error_wh": chain_balance_error_wh,
            "powertrain_chain_balance_relative": (
                chain_balance_error_wh
                / max(abs(cumulative_terminal_energy_wh), 1e-12)
            ),
        },
        "active_limiter_counts": limiter_counts,
        "per_lap": per_lap,
        "limitations": [
            (
                "Regeneration is disabled; braking is mechanical only."
                if regen_terminal_power_target_w <= 0.0
                else (
                    "Endurance-only regen uses a constant active battery-"
                    "terminal target derived from telemetry RMS; source, "
                    "motor/inverter, pack-voltage, inverter-current, tire-force, "
                    "and SOC-headroom limits can reduce achieved power."
                )
            ),
            (
                "No cell-specific charge-current limits are available in the "
                "candidate source file; the measured terminal-power target, "
                "inverter limit, maximum cell voltage, and SOC headroom govern "
                "charge acceptance."
            ),
            "Cell and motor temperatures are fixed configuration inputs.",
            "No driver-change stop or thermal derating.",
            (
                "Default OCV/resistance and inverter-loss values are "
                "provisional until replaced by the validated battery "
                "architecture model."
            ),
        ],
    }
    return trace, summary
