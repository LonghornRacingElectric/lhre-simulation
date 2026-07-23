"""Headless Python execution port of the OpenLAP point-mass equations.

The equations and sign conventions follow OpenLAP.m and OpenVEHICLE.m from
Michael Halkiopoulos's OpenLAP Lap Time Simulator. The velocity-envelope
iteration is an equivalent forward/backward implementation suitable for a
closed course and makes the model executable without a MATLAB installation.

SPDX-License-Identifier: GPL-3.0-or-later
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd


G = 9.81


@dataclass(frozen=True)
class Vehicle:
    mass: float
    rho: float
    cl: float
    cd: float
    area: float
    aero_factor: float
    cr: float
    mu_x: float
    mu_y: float
    ref_mass_x: float
    ref_mass_y: float
    sens_x: float
    sens_y: float
    factor_drive: float
    factor_aero: float
    driven_wheels: int
    power_factor: float
    v_max: float
    vehicle_speed: np.ndarray
    fx_engine: np.ndarray


def load_vehicle(path: Path) -> Vehicle:
    data = json.loads(path.read_text(encoding="utf-8"))
    return Vehicle(
        mass=float(data["mass_kg"]),
        rho=float(data["air_density_kg_m3"]),
        cl=float(data["cl_downforce_positive"]),
        cd=float(data["cd_drag_positive"]),
        area=float(data["reference_area_m2"]),
        aero_factor=float(data["aero_factor"]),
        cr=float(data["rolling_resistance_coefficient"]),
        mu_x=float(data["mu_x_at_reference_load"]),
        mu_y=float(data["mu_y_at_reference_load"]),
        ref_mass_x=float(data["reference_mass_x_per_tire_kg"]),
        ref_mass_y=float(data["reference_mass_y_per_tire_kg"]),
        sens_x=float(data["sensitivity_x_per_n"]),
        sens_y=float(data["sensitivity_y_per_n"]),
        factor_drive=float(data["rear_static_fraction"]),
        factor_aero=float(data["rear_aero_fraction"]),
        driven_wheels=int(data["driven_wheels"]),
        power_factor=float(data["power_factor"]),
        v_max=float(data["top_speed_mps"]),
        vehicle_speed=np.asarray(data["vehicle_speed_mps"], dtype=float),
        fx_engine=np.asarray(data["tractive_force_n"], dtype=float),
    )


def aero_and_loads(vehicle: Vehicle, speed: float) -> tuple[float, float, float]:
    downforce = (
        0.5
        * vehicle.rho
        * vehicle.aero_factor
        * vehicle.cl
        * vehicle.area
        * speed**2
    )
    drag = (
        0.5
        * vehicle.rho
        * vehicle.aero_factor
        * vehicle.cd
        * vehicle.area
        * speed**2
    )
    total_load = vehicle.mass * G + downforce
    return downforce, drag, total_load


def load_sensitive_mu(
    mu_reference: float,
    sensitivity_per_n: float,
    reference_mass_per_tire: float,
    load_per_tire_n: float,
) -> float:
    return mu_reference + sensitivity_per_n * (
        reference_mass_per_tire * G - load_per_tire_n
    )


def engine_force(vehicle: Vehicle, speed: float) -> float:
    if speed < vehicle.vehicle_speed[0] or speed > vehicle.vehicle_speed[-1]:
        return 0.0
    return vehicle.power_factor * float(
        np.interp(speed, vehicle.vehicle_speed, vehicle.fx_engine)
    )


def lateral_margin(vehicle: Vehicle, curvature: float, speed: float) -> float:
    if abs(curvature) < 1e-15:
        return math.inf
    downforce, drag, total_load = aero_and_loads(vehicle, speed)
    mu_y = load_sensitive_mu(
        vehicle.mu_y, vehicle.sens_y, vehicle.ref_mass_y, total_load / 4.0
    )
    ay_max = max(0.0, mu_y * total_load / vehicle.mass)
    driven_load_per_tire = (
        vehicle.factor_drive * vehicle.mass * G
        + vehicle.factor_aero * downforce
    ) / vehicle.driven_wheels
    mu_x = load_sensitive_mu(
        vehicle.mu_x,
        vehicle.sens_x,
        vehicle.ref_mass_x,
        driven_load_per_tire,
    )
    ax_tyre_max = max(
        0.0,
        mu_x
        * driven_load_per_tire
        * vehicle.driven_wheels
        / vehicle.mass,
    )
    drag_deceleration = (drag + vehicle.cr * total_load) / vehicle.mass
    if ax_tyre_max <= 0.0 or drag_deceleration >= ax_tyre_max:
        ay_available = 0.0
    else:
        ay_available = ay_max * math.sqrt(
            max(0.0, 1.0 - (drag_deceleration / ax_tyre_max) ** 2)
        )
    return ay_available - speed**2 * abs(curvature)


def lateral_speed_limit(vehicle: Vehicle, curvature: float) -> float:
    if abs(curvature) < 1e-15:
        return vehicle.v_max
    if lateral_margin(vehicle, curvature, vehicle.v_max) >= 0.0:
        return vehicle.v_max
    lower = 0.0
    upper = vehicle.v_max
    for _ in range(80):
        middle = 0.5 * (lower + upper)
        if lateral_margin(vehicle, curvature, middle) >= 0.0:
            lower = middle
        else:
            upper = middle
    return lower


def ellipse_multiplier(
    vehicle: Vehicle, curvature: float, speed: float, total_load: float
) -> float:
    if abs(curvature) < 1e-15:
        return 1.0
    mu_y = load_sensitive_mu(
        vehicle.mu_y, vehicle.sens_y, vehicle.ref_mass_y, total_load / 4.0
    )
    ay_max = max(1e-12, mu_y * total_load / vehicle.mass)
    ay = speed**2 * abs(curvature)
    ratio = min(1.0, ay / ay_max)
    return math.sqrt(max(0.0, 1.0 - ratio**2))


def acceleration(vehicle: Vehicle, curvature: float, speed: float) -> float:
    downforce, drag, total_load = aero_and_loads(vehicle, speed)
    ellipse = ellipse_multiplier(vehicle, curvature, speed, total_load)
    driven_load_per_tire = (
        vehicle.factor_drive * vehicle.mass * G
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
    drive_force = min(tyre_force, engine_force(vehicle, speed))
    resistance = drag + vehicle.cr * total_load
    return (drive_force - resistance) / vehicle.mass


def deceleration(vehicle: Vehicle, curvature: float, speed: float) -> float:
    _, drag, total_load = aero_and_loads(vehicle, speed)
    ellipse = ellipse_multiplier(vehicle, curvature, speed, total_load)
    load_per_tire = total_load / 4.0
    mu_x = load_sensitive_mu(
        vehicle.mu_x, vehicle.sens_x, vehicle.ref_mass_x, load_per_tire
    )
    braking_force = max(0.0, mu_x * total_load * ellipse)
    resistance = drag + vehicle.cr * total_load
    return (braking_force + resistance) / vehicle.mass


def solve_closed_track(
    vehicle: Vehicle,
    track: pd.DataFrame,
    tolerance: float = 1e-9,
    max_iterations: int = 2000,
) -> tuple[pd.DataFrame, dict]:
    dx = track["dx_m"].to_numpy(dtype=float)
    curvature = track["curvature_1pm"].to_numpy(dtype=float)
    count = len(track)
    lateral_limit = np.array(
        [lateral_speed_limit(vehicle, value) for value in curvature], dtype=float
    )
    speed = lateral_limit.copy()

    converged = False
    max_change = math.inf
    for iteration in range(1, max_iterations + 1):
        old_speed = speed.copy()
        for index in range(count):
            next_index = (index + 1) % count
            ax = acceleration(vehicle, curvature[index], speed[index])
            proposed_sq = speed[index] ** 2 + 2.0 * ax * dx[index]
            proposed = math.sqrt(max(0.0, proposed_sq))
            if proposed < speed[next_index]:
                speed[next_index] = proposed

        for index in range(count - 1, -1, -1):
            previous_index = (index - 1) % count
            proposed = speed[index]
            for _ in range(12):
                decel = deceleration(
                    vehicle, curvature[previous_index], proposed
                )
                updated = math.sqrt(
                    max(0.0, speed[index] ** 2 + 2.0 * decel * dx[previous_index])
                )
                updated = min(updated, lateral_limit[previous_index])
                if abs(updated - proposed) < 1e-11:
                    proposed = updated
                    break
                proposed = updated
            if proposed < speed[previous_index]:
                speed[previous_index] = proposed

        speed = np.minimum(speed, lateral_limit)
        max_change = float(np.max(np.abs(speed - old_speed)))
        if max_change < tolerance:
            converged = True
            break

    next_speed = np.roll(speed, -1)
    longitudinal_accel = np.divide(
        next_speed**2 - speed**2,
        2.0 * dx,
        out=np.zeros_like(speed),
        where=dx > 0,
    )
    lateral_accel = speed**2 * curvature
    time_in_segment = np.divide(
        dx,
        speed,
        out=np.full_like(dx, np.inf),
        where=speed > 0,
    )
    elapsed_time = np.cumsum(time_in_segment)
    downforce = np.empty(count)
    drag = np.empty(count)
    total_load = np.empty(count)
    mu_x = np.empty(count)
    mu_y = np.empty(count)
    tractive_force = np.empty(count)
    for index, value in enumerate(speed):
        downforce[index], drag[index], total_load[index] = aero_and_loads(
            vehicle, value
        )
        mu_x[index] = load_sensitive_mu(
            vehicle.mu_x,
            vehicle.sens_x,
            vehicle.ref_mass_x,
            total_load[index] / 4.0,
        )
        mu_y[index] = load_sensitive_mu(
            vehicle.mu_y,
            vehicle.sens_y,
            vehicle.ref_mass_y,
            total_load[index] / 4.0,
        )
        tractive_force[index] = engine_force(vehicle, value)

    result = track.copy()
    result["lateral_speed_limit_mps"] = lateral_limit
    result["speed_mps"] = speed
    result["time_in_segment_s"] = time_in_segment
    result["elapsed_time_s"] = elapsed_time
    result["longitudinal_accel_mps2"] = longitudinal_accel
    result["lateral_accel_mps2"] = lateral_accel
    result["downforce_n"] = downforce
    result["drag_n"] = drag
    result["total_normal_load_n"] = total_load
    result["mu_x_per_tire"] = mu_x
    result["mu_y_per_tire"] = mu_y
    result["available_tractive_force_n"] = tractive_force

    summary = {
        "solver": "OpenLAP equations, contained Python forward/backward port",
        "lap_time_s": float(time_in_segment.sum()),
        "track_length_m": float(dx.sum()),
        "segments": count,
        "minimum_speed_mps": float(speed.min()),
        "maximum_speed_mps": float(speed.max()),
        "distance_weighted_average_speed_mps": float(dx.sum() / time_in_segment.sum()),
        "maximum_lateral_accel_mps2": float(np.max(np.abs(lateral_accel))),
        "maximum_longitudinal_accel_mps2": float(longitudinal_accel.max()),
        "maximum_longitudinal_decel_mps2": float(longitudinal_accel.min()),
        "iterations": iteration,
        "converged": converged,
        "final_max_speed_change_mps": max_change,
        "canonical_time_formula": "sum(dx / speed), matching OpenLAP.m for a closed track",
    }
    return result, summary


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--vehicle",
        type=Path,
        default=root / "inputs" / "openlap_vehicle.json",
    )
    parser.add_argument(
        "--track",
        type=Path,
        default=root / "inputs" / "michigan_openlap_track.csv",
    )
    parser.add_argument("--output-root", type=Path, default=root / "outputs")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_root = args.output_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    vehicle = load_vehicle(args.vehicle.resolve())
    track = pd.read_csv(args.track.resolve())
    result, summary = solve_closed_track(vehicle, track)
    result.to_csv(output_root / "openlap_trace.csv", index=False)
    (output_root / "openlap_summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
