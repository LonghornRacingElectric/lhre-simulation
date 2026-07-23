"""Compare the matched OptimumLap and OpenLAP-equation baseline runs."""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


G_OPTIMUM = 9.80665


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    inputs = root / "inputs"
    outputs = root / "outputs"
    outputs.mkdir(parents=True, exist_ok=True)

    optimum_summary = json.loads(
        (outputs / "optimumlap_summary.json").read_text(encoding="utf-8-sig")
    )
    openlap_summary = json.loads(
        (outputs / "openlap_summary.json").read_text(encoding="utf-8")
    )
    optimum_inputs = json.loads(
        (inputs / "optimumlap_baseline.json").read_text(encoding="utf-8-sig")
    )
    openlap_vehicle = json.loads(
        (inputs / "openlap_vehicle.json").read_text(encoding="utf-8")
    )

    optimum_trace = pd.read_csv(outputs / "optimumlap_trace.csv")
    openlap_trace = pd.read_csv(outputs / "openlap_trace.csv")
    if len(optimum_trace) != len(openlap_trace):
        raise ValueError("Trace lengths differ")
    distance_delta = (
        optimum_trace["elapsedDistance"].to_numpy()
        - openlap_trace["distance_m"].to_numpy()
    )
    if float(np.max(np.abs(distance_delta))) > 1e-8:
        raise ValueError("OptimumLap and OpenLAP distance meshes differ")

    optimum_speed = optimum_trace["speed"].to_numpy(dtype=float)
    openlap_speed = openlap_trace["speed_mps"].to_numpy(dtype=float)
    dx = openlap_trace["dx_m"].to_numpy(dtype=float)
    speed_delta = openlap_speed - optimum_speed
    lap_delta = (
        float(openlap_summary["lap_time_s"])
        - float(optimum_summary["LapTime_s"])
    )
    lap_delta_percent = 100.0 * lap_delta / float(optimum_summary["LapTime_s"])
    optimum_profile_openlap_time_formula = float(np.sum(dx / optimum_speed))
    openlap_profile_openlap_time_formula = float(np.sum(dx / openlap_speed))
    normalized_profile_delta = (
        openlap_profile_openlap_time_formula
        - optimum_profile_openlap_time_formula
    )
    normalized_profile_delta_percent = (
        100.0 * normalized_profile_delta / optimum_profile_openlap_time_formula
    )

    comparison = {
        "optimumlap": {
            "solver": optimum_summary["Solver"],
            "lap_time_s": float(optimum_summary["LapTime_s"]),
            "minimum_speed_mps": float(optimum_summary["LowestSpeed_mps"]),
            "maximum_speed_mps": float(optimum_summary["HighestSpeed_mps"]),
            "maximum_lateral_accel_mps2": float(
                optimum_summary["MaxLatAccel_mps2"]
            ),
            "maximum_longitudinal_accel_mps2": float(
                optimum_summary["MaxLongAccel_mps2"]
            ),
            "maximum_longitudinal_decel_mps2": float(
                optimum_summary["MaxLongDecel_mps2"]
            ),
        },
        "openlap": {
            "solver": openlap_summary["solver"],
            "lap_time_s": float(openlap_summary["lap_time_s"]),
            "minimum_speed_mps": float(openlap_summary["minimum_speed_mps"]),
            "maximum_speed_mps": float(openlap_summary["maximum_speed_mps"]),
            "maximum_lateral_accel_mps2": float(
                openlap_summary["maximum_lateral_accel_mps2"]
            ),
            "maximum_longitudinal_accel_mps2": float(
                openlap_summary["maximum_longitudinal_accel_mps2"]
            ),
            "maximum_longitudinal_decel_mps2": float(
                openlap_summary["maximum_longitudinal_decel_mps2"]
            ),
        },
        "difference_openlap_minus_optimumlap": {
            "lap_time_s": lap_delta,
            "lap_time_percent": lap_delta_percent,
            "speed_rmse_mps": float(np.sqrt(np.mean(speed_delta**2))),
            "speed_mae_mps": float(np.mean(np.abs(speed_delta))),
            "maximum_absolute_speed_delta_mps": float(
                np.max(np.abs(speed_delta))
            ),
            "speed_profile_correlation": float(
                np.corrcoef(optimum_speed, openlap_speed)[0, 1]
            ),
            "same_openlap_time_formula_delta_s": normalized_profile_delta,
            "same_openlap_time_formula_delta_percent": (
                normalized_profile_delta_percent
            ),
            "optimumlap_speed_profile_with_openlap_time_formula_s": (
                optimum_profile_openlap_time_formula
            ),
            "openlap_speed_profile_with_openlap_time_formula_s": (
                openlap_profile_openlap_time_formula
            ),
            "optimumlap_reported_minus_openlap_formula_s": (
                float(optimum_summary["LapTime_s"])
                - optimum_profile_openlap_time_formula
            ),
        },
        "mesh_validation": {
            "track_length_m": float(openlap_summary["track_length_m"]),
            "segments": int(openlap_summary["segments"]),
            "maximum_absolute_distance_delta_m": float(
                np.max(np.abs(distance_delta))
            ),
        },
    }
    (outputs / "comparison_summary.json").write_text(
        json.dumps(comparison, indent=2), encoding="utf-8"
    )

    rows = [
        {
            "Metric": "Lap time",
            "Unit": "s",
            "OptimumLap": comparison["optimumlap"]["lap_time_s"],
            "OpenLAP": comparison["openlap"]["lap_time_s"],
        },
        {
            "Metric": "Minimum speed",
            "Unit": "m/s",
            "OptimumLap": comparison["optimumlap"]["minimum_speed_mps"],
            "OpenLAP": comparison["openlap"]["minimum_speed_mps"],
        },
        {
            "Metric": "Maximum speed",
            "Unit": "m/s",
            "OptimumLap": comparison["optimumlap"]["maximum_speed_mps"],
            "OpenLAP": comparison["openlap"]["maximum_speed_mps"],
        },
        {
            "Metric": "Maximum lateral acceleration",
            "Unit": "m/s^2",
            "OptimumLap": comparison["optimumlap"][
                "maximum_lateral_accel_mps2"
            ],
            "OpenLAP": comparison["openlap"]["maximum_lateral_accel_mps2"],
        },
        {
            "Metric": "Maximum longitudinal acceleration",
            "Unit": "m/s^2",
            "OptimumLap": comparison["optimumlap"][
                "maximum_longitudinal_accel_mps2"
            ],
            "OpenLAP": comparison["openlap"][
                "maximum_longitudinal_accel_mps2"
            ],
        },
        {
            "Metric": "Maximum longitudinal deceleration",
            "Unit": "m/s^2",
            "OptimumLap": comparison["optimumlap"][
                "maximum_longitudinal_decel_mps2"
            ],
            "OpenLAP": comparison["openlap"][
                "maximum_longitudinal_decel_mps2"
            ],
        },
    ]
    comparison_table = pd.DataFrame(rows)
    comparison_table["OpenLAP_minus_OptimumLap"] = (
        comparison_table["OpenLAP"] - comparison_table["OptimumLap"]
    )
    comparison_table["Difference_percent_of_OptimumLap"] = (
        100.0
        * comparison_table["OpenLAP_minus_OptimumLap"]
        / comparison_table["OptimumLap"].abs()
    )
    comparison_table.to_csv(outputs / "comparison_summary.csv", index=False)

    runtime = optimum_inputs["Vehicle"]["Runtime"]
    input_rows = [
        ("Mass", "kg", float(runtime["Mass"]), openlap_vehicle["mass_kg"]),
        (
            "Rear static/driven fraction",
            "-",
            float(runtime["WeightOnDrivenWheel"]),
            openlap_vehicle["rear_static_fraction"],
        ),
        (
            "Tire radius",
            "m",
            float(runtime["TireRadius"]),
            openlap_vehicle["tire_radius_m"],
        ),
        (
            "Lift coefficient (downforce-positive reporting)",
            "-",
            float(runtime["LiftCoefficient"]),
            openlap_vehicle["cl_downforce_positive"],
        ),
        (
            "Drag coefficient (drag-positive reporting)",
            "-",
            float(runtime["DragCoefficient"]),
            openlap_vehicle["cd_drag_positive"],
        ),
        (
            "Reference area",
            "m^2",
            float(runtime["FrontArea"]),
            openlap_vehicle["reference_area_m2"],
        ),
        (
            "Air density",
            "kg/m^3",
            float(runtime["AirDensity"]),
            openlap_vehicle["air_density_kg_m3"],
        ),
        (
            "Longitudinal mu at reference load",
            "-",
            float(runtime["LongFriction"]),
            openlap_vehicle["mu_x_at_reference_load"],
        ),
        (
            "Lateral mu at reference load",
            "-",
            float(runtime["LatFriction"]),
            openlap_vehicle["mu_y_at_reference_load"],
        ),
        (
            "Top speed",
            "m/s",
            float(runtime["TopSpeed"]),
            openlap_vehicle["top_speed_mps"],
        ),
        (
            "Maximum power",
            "kW",
            max(float(value) for value in runtime["Power"]) / 1000.0,
            max(openlap_vehicle["engine_power_w"]) / 1000.0,
        ),
        (
            "Track length",
            "m",
            float(optimum_inputs["Track"]["TotalLength_m"]),
            float(openlap_summary["track_length_m"]),
        ),
        (
            "Track segments",
            "count",
            float(optimum_inputs["Track"]["SegmentCount"]),
            float(openlap_summary["segments"]),
        ),
    ]
    parity = pd.DataFrame(
        input_rows,
        columns=["Parameter", "Unit", "OptimumLap", "OpenLAP"],
    )
    parity["OpenLAP_minus_OptimumLap"] = (
        parity["OpenLAP"] - parity["OptimumLap"]
    )
    parity.to_csv(outputs / "input_equivalence.csv", index=False)

    normal_load = np.linspace(250.0, 1600.0, 136)
    ref_load_x = openlap_vehicle["reference_load_x_n_in_openlap"]
    ref_load_y = openlap_vehicle["reference_load_y_n_in_openlap"]
    optimum_sens_x_kg = float(runtime["LongFrictionSensitivity"])
    optimum_sens_y_kg = float(runtime["LatFrictionSensitivity"])
    optimum_mu_x = float(runtime["LongFriction"]) - optimum_sens_x_kg * (
        (normal_load - ref_load_x) / G_OPTIMUM
    )
    optimum_mu_y = float(runtime["LatFriction"]) - optimum_sens_y_kg * (
        (normal_load - ref_load_y) / G_OPTIMUM
    )
    openlap_mu_x = openlap_vehicle["mu_x_at_reference_load"] + openlap_vehicle[
        "sensitivity_x_per_n"
    ] * (ref_load_x - normal_load)
    openlap_mu_y = openlap_vehicle["mu_y_at_reference_load"] + openlap_vehicle[
        "sensitivity_y_per_n"
    ] * (ref_load_y - normal_load)
    tire_validation = pd.DataFrame(
        {
            "normal_load_per_tire_n": normal_load,
            "optimumlap_mu_x": optimum_mu_x,
            "openlap_mu_x": openlap_mu_x,
            "mu_x_difference": openlap_mu_x - optimum_mu_x,
            "optimumlap_mu_y": optimum_mu_y,
            "openlap_mu_y": openlap_mu_y,
            "mu_y_difference": openlap_mu_y - optimum_mu_y,
        }
    )
    tire_validation.to_csv(
        outputs / "tire_load_sensitivity_validation.csv", index=False
    )

    distance = openlap_trace["distance_m"].to_numpy(dtype=float)
    fig, axes = plt.subplots(
        2,
        1,
        figsize=(12, 7.5),
        sharex=True,
        gridspec_kw={"height_ratios": [3, 1]},
    )
    axes[0].plot(
        distance,
        optimum_speed * 3.6,
        label=f"OptimumLap: {optimum_summary['LapTime_s']:.3f} s",
        linewidth=1.5,
    )
    axes[0].plot(
        distance,
        openlap_speed * 3.6,
        label=f"OpenLAP equations: {openlap_summary['lap_time_s']:.3f} s",
        linewidth=1.2,
        alpha=0.85,
    )
    axes[0].set_ylabel("Speed (km/h)")
    axes[0].set_title("Matched Michigan 2014 Endurance Lap")
    axes[0].grid(alpha=0.25)
    axes[0].legend(loc="upper right")

    axes[1].plot(distance, speed_delta * 3.6, color="#7b3294", linewidth=1.0)
    axes[1].axhline(0.0, color="black", linewidth=0.7)
    axes[1].set_xlabel("Distance (m)")
    axes[1].set_ylabel("OpenLAP -\nOptimumLap\n(km/h)")
    axes[1].grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(outputs / "optimumlap_vs_openlap_speed.png", dpi=180)
    plt.close(fig)

    print(json.dumps(comparison, indent=2))


if __name__ == "__main__":
    main()
