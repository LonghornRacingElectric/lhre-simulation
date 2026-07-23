"""Build OpenLAP-compatible vehicle and track inputs from the matched baseline.

SPDX-License-Identifier: GPL-3.0-or-later
"""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path

import numpy as np
import pandas as pd
import yaml
from scipy.io import savemat


G_OPTIMUM = 9.80665
G_OPENLAP = 9.81
TIRE_MU_SCALE = 0.6225437130779028


def parse_args() -> argparse.Namespace:
    script_root = Path(__file__).resolve().parents[1]
    repo_root = script_root.parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--lapsims-root", type=Path, default=script_root)
    parser.add_argument(
        "--vehicle-yaml",
        type=Path,
        default=repo_root / "vehicles" / "current" / "vehicle.yml",
    )
    parser.add_argument(
        "--tire-file",
        type=Path,
        default=repo_root
        / "vehicles"
        / "current"
        / "tires"
        / "16x7p5_10_12psi.tir",
    )
    return parser.parse_args()


def tire_parameters(path: Path) -> dict[str, float]:
    wanted = {"FNOMIN", "UNLOADED_RADIUS", "PDX1", "PDX2", "PDY1", "PDY2"}
    found: dict[str, float] = {}
    pattern = re.compile(
        r"^\s*([A-Z0-9_]+)\s*=\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][-+]?\d+)?)"
    )
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        match = pattern.match(line)
        if match and match.group(1) in wanted:
            found[match.group(1)] = float(match.group(2))
    missing = wanted - found.keys()
    if missing:
        raise ValueError(f"Missing tire parameters in {path}: {sorted(missing)}")
    return found


def total_mass_and_cg(vehicle: dict) -> tuple[float, float]:
    components: list[tuple[float, float]] = [
        (
            float(vehicle["sprung_mass"]["mass_kg"]),
            float(vehicle["sprung_mass"]["cg_m"][0]),
        ),
        (
            float(vehicle["driver_mass"]["mass_kg"]),
            float(vehicle["driver_mass"]["cg_m"][0]),
        ),
    ]
    for axle in ("front", "rear"):
        for component in vehicle[axle]["masses"].values():
            components.append(
                (2.0 * float(component["mass_kg"]), float(component["cg_m"][0]))
            )
    mass = sum(component_mass for component_mass, _ in components)
    cg_x = sum(component_mass * x for component_mass, x in components) / mass
    return mass, cg_x


def output_value(parameters: dict, key: str) -> float:
    return float(parameters[key]["Value"])


def first_float(value: object) -> float:
    if isinstance(value, list):
        return float(value[0])
    return float(value)


def main() -> None:
    args = parse_args()
    lapsims_root = args.lapsims_root.resolve()
    inputs_dir = lapsims_root / "inputs"
    inputs_dir.mkdir(parents=True, exist_ok=True)

    optimum = json.loads(
        (inputs_dir / "optimumlap_baseline.json").read_text(encoding="utf-8-sig")
    )
    runtime = optimum["Vehicle"]["Runtime"]
    parameters = optimum["Vehicle"]["ParameterDictionary"]
    track_meta = optimum["Track"]
    segments = pd.read_csv(inputs_dir / "michigan_optimumlap_segments.csv")

    vehicle_doc = yaml.safe_load(args.vehicle_yaml.read_text(encoding="utf-8"))
    vehicle = vehicle_doc["vehicle"]
    mass_yaml, cg_x = total_mass_and_cg(vehicle_doc)
    wheelbase = abs(
        float(vehicle_doc["rear"]["suspension"]["wheel_center_m"][0])
        - float(vehicle_doc["front"]["suspension"]["wheel_center_m"][0])
    )
    rear_static_fraction = -cg_x / wheelbase

    mass_optimum = float(runtime["Mass"])
    rear_fraction_optimum = float(runtime["WeightOnDrivenWheel"])
    if not math.isclose(mass_yaml, mass_optimum, rel_tol=0.0, abs_tol=1e-9):
        raise ValueError(
            f"BobSim mass {mass_yaml:.12f} does not match OptimumLap "
            f"{mass_optimum:.12f}"
        )
    if not math.isclose(
        rear_static_fraction, rear_fraction_optimum, rel_tol=0.0, abs_tol=1e-12
    ):
        raise ValueError(
            "BobSim rear static fraction does not match the OptimumLap baseline"
        )

    tire = tire_parameters(args.tire_file)
    mu_x_tir = tire["PDX1"] * TIRE_MU_SCALE
    mu_y_tir = abs(tire["PDY1"]) * TIRE_MU_SCALE
    sensitivity_x_per_n_tir = abs(tire["PDX2"]) * TIRE_MU_SCALE / tire["FNOMIN"]
    sensitivity_y_per_n_tir = abs(tire["PDY2"]) * TIRE_MU_SCALE / tire["FNOMIN"]

    mu_x = float(runtime["LongFriction"])
    mu_y = float(runtime["LatFriction"])
    sensitivity_x_per_n = float(runtime["LongFrictionSensitivity"]) / G_OPTIMUM
    sensitivity_y_per_n = float(runtime["LatFrictionSensitivity"]) / G_OPTIMUM
    reference_load_x_n = (
        float(runtime["MassLongFriction"]) * G_OPTIMUM / 4.0
    )
    reference_load_y_n = (
        float(runtime["MassLatFriction"]) * G_OPTIMUM / 4.0
    )
    reference_mass_x_per_tire = reference_load_x_n / G_OPENLAP
    reference_mass_y_per_tire = reference_load_y_n / G_OPENLAP

    validation_pairs = {
        "mu_x": (mu_x, mu_x_tir),
        "mu_y": (mu_y, mu_y_tir),
        "sensitivity_x_per_n": (
            sensitivity_x_per_n,
            sensitivity_x_per_n_tir,
        ),
        "sensitivity_y_per_n": (
            sensitivity_y_per_n,
            sensitivity_y_per_n_tir,
        ),
    }
    for name, (optimum_value, tire_value) in validation_pairs.items():
        if not math.isclose(
            optimum_value, tire_value, rel_tol=0.0, abs_tol=2e-12
        ):
            raise ValueError(
                f"{name} mismatch: OptimumLap={optimum_value}, tire={tire_value}"
            )

    engine_speed_rad_s = np.asarray(runtime["EngineSpeed"], dtype=float)
    engine_torque_nm = np.asarray(runtime["Torque"], dtype=float)
    engine_power_w = np.asarray(runtime["Power"], dtype=float)
    gear_ratio = first_float(runtime["GearRatios"])
    final_drive = float(runtime["FinalDriveRatio"])
    drive_efficiency = float(runtime["DriveEfficiency"])
    tire_radius = float(runtime["TireRadius"])
    vehicle_speed = (
        engine_speed_rad_s / (gear_ratio * final_drive) * tire_radius
    )
    wheel_torque = (
        engine_torque_nm * gear_ratio * final_drive * drive_efficiency
    )
    fx_engine = wheel_torque / tire_radius
    if vehicle_speed[0] > 0:
        vehicle_speed = np.insert(vehicle_speed, 0, 0.0)
        fx_engine = np.insert(fx_engine, 0, fx_engine[0])
        wheel_torque = np.insert(wheel_torque, 0, wheel_torque[0])
        engine_torque_nm = np.insert(engine_torque_nm, 0, engine_torque_nm[0])
        engine_power_w = np.insert(engine_power_w, 0, 0.0)
        engine_speed_rad_s = np.insert(engine_speed_rad_s, 0, 0.0)

    cl = float(runtime["LiftCoefficient"])
    cd = float(runtime["DragCoefficient"])
    area = float(runtime["FrontArea"])
    rho = float(runtime["AirDensity"])

    vehicle_config = {
        "name": f"{vehicle['name']} OpenLAP matched baseline",
        "sources": {
            "optimumlap_vehicle": optimum["Vehicle"]["SourcePath"],
            "optimumlap_vehicle_sha256": optimum["Vehicle"]["SourceSha256"],
            "bobsim_vehicle_yaml": str(args.vehicle_yaml.resolve()),
            "bobsim_tire_file": str(args.tire_file.resolve()),
            "tire_mu_scale": TIRE_MU_SCALE,
        },
        "mass_kg": mass_yaml,
        "cg_x_m": cg_x,
        "wheelbase_m": wheelbase,
        "rear_static_fraction": rear_static_fraction,
        "driven_wheels": 2,
        "drive_layout": "RWD",
        "rear_aero_fraction": rear_static_fraction,
        "tire_radius_m": tire_radius,
        "rolling_resistance_coefficient": float(runtime["TireRollingDrag"]),
        "mu_x_at_reference_load": mu_x,
        "mu_y_at_reference_load": mu_y,
        "reference_mass_x_per_tire_kg": reference_mass_x_per_tire,
        "reference_mass_y_per_tire_kg": reference_mass_y_per_tire,
        "optimumlap_reference_mass_x_total_kg": float(
            runtime["MassLongFriction"]
        ),
        "optimumlap_reference_mass_y_total_kg": float(
            runtime["MassLatFriction"]
        ),
        "sensitivity_x_per_n": sensitivity_x_per_n,
        "sensitivity_y_per_n": sensitivity_y_per_n,
        "reference_load_x_n_in_openlap": reference_load_x_n,
        "reference_load_y_n_in_openlap": reference_load_y_n,
        "cl_downforce_positive": cl,
        "cd_drag_positive": cd,
        "reference_area_m2": area,
        "air_density_kg_m3": rho,
        "aero_factor": float(runtime["AeroFactor"]),
        "power_factor": float(runtime["PowerFactor"]),
        "drive_efficiency": drive_efficiency,
        "engine_efficiency": float(runtime["EngineEfficiency"]),
        "final_drive_ratio": final_drive,
        "gear_ratio": gear_ratio,
        "top_speed_mps": float(runtime["TopSpeed"]),
        "engine_speed_rad_s": engine_speed_rad_s.tolist(),
        "engine_torque_nm": engine_torque_nm.tolist(),
        "engine_power_w": engine_power_w.tolist(),
        "vehicle_speed_mps": vehicle_speed.tolist(),
        "wheel_torque_nm": wheel_torque.tolist(),
        "tractive_force_n": fx_engine.tolist(),
        "validation": {
            "mass_difference_yaml_minus_optimumlap_kg": mass_yaml - mass_optimum,
            "rear_fraction_difference_yaml_minus_optimumlap": (
                rear_static_fraction - rear_fraction_optimum
            ),
            "tire_mu_x_difference": mu_x - mu_x_tir,
            "tire_mu_y_difference": mu_y - mu_y_tir,
            "tire_sensitivity_x_per_n_difference": (
                sensitivity_x_per_n - sensitivity_x_per_n_tir
            ),
            "tire_sensitivity_y_per_n_difference": (
                sensitivity_y_per_n - sensitivity_y_per_n_tir
            ),
        },
    }
    (inputs_dir / "openlap_vehicle.json").write_text(
        json.dumps(vehicle_config, indent=2), encoding="utf-8"
    )

    length = segments["Length_m"].to_numpy(dtype=float)
    distance = segments["TotalLength_m"].to_numpy(dtype=float)
    radius = segments["Radius_m"].to_numpy(dtype=float)
    curvature = np.divide(
        1.0,
        radius,
        out=np.zeros_like(radius),
        where=np.abs(radius) > 1e-15,
    )
    track = pd.DataFrame(
        {
            "distance_m": distance,
            "dx_m": length,
            "curvature_1pm": curvature,
            "bank_deg": np.zeros(len(segments)),
            "inclination_deg": -segments["Grade"].to_numpy(dtype=float),
            "grip_factor": np.ones(len(segments)),
            "sector": segments["Sector"].to_numpy(dtype=int),
            "x_m": segments["X_m"].to_numpy(dtype=float),
            "y_m": segments["Y_m"].to_numpy(dtype=float),
            "z_m": segments["Z_m"].to_numpy(dtype=float),
        }
    )
    track.to_csv(inputs_dir / "michigan_openlap_track.csv", index=False)

    if not math.isclose(
        float(length.sum()),
        float(track_meta["TotalLength_m"]),
        rel_tol=0.0,
        abs_tol=1e-9,
    ):
        raise ValueError("Converted OpenLAP track length does not match OptimumLap")

    info = {
        "name": track_meta["Name"],
        "country": "United States",
        "city": "Brooklyn, Michigan",
        "type": track_meta["Type"],
        "config": "Closed",
        "direction": "Forward",
        "mirror": "Off",
    }
    savemat(
        inputs_dir / "OpenTRACK_FSAE_Michigan_Endurance_2014.mat",
        {
            "info": info,
            "x": distance[:, None],
            "dx": length[:, None],
            "n": np.array([[len(track)]], dtype=np.uint32),
            "r": curvature[:, None],
            "bank": track["bank_deg"].to_numpy()[:, None],
            "incl": track["inclination_deg"].to_numpy()[:, None],
            "factor_grip": track["grip_factor"].to_numpy()[:, None],
            "sector": track["sector"].to_numpy()[:, None],
            "r_apex": np.empty((0, 1)),
            "apex": np.empty((0, 1)),
            "X": track["x_m"].to_numpy()[:, None],
            "Y": track["y_m"].to_numpy()[:, None],
            "Z": track["z_m"].to_numpy()[:, None],
            "arrow": np.empty((0, 1)),
        },
        do_compression=True,
    )

    # This is a native OpenLAP vehicle structure. OpenLAP uses negative CL, CD,
    # and rolling resistance to represent forces opposing the positive axes.
    df_front = 1.0 - rear_static_fraction
    da_front = 1.0 - rear_static_fraction
    cf = 1000.0
    cr = 1000.0
    a = (1.0 - df_front) * wheelbase
    b = -df_front * wheelbase
    steering_matrix = 2.0 * np.array(
        [[cf, cf + cr], [cf * a, cf * a + cr * b]], dtype=float
    )
    savemat(
        inputs_dir / "OpenVEHICLE_LHRe_Matched_Baseline.mat",
        {
            "name": vehicle_config["name"],
            "type": "FSAE EV",
            "M": mass_yaml,
            "df": df_front,
            "L": wheelbase,
            "rack": 1.0,
            "Cl": -cl,
            "Cd": -cd,
            "factor_Cl": float(runtime["AeroFactor"]),
            "factor_Cd": float(runtime["AeroFactor"]),
            "da": da_front,
            "A": area,
            "rho": rho,
            "factor_grip": float(runtime["GripFactor"]),
            "tyre_radius": tire_radius,
            "Cr": -float(runtime["TireRollingDrag"]),
            "mu_x": mu_x,
            "mu_x_M": reference_mass_x_per_tire,
            "sens_x": sensitivity_x_per_n,
            "mu_y": mu_y,
            "mu_y_M": reference_mass_y_per_tire,
            "sens_y": sensitivity_y_per_n,
            "CF": cf,
            "CR": cr,
            "factor_power": float(runtime["PowerFactor"]),
            "n_thermal": float(runtime["EngineEfficiency"]),
            "fuel_LHV": float(runtime["FuelEnergyDensity"]),
            "drive": "RWD",
            "shift_time": 0.0,
            "n_primary": 1.0,
            "n_final": drive_efficiency,
            "n_gearbox": 1.0,
            "ratio_primary": 1.0,
            "ratio_final": final_drive,
            "ratio_gearbox": np.array([[gear_ratio]]),
            "nog": np.array([[1]], dtype=np.uint32),
            "vehicle_speed": vehicle_speed[:, None],
            "fx_engine": fx_engine[:, None],
            "wheel_torque": wheel_torque[:, None],
            "engine_torque": engine_torque_nm[:, None],
            "engine_power": engine_power_w[:, None],
            "engine_speed": engine_speed_rad_s[:, None],
            "gear": np.ones((len(vehicle_speed), 1)),
            "v_max": float(runtime["TopSpeed"]),
            "factor_drive": rear_static_fraction,
            "factor_aero": rear_static_fraction,
            "driven_wheels": np.array([[2]], dtype=np.uint32),
            "C": steering_matrix,
            "beta": 1.0,
            "phi": 1.0,
        },
        do_compression=True,
    )

    manifest = {
        "track": {
            "name": track_meta["Name"],
            "length_m": float(length.sum()),
            "segments": int(len(track)),
            "mesh_min_m": float(length.min()),
            "mesh_max_m": float(length.max()),
            "maximum_abs_curvature_1pm": float(np.max(np.abs(curvature))),
        },
        "vehicle": {
            "mass_kg": mass_yaml,
            "rear_static_fraction": rear_static_fraction,
            "maximum_power_kw": float(engine_power_w.max() / 1000.0),
            "cl": cl,
            "cd": cd,
            "reference_area_m2": area,
            "mu_x": mu_x,
            "mu_y": mu_y,
        },
        "files": {
            "track_csv": "inputs/michigan_openlap_track.csv",
            "track_mat": "inputs/OpenTRACK_FSAE_Michigan_Endurance_2014.mat",
            "vehicle_json": "inputs/openlap_vehicle.json",
            "vehicle_mat": "inputs/OpenVEHICLE_LHRe_Matched_Baseline.mat",
        },
    }
    (inputs_dir / "conversion_manifest.json").write_text(
        json.dumps(manifest, indent=2), encoding="utf-8"
    )
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
