"""Screen parameterized suspension cases on the anti-geometry study's three ovals."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

from _0_Utils.dyn_py.parameters import G
from _0_Utils.dyn_py.parametric import ParametricVehicle, parameters_from_mapping
from _0_Utils.dyn_py.parametric_projection import with_longitudinal_anti
from _0_Utils.lap_sim import GGVMap, TrackCorridor, simulate_transient_lap, solve_qss_lap
from oval import oval_centerline, oval_layout


TRACKS = {
    "hairpin": {"length_m": 70.0, "mean_radius_m": (9.0, 9.0), "radius_rate_m_per_rad": (2.5, -2.5)},
    "template": {"length_m": 60.0, "mean_radius_m": (15.0, 9.0), "radius_rate_m_per_rad": (2.0, -1.5)},
    "sweeper": {"length_m": 40.0, "mean_radius_m": (25.0, 15.0), "radius_rate_m_per_rad": (0.0, 0.0)},
}
LEVELS = (0, 20, 40, 60)
RC_MM = (0, 25, 50)
FIELDS = (
    "case",
    "track",
    "status",
    "anti_dive_pct",
    "anti_squat_pct",
    "front_rc_mm",
    "rear_rc_mm",
    "target_lap_s",
    "lap_s",
    "lap_length_m",
    "rms_lateral_error_m",
    "max_lateral_error_m",
    "peak_abs_pitch_deg",
    "peak_abs_roll_deg",
    "min_jounce_mm",
    "max_jounce_mm",
    "error",
)


def cases():
    yield "baseline", None, None, None
    for dive in LEVELS:
        for squat in LEVELS:
            yield f"ad{dive:02d}_as{squat:02d}", dive / 100, squat / 100, None
    for rc in RC_MM:
        yield f"rc{rc:02d}", 0.0, 0.0, rc


def corridor(center):
    tangent = np.roll(center, -1, axis=0) - np.roll(center, 1, axis=0)
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0])) / np.linalg.norm(tangent, axis=1)[:, None]
    return TrackCorridor(center + 2.0 * normal, center - 2.0 * normal)


def driver_target(parameters, ay_g, brake_g, drive_g):
    speed = np.linspace(5.0, 30.0, 26)
    lateral = np.linspace(0.0, ay_g * G, 27)
    ellipse = np.sqrt(1.0 - (lateral / lateral[-1]) ** 2)
    drag = 0.5 * parameters.rho_air_kg_m3 * parameters.cd_area_m2 * speed**2
    drive_force = np.minimum(parameters.peak_drive_force_n, parameters.peak_drive_power_w / speed) - drag
    drive = np.clip(drive_force / parameters.mass_kg, 0.0, drive_g * G)
    return GGVMap.from_arrays(
        speed, lateral, np.outer(drive, ellipse), np.outer(np.full_like(speed, -brake_g * G), ellipse)
    )


def run_case(data, vehicle_dir, name, dive, squat, rc, track, qss, period):
    changed = data if dive is None else with_longitudinal_anti(data, anti_dive=dive, anti_squat=squat)
    if rc is not None:
        changed["front"]["roll_center_height_m"] = rc / 1000
        changed["rear"]["roll_center_height_m"] = rc / 1000
    parameters = parameters_from_mapping(changed, base_dir=vehicle_dir)
    model = ParametricVehicle(parameters).model(6)
    row = dict.fromkeys(FIELDS, "")
    row.update(
        case=name,
        track=track,
        status="failed",
        anti_dive_pct="" if dive is None else 100 * dive,
        anti_squat_pct="" if squat is None else 100 * squat,
        front_rc_mm=1000 * changed["front"]["roll_center_height_m"],
        rear_rc_mm=1000 * changed["rear"]["roll_center_height_m"],
        target_lap_s=qss.lap_time_s,
        lap_length_m=qss.line.track_length_m,
    )
    try:
        lap = simulate_transient_lap(model, qss, sample_period_s=period, stop_time_s=1.2 * qss.lap_time_s + 2.0)
        signals = lap.transient.signals
        count = len(lap.transient.time_s)
        if not lap.transient.success or not lap.completed_lap:
            raise RuntimeError(lap.transient.message if not lap.transient.success else "lap not completed")
        jounce = np.column_stack([signals[f"jounce{corner}"] for corner in ("FL", "FR", "RL", "RR")])
        row.update(
            status="completed",
            lap_s=lap.lap_time_s,
            rms_lateral_error_m=float(np.sqrt(np.mean(lap.lateral_error_m[:count] ** 2))),
            max_lateral_error_m=float(np.max(np.abs(lap.lateral_error_m[:count]))),
            peak_abs_pitch_deg=float(np.degrees(np.max(np.abs(signals["pitch"])))),
            peak_abs_roll_deg=float(np.degrees(np.max(np.abs(signals["roll"])))),
            min_jounce_mm=float(np.min(jounce) * 1000),
            max_jounce_mm=float(np.max(jounce) * 1000),
        )
    except (RuntimeError, ValueError, FloatingPointError) as exc:
        row["error"] = f"{type(exc).__name__}: {exc}"
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--vehicle", type=Path, default=Path("../studies/parameterized-bobsim/outputs/targets_2027/vehicle.yml")
    )
    parser.add_argument("--output", type=Path, default=Path("../studies/parameterized-bobsim/asymmetric_oval/results"))
    parser.add_argument("--case", action="append", choices=[case[0] for case in cases()])
    parser.add_argument("--track", action="append", choices=list(TRACKS))
    parser.add_argument("--sample-period", type=float, default=0.005)
    parser.add_argument("--ay-g", type=float, default=1.3)
    parser.add_argument("--brake-g", type=float, default=1.3)
    parser.add_argument("--drive-g", type=float, default=0.6)
    parser.add_argument("--geometry-only", action="store_true")
    args = parser.parse_args()
    if min(args.sample_period, args.ay_g, args.brake_g, args.drive_g) <= 0:
        parser.error("sample period and driver limits must be positive")
    tracks = args.track or list(TRACKS)
    selected = [case for case in cases() if args.case is None or case[0] in args.case]
    layouts = {name: oval_layout(**TRACKS[name]) for name in tracks}
    if args.geometry_only:
        print(json.dumps({"tracks": layouts, "cases": [case[0] for case in selected]}, indent=2))
        return
    vehicle_path = args.vehicle.resolve()
    data = yaml.safe_load(vehicle_path.read_text(encoding="utf-8"))
    parameters = parameters_from_mapping(data, base_dir=vehicle_path.parent)
    target = driver_target(parameters, args.ay_g, args.brake_g, args.drive_g)
    laps = {}
    for name in tracks:
        track_corridor = corridor(oval_centerline(**TRACKS[name], spacing_m=0.5))
        line = track_corridor.line_from_offsets(np.zeros(track_corridor.gate_count))
        laps[name] = solve_qss_lap(line, target)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    manifest = {
        "vehicle_sha256": hashlib.sha256(vehicle_path.read_bytes()).hexdigest(),
        "tire_sha256": hashlib.sha256((vehicle_path.parent / data["tire_file"]).read_bytes()).hexdigest(),
        "source_study": "https://github.com/LonghornRacingElectric/lhre/tree/310e3487cd9ff62eadfde91a7d66c5a7096b28a6/simulation/studies/anti-geometry",
        "sample_period_s": args.sample_period,
        "driver": {"ay_g": args.ay_g, "brake_g": args.brake_g, "drive_g": args.drive_g},
        "tracks": {
            name: {"parameters": TRACKS[name], "layout": layouts[name], "target_lap_s": laps[name].lap_time_s}
            for name in tracks
        },
        "model": "BobSimParametric hardpoint-free 6DOF; mechanical-only 2027 vehicle; no aero map",
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    failed = 0
    with (output / "summary.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        for name, dive, squat, rc in selected:
            for track in tracks:
                row = run_case(data, vehicle_path.parent, name, dive, squat, rc, track, laps[track], args.sample_period)
                writer.writerow(row)
                handle.flush()
                print(name, track, row["status"], row["error"], flush=True)
                failed += row["status"] != "completed"
    if failed:
        raise SystemExit(f"{failed} case(s) failed; see summary.csv")


if __name__ == "__main__":
    main()
