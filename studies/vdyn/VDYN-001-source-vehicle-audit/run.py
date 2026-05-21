from __future__ import annotations

import csv
import math
import os
import re
from pathlib import Path
from typing import Any

os.environ.setdefault(
    "MPLCONFIGDIR",
    str(Path(__file__).resolve().parent / ".matplotlib-cache"),
)

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import yaml


STUDY_DIR = Path(__file__).resolve().parent


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise TypeError(f"Expected YAML mapping: {path}")
    return data


def resolve_from(base: Path, raw: str) -> Path:
    candidate = Path(raw)
    if candidate.is_absolute():
        return candidate
    return (base.parent / candidate).resolve()


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def parse_tir(path: Path) -> dict[str, float]:
    values: dict[str, float] = {}
    pattern = re.compile(r"^([A-Za-z0-9_]+)\s*=\s*([-+0-9.Ee]+)")
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("$", 1)[0].strip()
        match = pattern.match(line)
        if not match:
            continue
        key, raw_value = match.groups()
        try:
            values[key.upper()] = float(raw_value)
        except ValueError:
            continue
    return values


def side_mass_rows(vehicle: dict[str, Any], side: str) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    masses = vehicle[side]["masses"]
    for name, entry in masses.items():
        mass = float(entry["mass_kg"])
        cg = [float(v) for v in entry["cg_m"]]
        rows.append(
            {
                "name": f"{side}_{name}",
                "count": 2,
                "mass_kg_each": mass,
                "effective_mass_kg": 2.0 * mass,
                "cg_x_m": cg[0],
                "cg_y_m": 0.0,
                "cg_z_m": cg[2],
            }
        )
    return rows


def mass_breakdown(vehicle: dict[str, Any]) -> list[dict[str, Any]]:
    rows = [
        {
            "name": "sprung_mass",
            "count": 1,
            "mass_kg_each": float(vehicle["sprung_mass"]["mass_kg"]),
            "effective_mass_kg": float(vehicle["sprung_mass"]["mass_kg"]),
            "cg_x_m": float(vehicle["sprung_mass"]["cg_m"][0]),
            "cg_y_m": float(vehicle["sprung_mass"]["cg_m"][1]),
            "cg_z_m": float(vehicle["sprung_mass"]["cg_m"][2]),
        },
        {
            "name": "driver_mass",
            "count": 1,
            "mass_kg_each": float(vehicle["driver_mass"]["mass_kg"]),
            "effective_mass_kg": float(vehicle["driver_mass"]["mass_kg"]),
            "cg_x_m": float(vehicle["driver_mass"]["cg_m"][0]),
            "cg_y_m": float(vehicle["driver_mass"]["cg_m"][1]),
            "cg_z_m": float(vehicle["driver_mass"]["cg_m"][2]),
        },
    ]
    rows.extend(side_mass_rows(vehicle, "front"))
    rows.extend(side_mass_rows(vehicle, "rear"))
    return rows


def weighted_cg(rows: list[dict[str, Any]]) -> tuple[float, float, float, float]:
    total = sum(float(row["effective_mass_kg"]) for row in rows)
    x = sum(float(row["effective_mass_kg"]) * float(row["cg_x_m"]) for row in rows) / total
    y = sum(float(row["effective_mass_kg"]) * float(row["cg_y_m"]) for row in rows) / total
    z = sum(float(row["effective_mass_kg"]) * float(row["cg_z_m"]) for row in rows) / total
    return total, x, y, z


def average_point(a: list[float], b: list[float]) -> list[float]:
    return [0.5 * (float(a[i]) + float(b[i])) for i in range(3)]


def norm(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values))


def plot_mass_breakdown(rows: list[dict[str, Any]], output_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(8.5, 4.8))
    labels = [row["name"].replace("_", "\n") for row in rows]
    masses = [float(row["effective_mass_kg"]) for row in rows]
    ax.bar(labels, masses, color="#3b6ea8")
    ax.set_ylabel("Effective mass [kg]")
    ax.set_title("Source Vehicle Mass Rollup")
    ax.tick_params(axis="x", labelrotation=45)
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def write_results(
    cfg: dict[str, Any],
    summary: dict[str, Any],
    tire: dict[str, float],
    output_path: Path,
) -> None:
    status = "PASS" if summary["max_aero_reference_error_m"] < 1e-9 else "CHECK"
    lines = [
        "# VDYN-001 Results",
        "",
        "## Decision Question",
        "",
        cfg["study"]["decision_question"],
        "",
        "## Finding",
        "",
        f"**{status}:** the source vehicle definition is coherent enough to be the starting point for vehicle dynamics studies.",
        "",
        "This study does not prove performance. It only establishes the source-of-truth vehicle that later performance studies are allowed to use.",
        "",
        "## Source Vehicle Metrics",
        "",
        f"- Total audited mass: `{summary['total_mass_kg']:.3f} kg`",
        f"- Audited CG: `x={summary['cg_x_m']:.4f} m`, `y={summary['cg_y_m']:.4f} m`, `z={summary['cg_z_m']:.4f} m`",
        f"- Wheelbase: `{summary['wheelbase_m']:.4f} m`",
        f"- Front/rear track: `{summary['front_track_m']:.4f} m` / `{summary['rear_track_m']:.4f} m`",
        f"- Static load split: `{100.0 * summary['front_static_load_fraction']:.2f} %` front / `{100.0 * summary['rear_static_load_fraction']:.2f} %` rear",
        f"- Body torsional stiffness input: `{summary['torsional_stiffness_n_m_per_rad']:.0f} N*m/rad`",
        "",
        "## Tire Source Metrics",
        "",
        f"- Tire template: `{summary['tire_template']}`",
        f"- Unloaded tire radius: `{tire.get('UNLOADED_RADIUS', math.nan):.4f} m`",
        f"- Nominal vertical load: `{tire.get('FNOMIN', math.nan):.1f} N`",
        f"- Vertical load range: `{tire.get('FZMIN', math.nan):.1f}` to `{tire.get('FZMAX', math.nan):.1f} N`",
        f"- Lateral relaxation coefficients: `PTY1={tire.get('PTY1', math.nan):.6f}`, `PTY2={tire.get('PTY2', math.nan):.6f}`",
        "",
        "## Aero Reference Audit",
        "",
        f"- Front lower-inboard average: `{summary['front_lower_inboard_average_m']}`",
        f"- YAML front aero reference: `{summary['front_aero_reference_m']}`",
        f"- Rear lower-inboard average: `{summary['rear_lower_inboard_average_m']}`",
        f"- YAML rear aero reference: `{summary['rear_aero_reference_m']}`",
        f"- Max aero reference error: `{summary['max_aero_reference_error_m']:.6e} m`",
        "",
        "![Mass breakdown](plots/mass_breakdown.png)",
        "",
        "## What This Enables",
        "",
        "- `VDYN-002` may use these mass, CG, wheelbase, track, tire, brake, power, and aero inputs for baseline envelope work.",
        "- `AERO-001` may use the audited lower-inboard reference points as the starting point for aero map convention checks.",
        "- `CHASSIS-001` may use this audit as the first hardpoint and mass-property source check, but should still add structural load-path detail.",
        "",
        "## Correlation Closure",
        "",
        "Before first dynamic correlation, measure total mass, corner weights, wheelbase, track, ride heights, and static alignment. If the built car differs materially, update the source YAML or explicitly document why design-intent values remain in the model.",
    ]
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    cfg = load_yaml(STUDY_DIR / "study.yml")
    vehicle_path = resolve_from(STUDY_DIR / "study.yml", cfg["inputs"]["vehicle_config"])
    tire_path = resolve_from(STUDY_DIR / "study.yml", cfg["inputs"]["tire_file"])
    vehicle = load_yaml(vehicle_path)
    tire = parse_tir(tire_path)

    rows = mass_breakdown(vehicle)
    total_mass, cg_x, cg_y, cg_z = weighted_cg(rows)
    front_wc = [float(v) for v in vehicle["front"]["suspension"]["wheel_center_m"]]
    rear_wc = [float(v) for v in vehicle["rear"]["suspension"]["wheel_center_m"]]
    wheelbase = abs(front_wc[0] - rear_wc[0])
    front_track = 2.0 * abs(front_wc[1])
    rear_track = 2.0 * abs(rear_wc[1])
    front_static = (cg_x - rear_wc[0]) / (front_wc[0] - rear_wc[0])
    rear_static = 1.0 - front_static

    front_avg = average_point(
        vehicle["front"]["suspension"]["lower_fore_i_m"],
        vehicle["front"]["suspension"]["lower_aft_i_m"],
    )
    rear_avg = average_point(
        vehicle["rear"]["suspension"]["lower_fore_i_m"],
        vehicle["rear"]["suspension"]["lower_aft_i_m"],
    )
    front_ref = [float(v) for v in vehicle["aero"]["front_left_ride_height_ref_m"]]
    rear_ref = [float(v) for v in vehicle["aero"]["rear_left_ride_height_ref_m"]]
    front_error = norm([front_ref[i] - front_avg[i] for i in range(3)])
    rear_error = norm([rear_ref[i] - rear_avg[i] for i in range(3)])

    summary = {
        "vehicle_config": str(vehicle_path),
        "tire_file": str(tire_path),
        "vehicle_name": vehicle["vehicle"]["name"],
        "architecture_front": vehicle["architecture"]["front"],
        "architecture_rear": vehicle["architecture"]["rear"],
        "total_mass_kg": total_mass,
        "cg_x_m": cg_x,
        "cg_y_m": cg_y,
        "cg_z_m": cg_z,
        "wheelbase_m": wheelbase,
        "front_track_m": front_track,
        "rear_track_m": rear_track,
        "front_static_load_fraction": front_static,
        "rear_static_load_fraction": rear_static,
        "torsional_stiffness_n_m_per_rad": float(vehicle["body"]["torsional_stiff_n_m_per_rad"]),
        "tire_template": vehicle["front"]["tire"]["template"],
        "front_lower_inboard_average_m": front_avg,
        "rear_lower_inboard_average_m": rear_avg,
        "front_aero_reference_m": front_ref,
        "rear_aero_reference_m": rear_ref,
        "front_aero_reference_error_m": front_error,
        "rear_aero_reference_error_m": rear_error,
        "max_aero_reference_error_m": max(front_error, rear_error),
    }

    outputs = STUDY_DIR / cfg["outputs"]["data_dir"]
    plots = STUDY_DIR / cfg["outputs"]["figures_dir"]
    write_csv(outputs / "mass_breakdown.csv", rows)
    write_csv(outputs / "summary.csv", [summary])
    write_csv(
        outputs / "tire_metadata.csv",
        [
            {
                "key": key,
                "value": tire[key],
            }
            for key in sorted(tire)
            if key
            in {
                "UNLOADED_RADIUS",
                "WIDTH",
                "RIM_RADIUS",
                "FNOMIN",
                "FZMIN",
                "FZMAX",
                "VERTICAL_STIFFNESS",
                "PTY1",
                "PTY2",
                "PDY1",
                "PDY2",
                "PKY1",
                "PKY2",
                "PDX1",
                "PDX2",
            }
        ],
    )
    plot_mass_breakdown(rows, plots / "mass_breakdown.png")
    write_results(cfg, summary, tire, STUDY_DIR / cfg["outputs"]["results_markdown"])


if __name__ == "__main__":
    main()
