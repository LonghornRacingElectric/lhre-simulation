from __future__ import annotations

import csv
import math
import os
from pathlib import Path
from typing import Any

os.environ.setdefault(
    "MPLCONFIGDIR",
    str(Path(__file__).resolve().parent / ".matplotlib-cache"),
)

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import yaml


STUDY_DIR = Path(__file__).resolve().parent
RHO = 1.225


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


def average_point(a: list[float], b: list[float]) -> list[float]:
    return [0.5 * (float(a[i]) + float(b[i])) for i in range(3)]


def norm(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values))


def is_strictly_increasing(values: list[float]) -> bool:
    return all(float(b) > float(a) for a, b in zip(values, values[1:]))


def force_to_area(force_n: float, speed_mps: float) -> float:
    return force_n / (0.5 * RHO * speed_mps * speed_mps)


def table_shape_ok(table: list[list[float]], nx: int, ny: int) -> bool:
    return len(table) == nx and all(len(row) == ny for row in table)


def map_rows(vehicle: dict[str, Any]) -> list[dict[str, float]]:
    aero = vehicle["aero"]
    front_grid = [float(v) for v in aero["front_ride_height_grid_m"]]
    rear_grid = [float(v) for v in aero["rear_ride_height_grid_m"]]
    downforce = np.asarray(aero["downforce_table_n"], dtype=float)
    drag = np.asarray(aero["drag_table_n"], dtype=float)
    my = np.asarray(aero["my_table_nm"], dtype=float)
    ref_x = float(aero["aero_ref_m"][0])
    speed = float(aero["reference_speed_m_per_s"])

    rows: list[dict[str, float]] = []
    for i, front_rh in enumerate(front_grid):
        for j, rear_rh in enumerate(rear_grid):
            df = float(downforce[i, j])
            dr = float(drag[i, j])
            my_free = float(my[i, j])
            rows.append(
                {
                    "front_ride_height_m": front_rh,
                    "rear_ride_height_m": rear_rh,
                    "rake_m": rear_rh - front_rh,
                    "downforce_n": df,
                    "drag_n": dr,
                    "cl_a_m2": force_to_area(df, speed),
                    "cd_a_m2": force_to_area(dr, speed),
                    "downforce_to_drag": df / dr if dr else math.nan,
                    "my_free_nm": my_free,
                    "equivalent_vertical_load_x_m": ref_x + my_free / df if df else math.nan,
                }
            )
    return rows


def plot_heatmap(rows: list[dict[str, float]], key: str, title: str, output_path: Path) -> None:
    front = sorted({row["front_ride_height_m"] for row in rows})
    rear = sorted({row["rear_ride_height_m"] for row in rows})
    z = np.full((len(front), len(rear)), np.nan)
    for row in rows:
        i = front.index(row["front_ride_height_m"])
        j = rear.index(row["rear_ride_height_m"])
        z[i, j] = row[key]

    fig, ax = plt.subplots(figsize=(7.2, 5.0))
    image = ax.imshow(z, origin="lower", aspect="auto", cmap="viridis")
    fig.colorbar(image, ax=ax, label=key)
    ax.set_xticks(range(len(rear)), [f"{v * 1000:.0f}" for v in rear])
    ax.set_yticks(range(len(front)), [f"{v * 1000:.0f}" for v in front])
    ax.set_xlabel("Rear ride height [mm]")
    ax.set_ylabel("Front ride height [mm]")
    ax.set_title(title)
    for i in range(len(front)):
        for j in range(len(rear)):
            ax.text(j, i, f"{z[i, j]:.0f}", ha="center", va="center", color="white", fontsize=8)
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def write_results(
    cfg: dict[str, Any],
    summary: dict[str, Any],
    output_path: Path,
) -> None:
    status = "PASS" if summary["accepted"] else "CHECK"
    lines = [
        "# AERO-001 Results",
        "",
        "## Decision Question",
        "",
        cfg["study"]["decision_question"],
        "",
        "## Finding",
        "",
        f"**{status}:** the aero map is coherent enough to proceed to platform sensitivity studies.",
        "",
        "This study does not claim the aero package is optimal. It only verifies that the map can be interpreted and cited.",
        "",
        "## Reference And Shape Checks",
        "",
        f"- Front/rear ride-height grids monotonic: `{summary['ride_height_grids_monotonic']}`",
        f"- Force/moment table shapes valid: `{summary['table_shapes_valid']}`",
        f"- Front reference error: `{summary['front_reference_error_m']:.6e} m`",
        f"- Rear reference error: `{summary['rear_reference_error_m']:.6e} m`",
        "",
        "## Baseline Map Point",
        "",
        f"- Baseline front/rear ride height: `{summary['baseline_front_ride_height_m']:.5f} m` / `{summary['baseline_rear_ride_height_m']:.5f} m`",
        f"- Downforce/drag at 15 m/s: `{summary['baseline_downforce_n']:.1f} N` / `{summary['baseline_drag_n']:.1f} N`",
        f"- ClA/CdA: `{summary['baseline_cl_a_m2']:.3f} m^2` / `{summary['baseline_cd_a_m2']:.3f} m^2`",
        f"- Downforce/drag ratio: `{summary['baseline_downforce_to_drag']:.3f}`",
        "",
        "## Map Ranges",
        "",
        f"- Downforce range: `{summary['downforce_min_n']:.1f}` to `{summary['downforce_max_n']:.1f} N`",
        f"- Drag range: `{summary['drag_min_n']:.1f}` to `{summary['drag_max_n']:.1f} N`",
        f"- Equivalent vertical-load x range from pitch moment: `{summary['equivalent_x_min_m']:.3f}` to `{summary['equivalent_x_max_m']:.3f} m`",
        "",
        "![Downforce map](plots/downforce_map.png)",
        "",
        "![Drag map](plots/drag_map.png)",
        "",
        "## What This Enables",
        "",
        "`AERO-002-platform-sensitivity` may now evaluate ride-height, rake, downforce, and drag effects with the map references and dimensions established.",
        "",
        "## Correlation Closure",
        "",
        "Use coastdown, aero-on/off, and ride-height-vs-speed testing to validate drag and load trends. Convert pitch moment into a front/rear load split before making final balance claims.",
    ]
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    cfg = load_yaml(STUDY_DIR / "study.yml")
    vehicle_path = resolve_from(STUDY_DIR / "study.yml", cfg["inputs"]["vehicle_config"])
    vehicle = load_yaml(vehicle_path)
    aero = vehicle["aero"]
    front_grid = [float(v) for v in aero["front_ride_height_grid_m"]]
    rear_grid = [float(v) for v in aero["rear_ride_height_grid_m"]]
    nx = len(front_grid)
    ny = len(rear_grid)

    front_avg = average_point(
        vehicle["front"]["suspension"]["lower_fore_i_m"],
        vehicle["front"]["suspension"]["lower_aft_i_m"],
    )
    rear_avg = average_point(
        vehicle["rear"]["suspension"]["lower_fore_i_m"],
        vehicle["rear"]["suspension"]["lower_aft_i_m"],
    )
    front_ref = [float(v) for v in aero["front_left_ride_height_ref_m"]]
    rear_ref = [float(v) for v in aero["rear_left_ride_height_ref_m"]]
    front_error = norm([front_ref[i] - front_avg[i] for i in range(3)])
    rear_error = norm([rear_ref[i] - rear_avg[i] for i in range(3)])
    rows = map_rows(vehicle)
    baseline = next(
        row
        for row in rows
        if row["front_ride_height_m"] == front_grid[0] and row["rear_ride_height_m"] == rear_grid[0]
    )
    table_shapes_valid = all(
        table_shape_ok(aero[name], nx, ny)
        for name in [
            "downforce_table_n",
            "drag_table_n",
            "mx_table_nm",
            "my_table_nm",
            "mz_table_nm",
        ]
    )
    grids_monotonic = is_strictly_increasing(front_grid) and is_strictly_increasing(rear_grid)
    accepted = table_shapes_valid and grids_monotonic and front_error < 1e-9 and rear_error < 1e-9
    summary = {
        "accepted": accepted,
        "ride_height_grids_monotonic": grids_monotonic,
        "table_shapes_valid": table_shapes_valid,
        "front_reference_error_m": front_error,
        "rear_reference_error_m": rear_error,
        "baseline_front_ride_height_m": baseline["front_ride_height_m"],
        "baseline_rear_ride_height_m": baseline["rear_ride_height_m"],
        "baseline_downforce_n": baseline["downforce_n"],
        "baseline_drag_n": baseline["drag_n"],
        "baseline_cl_a_m2": baseline["cl_a_m2"],
        "baseline_cd_a_m2": baseline["cd_a_m2"],
        "baseline_downforce_to_drag": baseline["downforce_to_drag"],
        "downforce_min_n": min(row["downforce_n"] for row in rows),
        "downforce_max_n": max(row["downforce_n"] for row in rows),
        "drag_min_n": min(row["drag_n"] for row in rows),
        "drag_max_n": max(row["drag_n"] for row in rows),
        "equivalent_x_min_m": min(row["equivalent_vertical_load_x_m"] for row in rows),
        "equivalent_x_max_m": max(row["equivalent_vertical_load_x_m"] for row in rows),
    }

    outputs = STUDY_DIR / cfg["outputs"]["data_dir"]
    plots = STUDY_DIR / cfg["outputs"]["figures_dir"]
    write_csv(outputs / "aero_map_points.csv", rows)
    write_csv(outputs / "summary.csv", [summary])
    plot_heatmap(rows, "downforce_n", "AERO-001 Downforce Map", plots / "downforce_map.png")
    plot_heatmap(rows, "drag_n", "AERO-001 Drag Map", plots / "drag_map.png")
    write_results(cfg, summary, STUDY_DIR / cfg["outputs"]["results_markdown"])


if __name__ == "__main__":
    main()
