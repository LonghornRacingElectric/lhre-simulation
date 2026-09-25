"""Calibrate Orion pitch moments to a stated, provisional BobSim CoP prior.

Run inside bobdyn/bobsim:latest with this study directory mounted at /study.
The checked-in pre-calibration vehicle is the immutable input for reruns.
"""

from __future__ import annotations

import csv
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml


HERE = Path(__file__).resolve().parent
RAW = HERE / "orion_aeromap_precalibration_vehicle.yml"
VEHICLE = HERE / "vehicle_wip_2027_frontv19_rearv35.yml"
SOURCE = HERE / "orion_2026_aeromap_converged.csv"
PLOT = HERE / "orion_cop_calibration.png"
CSV = HERE / "orion_cop_sweep.csv"
TARGET = 0.5  # Front downforce fraction at middle grid point.
MIN_FRACTION = 0.05  # Explicit whole-grid modeling guardrail, not CFD evidence.


def fraction(my: np.ndarray, downforce: np.ndarray, ref_x: float,
             front_x: float, rear_x: float) -> np.ndarray:
    wheelbase = front_x - rear_x
    return (front_x + my / downforce + ref_x - front_x - rear_x) / wheelbase


def main() -> None:
    raw_text = RAW.read_text(encoding="utf-8")
    raw = yaml.safe_load(raw_text)
    aero = raw["aero"]
    downforce = np.asarray(aero["downforce_table_n"], dtype=float)
    drag = np.asarray(aero["drag_table_n"], dtype=float)
    original_my = np.asarray(aero["my_table_nm"], dtype=float)
    front = np.asarray(aero["front_ride_height_grid_m"], dtype=float)
    rear = np.asarray(aero["rear_ride_height_grid_m"], dtype=float)
    front_x = float(raw["front"]["suspension"]["wheel_center_m"][0])
    rear_x = float(raw["rear"]["suspension"]["wheel_center_m"][0])
    ref_x = float(aero["aero_ref_m"][0])
    wheelbase = front_x - rear_x
    assert wheelbase > 0 and downforce.shape == original_my.shape == drag.shape == (5, 5)
    assert np.all(np.isfinite(downforce)) and np.all(downforce > 0)
    assert np.all(np.isfinite(drag)) and np.all(drag > 0)
    assert np.all(np.diff(front) > 0) and np.all(np.diff(rear) > 0)

    with SOURCE.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    source_points = set()
    for row in rows:
        i = int(np.argmin(abs(front - float(row["front_ride_height_in"]) * 0.0254)))
        j = int(np.argmin(abs(rear - float(row["rear_ride_height_in"]) * 0.0254)))
        assert abs(front[i] - float(row["front_ride_height_in"]) * 0.0254) < 1e-8
        assert abs(rear[j] - float(row["rear_ride_height_in"]) * 0.0254) < 1e-8
        assert abs(downforce[i, j] - 2 * float(row["fz_n"])) < 1e-4
        assert abs(drag[i, j] - 2 * float(row["fx_n"])) < 1e-4
        assert abs(original_my[i, j] - 2 * (float(row["my_nm"]) - float(row["fz_n"]) * ref_x)) < 1e-4
        source_points.add((i, j))
    assert len(source_points) == 21

    original = fraction(original_my, downforce, ref_x, front_x, rear_x)
    anchor = float(original[2, 2])  # FRH 2.8 in, RRH 3.3 in; accepted CFD row.
    migration = original - anchor
    # Largest common gain that keeps *all 25 stored cells* at least 5% from
    # either axle. This also scales the four inferred cells; it never invents
    # a new CFD convergence result. Sign and relative migration are retained.
    lower_limit = (TARGET - MIN_FRACTION) / max(0.0, -float(migration.min()))
    upper_limit = (1 - MIN_FRACTION - TARGET) / max(0.0, float(migration.max()))
    gain = min(1.0, lower_limit, upper_limit)
    adjusted = TARGET + gain * migration
    adjusted_my = downforce * (rear_x + wheelbase * adjusted - ref_x)
    assert np.allclose(fraction(adjusted_my, downforce, ref_x, front_x, rear_x), adjusted)
    assert np.isclose(adjusted[2, 2], TARGET)
    assert np.all(adjusted >= MIN_FRACTION - 1e-12)
    assert np.all(adjusted <= 1 - MIN_FRACTION + 1e-12)
    # BobLib bilinearly interpolates force and moment separately. Sweep cell
    # interiors under that operation, rather than interpolating CoP directly.
    sampled = []
    for i in range(4):
        for j in range(4):
            for u in np.linspace(0, 1, 11):
                for v in np.linspace(0, 1, 11):
                    weights = np.array([[(1-u)*(1-v), (1-u)*v],
                                        [u*(1-v), u*v]])
                    sample_fz = float(np.sum(weights * downforce[i:i+2, j:j+2]))
                    sample_my = float(np.sum(weights * adjusted_my[i:i+2, j:j+2]))
                    sampled.append(float(fraction(np.array(sample_my), np.array(sample_fz),
                                                  ref_x, front_x, rear_x)))
    assert min(sampled) >= MIN_FRACTION - 1e-12
    assert max(sampled) <= 1 - MIN_FRACTION + 1e-12

    # Replace only the numeric My block; retain the rest of the WIP YAML verbatim.
    block = "  my_table_nm:\n" + "\n".join(
        "  - - " + format(float(row[0]), ".10f") + "\n" +
        "\n".join("    - " + format(float(v), ".10f") for v in row[1:])
        for row in adjusted_my
    ) + "\n"
    output, count = re.subn(r"(?ms)^  my_table_nm:\n.*?(?=^  mz_table_nm:)", lambda _: block, raw_text)
    assert count == 1
    VEHICLE.write_text(output, encoding="utf-8")
    loaded = yaml.safe_load(VEHICLE.read_text(encoding="utf-8"))
    assert np.allclose(loaded["aero"]["my_table_nm"], adjusted_my, atol=1e-9)
    for key in ("drag_table_n", "downforce_table_n", "aero_ref_m"):
        assert loaded["aero"][key] == aero[key]

    with CSV.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("front_ride_height_in", "rear_ride_height_in", "source_status",
                         "original_front_fraction", "adjusted_front_fraction", "downforce_n",
                         "drag_n", "adjusted_my_nm"))
        for i in range(5):
            for j in range(5):
                writer.writerow((f"{front[i]/0.0254:.4f}", f"{rear[j]/0.0254:.4f}",
                                 "accepted" if (i, j) in source_points else "inferred_fill",
                                 f"{original[i,j]:.9f}", f"{adjusted[i,j]:.9f}",
                                 f"{downforce[i,j]:.6f}", f"{drag[i,j]:.6f}",
                                 f"{adjusted_my[i,j]:.9f}"))

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.7), constrained_layout=True)
    for ax, data, title in zip(axes, (original, adjusted),
                               ("Stored map before calibration", "Provisional 50% CoP calibration")):
        im = ax.imshow(100 * data, origin="lower", vmin=-120, vmax=100,
                       extent=(0, 5, 0, 5), cmap="coolwarm")
        ax.set_xticks(np.arange(5) + .5, [f"{x/0.0254:g}" for x in rear])
        ax.set_yticks(np.arange(5) + .5, [f"{x/0.0254:g}" for x in front])
        ax.set_xlabel("Rear ride height (in)")
        ax.set_ylabel("Front ride height (in)")
        ax.set_title(title)
        for i in range(5):
            for j in range(5):
                ax.text(j + .5, i + .5, f"{100*data[i,j]:.1f}" + ("*" if (i,j) not in source_points else ""),
                        ha="center", va="center", fontsize=8)
    fig.colorbar(im, ax=axes, label="Front downforce fraction (%)", shrink=.8)
    fig.suptitle("Orion FRH/RRH CoP: * = inferred fill, middle cell anchored at 50%")
    fig.savefig(PLOT, dpi=170)
    plt.close(fig)
    print(f"Source rows: {len(source_points)}/21; inferred cells: {25-len(source_points)}")
    print(f"Anchor original={anchor:.6f}, target={TARGET:.6f}, migration gain={gain:.6f}")
    print(f"Original range={original.min():.6f}..{original.max():.6f}")
    print(f"Adjusted range={adjusted.min():.6f}..{adjusted.max():.6f}; out of wheelbase=0/25")
    print(f"Accepted source adjusted range={min(adjusted[i,j] for i,j in source_points):.6f}.."
          f"{max(adjusted[i,j] for i,j in source_points):.6f}")
    print(f"Bilinear interior sweep={len(sampled)} samples; range={min(sampled):.6f}..{max(sampled):.6f}")
    print("Static grid sanity check PASS; physical CFD frame and dynamic vehicle NOT validated")


if __name__ == "__main__":
    main()
